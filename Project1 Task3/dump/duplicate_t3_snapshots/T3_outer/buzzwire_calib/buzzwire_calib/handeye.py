"""General multi-camera hand-eye / eye-to-hand solver.

Model (all transforms are 4x4, T_a_b = pose of b expressed in a):
  * board_mount == "ee"    : board rigidly on the end effector   -> board in base:  B_i = T_base_ee_i @ Y   (Y unknown)
  * board_mount == "world"  : board static during one 'matrix'   -> board in base:  B_i = B_group(i)        (unknown per matrix)
  * camera mount "fixed"    : camera static in the world         -> T_base_cam = X_c                         (unknown)
  * camera mount "hand"     : camera on the arm                  -> T_base_cam_i = T_base_ee_i @ X_c         (X_c = T_ee_cam)
  Measurement: T_cam_board_i (from PnP).   Prediction: inv(T_base_cam_i) @ B_i.

All cameras are fitted JOINTLY (they share Y or B_k), which is what ties the two cameras together
and averages out per-camera noise. A linear AX=XB solve initialises; robust non-linear least squares refines.
"""
from __future__ import annotations
from dataclasses import dataclass
import itertools
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

from .transforms import inv_T, make_T, T_to_params, params_to_T, average_poses

W_ROT = 0.3   # metres per radian when mixing rotation & translation residuals


@dataclass
class Obs:
    cap: int
    group: int
    cam: str
    T_be: np.ndarray       # base <- ee  (robot FK)
    T_cb: np.ndarray       # cam  <- board (PnP, already burst-averaged)


# --------------------------------------------------------------------------- linear initialisation
def _solve_AXXB(pairs):
    """Solve A X = X B for X from many (A, B) pairs (Park-Martin style, pooled)."""
    a, b = [], []
    use = []
    for A, B in pairs:
        ra, rb = Rot.from_matrix(A[:3, :3]).as_rotvec(), Rot.from_matrix(B[:3, :3]).as_rotvec()
        if np.linalg.norm(ra) < np.radians(4) or np.linalg.norm(rb) < np.radians(4):
            continue
        a.append(ra); b.append(rb); use.append((A, B))
    if len(a) < 2:
        raise RuntimeError("not enough rotational motion between captures for hand-eye initialisation "
                           "(need >= 2 motions with >5 deg rotation about DIFFERENT axes)")
    a, b = np.array(a), np.array(b)
    H = b.T @ a
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rx = Vt.T @ D @ U.T
    M = np.vstack([A[:3, :3] - np.eye(3) for A, _ in use])
    v = np.concatenate([Rx @ B[:3, 3] - A[:3, 3] for A, B in use])
    tx = np.linalg.lstsq(M, v, rcond=None)[0]
    return make_T(Rx, tx)


def _pairs(obs_list, kind, max_pairs=1500):
    """kind 'hand'(world board) : A=inv(Tbe_j) Tbe_i ; kind 'fixed_ee': A=Tbe_j inv(Tbe_i)."""
    by_group = {}
    for o in obs_list:
        by_group.setdefault(o.group, []).append(o)
    out = []
    for g, lst in by_group.items():
        for i, j in itertools.combinations(range(len(lst)), 2):
            oi, oj = lst[i], lst[j]
            A = inv_T(oj.T_be) @ oi.T_be if kind == "hand" else oj.T_be @ inv_T(oi.T_be)
            B = oj.T_cb @ inv_T(oi.T_cb)
            out.append((A, B))
    if len(out) > max_pairs:
        rng = np.random.default_rng(0)
        out = [out[k] for k in rng.choice(len(out), max_pairs, replace=False)]
    return out


# --------------------------------------------------------------------------- model
class Problem:
    def __init__(self, obs, cam_mounts, board_mount):
        self.obs = list(obs)
        self.cams = sorted(cam_mounts)
        self.mounts = cam_mounts
        self.bm = board_mount
        self.groups = sorted({o.group for o in obs}) if board_mount == "world" else []
        # parameter layout
        self.idx = {}
        k = 0
        for c in self.cams:
            self.idx[("X", c)] = slice(k, k + 6); k += 6
        if board_mount == "ee":
            self.idx[("Y",)] = slice(k, k + 6); k += 6
        else:
            for g in self.groups:
                self.idx[("B", g)] = slice(k, k + 6); k += 6
        self.n = k

    def unpack(self, p):
        X = {c: params_to_T(p[self.idx[("X", c)]]) for c in self.cams}
        Y = params_to_T(p[self.idx[("Y",)]]) if self.bm == "ee" else None
        B = {g: params_to_T(p[self.idx[("B", g)]]) for g in self.groups}
        return X, Y, B

    def predict(self, o: Obs, X, Y, B):
        B_i = o.T_be @ Y if self.bm == "ee" else B[o.group]
        Tb_cam = X[o.cam] if self.mounts[o.cam] == "fixed" else o.T_be @ X[o.cam]
        return inv_T(Tb_cam) @ B_i

    def residual_one(self, o, X, Y, B):
        d = inv_T(self.predict(o, X, Y, B)) @ o.T_cb
        return np.concatenate([Rot.from_matrix(d[:3, :3]).as_rotvec() * W_ROT, d[:3, 3]])

    def residuals(self, p, mask=None):
        X, Y, B = self.unpack(p)
        return np.concatenate([self.residual_one(o, X, Y, B) for k, o in enumerate(self.obs)
                               if mask is None or mask[k]])


def _initial(prob: Problem, log):
    p = np.zeros(prob.n)
    X, Y, B = {}, None, {}
    by_cam = {c: [o for o in prob.obs if o.cam == c] for c in prob.cams}
    if prob.bm == "ee":
        for c in prob.cams:
            if prob.mounts[c] != "fixed":
                raise ValueError("board_mount 'ee' (board on gripper) only makes sense with FIXED cameras")
            X[c] = _solve_AXXB(_pairs(by_cam[c], "fixed_ee"))
        ys = [inv_T(o.T_be) @ X[o.cam] @ o.T_cb for o in prob.obs]
        Y, _, _ = average_poses(ys, rot_tol_deg=10, trans_tol_m=0.05)
    else:
        hand = [c for c in prob.cams if prob.mounts[c] == "hand"]
        fixed = [c for c in prob.cams if prob.mounts[c] == "fixed"]
        if not hand:
            raise ValueError("board_mount 'world' with only FIXED cameras is unobservable (nothing links the "
                             "camera to the robot). Put the board on the gripper (board_mount: ee) or use a hand camera.")
        for c in hand:
            X[c] = _solve_AXXB(_pairs(by_cam[c], "hand"))
        # board pose per matrix from hand cameras
        for g in prob.groups:
            cand = [o.T_be @ X[o.cam] @ o.T_cb for o in prob.obs if o.group == g and o.cam in hand]
            if cand:
                B[g] = average_poses(cand, 10, 0.05)[0]
        for c in fixed:
            est = [B[o.group] @ inv_T(o.T_cb) for o in by_cam[c] if o.group in B]
            if not est:
                raise ValueError(f"fixed camera {c} never shares a capture with a hand camera")
            X[c] = average_poses(est, 10, 0.05)[0]
        for g in prob.groups:
            if g not in B:   # matrix seen only by fixed cams: use fixed-cam estimate
                cand = [X[o.cam] @ o.T_cb for o in prob.obs if o.group == g]
                B[g] = average_poses(cand, 10, 0.05)[0]
    for c in prob.cams:
        p[prob.idx[("X", c)]] = T_to_params(X[c])
    if prob.bm == "ee":
        p[prob.idx[("Y",)]] = T_to_params(Y)
    for g in prob.groups:
        p[prob.idx[("B", g)]] = T_to_params(B[g])
    return p


def solve(obs, cam_mounts: dict, board_mount: str, log=print, reject: bool = True):
    """Returns dict with transforms, per-observation residuals and quality warnings."""
    obs = list(obs)
    prob = Problem(obs, cam_mounts, board_mount)
    p0 = _initial(prob, log)
    mask = np.ones(len(obs), bool)
    p = p0
    for rnd in range(3):
        sub = [o for o, m in zip(obs, mask) if m]
        sprob = Problem(sub, cam_mounts, board_mount)
        # keep parameter layout identical even if a whole group vanished
        if sprob.n != prob.n:
            break
        sol = least_squares(sprob.residuals, p, loss="soft_l1", f_scale=0.004, x_scale=1.0, max_nfev=300)
        p = sol.x
        r = np.linalg.norm(prob.residuals(p).reshape(-1, 6), axis=1)
        if not reject:
            break
        thr = max(4.0 * np.median(r), 0.006)
        new_mask = r <= thr
        if new_mask.sum() < max(6, 0.6 * len(obs)) or (new_mask == mask).all():
            break
        log(f"  rejecting {int((~new_mask).sum())} outlier observation(s) (residual > {thr*1000:.1f} mm-equiv.)")
        mask = new_mask
    X, Y, B = prob.unpack(p)
    res = np.array([prob.residual_one(o, X, Y, B) for o in obs])
    rot_deg = np.degrees(np.linalg.norm(res[:, :3] / W_ROT, axis=1))
    trans_mm = np.linalg.norm(res[:, 3:], axis=1) * 1000
    per_obs = [{"cap": o.cap, "group": o.group, "cam": o.cam, "trans_mm": float(t), "rot_deg": float(r_),
                "used": bool(m)} for o, t, r_, m in zip(obs, trans_mm, rot_deg, mask)]
    stats = {}
    for c in prob.cams:
        sel = [i for i, o in enumerate(obs) if o.cam == c and mask[i]]
        stats[c] = {"n_used": len(sel),
                    "trans_rms_mm": float(np.sqrt(np.mean(trans_mm[sel] ** 2))) if sel else None,
                    "rot_rms_deg": float(np.sqrt(np.mean(rot_deg[sel] ** 2))) if sel else None}
    warnings = _quality_warnings(prob, obs, mask, stats)
    return {"X": X, "Y": Y, "B": B, "mounts": cam_mounts, "board_mount": board_mount, "per_obs": per_obs,
            "stats": stats, "warnings": warnings}


def _quality_warnings(prob, obs, mask, stats):
    w = []
    for c in prob.cams:
        sel = [o for o, m in zip(obs, mask) if o.cam == c and m]
        if len(sel) < 10:
            w.append(f"{c}: only {len(sel)} usable observations (aim for >= 15-20 spread over several matrices).")
        if len(sel) >= 2:
            Rs = Rot.from_matrix(np.array([o.T_be[:3, :3] for o in sel]))
            ang = max(np.degrees(np.linalg.norm((Rs[0].inv() * r).as_rotvec())) for r in Rs)
            if ang < 30:
                w.append(f"{c}: end-effector orientation varied by only {ang:.0f} deg - rotate the tool more "
                         "(>= 45 deg about at least two axes) for a well-conditioned hand-eye solution.")
    if prob.bm == "world" and len(prob.groups) < 3:
        w.append("Only %d matrix placement(s) - use >= 4 for a robust result." % len(prob.groups))
    for c, s in stats.items():
        if s["trans_rms_mm"] is not None and s["trans_rms_mm"] > 5:
            w.append(f"{c}: residual {s['trans_rms_mm']:.1f} mm RMS is high (good: < 3 mm). Check intrinsics, board "
                     "square size, camera 'mount' setting in the config, and that the board was flat/rigid.")
    return w
