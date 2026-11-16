"""Small SE(3) helpers + robust pose averaging."""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    return make_T(Rot.from_rotvec(np.asarray(rvec, float).reshape(3)).as_matrix(),
                  np.asarray(tvec, float).reshape(3))


def T_to_params(T: np.ndarray) -> np.ndarray:
    """4x4 -> [rotvec(3), t(3)]"""
    return np.concatenate([Rot.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


def params_to_T(p) -> np.ndarray:
    return rvec_tvec_to_T(p[:3], p[3:6])


def quat_to_T(xyz, quat_xyzw) -> np.ndarray:
    return make_T(Rot.from_quat(quat_xyzw).as_matrix(), xyz)


def T_to_xyz_quat(T: np.ndarray):
    return T[:3, 3].tolist(), Rot.from_matrix(T[:3, :3]).as_quat().tolist()  # xyzw


def pose_diff(T_a: np.ndarray, T_b: np.ndarray):
    """(rotation error deg, translation error m) between two poses."""
    d = inv_T(T_a) @ T_b
    return float(np.degrees(np.linalg.norm(Rot.from_matrix(d[:3, :3]).as_rotvec()))), float(np.linalg.norm(d[:3, 3]))


def average_poses(Ts, rot_tol_deg: float = 3.0, trans_tol_m: float = 0.01):
    """Robust mean of a list of 4x4 poses.

    Outliers (far from the median pose) are rejected first, then translation is
    averaged arithmetically and rotation with the chordal (quaternion) mean.
    Returns (T_mean, inlier_mask, stats_dict).
    """
    Ts = [np.asarray(T, float) for T in Ts]
    n = len(Ts)
    if n == 0:
        raise ValueError("no poses")
    if n == 1:
        return Ts[0].copy(), np.array([True]), {"n": 1, "n_inliers": 1, "rot_std_deg": 0.0, "trans_std_m": 0.0}
    t = np.array([T[:3, 3] for T in Ts])
    Rs = np.array([T[:3, :3] for T in Ts])
    ok = np.ones(n, bool)
    for _ in range(2):
        R_ref = Rot.from_matrix(Rs[ok]).mean()
        t_ref = np.median(t[ok], axis=0)
        d_t = np.linalg.norm(t - t_ref, axis=1)
        d_r = np.array([np.degrees(np.linalg.norm((R_ref.inv() * Rot.from_matrix(R)).as_rotvec())) for R in Rs])
        new_ok = (d_t <= max(trans_tol_m, 3 * np.median(d_t))) & (d_r <= max(rot_tol_deg, 3 * np.median(d_r)))
        if new_ok.sum() < max(1, n // 2):
            break
        ok = new_ok
    Rm = Rot.from_matrix(Rs[ok]).mean()
    tm = t[ok].mean(axis=0)
    d_r = [np.degrees(np.linalg.norm((Rm.inv() * Rot.from_matrix(R)).as_rotvec())) for R in Rs[ok]]
    stats = {"n": n, "n_inliers": int(ok.sum()),
             "rot_std_deg": float(np.sqrt(np.mean(np.square(d_r)))),
             "trans_std_m": float(np.sqrt(np.mean(np.sum((t[ok] - tm) ** 2, axis=1))))}
    return make_T(Rm.as_matrix(), tm), ok, stats
