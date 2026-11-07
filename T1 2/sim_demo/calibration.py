"""
Eye-to-hand calibration from arm motion.

Idea: the arm's geometry is known (arm_model.py).  We move the arm to a set of poses, read the joint
encoders -> forward kinematics gives the tool-marker position in the ROBOT frame.  At the same time
the mounted RGB-D camera sees the marker -> its position in the CAMERA frame.  The rigid transform
that maps camera points onto robot points (Kabsch / SVD) is the camera extrinsic T_robot_cam.

Nothing here reads simulator ground truth; the sim's true camera pose is only used afterwards, in
run_pipeline.py, to REPORT how accurate the estimate was.
"""
import numpy as np
import cv2

import arm_model as A
from perception import detect_marker
from scene_builder import MARKER_R


def kabsch(P_cam: np.ndarray, P_rob: np.ndarray):
    """Least-squares rigid transform with P_rob ~= R P_cam + t. Returns 4x4."""
    cc, cr = P_cam.mean(0), P_rob.mean(0)
    H = (P_cam - cc).T @ (P_rob - cr)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = cr - R @ cc
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def calibration_targets(max_poses=16):
    """Candidate TCP targets (robot frame) + tilt that keep the marker over the visible workspace."""
    cands = []
    for z in (0.07, 0.12, 0.17):
        for x in (0.10, 0.15, 0.20):
            for y in (-0.12, -0.04, 0.04, 0.12):
                for tilt_deg in (0.0, 35.0):
                    q = A.ik(x, y, z, np.radians(tilt_deg))
                    if q is not None:
                        cands.append((x, y, z, tilt_deg, q))
    rng = np.random.default_rng(3)
    order = rng.permutation(len(cands))
    return [cands[i] for i in order[:max_poses]]


def run_calibration(env, cfg, on_pose=None, max_poses=16, settle_s=0.35, move_s=1.1):
    """
    Drives the arm through calibration poses and solves for T_robot_cam.
    on_pose(i, bgr, depth, det, P_fk_robot) lets the caller save images.
    Returns dict(T_robot_cam, rms_mm, per_pose=[...], n_used).
    """
    samples = []
    for i, (x, y, z, tilt_deg, q) in enumerate(calibration_targets(max_poses)):
        env.move_joints(q, move_s); env.settle(settle_s)
        enc = env.read_joints()                                   # what the real encoders would report
        P_rob = A.fk(enc)["marker"]                               # known arm geometry -> marker in robot frame
        bgr, depth = env.capture_rgbd()
        det = detect_marker(bgr, depth, env.cam, cfg, MARKER_R)
        if on_pose:
            on_pose(i, bgr, depth, det, P_rob)
        if det is None:
            samples.append(dict(i=i, ok=False, target=(x, y, z, tilt_deg)))
            continue
        samples.append(dict(i=i, ok=True, target=(x, y, z, tilt_deg), P_cam=det["P_cam"], P_rob=P_rob,
                            uv=(det["u"], det["v"])))
    ok = [s for s in samples if s["ok"]]
    if len(ok) < 6:
        raise RuntimeError(f"calibration needs >=6 marker detections, got {len(ok)}")
    Pc = np.array([s["P_cam"] for s in ok]); Pr = np.array([s["P_rob"] for s in ok])
    T = kabsch(Pc, Pr)
    res = np.linalg.norm((T[:3, :3] @ Pc.T).T + T[:3, 3] - Pr, axis=1)
    keep = res < max(3.0 * np.median(res), 0.006)          # one outlier-rejection pass
    if keep.sum() >= 6 and keep.sum() < len(ok):
        T = kabsch(Pc[keep], Pr[keep])
        res = np.linalg.norm((T[:3, :3] @ Pc.T).T + T[:3, 3] - Pr, axis=1)
    for s, r, k in zip(ok, res, keep):
        s["residual_mm"] = float(r * 1000); s["used"] = bool(k)
    used = [s for s in ok if s["used"]]
    rms = float(np.sqrt(np.mean([s["residual_mm"] ** 2 for s in used])))
    # pixel reprojection error: project the FK marker through the estimated transform
    Tinv = np.linalg.inv(T)
    for s in ok:
        pc = (Tinv[:3, :3] @ s["P_rob"]) + Tinv[:3, 3]
        uv = env.cam.project(pc)[0]
        s["reproj_px"] = float(np.hypot(uv[0] - s["uv"][0], uv[1] - s["uv"][1]))
    return dict(T_robot_cam=T, rms_mm=rms, per_pose=samples, n_detected=len(ok), n_used=len(used),
                n_attempted=len(samples))


def transform_error(T_est, T_true):
    """(translation error mm, rotation error deg) between two 4x4 transforms."""
    dt = np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]) * 1000.0
    R = T_est[:3, :3].T @ T_true[:3, :3]
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    return float(dt), float(ang)
