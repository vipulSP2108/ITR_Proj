"""Glue: turn a collected session into observations, run the joint solver, cross-check the cameras."""
from __future__ import annotations
import csv, glob, json
from pathlib import Path
import cv2
import numpy as np

from .handeye import Obs, solve
from .intrinsics import calibrate_intrinsics, load_intrinsics, save_intrinsics
from .session import SessionStore
from .transforms import average_poses, inv_T, pose_diff, T_to_xyz_quat


def capture_board_pose(store, cid, cam, K, dist, board, min_corners=8):
    z = np.load(store.capture_path(cid) / "detections.npz")
    Ts = []
    for key in z.files:
        if key.startswith(cam + "__") and key.endswith("__corners"):
            c, ids = z[key], z[key.replace("__corners", "__ids")]
            T, _ = board.estimate_pose(c, ids, K, dist, min_corners=min_corners)
            if T is not None:
                Ts.append(T)
    if len(Ts) < 3:
        return None, None
    T, _, st = average_poses(Ts, rot_tol_deg=2.0, trans_tol_m=0.005)
    if st["n_inliers"] < 3:
        return None, None
    return T, st


def session_images(store: SessionStore, cam: str):
    out = []
    for cid in store.state()["live_ids"]:
        for f in sorted(glob.glob(str(store.capture_path(cid) / f"{cam}*.png"))):
            img = cv2.imread(f)
            if img is not None:
                out.append(img)
    return out


def get_intrinsics(cam, store, board, intr_dir: Path, cfg, log):
    f = Path(intr_dir) / f"{cam}.json"
    if f.exists():
        K, d, meta = load_intrinsics(f)
        log(f"[{cam}] using intrinsics {f} (RMS {meta.get('rms_px', float('nan')):.3f}px)")
        return K, d
    log(f"[{cam}] !! no {f} - SELF-CALIBRATING from the session images. This works but a dedicated intrinsics "
        f"run (capture_intrinsics.py + calibrate_intrinsics.py) is more accurate.")
    res = calibrate_intrinsics(board, session_images(store, cam), min_corners=cfg["capture"]["min_corners"], log=log)
    save_intrinsics(res, f)
    for w in res["warnings"]:
        log("   WARNING: " + w)
    return np.array(res["K"]), np.array(res["dist"])


def build_observations(store, cfg, board, intr, log):
    obs, board_poses = [], {}
    cams = list(cfg["cameras"])
    for cid in store.state()["live_ids"]:
        meta = store.load_capture_meta(cid)
        T_be = np.array(meta["T_base_ee"])
        for cam in cams:
            K, dist = intr[cam]
            T_cb, st = capture_board_pose(store, cid, cam, K, dist, board, cfg["capture"]["min_corners"])
            if T_cb is None:
                log(f"  capture #{cid} {cam}: board not usable, skipped")
                continue
            board_poses[(cid, cam)] = T_cb
            obs.append(Obs(cid, meta["matrix"], cam, T_be, T_cb))
    return obs, board_poses


def direct_stereo(board_poses, cams):
    """Camera-to-camera transform measured straight from simultaneous board views (needs no robot at all)."""
    if len(cams) < 2:
        return None
    c0, c1 = cams[0], cams[1]
    Ts = [board_poses[(cid, c1)] @ inv_T(board_poses[(cid, c0)]) for (cid, c) in board_poses
          if c == c0 and (cid, c1) in board_poses]
    if not Ts:
        return None
    T, ok, st = average_poses(Ts, rot_tol_deg=2.0, trans_tol_m=0.01)
    return {"pair": [c0, c1], "T_c1_c0": T, "stats": st}


def implied_stereo(sol, obs, cams):
    if len(cams) < 2:
        return None
    c0, c1 = cams[0], cams[1]
    by = {}
    for o in obs:
        by.setdefault(o.cap, {})[o.cam] = o

    def Tb(o, c):
        return sol["X"][c] if sol["mounts"][c] == "fixed" else o.T_be @ sol["X"][c]
    Ts = [inv_T(Tb(d[c1], c1)) @ Tb(d[c0], c0) for d in by.values() if c0 in d and c1 in d]
    if not Ts:
        return None
    T, _, st = average_poses(Ts, 2.0, 0.01)
    return {"pair": [c0, c1], "T_c1_c0": T, "stats": st}


def xyzq(T):
    p, q = T_to_xyz_quat(T)
    return {"T": T.tolist(), "xyz_m": p, "quat_xyzw": q}


def build_report(sol, obs, stereo_direct, stereo_implied, cfg):
    out = {"board_mount": sol["board_mount"], "cameras": {}, "stats": sol["stats"], "warnings": list(sol["warnings"])}
    tip, base = cfg["robot"]["tip_link"], cfg["robot"]["base_link"]
    for c, T in sol["X"].items():
        if sol["mounts"][c] == "fixed":
            out["cameras"][c] = {"mount": "fixed", "parent": base, "T_base_cam": xyzq(T)}
        else:
            out["cameras"][c] = {"mount": "hand", "parent": tip, "T_ee_cam": xyzq(T)}
    if sol["Y"] is not None:
        out["T_ee_board"] = xyzq(sol["Y"])
    out["T_base_board_per_matrix"] = {str(g): xyzq(T) for g, T in sol["B"].items()}
    if stereo_implied:
        out["stereo_from_model"] = {"pair": stereo_implied["pair"], **xyzq(stereo_implied["T_c1_c0"]),
                                    "stats": stereo_implied["stats"]}
    if stereo_direct:
        out["stereo_direct"] = {"pair": stereo_direct["pair"], **xyzq(stereo_direct["T_c1_c0"]),
                                "stats": stereo_direct["stats"]}
    if stereo_direct and stereo_implied:
        r, t = pose_diff(stereo_direct["T_c1_c0"], stereo_implied["T_c1_c0"])
        out["stereo_consistency"] = {"rot_deg": r, "trans_mm": t * 1000}
        if r > 1.0 or t > 0.01:
            out["warnings"].append(f"Camera-to-camera transform from the robot model disagrees with the direct "
                                   f"board measurement by {r:.2f} deg / {t*1000:.1f} mm -> check mount settings, "
                                   "intrinsics, or look at per-observation residuals for bad captures.")
    return out


def write_outputs(out_dir: Path, report: dict, sol: dict, cfg: dict):
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "extrinsics.json").write_text(json.dumps(report, indent=1))
    with open(out_dir / "per_observation_residuals.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["cap", "group", "cam", "trans_mm", "rot_deg", "used"])
        w.writeheader()
        w.writerows(sol["per_obs"])
    lines = ["# Paste into a launch script / run in a terminal to publish the calibration as TF:"]
    for c, d in report["cameras"].items():
        T = d.get("T_base_cam") or d.get("T_ee_cam")
        x, y, z = T["xyz_m"]; qx, qy, qz, qw = T["quat_xyzw"]
        lines.append(f"ros2 run tf2_ros static_transform_publisher --x {x:.6f} --y {y:.6f} --z {z:.6f} "
                     f"--qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} "
                     f"--frame-id {d['parent']} --child-frame-id {c}_optical_frame")
    (out_dir / "publish_tf.sh").write_text("\n".join(lines) + "\n")
