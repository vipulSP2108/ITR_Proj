"""Per-camera intrinsic calibration from ChArUco views (with automatic bad-view pruning)."""
from __future__ import annotations
import json
from pathlib import Path
import cv2
import numpy as np


def calibrate_intrinsics(board, images, min_corners: int = 10, prune: bool = True, rational: bool = False,
                         log=print):
    obj_l, img_l, used = [], [], []
    size = None
    for idx, img in enumerate(images):
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            log(f"  skipping image {idx}: size {img.shape[1]}x{img.shape[0]} != {size}")
            continue
        c, ids = board.detect(img)
        if c is None or len(ids) < min_corners:
            continue
        obj_l.append(board.obj_points(ids).reshape(-1, 1, 3).astype(np.float32))
        img_l.append(c.reshape(-1, 1, 2).astype(np.float32))
        used.append(idx)
    # drop near-duplicate views (same board placement seen repeatedly adds no information, only bias)
    keep, descr = [], []
    for k, c in enumerate(img_l):
        pts = c.reshape(-1, 2)
        d = np.array([pts[:, 0].mean(), pts[:, 1].mean(), 3.0 * pts.std()])
        if all(np.linalg.norm(d[:2] - e[:2]) > 25 or abs(d[2] - e[2]) > 0.05 * e[2] for e in descr):
            descr.append(d); keep.append(k)
    if len(keep) < len(obj_l):
        log(f"  {len(obj_l) - len(keep)} near-duplicate views ignored ({len(keep)} distinct views remain)")
    obj_l, img_l, used = [obj_l[k] for k in keep], [img_l[k] for k in keep], [used[k] for k in keep]
    if len(obj_l) < 8:
        raise RuntimeError(f"only {len(obj_l)} DISTINCT board views (need >= 8, ideally 20-30 at varied tilt/distance "
                           "covering the whole image). A camera that sees the board in only a few fixed placements "
                           "cannot be calibrated reliably from those - run scripts/capture_intrinsics.py for it.")
    flags = cv2.CALIB_RATIONAL_MODEL if rational else 0
    for it in range(4):
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_l, img_l, size, None, None, flags=flags)
        errs = []
        for o, i, rv, tv in zip(obj_l, img_l, rvecs, tvecs):
            p, _ = cv2.projectPoints(o, rv, tv, K, dist)
            errs.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
        errs_a = np.array(errs)
        log(f"  intrinsics pass {it}: {len(obj_l)} views, RMS {rms:.3f}px, worst view {errs_a.max():.2f}px")
        if not prune:
            break
        bad = errs_a > max(2.0 * np.median(errs_a), 0.8)
        if not bad.any() or len(obj_l) - bad.sum() < 8:
            break
        keep = ~bad
        log(f"    pruning {int(bad.sum())} bad views")
        obj_l = [o for o, k in zip(obj_l, keep) if k]
        img_l = [o for o, k in zip(img_l, keep) if k]
        used = [u for u, k in zip(used, keep) if k]
        errs = [e for e, k in zip(errs, keep) if k]
    fx = K[0, 0]
    if not (0.3 * size[0] < fx < 4.0 * size[0]):
        raise RuntimeError(f"implausible focal length {fx:.0f}px for a {size[0]}px-wide image - degenerate view set")
    allpts = np.vstack([i.reshape(-1, 2) for i in img_l])
    cov = {"x": [float(allpts[:, 0].min() / size[0]), float(allpts[:, 0].max() / size[0])],
           "y": [float(allpts[:, 1].min() / size[1]), float(allpts[:, 1].max() / size[1])]}
    warn = []
    if rms > 0.8:
        warn.append(f"RMS {rms:.2f}px is high (good: < 0.5px). Check board flatness / printed square size / focus.")
    if cov["x"][0] > 0.2 or cov["x"][1] < 0.8 or cov["y"][0] > 0.2 or cov["y"][1] < 0.8:
        warn.append("Board never reached the image edges/corners - distortion is poorly constrained there. "
                    "Capture more views with the board near the borders.")
    return {"image_size": list(size), "K": K.tolist(), "dist": dist.reshape(-1).tolist(), "rms_px": float(rms),
            "n_views": len(obj_l), "per_view_err_px": errs, "coverage": cov, "warnings": warn}


def save_intrinsics(res: dict, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(res, indent=1))


def load_intrinsics(path):
    d = json.loads(Path(path).read_text())
    return np.array(d["K"], float), np.array(d["dist"], float).reshape(-1), d
