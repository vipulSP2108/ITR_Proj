"""
Perception: colour segmentation, shape classification, plane fitting and pixel->robot-frame lifting.

Image-space functions need only the RGB frame.  3-D functions additionally need the depth frame, the
camera intrinsics and the calibrated camera->robot transform T_robot_cam (see calibration.py).
"""
import os
import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def load_vision_cfg(path=os.path.join(HERE, "config", "vision.yaml")):
    with open(path) as f:
        return yaml.safe_load(f)


def _rng(cfg, key):
    return np.array(cfg["hsv"][key]["lo"], np.uint8), np.array(cfg["hsv"][key]["hi"], np.uint8)


# ------------------------------------------------------------------------------------------------
# masks
# ------------------------------------------------------------------------------------------------
def color_masks(bgr, cfg):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = {}
    for k in ("peg_green", "mat", "board", "marker"):
        lo, hi = _rng(cfg, k)
        m[k] = cv2.inRange(hsv, lo, hi)
    la, ha = _rng(cfg, "red_ring_a"); lb, hb = _rng(cfg, "red_ring_b")
    m["red_ring"] = cv2.inRange(hsv, la, ha) | cv2.inRange(hsv, lb, hb)
    k3 = np.ones((3, 3), np.uint8)
    for k in ("peg_green", "marker"):
        m[k] = cv2.morphologyEx(m[k], cv2.MORPH_OPEN, k3)
    return m


def blobs(mask, min_px):
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_px:
            out.append(dict(label=i, area=int(stats[i, cv2.CC_STAT_AREA]), centroid=tuple(cent[i]),
                            bbox=tuple(int(v) for v in stats[i, :4]), mask=(lab == i)))
    out.sort(key=lambda b: -b["area"])
    return out


# ------------------------------------------------------------------------------------------------
# shape classification (circle / hexagon / square / triangle)
# ------------------------------------------------------------------------------------------------
def classify_shape(filled_mask: np.ndarray, cfg) -> dict:
    cnts, _ = cv2.findContours(filled_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return dict(shape="unknown", ratio=0.0, vertices=0)
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    (_, _), R = cv2.minEnclosingCircle(c)
    ratio = float(area / (np.pi * R * R + 1e-9))
    peri = cv2.arcLength(c, True)
    nv = len(cv2.approxPolyDP(cv2.convexHull(c), 0.04 * peri, True))
    th = cfg["shape_thresholds"]
    if ratio >= th["circle_min"]:
        shape = "circle"
    elif ratio >= th["hexagon_min"]:
        shape = "hexagon"
    elif ratio >= th["square_min"]:
        shape = "square"
    else:
        shape = "triangle"
    return dict(shape=shape, ratio=ratio, vertices=int(nv), contour=c)


# ------------------------------------------------------------------------------------------------
# geometry helpers
# ------------------------------------------------------------------------------------------------
def to_robot(P_cam: np.ndarray, T_robot_cam: np.ndarray) -> np.ndarray:
    return (T_robot_cam[:3, :3] @ P_cam.T).T + T_robot_cam[:3, 3]


def mask_points_robot(mask, depth, cam, T_robot_cam):
    vs, us = np.where(mask & (depth > 0.05))
    if len(us) == 0:
        return np.zeros((0, 3)), us, vs
    P = cam.deproject(us, vs, depth[vs, us])
    return to_robot(P, T_robot_cam), us, vs


def ransac_plane_z(P: np.ndarray, iters=200, tol=0.003, rng=None):
    """Fit z = a x + b y + c robustly; returns (a, b, c, inlier_mask)."""
    rng = rng or np.random.default_rng(0)
    best, best_in = None, None
    n = len(P)
    if n < 20:
        return None
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        A = np.c_[P[idx, 0], P[idx, 1], np.ones(3)]
        try:
            sol = np.linalg.solve(A, P[idx, 2])
        except np.linalg.LinAlgError:
            continue
        res = np.abs(P[:, 0] * sol[0] + P[:, 1] * sol[1] + sol[2] - P[:, 2])
        inl = res < tol
        if best is None or inl.sum() > best_in.sum():
            best, best_in = sol, inl
    A = np.c_[P[best_in, 0], P[best_in, 1], np.ones(best_in.sum())]
    sol, *_ = np.linalg.lstsq(A, P[best_in, 2], rcond=None)
    return sol[0], sol[1], sol[2], best_in


def ray_plane_xy(u, v, cam, T_robot_cam, plane_z):
    """Intersect the pixel ray with the horizontal plane z = plane_z (robot frame) -> (x, y)."""
    d_cam = np.array([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0])
    d = T_robot_cam[:3, :3] @ d_cam
    o = T_robot_cam[:3, 3]
    s = (plane_z - o[2]) / d[2]
    p = o + s * d
    return float(p[0]), float(p[1])


def fit_sphere_center(P: np.ndarray, r: float, iters=15):
    """Least-squares centre of a sphere of KNOWN radius r from surface points P (N,3)."""
    c = P.mean(axis=0) + np.array([0, 0, 0.8 * r])     # start slightly behind the visible cap
    for _ in range(iters):
        d = P - c
        n = np.linalg.norm(d, axis=1) + 1e-12
        J = -d / n[:, None]
        res = n - r
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        c = c + step
        if np.linalg.norm(step) < 1e-7:
            break
    return c


# ------------------------------------------------------------------------------------------------
# object detectors
# ------------------------------------------------------------------------------------------------
def detect_marker(bgr, depth, cam, cfg, marker_radius):
    """Find the magenta tool marker. Returns dict(u, v, P_cam (3,), n_px) or None."""
    m = color_masks(bgr, cfg)["marker"]
    bl = blobs(m, 15)
    if not bl:
        return None
    b = bl[0]
    mask = cv2.erode(b["mask"].astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    if mask.sum() < 8:
        mask = b["mask"]
    vs, us = np.where(mask & (depth > 0.05))
    if len(us) < 8:
        return None
    P = cam.deproject(us, vs, depth[vs, us])
    c = fit_sphere_center(P, marker_radius)
    return dict(u=b["centroid"][0], v=b["centroid"][1], P_cam=c, n_px=b["area"])


def detect_holes(bgr, depth, cam, cfg, T_robot_cam, board_z):
    """Red-outlined sockets -> list of dict(shape, x, y, z, radius, centroid_px). Sorted top->bottom of image."""
    red = color_masks(bgr, cfg)["red_ring"]
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    holes = []
    for c in cnts:
        if cv2.contourArea(c) < cfg["min_blob_px"]:
            continue
        filled = np.zeros(red.shape, np.uint8)
        cv2.drawContours(filled, [c], -1, 255, -1)
        info = classify_shape(filled, cfg)
        M = cv2.moments(filled, binaryImage=True)
        u, v = M["m10"] / M["m00"], M["m01"] / M["m00"]
        x, y = ray_plane_xy(u, v, cam, T_robot_cam, board_z)
        area_m2 = M["m00"] * (cam_depth_at(T_robot_cam, cam, board_z)) ** 2 / (cam.fx * cam.fy)
        holes.append(dict(shape=info["shape"], ratio=info["ratio"], vertices=info["vertices"],
                          x=x, y=y, z=board_z, radius=float(np.sqrt(area_m2 / np.pi)),
                          centroid_px=(float(u), float(v)), contour=c))
    holes.sort(key=lambda h: h["centroid_px"][1])
    return holes


def cam_depth_at(T_robot_cam, cam, plane_z):
    """Camera-frame depth of the horizontal plane z=plane_z along the optical axis (for area scaling)."""
    o = T_robot_cam[:3, 3]; d = T_robot_cam[:3, 2]
    return float((plane_z - o[2]) / d[2])


def detect_green_peg(bgr, depth, cam, cfg, T_robot_cam, mat_z, min_top_px=25):
    """
    Circle (green) peg. Uses depth slicing: keep only the pixels within ~12 mm of the highest peg
    point (its flat top face), so the visible side wall (perspective) does not bias the centre.
    Returns dict or None (None when the peg is occluded / not visible).
    """
    m = color_masks(bgr, cfg)["peg_green"]
    bl = blobs(m, cfg["min_blob_px"] // 2)
    if not bl:
        return None
    b = bl[0]
    P, us, vs = mask_points_robot(b["mask"], depth, cam, T_robot_cam)
    if len(P) < min_top_px:
        return None
    z_top = float(np.percentile(P[:, 2], 97))
    keep = P[:, 2] >= z_top - 0.012
    if keep.sum() < min_top_px:
        return None
    top = P[keep]
    top_mask = np.zeros(m.shape, np.uint8)
    top_mask[vs[keep], us[keep]] = 255
    top_mask = cv2.morphologyEx(top_mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    info = classify_shape(top_mask, cfg)
    cx, cy = top[:, 0].mean(), top[:, 1].mean()
    area_m2 = keep.sum() * (cam_depth_at(T_robot_cam, cam, z_top)) ** 2 / (cam.fx * cam.fy)
    return dict(shape=info["shape"], ratio=info["ratio"], x=float(cx), y=float(cy), z_top=z_top,
                height=float(z_top - mat_z), radius=float(np.sqrt(area_m2 / np.pi)),
                n_px=int(b["area"]), n_top_px=int(keep.sum()),
                centroid_px=(float(us[keep].mean()), float(vs[keep].mean())),
                visible_fraction=None)


def detect_surfaces(bgr, depth, cam, cfg, T_robot_cam):
    """Mat and board planes + bounding boxes in the robot frame."""
    masks = color_masks(bgr, cfg)
    out = {}
    for key in ("mat", "board"):
        mk = masks[key]
        mk = cv2.morphologyEx(mk, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        bl = blobs(mk, 400)
        if not bl:
            out[key] = None
            continue
        mask = bl[0]["mask"]
        mask_e = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
        P, us, vs = mask_points_robot(mask_e, depth, cam, T_robot_cam)
        pl = ransac_plane_z(P)
        a, b_, c, inl = pl
        Pin = P[inl]
        out[key] = dict(z=float(np.median(Pin[:, 2])), plane=(float(a), float(b_), float(c)),
                        x_range=(float(np.percentile(Pin[:, 0], 0.5)), float(np.percentile(Pin[:, 0], 99.5))),
                        y_range=(float(np.percentile(Pin[:, 1], 0.5)), float(np.percentile(Pin[:, 1], 99.5))),
                        n_px=int(mask.sum()), mask=mask)
    return out
