"""
Workspace analysis: turns calibrated camera detections + the arm's kinematics into
  * the metric layout of the table (mat, board, holes, peg) in the robot frame,
  * a reachability map (minimum tool tilt needed to reach each table position),
  * a per-object verdict that the planner uses to choose its grasp strategy.
"""
import numpy as np
import cv2

import arm_model as A

TILT_STEPS_DEG = list(range(0, 61, 5))


def min_tilt_deg(x, y, z, tilts_deg=TILT_STEPS_DEG, margin_m=0.0):
    """Smallest tool tilt (deg) at which the TCP can reach (x, y, z); None if unreachable <= 60 deg.
    margin_m: also require the point +-margin radially (robustness against small position errors)."""
    r = np.hypot(x, y)
    for td in tilts_deg:
        t = np.radians(td)
        ok = True
        for dr in ((0.0,) if margin_m == 0 else (-margin_m, 0.0, margin_m)):
            rr = max(r + dr, 1e-6)
            if A.ik(rr * x / max(r, 1e-9), rr * y / max(r, 1e-9), z, t) is None:
                ok = False; break
        if ok:
            return td
    return None


def reachability_grid(z, x_range=(0.0, 0.42), y_range=(-0.24, 0.24), step=0.01):
    xs = np.arange(x_range[0], x_range[1] + 1e-9, step)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, step)
    G = np.full((len(ys), len(xs)), np.nan)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if np.hypot(x, y) < 0.06:
                continue
            m = min_tilt_deg(x, y, z)
            if m is not None:
                G[j, i] = m
    return xs, ys, G


CLASS_COLORS_BGR = {          # (label, colour)
    "vertical": ((60, 200, 60), "vertical top-grasp (tilt <= 5 deg)"),
    "tilt30":   ((40, 220, 240), "tilted grasp, <= 30 deg"),
    "tilt60":   ((30, 140, 255), "tilted grasp, 35-60 deg"),
    "none":     ((60, 60, 220), "unreachable"),
}


def classify_cell(v):
    if np.isnan(v): return "none"
    if v <= 5: return "vertical"
    if v <= 30: return "tilt30"
    return "tilt60"


def draw_reachability_overlay(bgr, cam, T_robot_cam, xs, ys, G, z_plane, extent_xy, alpha=0.5):
    """Paint every grid cell (that lies on the mat/board footprint) on the camera image."""
    out = bgr.copy(); layer = bgr.copy()
    Tinv = np.linalg.inv(T_robot_cam)
    (x0, x1), (y0, y1) = extent_xy
    cell_px = max(2, int(0.01 * cam.fx / 0.6) + 1)
    counts = {k: 0 for k in CLASS_COLORS_BGR}
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            c = classify_cell(G[j, i]); counts[c] += 1
            pc = Tinv[:3, :3] @ np.array([x, y, z_plane]) + Tinv[:3, 3]
            u, v = cam.project(pc)[0]
            cv2.circle(layer, (int(round(u)), int(round(v))), cell_px, CLASS_COLORS_BGR[c][0], -1)
    out = cv2.addWeighted(layer, alpha, out, 1 - alpha, 0)
    y0_ = 18
    cv2.rectangle(out, (8, 8), (330, 12 + 18 * (len(CLASS_COLORS_BGR) + 1)), (25, 25, 25), -1)
    cv2.putText(out, "Reachability (min tool tilt)", (14, y0_), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    for k, (col, label) in CLASS_COLORS_BGR.items():
        y0_ += 18
        cv2.rectangle(out, (14, y0_ - 10), (26, y0_ + 1), col, -1)
        cv2.putText(out, f"{label}", (32, y0_), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1)
    return out, counts
