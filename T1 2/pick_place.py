#!/usr/bin/env python3
"""
Vision-guided TOP-GRASP peg sorting for the OpenMANIPULATOR-X.
Needs the calibration in config/camera_robot_calib.yaml (run ./run_calibration.sh first).

    python pick_place.py --detect-only        # look only  -> runs/<time>/scan1_annotated.jpg
    python pick_place.py --plan-only          # look + plan -> runs/<time>/plan1.jpg, nothing is picked
    python pick_place.py --shape circle       # only the circular peg
    python pick_place.py                      # every peg it can, best order first
    python pick_place.py --order circle,square    # fix the order yourself (others are obstacles meanwhile)
    python pick_place.py --force              # also try pegs whose rotation does not match their hole

No table coordinates are written in this file or its config: pegs, holes, the mat outline, the
planning area and parking spots are measured by the camera on every scan and converted with the
current calibration. Sizes and safety buffers: config/pick_place.yaml, objects: config/task_objects.yaml.

How it avoids collisions
  * All pegs are 100 mm tall and the arm cannot lift one over another, so a carried peg is routed
    AROUND every other peg: A* on the table plane (src/planning/path2d.py). The moving body is the
    carried peg PLUS the fingers holding it; every other peg is an obstacle with a hard buffer
    and a wider preferred buffer.
  * Before picking / releasing it checks that the open fingers keep a buffer to neighbouring pegs.
  * With several pegs every order is tried; the fully feasible one with the best (short + roomy)
    paths wins. If nothing is feasible, one peg is first moved to a free parking spot on the mat.
  * The gripper only moves vertically near pegs and rises straight up off a placed peg.
  * The base only turns with the arm tucked up in the travel pose, gripper ~140 mm above the peg tops.
After every peg the scene is scanned again, so nothing depends on stale positions.

Every run writes runs/<time>/ : scan and plan pictures, a photo at every step and run.mp4.
"""
import os, sys, time, math, argparse, threading, itertools
import numpy as np
import cv2
import yaml

T1 = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, T1)
sys.path.insert(0, os.path.join(T1, "calibration"))
from hw_common import (LiveCamera, connect_arm, move_joints, read_joints, gripper_goto,
                       GRIP_OPEN, GRIP_SQUEEZE, load_calib, ik_cmd)
from src.vision.camera_model import CameraModel
from src.planning import omx_kinematics as K
from src.planning import path2d


def _load(name):
    with open(os.path.join(T1, "config", name)) as f:
        return yaml.safe_load(f)


OBJ, CFG = _load("task_objects.yaml"), _load("pick_place.yaml")
PEG_H, PEGS, BOARD_T = float(OBJ["peg_height_mm"]), OBJ["pegs"], float(OBJ["board_thickness_mm"])
COLOUR_TO_SHAPE = {v["colour"]: k for k, v in PEGS.items()}
HT, GR, BUF, TG, ORI = CFG["heights_above_mat_mm"], CFG["gripper"], CFG["safety_buffers_mm"], CFG["top_grasp"], CFG["orientation"]
INS = CFG["insertion_search"]
TRV = CFG["travel_pose"]
HOVER_ABOVE = PEG_H + HT["hover_over_peg_top"]


def grip_counts(gap_mm):
    return int(min(GRIP_OPEN, GR["touch_counts"] + gap_mm * GR["counts_per_mm"]))


def wrap(a, period):
    """Angle (deg) folded into [-period/2, period/2)."""
    return (a + period / 2) % period - period / 2


def jaw_width(shape, mis_deg=0.0):
    """Finger gap (mm) needed to hold a peg whose flat faces are mis_deg off the jaws."""
    p = PEGS[shape]
    if shape == "circle":
        return p["diameter_mm"]
    if shape == "square":
        m = math.radians(abs(mis_deg))
        return p["side_mm"] * (math.cos(m) + math.sin(m))
    if shape == "hexagon":
        return p["side_mm"] * math.sqrt(3) + 4 * abs(mis_deg) / 30.0
    return p["side_mm"] * math.sqrt(3) / 2 + 4 * abs(mis_deg) / 30.0       # triangle: face against apex


class Recorder:
    """Writes the live camera to an mp4 with the current step burned in."""

    def __init__(self, cam, path, fps=10):
        self.cam, self.label, self.alive = cam, "", True
        self.vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (cam.w // 2, cam.h // 2))
        self.dt = 1.0 / fps
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while self.alive:
            f = self.cam.read(fresh=False)
            if f is not None:
                f = cv2.resize(f, (self.cam.w // 2, self.cam.h // 2))
                cv2.rectangle(f, (0, 0), (f.shape[1], 24), (0, 0, 0), -1)
                cv2.putText(f, self.label, (6, 17), 0, 0.5, (255, 255, 255), 1)
                self.vw.write(f)
            time.sleep(self.dt)

    def close(self):
        self.alive = False
        self.t.join(timeout=2)
        self.vw.release()


# ------------------------------------------------------------------ vision --
def mat_mask(hsv):
    """Pixels that look like the foam mat (dominant saturated hue of the picture)."""
    sat = hsv[..., 1] > 90
    hist = np.bincount(hsv[..., 0][sat].ravel(), minlength=180)
    hue = int(np.argmax(np.convolve(hist, np.ones(5), "same")))
    return cv2.inRange(hsv, (max(0, hue - 9), 70, 60), (min(179, hue + 9), 255, 255)), hue


def find_mat(img, cm):
    """Outline of the mat in robot coordinates (convex polygon, Nx2 float32) - this IS the workspace."""
    m, _ = mat_mask(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        raise RuntimeError("cannot see the mat")
    hull = cv2.convexHull(max(cnts, key=cv2.contourArea)).reshape(-1, 2)
    return np.array([cm.pixel_to_robot(u, v) for u, v in hull], np.float32)


def find_board(img, cm):
    """Outline of the wooden hole board in robot coordinates (convex polygon), or None if not seen."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (8, 35, 110), (30, 170, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > 8000]
    if not cnts:
        return None
    hull = cv2.convexHull(np.vstack(cnts)).reshape(-1, 2)
    return np.array([cm.pixel_to_robot(u, v, cm.table_z + BOARD_T) for u, v in hull], np.float32)


def inside(poly, xy):
    """Signed distance (mm) of a robot point to the polygon edge: positive = inside."""
    return float(cv2.pointPolygonTest(poly.reshape(-1, 1, 2), (float(xy[0]), float(xy[1])), True))


def classify(cnt):
    """Shape from how much of its enclosing circle the outline fills
    (circle 1.00, hexagon 0.83, square 0.64, triangle 0.41)."""
    hull = cv2.convexHull(cnt)
    (_, _), r = cv2.minEnclosingCircle(hull)
    fill = cv2.contourArea(hull) / (math.pi * r * r + 1e-9)
    if len(cv2.approxPolyDP(hull, 0.06 * cv2.arcLength(hull, True), True)) == 3:
        return "triangle"
    if fill > 0.90:
        return "circle"
    if fill > 0.74:
        return "hexagon"
    if fill > 0.52:
        return "square"
    return "triangle"


def edge_yaw(cnt_px, cm, z, shape):
    """Direction (deg, robot frame) of the outline's longest straight edge and its length (mm); None for a circle."""
    if shape == "circle":
        return None, None
    hull = cv2.convexHull(cnt_px)
    poly = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True).reshape(-1, 2)
    P = np.array([cm.pixel_to_robot(u, v, z) for u, v in poly])
    e = np.roll(P, -1, axis=0) - P
    L = np.linalg.norm(e, axis=1)
    k = int(np.argmax(L))
    return math.degrees(math.atan2(e[k, 1], e[k, 0])), float(np.median(L))


def find_holes(img, cm):
    """Board cut-outs = regions enclosed by a red/orange outline."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 90, 90), (12, 255, 255)) | cv2.inRange(hsv, (168, 90, 90), (179, 255, 255))
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    cnts, hier = cv2.findContours(red, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    if hier is None:
        return out
    z = cm.table_z + BOARD_T
    for c, h in zip(cnts, hier[0]):
        if h[3] < 0 or not 1500 < cv2.contourArea(c) < 20000:      # inner boundary of each red ring only
            continue
        m = cv2.moments(c)
        u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
        shape = classify(c)
        yaw, side = edge_yaw(c, cm, z, shape)
        out.append({"shape": shape, "px": (u, v), "xy": cm.pixel_to_robot(u, v, z), "yaw": yaw, "side": side, "cnt": c})
    return out


def measure_peg(cnt, hsv, cm, shape):
    """Position (robot XY of the peg axis) and rotation of a peg standing on the mat.

    Seen from the low camera a standing peg is a smeared blob: its foot is the end nearest to the
    point under the camera, its (brighter) top face the far end. All pegs have the same known
    height, so the top face - the cleanest part of the picture - gives the position directly;
    the foot gives a second, independent estimate used as a cross-check.
    """
    z_top = cm.table_z + PEG_H
    pts = cnt.reshape(-1, 2).astype(np.float64)
    c = pts.mean(0)
    nadir = np.array(cm.robot_to_pixel(cm.cam_pos[0], cm.cam_pos[1]))
    u = (c - nadir) / np.linalg.norm(c - nadir)
    v = np.array([-u[1], u[0]])
    t, w = (pts - c) @ u, (pts - c) @ v
    r_top = (w.max() - w.min()) / 2
    lat = (w.max() + w.min()) / 2
    H = cm.cam_pos[2] - cm.table_z
    foot_px = c + (t.min() + r_top * (H - PEG_H) / H) * u + lat * v
    far_px = c + (t.max() - r_top) * u + lat * v
    xy_foot = np.array(cm.pixel_to_robot(*foot_px))
    xy_far = np.array(cm.pixel_to_robot(far_px[0], far_px[1], z_top))
    xy, top_px, yaw = (xy_foot + xy_far) / 2, far_px, None           # errors in r_top cancel in the average

    # brighter top face inside the blob -> direct position (and rotation for cornered pegs)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, -1)
    vals = hsv[..., 2][mask > 0]
    thr, _ = cv2.threshold(vals.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    top = ((hsv[..., 2] > thr) & (mask > 0)).astype(np.uint8) * 255
    top = cv2.morphologyEx(top, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    tc, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if tc:
        tcn = cv2.convexHull(max(tc, key=cv2.contourArea))
        frac = cv2.contourArea(tcn) / max(1.0, cv2.contourArea(cnt))
        m = cv2.moments(tcn)
        if 0.2 < frac < 0.75 and m["m00"] > 0:
            tpx = (m["m10"] / m["m00"], m["m01"] / m["m00"])
            xy_face = np.array(cm.pixel_to_robot(tpx[0], tpx[1], z_top))
            if np.linalg.norm(xy_face - xy) < 15:                    # the two methods must agree
                xy, top_px = xy_face, np.array(tpx)
                yaw, _ = edge_yaw(tcn, cm, z_top, shape)
    return (float(xy[0]), float(xy[1])), yaw, tuple(foot_px), tuple(top_px), float(np.linalg.norm(xy_foot - xy_far))


def find_pegs(img, cm, mat_poly, holes=(), board_poly=None):
    """Coloured pegs standing on the mat. Shape comes from the colour (config/task_objects.yaml)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mat, mat_hue = mat_mask(hsv)
    # hue bands of the four peg colours (strongly saturated only: the wooden board and the red hole
    # outlines share these hues but are paler / thinner)
    ranges = {"orange": [((3, 150, 120), (15, 255, 255))],
              "yellow": [((20, 170, 110), (32, 255, 255))],
              "green": [((34, 70, 60), (max(35, mat_hue - 12), 255, 255))],
              "blue": [((min(178, mat_hue + 10), 120, 40), (135, 255, 255))]}
    edge = 4                                                    # a blob touching the picture border is cut off
    out = []
    for name, rs in ranges.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in rs:
            m |= cv2.inRange(hsv, lo, hi)
        m &= ~mat
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((11, 11), np.uint8))   # also removes the thin red hole outlines
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if not 1500 < cv2.contourArea(c) < 60000:
                continue
            c = cv2.convexHull(c)
            shape = COLOUR_TO_SHAPE.get(name)
            if shape is None:
                continue
            bx, by, bw, bh = cv2.boundingRect(c)
            cut = bx <= edge or by <= edge or bx + bw >= hsv.shape[1] - edge or by + bh >= hsv.shape[0] - edge
            xy, yaw, foot_px, top_px, spread = measure_peg(c, hsv, cm, shape)
            on_mat = inside(mat_poly, xy) >= -BUF["mat_edge"]
            in_hole = any(math.hypot(xy[0] - h["xy"][0], xy[1] - h["xy"][1]) < 40 for h in holes)
            near_board = board_poly is not None and inside(board_poly, xy) >= -25
            if not (on_mat or in_hole or near_board):
                continue
            hidden_hole = not on_mat and not in_hole and spread <= BUF["peg_shape_check"]   # upright on the board = in a hole it hides
            if cut and (on_mat or in_hole or near_board):             # cannot be measured: part of it is outside the picture
                out.append({"shape": "cut-off", "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": None,
                            "spread": spread, "across": 90.0, "cnt": c, "claims": shape})
                continue
            if spread > BUF["peg_shape_check"]:                       # coloured thing that is not a standing peg
                if cv2.contourArea(c) > 7000 and (inside(mat_poly, xy) >= 0 or near_board):   # big enough to be a peg lying down
                    out.append({"shape": "fallen", "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": None,
                                "spread": spread, "across": 120.0, "cnt": c})
                continue
            out.append({"shape": shape, "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": yaw,
                        "spread": spread, "across": PEGS[shape]["across_mm"], "cnt": c, "hidden_hole": hidden_hole})
    return out


def stable_pegs(cam, cm, mat_poly, holes, board_poly, first):
    """Pegs seen at the same place in two pictures taken a moment apart (hands, sleeves and people
    passing through the camera view are not)."""
    time.sleep(0.8)
    second = find_pegs(cam.read_avg(4), cm, mat_poly, holes, board_poly)
    return [p for p in first if any(q["shape"] == p["shape"] and math.hypot(p["xy"][0] - q["xy"][0], p["xy"][1] - q["xy"][1]) < 12
                                    for q in second)]


def ring(cm, x, y, r, n=48):
    return np.array([cm.robot_to_pixel(x + r * math.cos(t), y + r * math.sin(t)) for t in np.linspace(0, 2 * math.pi, n)], np.int32)


def annotate(img, cm, pegs, holes, mat_poly=None, plan=None):
    vis = img.copy()
    if mat_poly is not None:
        cv2.polylines(vis, [np.array([cm.robot_to_pixel(x, y) for x, y in mat_poly], np.int32)], True, (255, 255, 255), 1)
    if plan:                                                    # swept area of (carried peg + fingers)
        layer = vis.copy()
        pts = [cm.robot_to_pixel(x, y) for x, y in plan["path"]]
        width = int(2 * plan["mover_r"] / cm.mm_per_px())
        cv2.polylines(layer, [np.array(pts, np.int32)], False, (0, 220, 0), width, cv2.LINE_AA)
        vis = cv2.addWeighted(layer, 0.35, vis, 0.65, 0)
        cv2.polylines(vis, [np.array(pts, np.int32)], False, (0, 255, 0), 2)
        for ox, oy, orad in plan["obstacles"]:
            cv2.polylines(vis, [ring(cm, ox, oy, orad)], True, (0, 0, 255), 3)                       # the peg itself
            cv2.polylines(vis, [ring(cm, ox, oy, orad + BUF["hard_clearance"])], True, (0, 0, 255), 1)      # hard buffer
            cv2.polylines(vis, [ring(cm, ox, oy, orad + BUF["preferred_clearance"])], True, (0, 140, 255), 1)  # preferred
        cv2.rectangle(vis, (0, 660), (1280, 720), (0, 0, 0), -1)
        cv2.putText(vis, f"PLAN {plan['label']}: path {plan['length']:.0f} mm | smallest free gap (peg+fingers to other pegs) "
                         f"{plan['gap']:.0f} mm | released {90 - plan['tilt_release']:.0f} deg off upright",
                    (10, 683), 0, 0.52, (0, 255, 0), 1)
        cv2.putText(vis, "green band = swept by carried peg + fingers; red = other pegs, thin red = hard buffer, orange = preferred buffer",
                    (10, 708), 0, 0.45, (200, 200, 200), 1)
    for h in holes.values() if isinstance(holes, dict) else holes:
        u, v = cm.robot_to_pixel(h["xy"][0], h["xy"][1], cm.table_z + BOARD_T)
        cv2.circle(vis, (int(u), int(v)), 5, (255, 0, 255), -1)
        cv2.putText(vis, f"HOLE {h['shape']}", (int(u) - 45, int(v) - 12), 0, 0.5, (255, 0, 255), 2)
    for p in pegs:
        cv2.drawContours(vis, [p["cnt"]], -1, (0, 255, 255), 1)
        cv2.drawMarker(vis, (int(p["px"][0]), int(p["px"][1])), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        tag = "IN HOLE" if p.get("placed") else "PEG"
        yaw = "" if p["yaw"] is None else f" yaw {p['yaw']:.0f}"
        cv2.putText(vis, f"{tag} {p['shape']} ({p['xy'][0]:.0f},{p['xy'][1]:.0f}){yaw}", (int(p["px"][0]) - 70, int(p["px"][1]) + 70), 0, 0.5, (0, 255, 255), 2)
    cv2.drawMarker(vis, tuple(int(v) for v in cm.robot_to_pixel(0, 0)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    return vis


# ---------------------------------------------------------------- planning --
def footprint(o, n=48):
    """Outline points (robot mm) of a standing peg: its real cross-section when the rotation is known,
    otherwise the circle that encloses it (the cautious choice)."""
    x, y = o["xy"]
    shape, yaw = o["shape"], o.get("yaw")
    if shape in ("square", "triangle", "hexagon") and yaw is not None:
        k = {"square": 4, "triangle": 3, "hexagon": 6}[shape]
        r = o["across"] / 2
        a0 = math.radians(yaw) + math.pi / k            # an edge lies along `yaw`
        v = [(x + r * math.cos(a0 + 2 * math.pi * i / k), y + r * math.sin(a0 + 2 * math.pi * i / k)) for i in range(k)]
        pts = []
        for i in range(k):
            (x0, y0), (x1, y1) = v[i], v[(i + 1) % k]
            pts += [(x0 + (x1 - x0) * t / 10, y0 + (y1 - y0) * t / 10) for t in range(10)]
        return pts
    r = o["across"] / 2
    return [(x + r * math.cos(2 * math.pi * i / n), y + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def fingers_gap(at_xy, gap_mm, others):
    """Smallest free gap (mm) between the open fingers standing at at_xy and neighbouring pegs, and who is nearest.

    The gripper always faces away from the robot base: the fingers open along the tangential
    direction, so they occupy a box (gap + 2 fingers) x (finger width) around the peg. The
    neighbours are taken with their real outline.
    """
    r = math.hypot(*at_xy)
    rad, tan = (at_xy[0] / r, at_xy[1] / r), (-at_xy[1] / r, at_xy[0] / r)
    best, who = 1e9, None
    for o in others:
        g = 1e9
        for qx, qy in footprint(o):
            dx, dy = qx - at_xy[0], qy - at_xy[1]
            ex = max(0.0, abs(dx * tan[0] + dy * tan[1]) - (gap_mm / 2 + GR["finger_thickness_mm"]))
            ey = max(0.0, abs(dx * rad[0] + dy * rad[1]) - GR["finger_half_width_mm"])
            g = min(g, math.hypot(ex, ey))
        if g < best:
            best, who = g, f"{o['shape']} peg at ({o['xy'][0]:.0f},{o['xy'][1]:.0f})"
    return best, who


def orientation_plan(peg, goal_xy, hole):
    """How a cornered peg is gripped and how it arrives at the hole. Returns (jaw_mis_deg, arrive_mis_deg, tol_deg).

    The arm has no wrist roll: the jaws are always parallel to the line from the base. Closing them
    turns the peg until two faces lie along that line; carrying it turns it with the base.
    """
    sym = PEGS[peg["shape"]]["symmetry_deg"]
    if not sym or peg["yaw"] is None:
        return 0.0, 0.0, 180.0
    snap = 90.0 if peg["shape"] == "square" else 60.0
    th_pick = math.degrees(math.atan2(peg["xy"][1], peg["xy"][0]))
    th_goal = math.degrees(math.atan2(goal_xy[1], goal_xy[0]))
    jaw_mis = wrap(peg["yaw"] - th_pick, snap)
    if hole is None or hole["yaw"] is None:
        return jaw_mis, 0.0, 180.0
    arrive = (peg["yaw"] - jaw_mis) + (th_goal - th_pick)
    mis = wrap(arrive - hole["yaw"], sym)
    side = hole.get("side")
    play = ORI["position_play_needed_mm"]
    if peg["shape"] == "square" and side:
        # a square turned by m needs a hole of side*(cos m + sin m); keep `play` free on every side
        ratio = (side - 2 * play) / (PEGS["square"]["side_mm"] * math.sqrt(2))
        tol = math.degrees(math.asin(min(1.0, ratio)) - math.pi / 4) if ratio > math.sqrt(0.5) else 0.0
    elif peg["shape"] == "hexagon" and side:
        # the peg's corner circle fits inside the hole's inner circle -> it drops in at ANY rotation
        tol = 180.0 if PEGS["hexagon"]["side_mm"] + play < side * math.sqrt(3) / 2 else ORI["default_fit_deg"]
    elif peg["shape"] == "triangle" and side:
        tol = ORI["triangle_fit_deg"] if PEGS["triangle"]["side_mm"] + 2 * play < side else ORI["default_fit_deg"]
    else:
        tol = ORI["default_fit_deg"]
    return jaw_mis, (0.0 if tol >= 180 else mis), max(0.0, tol)


def plan_move(mv, peg, goal_xy, others, label, bounds, hole=None, force=False):
    """Plan pick(peg) -> goal_xy with `others` as obstacles. Returns a plan dict or {"fail": reason}."""
    jaw_mis, arrive_mis, tol = orientation_plan(peg, goal_xy, hole)
    if abs(jaw_mis) > ORI["max_grip_misalignment_deg"]:
        return {"fail": f"peg is turned {jaw_mis:.0f} deg to the jaws (corner grip) - turn it by hand"}
    if hole is not None and abs(arrive_mis) > tol and not force:
        return {"fail": f"it would arrive {arrive_mis:+.0f} deg rotated in its hole (hole play +-{tol:.0f} deg): "
                        f"turn the board so this hole's edges point at the robot base, or use --force"}
    width = jaw_width(peg["shape"], jaw_mis)
    # open the fingers as wide as the neighbours allow (wide = more tolerant of position error)
    open_extra = GR["open_extra_mm"]
    while True:
        g, who = fingers_gap(peg["xy"], width + open_extra, others)
        if g >= BUF["finger_clearance"]:
            break
        open_extra -= 4.0
        if open_extra < GR["open_extra_min_mm"]:
            return {"fail": f"cannot pick: even barely open, the fingers would pass {g:.0f} mm from the {who} "
                            f"(buffer required: {BUF['finger_clearance']:.0f} mm). Move those two pegs about "
                            f"{BUF['finger_clearance'] - g + 5:.0f} mm further apart, or put one nearer to / further from the robot"}
    g, who = fingers_gap(goal_xy, width + GR["release_extra_mm"], others)
    if g < BUF["finger_clearance"]:
        return {"fail": f"cannot release: fingers would be {g:.0f} mm from the {who} (need {BUF['finger_clearance']:.0f})"}
    drop = (BOARD_T if hole is not None else 0.0) + HT["release_gap"]
    # choose where on the peg to grip: the height that lets the peg hang most upright when released
    best = None
    for grip in np.arange(HT["grip_centre_max"], HT["grip_centre_min"] - 0.1, -5.0):
        t_pick = mv.tilt_for(peg["xy"][0], peg["xy"][1], grip)
        t_rel = mv.tilt_for(goal_xy[0], goal_xy[1], drop, grip)
        if t_pick is None or t_rel is None or mv.hover_height(peg["xy"][0], peg["xy"][1]) is None:
            continue
        if mv.hover_height(goal_xy[0], goal_xy[1], min_down=TG["retreat_min_down_deg"]) is None:
            continue
        if best is None or min(t_pick, t_rel) > min(best[1], best[2]) + 1e-6:
            best = (float(grip), t_pick, t_rel)
    if best is None:
        return {"fail": f"out of reach for a top grasp (pick r={math.hypot(*peg['xy']):.0f} mm, goal r={math.hypot(*goal_xy):.0f} mm)"}
    grip, t_pick, t_rel = best
    obstacles = [(o["xy"][0], o["xy"][1], o["across"] / 2) for o in others]
    mover_r = peg["across"] / 2 + GR["finger_thickness_mm"]
    path = path2d.plan(tuple(peg["xy"]), tuple(goal_xy), obstacles, mover_r, lambda x, y: mv.can_carry(x, y, grip), bounds,
                       hard=BUF["hard_clearance"], soft=BUF["preferred_clearance"], end=BUF["end_clearance"])
    if path is None:
        return {"fail": f"no path keeps {BUF['hard_clearance']:.0f} mm between the carried peg and the other pegs"}
    return {"peg": peg, "goal": tuple(goal_xy), "path": path, "obstacles": obstacles, "mover_r": mover_r, "label": label,
            "grip": grip, "drop": drop, "width": width, "open_extra": open_extra, "tilt_pick": t_pick, "tilt_release": t_rel,
            "jaw_mis": jaw_mis, "arrive_mis": arrive_mis, "fit_tol": tol, "into_hole": hole is not None,
            "length": path2d.path_length(path), "gap": path2d.min_gap(path, obstacles, mover_r) if obstacles else 999.0}


def choose(mv, todo, standing, holes, bounds, force):
    """Try every order of the pegs in `todo`; return (score, first move, order) of the best fully feasible order.

    standing = every peg currently standing anywhere (all are obstacles). A peg that has been
    placed is simulated as standing in its hole for the pegs that come after it.
    """
    best, reasons = None, {}
    for order in itertools.permutations(todo):
        sim = [dict(p) for p in standing]
        total, first, ok = 0.0, None, True
        for peg in order:
            me = next(s for s in sim if s["id"] == peg["id"])
            others = [s for s in sim if s["id"] != peg["id"]]
            hole = holes[peg["shape"]]
            pl = plan_move(mv, me, hole["xy"], others, f"{peg['shape']} -> hole", bounds, hole, force)
            if "fail" in pl:
                reasons.setdefault(peg["shape"], pl["fail"])
                ok = False
                break
            total += pl["length"] - 2.0 * min(pl["gap"], BUF["preferred_clearance"])     # short AND roomy is best
            first = first or pl
            sim[sim.index(me)] = dict(me, xy=hole["xy"])            # copy: never alter the planned peg
        if ok and (best is None or total < best[0]):
            best = (total, first, [p["shape"] for p in order])
    return best, reasons


def hole_line_distance(holes, xy):
    """Distance of a point from the row of holes (= the board)."""
    P = np.array([h["xy"] for h in holes.values()])
    if len(P) == 0:
        return 1e9
    if len(P) == 1:
        return float(np.linalg.norm(P[0] - xy))
    c = P.mean(0)
    d = np.linalg.svd(P - c)[2][0]
    return float(abs((xy[0] - c[0]) * d[1] - (xy[1] - c[1]) * d[0]))


def choose_parking(mv, loose, standing, holes, mat_poly, bounds):
    """Nothing can go to its hole: move one peg to a free spot on the mat so the others get room."""
    best = None
    for peg in loose:
        others = [s for s in standing if s["id"] != peg["id"]]
        for sx in np.arange(bounds[0], bounds[1], 15.0):
            for sy in np.arange(bounds[2], bounds[3], 15.0):
                if inside(mat_poly, (sx, sy)) < BUF["parking_from_mat_edge"] or hole_line_distance(holes, (sx, sy)) < BUF["parking_from_board"]:
                    continue
                if any(math.hypot(sx - o["xy"][0], sy - o["xy"][1]) < BUF["parking_from_pegs"] for o in others):
                    continue
                if math.hypot(sx - peg["xy"][0], sy - peg["xy"][1]) < 60:
                    continue
                pl = plan_move(mv, peg, (float(sx), float(sy)), others, f"{peg['shape']} -> parking spot", bounds)
                if "fail" not in pl and (best is None or pl["length"] < best["length"]):
                    best = pl
    return best


# ------------------------------------------------------------------ motion --
class Mover:
    def __init__(self, arm, cam, cm, calib, rec, run_dir):
        self.arm, self.cam, self.cm, self.calib, self.rec, self.dir = arm, cam, cm, calib, rec, run_dir
        self.n, self._cache = 0, {}

    def step(self, label):
        self.n += 1
        self.rec.label = f"{self.n:02d} {label}"
        print(f"  [{self.n:02d}] {label}")

    def photo(self, name):
        cv2.imwrite(os.path.join(self.dir, f"{self.n:02d}_{name}.jpg"), self.cam.read())

    def solve(self, x, y, above, extra=0.0, min_down=None):
        """(raw joints, tool angle) putting the point `extra` mm beyond the grip centre (0 = grip centre itself,
        grip height = bottom of a carried peg) at (x, y), `above` mm over the mat; (None, None) if impossible."""
        if above + extra < HT["never_below"] or above < 3.0:
            return None, None
        return ik_cmd(x, y, self.cm.table_z + above, 90.0, K.L4 + extra, self.calib,
                      min_down_deg=TG["min_down_deg"] if min_down is None else min_down)

    def tilt_for(self, x, y, above, extra=0.0, min_down=None):
        return self.solve(x, y, above, extra, min_down)[1]

    def raw_for(self, x, y, above, extra=0.0, min_down=None):
        raw, _ = self.solve(x, y, above, extra, min_down)
        if raw is None:
            raise RuntimeError(f"({x:.0f},{y:.0f}) at {above:.0f} mm is out of reach (r={math.hypot(x, y):.0f} mm)")
        return raw

    def can_carry(self, x, y, grip):
        k = (round(x), round(y), round(grip))
        if k not in self._cache:
            self._cache[k] = self.solve(x, y, HT["carry_peg_bottom"], grip)[0] is not None
        return self._cache[k]

    def goto(self, x, y, above, slow=False, extra=0.0, min_down=None):
        move_joints(self.arm, self.raw_for(x, y, above, extra, min_down), max_step=10 if slow else 25)

    def travel_joints(self):
        """Shoulder/elbow/wrist counts of the tucked-up travel pose (config travel_pose)."""
        a = K.ik(TRV["radius"], 0.0, self.cm.table_z + TRV["grip_centre_above_mat"], math.radians(TRV["tool_down_deg"]))
        if a is None:
            raise RuntimeError("travel_pose in config/pick_place.yaml is out of reach")
        off = self.calib.get("joint_offsets_rad", [0.0, 0.0, 0.0])
        return K.angles_to_raw([0.0, a[1] - off[0], a[2] - off[1], a[3] - off[2]])[1:]

    def tuck(self, towards=None, base_raw=None):
        """Fold the arm up into the travel pose where it stands, then (optionally) turn the base.

        The base only ever turns in this pose: the whole gripper is then far above every peg.
        """
        cur = read_joints(self.arm)
        tj = self.travel_joints()
        move_joints(self.arm, [cur[0]] + tj, max_step=12)
        if towards is not None:
            base_raw = K.angles_to_raw([math.atan2(towards[1], towards[0]), 0, 0, 0])[0]
        if base_raw is not None:
            move_joints(self.arm, [base_raw] + tj, max_step=15)

    def hover_height(self, x, y, min_down=None):
        """Highest reachable hover (grip centre above the mat) over (x, y) within the configured band, or None."""
        h = PEG_H + HT["hover_over_peg_top_max"]
        while h >= HOVER_ABOVE - 1e-6:
            if self.solve(x, y, h, min_down=min_down)[0] is not None:
                return h
            h -= 4.0
        return None

    def follow(self, path, above, extra):
        for a, b in zip(path, path[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 15))
            for i in range(1, n + 1):
                t = i / n
                move_joints(self.arm, self.raw_for(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, above, extra),
                            max_step=15, settle=0.0, tol=40)
        time.sleep(0.6)


def execute(mv, pl):
    peg, (sx, sy), grip = pl["peg"], pl["goal"], pl["grip"]
    px, py = peg["xy"]
    carry = HT["carry_peg_bottom"]
    mv.step(f"{pl['label']}: open fingers {pl['width'] + pl['open_extra']:.0f} mm, turn towards the peg, hover")
    gripper_goto(mv.arm, grip_counts(pl["width"] + pl["open_extra"]))
    mv.tuck(towards=(px, py))
    hover = mv.hover_height(px, py)
    print(f"       hovering {hover - PEG_H:.0f} mm (grip centre) above the peg top")
    mv.goto(px, py, hover); mv.photo("hover_pick")
    mv.step(f"descend on peg (grip {grip:.0f} mm above the mat)"); mv.goto(px, py, grip, slow=True); mv.photo("at_peg")
    mv.step("close gripper")
    g = gripper_goto(mv.arm, GRIP_SQUEEZE)
    held_mm = (g - GR["touch_counts"]) / GR["counts_per_mm"]
    if g <= GR["held_min_counts"]:
        print("       gripper closed completely: NOTHING between the fingers")
        gripper_goto(mv.arm, grip_counts(pl["width"] + pl["open_extra"]))
        mv.goto(px, py, hover, slow=True)
        mv.tuck()
        return False
    print(f"       holding the peg: fingers stopped {held_mm:.0f} mm apart (expected about {pl['width']:.0f} mm"
          f"{'' if abs(held_mm - pl['width']) < 8 else ' - OFF-CENTRE or wrong peg?'})")
    mv.step("lift"); mv.goto(px, py, carry, slow=True, extra=grip); mv.photo("lifted")
    mv.step(f"carry along the planned path ({len(pl['path']) - 1} segment(s))")
    mv.follow(pl["path"], carry, grip); mv.photo("over_goal")
    mv.step(f"lower (peg will be {90 - pl['tilt_release']:.0f} deg off upright)")
    mv.goto(sx, sy, pl["drop"], slow=True, extra=grip); time.sleep(0.4)
    if pl["into_hole"] and INS["enabled"]:
        seat_in_hole(mv, sx, sy, pl["drop"], grip)
    mv.photo("lowered")
    mv.step(f"release (fingers open only {GR['release_extra_mm']:.0f} mm wider than the peg)")
    gripper_goto(mv.arm, g + int(GR["release_extra_mm"] * GR["counts_per_mm"])); time.sleep(0.5)
    mv.step("retreat straight up, clear of the peg top")
    up = mv.hover_height(sx, sy, min_down=TG["retreat_min_down_deg"])
    mv.goto(sx, sy, up, slow=True, min_down=TG["retreat_min_down_deg"]); mv.photo("released")
    mv.tuck()                                              # fold up high where it stands before any turn
    return True


def seat_in_hole(mv, sx, sy, start_above, grip):
    """Lower the held peg until it touches, then feel sideways for the hole (spiral search).

    A peg resting on the hole's rim stops the arm early (the joints lag behind their commands).
    While pressing down very lightly the peg is moved in small circles round the hole centre;
    when it finds the hole it drops ~3 mm and the lag disappears. Then it is released there.
    """
    def lag(x, y, above):
        raw = mv.raw_for(x, y, above, grip)
        got = move_joints(mv.arm, raw, max_step=5, settle=0.3, tol=3)
        return sum(abs(a - b) for a, b in zip(got[1:], raw[1:]))

    base = lag(sx, sy, start_above)
    floor = BOARD_T - INS["max_below_board_mm"]
    above, touched = start_above, None
    while above - INS["step_down_mm"] >= max(floor, 3.0):
        above -= INS["step_down_mm"]
        if lag(sx, sy, above) - base > INS["contact_error_counts"]:
            touched = above
            break
    if touched is None:
        print("       insertion: went down freely - the peg is in the hole")
        return
    press = max(touched - INS["press_mm"], 3.0)
    for r in INS["radii_mm"]:
        for k in range(8):
            ox, oy = r * math.cos(k * math.pi / 4), r * math.sin(k * math.pi / 4)
            if lag(sx + ox, sy + oy, press) - base <= INS["contact_error_counts"] / 2:
                print(f"       insertion: touched at {touched:.0f} mm, found the hole {r:.1f} mm to the side - peg dropped in")
                return
    lag(sx, sy, touched + 1.0)                     # stop pressing before letting go
    print(f"       insertion: touched at {touched:.0f} mm and no sideways position freed it "
          "(either it already stands on the mat inside the hole, or it sits on the rim) - releasing here")


def advise_upright(mv, pl):
    """If the peg is released tilted, say how much closer the hole would have to be for an upright release."""
    if pl["tilt_release"] >= TG["upright_warn_deg"]:
        return
    sx, sy = pl["goal"]
    r = math.hypot(sx, sy)
    for d in range(5, 150, 5):
        k = (r - d) / r
        if mv.tilt_for(sx * k, sy * k, pl["drop"], HT["grip_centre_min"], min_down=89.0) is not None:
            print(f"       NOTE: at {r:.0f} mm from the base the peg is released {90 - pl['tilt_release']:.0f} deg off upright "
                  f"(it rights itself when let go). Moving the board about {d} mm closer to the robot would make this release upright.")
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shape", default="all", help="circle / square / hexagon / triangle / all")
    ap.add_argument("--detect-only", action="store_true")
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--force", action="store_true", help="try a peg even if its rotation does not match the hole")
    ap.add_argument("--order", default="", help="fix the order yourself, e.g. --order circle,square (default: best order is chosen)")
    a = ap.parse_args()

    run_dir = os.path.join(T1, "runs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir)
    calib, cm = load_calib(), CameraModel()
    cam = LiveCamera()
    arm = connect_arm()
    rec = Recorder(cam, os.path.join(run_dir, "run.mp4"))
    mv = Mover(arm, cam, cm, calib, rec, run_dir)
    holes, done, failed = {}, [], {}
    try:
        low = TRV["grip_centre_above_mat"] - TRV["gripper_below_centre"] - TRV["droop_allowance"] - PEG_H
        if low < TRV["min_gap_over_pegs"]:
            raise RuntimeError(f"travel_pose leaves only {low:.0f} mm above the pegs - raise it in config/pick_place.yaml")
        forced = [x.strip() for x in a.order.split(",")] if a.order else None
        for rnd in range(1, 10):
            rec.label = f"scan {rnd}"
            mv.tuck(base_raw=calib["scan_pose_raw"][0]); time.sleep(0.6)     # tucked, turned out of the camera's view
            scan = cam.read_avg(4)
            mat_poly = find_mat(scan, cm)
            for h in find_holes(scan, cm):                 # a hole hidden by a peg keeps its last seen position
                holes[h["shape"]] = h
            board_poly = find_board(scan, cm)
            hl = list(holes.values())
            pegs = stable_pegs(cam, cm, mat_poly, hl, board_poly, find_pegs(scan, cm, mat_poly, hl, board_poly))
            for i, p in enumerate(pegs):
                p["id"] = i
                p["placed"] = p["shape"] != "fallen" and p["shape"] != "cut-off" and (p.get("hidden_hole", False) or any(math.hypot(p["xy"][0] - h["xy"][0], p["xy"][1] - h["xy"][1]) < 30 for h in holes.values()))
            loose = [p for p in pegs if not p["placed"] and p["shape"] not in ("fallen", "cut-off")]
            allpts = np.vstack([mat_poly] + [np.array([h["xy"]]) for h in holes.values()])
            bounds = (float(allpts[:, 0].min()) - 40, float(allpts[:, 0].max()) + 40,
                      float(allpts[:, 1].min()) - 40, float(allpts[:, 1].max()) + 40)
            cv2.imwrite(os.path.join(run_dir, f"scan{rnd}_raw.jpg"), scan)
            cv2.imwrite(os.path.join(run_dir, f"scan{rnd}_annotated.jpg"), annotate(scan, cm, pegs, holes, mat_poly))
            print(f"\n--- scan {rnd} ---")
            print("holes :", {k: [round(v) for v in h["xy"]] for k, h in holes.items()})
            for p in pegs:
                print(f"peg   : {p['shape']:8s} ({p['colour']}) at ({p['xy'][0]:.0f},{p['xy'][1]:.0f})"
                      f"{'' if p['yaw'] is None else ' yaw %.0f deg' % p['yaw']}  {'in hole' if p['placed'] else 'on mat'}"
                      f"  [position check {p['spread']:.0f} mm]")
            if a.detect_only:
                break
            for c0 in [p for p in pegs if p["shape"] == "cut-off"]:
                print(f"NOTE: the {c0['colour']} peg near ({c0['xy'][0]:.0f},{c0['xy'][1]:.0f}) is partly outside the camera picture. "
                      "It is avoided as a (generously sized) obstacle but will not be picked - move it towards the middle of the mat.")
            if any(p["shape"] == "fallen" for p in pegs):
                print("STOP: a peg seems to be lying down (or something else coloured is on the table). "
                      "Stand it up / remove it and run again - see the scan picture.")
                break
            todo = [p for p in loose if p["shape"] in holes and a.shape in ("all", p["shape"]) and failed.get(p["shape"], 0) < 2]
            if forced:                                     # the user fixed the order: only the next shape in it is a candidate
                nxt = next((sh for sh in forced if any(p["shape"] == sh for p in todo)), None)
                todo = [p for p in todo if p["shape"] == nxt]
                if todo:
                    print(f"forced order {forced}: next is the {nxt}; every other peg is treated as an obstacle")
            if not todo:
                print("nothing (more) to do.")
                break
            best, reasons = choose(mv, todo, pegs, holes, bounds, a.force)
            if best is None and len(todo) > 1:             # not all can be done: do the ones that can
                for sub in todo:
                    b1, r1 = choose(mv, [sub], pegs, holes, bounds, a.force)
                    reasons.update(r1)
                    if b1 and (best is None or b1[0] < best[0]):
                        best = b1
            for s, why in reasons.items():
                print(f"  cannot do {s} now: {why}")
            if best:
                pl = best[1]
                print(f"order: {best[2]} -> now {pl['label']}: path {pl['length']:.0f} mm, smallest free gap {pl['gap']:.0f} mm, "
                      f"fingers open {pl['width'] + pl['open_extra']:.0f} mm, grip {pl['grip']:.0f} mm up, tool {pl['tilt_pick']:.0f} deg at pick / {pl['tilt_release']:.0f} deg at release"
                      + ("" if not pl["jaw_mis"] and not pl["arrive_mis"] else f", jaws turn the peg {-pl['jaw_mis']:+.0f} deg, arrives {pl['arrive_mis']:+.0f} deg off the hole"))
                advise_upright(mv, pl)
            else:
                blocked = [p for p in todo if "fingers" in reasons.get(p["shape"], "") or "path" in reasons.get(p["shape"], "")]
                pl = None
                if blocked:                                # crowded: move a peg that is in the way, the blocked one last
                    ids = {p["id"] for p in blocked}
                    pl = (choose_parking(mv, [p for p in loose if p["id"] not in ids], pegs, holes, mat_poly, bounds)
                          or choose_parking(mv, blocked, pegs, holes, mat_poly, bounds))
                if pl is None:
                    print("nothing can be moved safely - see the reasons above.")
                    break
                print(f"blocked -> first making room: {pl['label']} at ({pl['goal'][0]:.0f},{pl['goal'][1]:.0f})")
            cv2.imwrite(os.path.join(run_dir, f"plan{rnd}.jpg"), annotate(scan, cm, pegs, holes, mat_poly, pl))
            if a.plan_only:
                break
            ok = execute(mv, pl)
            if ok and pl["into_hole"]:
                done.append(pl["peg"]["shape"])
            elif not ok:
                failed[pl["peg"]["shape"]] = failed.get(pl["peg"]["shape"], 0) + 1
        print("\nRESULT: placed", done or "nothing", "| failed grasps:", failed or "none")
        mv.tuck(base_raw=calib["scan_pose_raw"][0])
    finally:
        rec.close()
        cam.release()
        arm.disconnect(disable_torque=False)
        print("run folder:", run_dir)


if __name__ == "__main__":
    main()
