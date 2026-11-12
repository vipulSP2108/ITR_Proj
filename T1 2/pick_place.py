#!/usr/bin/env python3
"""
Vision-guided TOP-GRASP peg sorting for the OpenMANIPULATOR-X.
Needs the calibration in config/camera_robot_calib.yaml (run ./run_calibration.sh first).

    python pick_place.py --detect-only        # look only  -> runs/<time>/scan1_annotated.jpg
    python pick_place.py --suggest-layout     # empty mat: picture of where to stand the pegs -> suggested_layout.jpg
    python pick_place.py --plan-only          # look + plan -> runs/<time>/plan1.jpg, nothing is picked
    python pick_place.py --check-calibration  # measure the calibration error now -> runs/<time>/check_sheet.jpg
    python pick_place.py --tune-calibration   # ... and store a small correction (no need to clear the table)
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
  * The base only turns with the arm folded up (config/robot_safety.yaml), never towards the camera stand.
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
from hw_common import (LiveCamera, connect_arm, move_joints, stream_path, read_joints, gripper_goto,
                       GRIP_OPEN, GRIP_SQUEEZE, load_calib, ik_cmd, grip_counts_for_gap, gap_for_grip_counts,
                       gripper_holding)
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
HOVER_ABOVE = PEG_H + HT["hover_over_peg_top"]


def grip_counts(gap_mm):
    """Gripper command for a finger gap of gap_mm (never wider than the widest point)."""
    return max(GRIP_OPEN, grip_counts_for_gap(gap_mm))


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
        yaw, _ = edge_yaw(c, cm, z, shape)
        # side length from the hole's AREA in mm^2 (far steadier than measuring single edges)
        area = cv2.contourArea(np.array([cm.pixel_to_robot(q[0][0], q[0][1], z) for q in c], np.float32))
        side = {"square": math.sqrt(area), "hexagon": math.sqrt(2 * area / (3 * math.sqrt(3))),
                "triangle": math.sqrt(4 * area / math.sqrt(3))}.get(shape)
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
    xy, top_px, yaw, verts = (xy_foot + xy_far) / 2, far_px, None, None   # errors in r_top cancel in the average

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
                poly = cv2.approxPolyDP(tcn, 0.04 * cv2.arcLength(tcn, True), True).reshape(-1, 2)
                verts = [cm.pixel_to_robot(u_, v_, z_top) for u_, v_ in poly]      # corners of the top face (robot mm)
    return (float(xy[0]), float(xy[1])), yaw, tuple(foot_px), tuple(top_px), float(np.linalg.norm(xy_foot - xy_far)), verts


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
            xy, yaw, foot_px, top_px, spread, verts = measure_peg(c, hsv, cm, shape)
            on_mat = inside(mat_poly, xy) >= -BUF["mat_edge"]
            in_hole = any(math.hypot(xy[0] - h["xy"][0], xy[1] - h["xy"][1]) < 40 for h in holes)
            near_board = board_poly is not None and inside(board_poly, xy) >= -25
            if not (on_mat or in_hole or near_board):
                continue
            hidden_hole = not on_mat and not in_hole and spread <= BUF["peg_shape_check"]   # upright on the board = in a hole it hides
            if cut and (on_mat or in_hole or near_board):             # cannot be measured: part of it is outside the picture
                nominal_across = (PEGS[shape]["across_mm"] + 15.0) if shape in PEGS else 90.0
                out.append({"shape": "cut-off", "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": None,
                            "spread": spread, "across": nominal_across, "cnt": c, "claims": shape})
                continue
            if spread > BUF["peg_shape_check"] and not in_hole:       # coloured thing with high spread
                if spread <= 50.0 or cv2.contourArea(c) < 7000:
                    # Standing cornered peg (like triangle) at an oblique camera angle or slightly leaning
                    out.append({"shape": shape, "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": yaw,
                                "spread": spread, "across": PEGS[shape]["across_mm"], "cnt": c, "hidden_hole": hidden_hole})
                    continue
                out.append({"shape": "fallen", "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": None,
                            "spread": spread, "across": 120.0, "cnt": c})
                continue
            out.append({"shape": shape, "colour": name, "px": foot_px, "top_px": top_px, "xy": xy, "yaw": yaw,
                        "spread": spread, "across": PEGS[shape]["across_mm"], "cnt": c, "hidden_hole": hidden_hole,
                        "verts": verts})
    return out


def stable_pegs(cam, cm, mat_poly, holes, board_poly, first):
    """Pegs seen at the same place in two pictures taken a moment apart (hands, sleeves and people
    passing through the camera view are not)."""
    time.sleep(0.8)
    second = find_pegs(cam.read_avg(4), cm, mat_poly, holes, board_poly)
    return [p for p in first if any(q["shape"] == p["shape"] and math.hypot(p["xy"][0] - q["xy"][0], p["xy"][1] - q["xy"][1]) < 12
                                    for q in second)]


def ring(cm, x, y, r, n=48, z=None):
    """Circle of radius r (mm) around (x, y) drawn at height z (default: half way up a peg, where the eye
    judges it - drawn on the table it looks shifted/shrunk far from the camera because of perspective)."""
    z = cm.table_z + PEG_H / 2 if z is None else z
    return np.array([cm.robot_to_pixel(x + r * math.cos(t), y + r * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, n)], np.int32)


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


def tangent(xy):
    """Unit vector along the gripper's jaw axis at (x, y): the jaws always open sideways to the line from the base."""
    r = math.hypot(*xy)
    return (-xy[1] / r, xy[0] / r)


def grasp_offset(peg):
    """Where the jaws centre on the peg, in mm along the jaw axis from the peg's centre (+ = counter-clockwise side).

    The jaws close symmetrically. A triangle is held between one flat face and the opposite corner,
    so the middle of the grip lies h/6 (~5 mm) from its centre, towards the corner. Aiming the gripper
    at the centre instead shoves the triangle 5 mm sideways and it lands off-centre. Square, hexagon
    and circle are symmetric: offset 0.
    """
    v = peg.get("verts")
    if peg["shape"] != "triangle" or not v or len(v) != 3:
        return 0.0
    t = tangent(peg["xy"])
    proj = [(vx - peg["xy"][0]) * t[0] + (vy - peg["xy"][1]) * t[1] for vx, vy in v]
    h = PEGS["triangle"]["side_mm"] * math.sqrt(3) / 2
    return (1.0 if max(proj) > -min(proj) else -1.0) * h / 6.0


def plan_move(mv, peg, goal_xy, others, label, bounds, hole=None, force=False):
    """Plan pick(peg) -> goal_xy with `others` as obstacles. Returns a plan dict or {"fail": reason}."""
    notes = []                                             # relaxed-buffer messages, printed only for the move actually made
    jaw_mis, arrive_mis, tol = orientation_plan(peg, goal_xy, hole)
    if abs(jaw_mis) > ORI["max_grip_misalignment_deg"]:
        return {"fail": f"peg is turned {jaw_mis:.0f} deg to the jaws (corner grip) - turn it by hand"}
    if hole is not None and abs(arrive_mis) > tol and not force:
        return {"fail": f"it would arrive {arrive_mis:+.0f} deg rotated in its hole (hole play +-{tol:.0f} deg): "
                        f"turn the board so this hole's edges point at the robot base, or use --force"}
    width = jaw_width(peg["shape"], jaw_mis)
    # open the fingers as wide as the neighbours allow (wide = more tolerant of position error)
    open_extra = GR["open_extra_mm"]
    min_open = GR.get("open_extra_min_mm", 16.0) if not force else 10.0
    min_pick_buf = BUF["finger_clearance"] if not force else 8.0
    while True:
        g, who = fingers_gap(peg["xy"], width + open_extra, others)
        if g >= BUF["finger_clearance"]:
            break
        open_extra -= 2.0
        if open_extra < min_open:
            if force and g >= min_pick_buf:
                notes.append(f"  [Adaptive Buffer] Pick finger clearance relaxed to {g:.1f} mm for {peg['shape']} (--force active)")
                open_extra += 2.0
                break
            return {"fail": f"cannot pick: even barely open, the fingers would pass {g:.0f} mm from the {who} "
                            f"(buffer required: {BUF['finger_clearance']:.0f} mm). Move those two pegs about "
                            f"{BUF['finger_clearance'] - g + 5:.0f} mm further apart, or put one nearer to / further from the robot"}
    # when letting go, open as wide as the neighbours allow (a wide opening is what reliably frees the peg)
    release_extra = GR["open_extra_mm"]
    min_rel_extra = GR.get("release_extra_min_mm", 4.0) if (hole is not None or force) else GR["release_extra_mm"]
    min_rel_buf = BUF.get("hole_finger_clearance", 6.0) if (hole is not None or force) else BUF["finger_clearance"]
    while True:
        g, who = fingers_gap(goal_xy, width + release_extra, others)
        if g >= BUF["finger_clearance"]:
            break
        release_extra -= 2.0
        if release_extra < min_rel_extra:
            if (hole is not None or force) and g >= min_rel_buf:
                notes.append(f"  [Adaptive Buffer] Release finger clearance relaxed to {g:.1f} mm (min: {min_rel_buf:.1f} mm, extra: {release_extra + 2.0:.1f} mm) to fit between seated neighbours.")
                release_extra += 2.0
                break
            return {"fail": f"cannot release: fingers would be {g:.0f} mm from the {who} (need {min_rel_buf:.0f} mm)"}
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
        if not mv.can_lift(peg["xy"][0], peg["xy"][1], grip) or not mv.can_carry(goal_xy[0], goal_xy[1], grip):
            continue
        if best is None or min(t_pick, t_rel) > min(best[1], best[2]) + 1e-6:
            best = (float(grip), t_pick, t_rel)
    if best is None:
        from hw_common import SAFE
        p_deg = math.degrees(math.atan2(peg["xy"][1], peg["xy"][0]))
        g_deg = math.degrees(math.atan2(goal_xy[1], goal_xy[0]))
        if p_deg > SAFE["base_max_deg"] or p_deg < SAFE["base_min_deg"]:
            return {"fail": f"peg is at base angle {p_deg:+.0f} deg (safety limit is [{SAFE['base_min_deg']}, {SAFE['base_max_deg']}] deg towards camera stand) - move peg towards center of mat"}
        if g_deg > SAFE["base_max_deg"] or g_deg < SAFE["base_min_deg"]:
            return {"fail": f"hole is at base angle {g_deg:+.0f} deg (safety limit is [{SAFE['base_min_deg']}, {SAFE['base_max_deg']}] deg)"}
        return {"fail": f"out of reach for a top grasp (pick r={math.hypot(*peg['xy']):.0f} mm, goal r={math.hypot(*goal_xy):.0f} mm)"}
    grip, t_pick, t_rel = best
    obstacles = [(o["xy"][0], o["xy"][1], o["across"] / 2) for o in others]
    mover_r = peg["across"] / 2 + GR["finger_thickness_mm"]
    tiers = [
        (BUF["hard_clearance"], BUF["preferred_clearance"], BUF["end_clearance"], 70.0),
    ]
    if hole is not None or force:
        tiers.extend([
            (20.0, BUF["preferred_clearance"], min(BUF["end_clearance"], 8.0), 95.0),
            (15.0, BUF["preferred_clearance"], min(BUF["end_clearance"], 6.0), 110.0),
            (10.0, BUF["preferred_clearance"], min(BUF["end_clearance"], 5.0), 120.0),
        ])
    path = None
    for h_buf, s_buf, e_buf, esc in tiers:
        path = path2d.plan(tuple(peg["xy"]), tuple(goal_xy), obstacles, mover_r, lambda x, y: mv.can_carry(x, y, grip), bounds,
                           hard=h_buf, soft=s_buf, end=e_buf, escape_r=esc)
        if path is not None:
            if h_buf < BUF["hard_clearance"] or e_buf < BUF["end_clearance"]:
                notes.append(f"  [Adaptive Buffer] Path planned with relaxed clearances: hard={h_buf:.0f} mm, end={e_buf:.0f} mm")
            break
    if path is None:
        return {"fail": f"no path keeps {BUF['hard_clearance']:.0f} mm between the carried peg and the other pegs"}
    return {"peg": peg, "goal": tuple(goal_xy), "path": path, "obstacles": obstacles, "mover_r": mover_r, "label": label,
            "grip": grip, "drop": drop, "width": width, "open_extra": open_extra, "release_extra": release_extra, "tilt_pick": t_pick, "tilt_release": t_rel,
            "jaw_mis": jaw_mis, "arrive_mis": arrive_mis, "fit_tol": tol, "into_hole": hole is not None,
            "grasp_offset": grasp_offset(peg), "notes": notes,
            "length": path2d.path_length(path), "gap": path2d.min_gap(path, obstacles, mover_r) if obstacles else 999.0}


def release_point(hole):
    """Where the peg's bottom is aimed: the measured hole centre plus the per-shape fine-tune along
    the line from the robot base (config placement_fine_tune_mm)."""
    x, y = hole["xy"]
    d = float(CFG.get("placement_fine_tune_mm", {}).get("outward", {}).get(hole["shape"], 0.0))
    r = math.hypot(x, y)
    return (x + d * x / r, y + d * y / r)


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
            pl = plan_move(mv, me, release_point(hole), others, f"{peg['shape']} -> hole", bounds, hole, force)
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


def choose_parking(mv, loose, standing, holes, mat_poly, bounds, already_parked=None):
    """Nothing can go to its hole: move one peg to a free spot on the mat so the others get room."""
    best = None
    min_r = CFG.get("suggested_layout", {}).get("min_from_base", 95.0)
    min_x = CFG.get("suggested_layout", {}).get("min_x_in_front", 60.0)
    already_parked = already_parked or set()
    candidates = [p for p in loose if p["shape"] not in already_parked]
    if not candidates:
        return None
    for peg in candidates:
        others = [s for s in standing if s["id"] != peg["id"]]
        for p_dist in (BUF["parking_from_pegs"], 80.0):
            for sx in np.arange(bounds[0], bounds[1], 15.0):
                for sy in np.arange(bounds[2], bounds[3], 15.0):
                    if math.hypot(sx, sy) < min_r or sx < min_x:
                        continue
                    if inside(mat_poly, (sx, sy)) < BUF["parking_from_mat_edge"] or hole_line_distance(holes, (sx, sy)) < BUF["parking_from_board"]:
                        continue
                    if any(math.hypot(sx - o["xy"][0], sy - o["xy"][1]) < p_dist for o in others):
                        continue
                    if math.hypot(sx - peg["xy"][0], sy - peg["xy"][1]) < 50:
                        continue
                    pl = plan_move(mv, peg, (float(sx), float(sy)), others, f"{peg['shape']} -> parking spot", bounds, force=True)
                    if "fail" not in pl and (best is None or pl["length"] < best["length"]):
                        best = pl
            if best is not None:
                break
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

    def can_lift(self, x, y, grip, carry=None):
        carry = HT["carry_peg_bottom"] if carry is None else carry
        if not self.can_carry(x, y, grip):
            return False
        n = max(2, int(carry / 5))
        return all(self.solve(x, y, carry * i / n, extra=grip)[0] is not None for i in range(1, n + 1))

    def goto(self, x, y, above, slow=False, extra=0.0, min_down=None):
        move_joints(self.arm, self.raw_for(x, y, above, extra, min_down), max_step=10 if slow else 25)

    def travel_joints(self):
        """Shoulder/elbow/wrist counts of the folded-up arm (config/robot_safety.yaml)."""
        from hw_common import FOLD_RAW
        return list(FOLD_RAW)

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

    def follow(self, path, above, extra, speed=260.0, offset=0.0):
        """Carry along the planned table path in ONE smooth motion (IK sampled every 10 mm).
        offset: the gripper sits this far along the jaw axis from the peg centre (see grasp_offset)."""
        wps = []
        for a, b in zip(path, path[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 10))
            for i in range(1, n + 1):
                t = i / n
                x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
                tx, ty = tangent((x, y))
                wps.append(self.raw_for(x + offset * tx, y + offset * ty, above, extra))
        stream_path(self.arm, wps, speed=speed, settle=0.5)

    def line(self, x, y, above_to, extra=0.0, min_down=None, speed=150.0):
        """Straight vertical move of the tool point from its present height to above_to (smooth, one motion)."""
        cur = read_joints(self.arm)
        a = K.raw_to_angles(cur)
        off = self.calib.get("joint_offsets_rad", [0.0, 0.0, 0.0])
        _, _, z_now, _ = K.fk([a[0], a[1] + off[0], a[2] + off[1], a[3] + off[2]], tool_len=K.L4 + extra)
        above_from = z_now - self.cm.table_z
        n = max(2, int(abs(above_to - above_from) / 5))
        wps = [self.raw_for(x, y, above_from + (above_to - above_from) * i / n, extra, min_down) for i in range(1, n + 1)]
        stream_path(self.arm, wps, speed=speed, settle=0.4)


def execute(mv, pl):
    peg, (sx, sy), grip = pl["peg"], pl["goal"], pl["grip"]
    px, py = peg["xy"]
    d = pl.get("grasp_offset", 0.0)
    if d:                                                  # aim at the middle of the grip, not the peg centre
        tx, ty = tangent((px, py)); px, py = px + d * tx, py + d * ty
        tx, ty = tangent((sx, sy)); sx, sy = sx + d * tx, sy + d * ty
        print(f"       {peg['shape']}: jaws centre {d:+.1f} mm from its middle - gripper aimed there, same offset at the hole")
    carry = HT["carry_peg_bottom"]
    mv.step(f"{pl['label']}: open fingers {pl['width'] + pl['open_extra']:.0f} mm, turn towards the peg, hover")
    gripper_goto(mv.arm, grip_counts(pl["width"] + pl["open_extra"]))
    mv.tuck(towards=(px, py))
    hover = mv.hover_height(px, py)
    print(f"       hovering {hover - PEG_H:.0f} mm (grip centre) above the peg top")
    mv.goto(px, py, hover); mv.photo("hover_pick")
    mv.step(f"descend on peg (grip {grip:.0f} mm above the mat)"); mv.line(px, py, grip); mv.photo("at_peg")
    mv.step("close gripper")
    g = gripper_goto(mv.arm, GRIP_SQUEEZE)
    held_mm = gap_for_grip_counts(g)
    if not gripper_holding(g):
        print("       gripper closed completely: NOTHING between the fingers")
        gripper_goto(mv.arm, grip_counts(pl["width"] + pl["open_extra"]))
        mv.goto(px, py, hover, slow=True)
        mv.tuck()
        return False
    print(f"       holding the peg: fingers stopped {held_mm:.0f} mm apart (expected about {pl['width']:.0f} mm"
          f"{'' if abs(held_mm - pl['width']) < 8 else ' - OFF-CENTRE or wrong peg?'})")
    mv.step("lift"); mv.line(px, py, carry, extra=grip); mv.photo("lifted")
    mv.step(f"carry along the planned path ({len(pl['path']) - 1} segment(s))")
    mv.follow(pl["path"], carry, grip, offset=d); mv.photo("over_goal")
    mv.step(f"lower (peg will be {90 - pl['tilt_release']:.0f} deg off upright)")
    mv.line(sx, sy, pl["drop"], extra=grip); time.sleep(0.3)
    if pl["into_hole"] and INS["enabled"]:
        seat_in_hole(mv, sx, sy, pl["drop"], grip)
    mv.photo("lowered")
    mv.step(f"release (fingers open {pl['release_extra']:.0f} mm wider than the peg)")
    gripper_goto(mv.arm, grip_counts(held_mm + pl["release_extra"])); time.sleep(0.6)
    mv.step("retreat straight up, clear of the peg top")
    up = mv.hover_height(sx, sy, min_down=TG["retreat_min_down_deg"])
    mv.line(sx, sy, up, min_down=TG["retreat_min_down_deg"]); mv.photo("released")
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


def check_calibration(mv, pegs, bounds, tune=False, mat_poly=None):
    """Show the fingertip to the camera at free spots above the pegs and MEASURE where the camera sees it.

    At every spot the gripper closes and opens once: the picture difference isolates the fingers
    (same detector as the calibration). The gap between "where the arm put its fingertip" and
    "where the camera model says that fingertip is" is the calibration error at that spot.
    With tune=True the errors are fitted and stored as a small correction (fine_tune_affine) in
    config/camera_robot_calib.yaml - a quick repair when the camera has moved a few mm,
    without clearing the table. A full ./run_calibration.sh resets it.
    """
    import importlib
    from hw_common import GRIP_CLOSED
    cal = importlib.import_module("01_calibrate_camera_robot")
    cc = CFG["calibration_check"]
    tilt, above = cc["tool_tilt_deg"], cc["fingertip_above_mat"]
    tip_len, z = mv.calib["fingertip_len_mm"], mv.cm.table_z + above
    cand = []
    for x in np.arange(bounds[0], bounds[1], 20.0):
        for y in np.arange(bounds[2], bounds[3], 20.0):
            if x < cc.get("min_x", 80) or math.hypot(x, y) > cc.get("max_r", 400) or \
                    any(math.hypot(x - p["xy"][0], y - p["xy"][1]) < cc["keep_from_pegs"] for p in pegs):
                continue                                   # (x < 80: too close to the robot's own base to see the fingers)
            u, v = mv.cm.robot_to_pixel(x, y, z)
            if not (60 < u < mv.cm.width - 60 and 60 < v < mv.cm.height - 60):
                continue                                   # the fingertip must be inside the picture
            raw, dn = ik_cmd(x, y, z, tilt, tip_len, mv.calib, min_down_deg=tilt)
            if raw is not None:
                cand.append((float(x), float(y), raw))
    if not cand:
        print("no free spot for the check")
        return
    n = cc["tune_points"] if tune else cc["points"]
    if mat_poly is not None:                               # the fingers are easiest to see against the mat
        on_mat = [c for c in cand if inside(mat_poly, (c[0], c[1])) >= 25]
        if len(on_mat) >= n:
            cand = on_mat
    chosen = [max(cand, key=lambda c: math.hypot(c[0], c[1]))]            # spread the test spots out
    while len(chosen) < min(n, len(cand)):
        chosen.append(max(cand, key=lambda c: min(math.hypot(c[0] - d[0], c[1] - d[1]) for d in chosen)))
    chosen.sort(key=lambda c: math.atan2(c[1], c[0]))                      # sweep across once, no zig-zag
    tiles, cmd, seen = [], [], []
    for i, (x, y, raw) in enumerate(chosen):
        mv.rec.label = f"calibration check {i + 1}/{len(chosen)}"
        mv.tuck(towards=(x, y))
        move_joints(mv.arm, raw, max_step=12)
        gripper_goto(mv.arm, GRIP_OPEN); time.sleep(0.5)
        img_open = mv.cam.read_avg(3)
        gripper_goto(mv.arm, GRIP_CLOSED); time.sleep(0.5)
        img = mv.cam.read_avg(3)
        u, v = mv.cm.robot_to_pixel(x, y, z)
        r = math.hypot(x, y)                                               # a point 50 mm back along the tool
        bx, by, bz = x - 50 * math.cos(math.radians(tilt)) * x / r, y - 50 * math.cos(math.radians(tilt)) * y / r, z + 50 * math.sin(math.radians(tilt))
        ub, vb = mv.cm.robot_to_pixel(bx, by, bz)
        blobs = [b_ for b_ in cal.finger_blobs(img_open, img) if np.linalg.norm(b_[1] - (u, v)) < 110]
        note = "fingers not found"
        if blobs:
            m, c0, _ = min(blobs, key=lambda b_: np.linalg.norm(b_[1] - (u, v)))
            m = cal.merge_fingers(m, c0, blobs)
            tu, tv = cal.tip_from_mask(m, (u - ub, v - vb))
            sx, sy = mv.cm.pixel_to_robot(tu, tv, z)
            cmd.append((x, y)); seen.append((sx, sy))
            note = f"camera sees it at ({sx:.0f},{sy:.0f}): off by {math.hypot(sx - x, sy - y):.1f} mm"
            cv2.circle(img, (int(tu), int(tv)), 9, (0, 0, 255), 2)
        cv2.drawMarker(img, (int(u), int(v)), (0, 255, 255), cv2.MARKER_CROSS, 40, 1)
        cv2.circle(img, (int(u), int(v)), 18, (0, 255, 255), 1)
        cv2.imwrite(os.path.join(mv.dir, f"check_{i}.jpg"), img)
        x0, y0 = int(np.clip(u - 160, 0, img.shape[1] - 320)), int(np.clip(v - 120, 0, img.shape[0] - 240))
        tiles.append(img[y0:y0 + 240, x0:x0 + 320])
        print(f"  spot {i + 1}: fingertip put at robot ({x:.0f},{y:.0f}); {note}")
        mv.tuck()
    gripper_goto(mv.arm, GRIP_OPEN)
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(os.path.join(mv.dir, "check_sheet.jpg"), np.vstack([np.hstack(tiles[k:k + 4]) for k in range(0, len(tiles), 4)]))
    print("picture:", os.path.join(mv.dir, "check_sheet.jpg"), "(yellow cross = expected fingertip, red circle = where the camera found it)")
    if len(cmd) < 3:
        print("too few fingertip detections to judge the calibration")
        return
    C, S = np.array(cmd), np.array(seen)
    err = np.linalg.norm(S - C, axis=1)
    print(f"calibration error now: mean {err.mean():.1f} mm, worst {err.max():.1f} mm over {len(err)} spots "
          f"({'fine' if err.mean() < 4 else 'worth tuning: run  python pick_place.py --tune-calibration' if not tune else 'tuning...'})")
    if not tune:
        return
    # fit  commanded = A @ [seen, 1]  (shift + small rotation/scale); drop spots that disagree with the rest
    keep = err < np.median(err) + 3 * (np.median(np.abs(err - np.median(err))) + 1.0)
    C, S = C[keep], S[keep]
    if len(C) >= 5:
        A = np.linalg.lstsq(np.column_stack([S, np.ones(len(S))]), C, rcond=None)[0].T
    else:
        A = np.array([[1.0, 0.0, (C - S)[:, 0].mean()], [0.0, 1.0, (C - S)[:, 1].mean()]])
    after = np.linalg.norm(np.column_stack([S, np.ones(len(S))]) @ A.T - C, axis=1)
    lin = A[:, :2]
    if abs(np.linalg.det(lin) - 1) > 0.08 or np.abs(A[:, 2]).max() > 30:
        print("the correction needed is too large for a fine-tune - the camera really moved: run ./run_calibration.sh")
        return
    total = np.vstack([A, [0, 0, 1]]) @ np.vstack([mv.cm.tune, [0, 0, 1]])        # on top of any earlier fine-tune
    mv.calib["fine_tune_affine"] = [[float(v) for v in row] for row in total[:2]]
    mv.calib["fine_tuned_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    from hw_common import calib_path
    with open(calib_path(), "w") as f:
        yaml.safe_dump(mv.calib, f, sort_keys=False)
    print(f"fine-tune saved ({keep.sum()} spots): shift ({A[0, 2]:+.1f},{A[1, 2]:+.1f}) mm, "
          f"turn {math.degrees(math.atan2(lin[1, 0], lin[0, 0])):+.2f} deg; error {err[keep].mean():.1f} -> {after.mean():.1f} mm. "
          "Run --check-calibration to confirm.")


def suggest_layout(mv, scan, holes, mat_poly, bounds, run_dir):
    """Draw, on the current picture of the table, where to stand each peg for the best results.

    Spots: inside the mat, in the arm's most accurate zone (fully vertical grasp), whole peg inside
    the camera picture, as far from each other as possible. Which shape goes on which spot is
    decided by running the real planner on every assignment and keeping the best one.
    """
    sl, cm = CFG["suggested_layout"], mv.cm
    shapes = [s_ for s_ in PEGS if s_ in holes]
    cand = []
    for x in np.arange(bounds[0], bounds[1], 5.0):
        for y in np.arange(bounds[2], bounds[3], 5.0):
            r = math.hypot(x, y)
            if x < sl.get("min_x_in_front", 80.0) or not (sl["min_from_base"] <= r <= sl["max_from_base"]) or inside(mat_poly, (x, y)) < sl["from_mat_edge"]:
                continue
            if hole_line_distance(holes, (x, y)) < sl["from_board"]:
                continue
            rr, mg = 30.0, sl["image_margin_px"]           # the whole peg (any shape), foot and top, must be in the picture
            px = [cm.robot_to_pixel(x + dx, y + dy, cm.table_z + h) for h in (0.0, PEG_H) for dx, dy in ((rr, 0), (-rr, 0), (0, rr), (0, -rr))]
            if any(not (mg < u < cm.width - mg and mg < v < cm.height - mg) for u, v in px):
                continue
            if mv.solve(x, y, HT["grip_centre_max"], min_down=89.0)[0] is None:
                continue
            cand.append((float(x), float(y)))
    if len(cand) < len(shapes):
        print("not enough room on the mat for a suggested layout")
        return
    C = np.array(cand)
    chosen = [tuple(C[np.argmin(C[:, 1])])]
    while len(chosen) < len(shapes):
        d = np.min([np.hypot(C[:, 0] - c[0], C[:, 1] - c[1]) for c in chosen], axis=0)
        chosen.append(tuple(C[int(np.argmax(d))]))
    for _ in range(40):                                    # relax: each spot moves to where it is furthest from the others
        for i in range(len(chosen)):
            others = [c for j, c in enumerate(chosen) if j != i]
            d = np.min([np.hypot(C[:, 0] - c[0], C[:, 1] - c[1]) for c in others], axis=0)
            chosen[i] = tuple(C[int(np.argmax(d))])
    spacing = min(math.hypot(a[0] - b[0], a[1] - b[1]) for i, a in enumerate(chosen) for b in chosen[i + 1:])
    best, reasons, pegs, trial = None, {}, None, None
    for perm in itertools.permutations(shapes):
        trial = []
        for i, (s_, (x, y)) in enumerate(zip(perm, chosen)):
            yaw = math.degrees(math.atan2(y, x)) if PEGS[s_]["symmetry_deg"] else None   # a flat side along the line to the robot
            trial.append({"id": i, "shape": s_, "colour": PEGS[s_]["colour"], "xy": (x, y), "yaw": yaw, "spread": 0.0,
                          "across": PEGS[s_]["across_mm"], "placed": False})
        b_, r_ = choose(mv, trial, trial, holes, bounds, False)
        if b_ is not None and (best is None or b_[0] < best[0]):
            best, pegs = b_, trial
        elif b_ is None and pegs is None:
            reasons = r_
    pegs = pegs or trial
    vis = scan.copy()
    cv2.polylines(vis, [np.array([cm.robot_to_pixel(x, y) for x, y in mat_poly], np.int32)], True, (255, 255, 255), 1)
    col = {"green": (60, 200, 60), "blue": (230, 120, 0), "orange": (0, 140, 255), "yellow": (0, 230, 255)}
    for p in pegs:
        c = col.get(p["colour"], (255, 255, 255))
        base = np.array([cm.robot_to_pixel(qx, qy) for qx, qy in footprint(p)], np.int32)
        top = np.array([cm.robot_to_pixel(qx, qy, cm.table_z + PEG_H) for qx, qy in footprint(p)], np.int32)
        layer = vis.copy()
        cv2.fillPoly(layer, [cv2.convexHull(np.vstack([base, top]))], c)       # how the standing peg will look
        vis = cv2.addWeighted(layer, 0.45, vis, 0.55, 0)
        cv2.polylines(vis, [base], True, (0, 0, 0), 4)
        cv2.polylines(vis, [base], True, c, 2)                                 # the foot on the mat
        u, v = cm.robot_to_pixel(*p["xy"])
        cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 0), cv2.MARKER_CROSS, 14, 2)
        hu, hv = cm.robot_to_pixel(*holes[p["shape"]]["xy"])
        cv2.arrowedLine(vis, (int(u), int(v)), (int(hu), int(hv)), c, 1, cv2.LINE_AA, tipLength=0.06)
        label = f"{p['colour'].upper()} {p['shape']}"
        cv2.putText(vis, label, (int(u) - 60, int(v) + 34), 0, 0.55, (0, 0, 0), 4)
        cv2.putText(vis, label, (int(u) - 60, int(v) + 34), 0, 0.55, c, 2)
    cv2.rectangle(vis, (0, 0), (1280, 58), (0, 0, 0), -1)
    cv2.putText(vis, "WHERE TO STAND THE PEGS: put each peg's foot on its outline (black cross = centre of the foot)", (10, 22), 0, 0.6, (255, 255, 255), 1)
    cv2.putText(vis, f"square / hexagon / triangle: one flat side pointing along the thin line to the robot, as drawn.   "
                     f"Pegs are {spacing:.0f} mm apart.", (10, 46), 0, 0.5, (200, 200, 200), 1)
    for out in (os.path.join(run_dir, "suggested_layout.jpg"), os.path.join(T1, "suggested_layout.jpg")):
        cv2.imwrite(out, vis)
    print("suggested layout picture:", os.path.join(T1, "suggested_layout.jpg"))
    for p in pegs:
        print(f"  {p['colour']:6s} {p['shape']:8s}: {math.hypot(*p['xy']):.0f} mm from the robot base, "
              f"{'left' if p['xy'][1] > 0 else 'right'} of the robot's centre line by {abs(p['xy'][1]):.0f} mm")
    print(f"planner check on this layout: all {len(pegs)} moves collision-free, order {best[2]}" if best
          else f"planner check on this layout: {reasons}")


def print_startup_layout_diagnostic(scan, cm, pegs, holes, mat_poly, bounds, mv):
    """Analyze the starting layout and print a structured feasibility diagnostic before motion begins."""
    from hw_common import SAFE
    print("\n" + "=" * 70)
    print("           STARTUP LAYOUT & FEASIBILITY DIAGNOSTIC (Scan 1)")
    print("=" * 70)

    advice = []

    # 1. Holes on the board
    print("[BOARD HOLES]")
    board_order = ["circle", "hexagon", "square", "triangle"]
    for sh in board_order:
        if sh in holes:
            h = holes[sh]
            seated = next((p for p in pegs if math.hypot(p["xy"][0] - h["xy"][0], p["xy"][1] - h["xy"][1]) < 30), None)
            status = f"OCCUPIED by seated {seated['colour']} {seated['shape']}" if seated else "EMPTY (ready for placement)"
            print(f"  {sh:8s} at ({h['xy'][0]:5.1f}, {h['xy'][1]:5.1f}) : {status}")
        else:
            print(f"  {sh:8s} : NOT DETECTED (check board lighting or calibration)")
            advice.append(f"Hole '{sh}' not detected on board. Verify board position / lighting.")

    # 2. Pegs detected on the table
    loose = [p for p in pegs if not p.get("placed", False)]
    seated_count = len(pegs) - len(loose)
    print(f"\n[PEGS DETECTED: {len(pegs)} total | {seated_count} seated in holes | {len(loose)} loose on mat]")
    for p in loose:
        x, y = p["xy"]
        r = math.hypot(x, y)
        deg = math.degrees(math.atan2(y, x))
        issues = []
        if r < 95.0:
            issues.append(f"TOO CLOSE TO BASE (r={r:.0f} mm, min 95 mm - singularity risk)")
            advice.append(f"Move {p['colour']} {p['shape']} forward (+X) away from robot base (current r={r:.0f} mm, need >= 95 mm)")
        elif r > 275.0:
            issues.append(f"TOO FAR (r={r:.0f} mm, max 275 mm - arm reach limit)")
            advice.append(f"Move {p['colour']} {p['shape']} closer to robot base (current r={r:.0f} mm)")

        if deg > SAFE["base_max_deg"]:
            issues.append(f"BASE ANGLE HIGH ({deg:+.0f} deg > {SAFE['base_max_deg']} deg - camera stand collision risk)")
            advice.append(f"Move {p['colour']} {p['shape']} towards center/right of mat (away from camera stand)")
        elif deg < SAFE["base_min_deg"]:
            issues.append(f"BASE ANGLE LOW ({deg:+.0f} deg < {SAFE['base_min_deg']} deg)")
            advice.append(f"Move {p['colour']} {p['shape']} towards center of mat")

        if p.get("top_px") and (p["top_px"][1] < 15 or p["top_px"][1] > cm.height - 15 or
                                p["top_px"][0] < 15 or p["top_px"][0] > cm.width - 15):
            issues.append("NEAR CAMERA FRAME EDGE")
            advice.append(f"Move {p['colour']} {p['shape']} further onto mat away from table/camera edge")

        if p.get("spread", 0.0) > 22.0 and p["shape"] != "fallen":
            issues.append(f"POSSIBLY LEANING (spread={p['spread']:.0f} mm)")

        status_str = (" | ALERT: " + "; ".join(issues)) if issues else " | OK"
        print(f"  {p['shape']:8s} ({p['colour']:6s}) at ({x:5.0f},{y:5.0f}) -> r={r:3.0f} mm, base angle={deg:+3.0f} deg{status_str}")

    # 3. Pairwise inter-peg spacing on the mat
    print("\n[INTER-PEG SPACING ON MAT]")
    if len(loose) <= 1:
        print("  Only 1 loose peg on mat - spacing check passed.")
    else:
        for i in range(len(loose)):
            for j in range(i + 1, len(loose)):
                p1, p2 = loose[i], loose[j]
                d = math.hypot(p1["xy"][0] - p2["xy"][0], p1["xy"][1] - p2["xy"][1])
                if d < 65.0:
                    print(f"  CRITICAL: {p1['shape']} and {p2['shape']} are TOO CLOSE ({d:.0f} mm apart, need >= 85 mm). Gripper cannot open safely between them!")
                    advice.append(f"Separate {p1['colour']} {p1['shape']} and {p2['colour']} {p2['shape']} by at least {85 - d:.0f} mm more")
                elif d < 85.0:
                    print(f"  TIGHT: {p1['shape']} and {p2['shape']} are {d:.0f} mm apart (narrow grasp / adaptive buffer will be used)")
                else:
                    print(f"  CLEAR: {p1['shape']} and {p2['shape']} spacing is good ({d:.0f} mm apart)")

    # 4. Placement Feasibility Check
    print("\n[PLACEMENT FEASIBILITY]")
    for p in loose:
        sh = p["shape"]
        if sh not in holes:
            print(f"  {sh:8s}: Target hole missing from detected holes!")
            continue
        h = holes[sh]
        others = [s for s in pegs if s["id"] != p["id"]]
        g_pick, who_p = fingers_gap(p["xy"], jaw_width(sh) + 20.0, others)
        g_rel, who_r = fingers_gap(release_point(h), jaw_width(sh) + 10.0, others)
        pick_status = "OK" if g_pick >= 12.0 else f"TIGHT ({g_pick:.0f} mm from {who_p})"
        rel_status = "OK" if g_rel >= 12.0 else f"TIGHT ({g_rel:.0f} mm from {who_r}, adaptive buffer active)"
        print(f"  {sh:8s}: Pick: {pick_status} | Release: {rel_status}")

    # 5. Actionable Advice
    print("\n" + "=" * 70)
    if advice:
        print("RECOMMENDED LAYOUT ADJUSTMENTS BEFORE ROBOT MOTION:")
        for idx, adv in enumerate(advice, 1):
            print(f"  {idx}. {adv}")
    else:
        print("ALL CHECKS PASSED: Layout is optimal and all pegs are in safe zones.")
    print("=" * 70 + "\n")


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
    ap.add_argument("--save-holes", action="store_true",
                    help="detect the holes and store them in config/detected_holes.yaml (run_calibration.sh does this)")
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--suggest-layout", action="store_true", help="draw where to stand the pegs for the best results (empty mat)")
    ap.add_argument("--check-calibration", action="store_true", help="measure how well the calibration still fits (pegs can stay)")
    ap.add_argument("--tune-calibration", action="store_true", help="same, then store a small correction (pegs can stay)")
    ap.add_argument("--force", action="store_true", help="try a peg even if its rotation does not match the hole")
    ap.add_argument("--order", default="", help="fix the order yourself, e.g. --order circle,square (default: best order is chosen)")
    a = ap.parse_args()

    run_dir = os.path.join(T1, "runs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir)
    from hw_common import calib_path
    calib, cm = load_calib(), CameraModel(calib_path())
    cam = LiveCamera()
    arm = connect_arm()
    rec = Recorder(cam, os.path.join(run_dir, "run.mp4"))
    mv = Mover(arm, cam, cm, calib, rec, run_dir)
    holes, done, failed, lost, pending = {}, [], {}, [], None
    holes_file = os.path.join(T1, "config", "detected_holes.yaml")
    if os.path.exists(holes_file) and not a.save_holes:      # written by ./run_calibration.sh with THIS calibration
        try:
            with open(holes_file) as f:
                saved = yaml.safe_load(f) or {}
            if saved.get("calibrated_at") == calib.get("calibrated_at"):
                holes.update({k: dict(v, xy=tuple(v["xy"]), side=v.get("side"), cnt=None) for k, v in saved["holes"].items()})
                print(f"holes from {os.path.basename(holes_file)} (saved with the current calibration): {sorted(holes)}")
            else:
                print(f"{os.path.basename(holes_file)} belongs to an older calibration - not used (holes are detected live)")
        except Exception as e:
            print(f"could not read {holes_file}: {e}")
    parked_pegs = set()
    try:
        forced = [x.strip() for x in a.order.split(",")] if a.order else None
        for rnd in range(1, 10):
            rec.label = f"scan {rnd}"
            mv.tuck(base_raw=calib["scan_pose_raw"][0]); time.sleep(0.6)     # tucked, turned out of the camera's view
            scan = cam.read_avg(4)
            mat_poly = find_mat(scan, cm)
            new_holes = find_holes(scan, cm)
            for h in new_holes:                 # live detection wins; a hole hidden by a peg keeps its saved / last position
                old = holes.get(h["shape"])
                if old is not None and math.hypot(h["xy"][0] - old["xy"][0], h["xy"][1] - old["xy"][1]) > 25.0:
                    print(f"NOTE: the {h['shape']} hole is {math.hypot(h['xy'][0] - old['xy'][0], h['xy'][1] - old['xy'][1]):.0f} mm "
                          "from where it was saved - was the board moved? Using the live position.")
                holes[h["shape"]] = h
            if a.save_holes:
                if len(holes) < 4:
                    print(f"only {len(holes)} of 4 holes seen ({sorted(holes)}) - nothing saved; is the board fully in view and empty?")
                else:
                    with open(holes_file, "w") as f:
                        yaml.safe_dump({"calibrated_at": calib.get("calibrated_at"),
                                        "holes": {k: {"shape": v["shape"], "xy": [float(v["xy"][0]), float(v["xy"][1])],
                                                      "yaw": None if v.get("yaw") is None else float(v["yaw"]),
                                                      "side": None if v.get("side") is None else float(v["side"])}
                                                  for k, v in holes.items()}}, f, sort_keys=False)
                    print(f"saved {len(holes)} holes to {holes_file}")
                break
            board_poly = find_board(scan, cm)
            hl = list(holes.values())
            pegs = stable_pegs(cam, cm, mat_poly, hl, board_poly, find_pegs(scan, cm, mat_poly, hl, board_poly))
            for i, p in enumerate(pegs):
                p["id"] = i
                at_h = next((h for h in holes.values() if math.hypot(p["xy"][0] - h["xy"][0], p["xy"][1] - h["xy"][1]) < 30), None)
                if at_h is not None and p["shape"] == "cut-off" and p.get("claims") in (at_h["shape"], None):
                    p["shape"] = at_h["shape"]
                    p["across"] = PEGS[at_h["shape"]]["across_mm"]
                p["placed"] = p["shape"] != "fallen" and (p.get("hidden_hole", False) or at_h is not None)
            loose = [p for p in pegs if not p["placed"] and p["shape"] not in ("fallen", "cut-off")]
            if pending:
                mine = next((p for p in pegs if p["shape"] == pending and p["placed"]), None)
                if mine is not None and mine["spread"] <= BUF["seated_check"]:
                    done.append(pending)
                    print(f"VERIFIED: the {pending} peg stands upright in its hole.")
                elif mine is not None:
                    lost.append(pending + " (leaning)")
                    print(f"WARNING: the {pending} peg is at its hole but LEANING (not seated) - push it in by hand; "
                          "if this repeats, run  python pick_place.py --check-calibration")
                else:
                    lost.append(pending)
                    print(f"WARNING: the {pending} peg is NOT in its hole after the move"
                          + (" and is nowhere in the picture - it probably stuck to the gripper and dropped outside the view."
                             if not any(p["shape"] == pending or p.get("claims") == pending for p in pegs) else "."))
                pending = None
            allpts = np.vstack([mat_poly] + [np.array([h["xy"]]) for h in holes.values()])
            bounds = (float(allpts[:, 0].min()) - 40, float(allpts[:, 0].max()) + 40,
                      float(allpts[:, 1].min()) - 40, float(allpts[:, 1].max()) + 40)
            cv2.imwrite(os.path.join(run_dir, f"scan{rnd}_raw.jpg"), scan)
            cv2.imwrite(os.path.join(run_dir, f"scan{rnd}_annotated.jpg"), annotate(scan, cm, pegs, holes, mat_poly))
            if rnd == 1:
                print_startup_layout_diagnostic(scan, cm, pegs, holes, mat_poly, bounds, mv)
            print(f"\n--- scan {rnd} ---")
            print("holes :", {k: [round(v) for v in h["xy"]] for k, h in holes.items()})
            for p in pegs:
                print(f"peg   : {p['shape']:8s} ({p['colour']}) at ({p['xy'][0]:.0f},{p['xy'][1]:.0f})"
                      f"{'' if p['yaw'] is None else ' yaw %.0f deg' % p['yaw']}  {'in hole' if p['placed'] else 'on mat'}"
                      f"  [position check {p['spread']:.0f} mm]")
            if a.suggest_layout:
                suggest_layout(mv, scan, holes, mat_poly, bounds, run_dir)
                break
            if a.check_calibration or a.tune_calibration:
                check_calibration(mv, pegs, bounds, tune=a.tune_calibration, mat_poly=mat_poly)
                break
            if a.detect_only:
                break
            for c0 in [p for p in pegs if p["shape"] == "cut-off"]:
                print(f"NOTE: the {c0['colour']} peg near ({c0['xy'][0]:.0f},{c0['xy'][1]:.0f}) is partly outside the camera picture. "
                      "It is avoided as a (generously sized) obstacle but will not be picked - move it towards the middle of the mat.")
            if any(p["shape"] == "fallen" for p in pegs):
                if a.force:
                    print("WARNING: a peg seems to be lying down, but proceeding because of --force.")
                else:
                    print("STOP: a peg seems to be lying down (or something else coloured is on the table). "
                          "Stand it up / remove it and run again (or use --force to proceed anyway) - see the scan picture.")
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
                for n_ in dict.fromkeys(pl.get("notes", [])):
                    print(n_)
                advise_upright(mv, pl)
            else:
                blocked = [p for p in todo if "fingers" in reasons.get(p["shape"], "") or "path" in reasons.get(p["shape"], "")]
                pl = None
                if blocked:                                # crowded: move a peg that is in the way, the blocked one last
                    ids = {p["id"] for p in blocked}
                    pl = (choose_parking(mv, [p for p in loose if p["id"] not in ids], pegs, holes, mat_poly, bounds, parked_pegs)
                          or choose_parking(mv, blocked, pegs, holes, mat_poly, bounds, parked_pegs))
                if pl is None:
                    if parked_pegs:
                        print(f"Pegs {list(parked_pegs)} were already parked, but remaining pegs still cannot reach their holes. Stopping safely.")
                    else:
                        print("nothing can be moved safely - see the reasons above.")
                    break
                parked_pegs.add(pl["peg"]["shape"])
                print(f"blocked -> first making room: {pl['label']} at ({pl['goal'][0]:.0f},{pl['goal'][1]:.0f})")
            cv2.imwrite(os.path.join(run_dir, f"plan{rnd}.jpg"), annotate(scan, cm, pegs, holes, mat_poly, pl))
            if a.plan_only:
                break
            ok = execute(mv, pl)
            if ok and pl["into_hole"]:
                pending = pl["peg"]["shape"]               # only counted once the next scan sees it in its hole
            elif not ok:
                failed[pl["peg"]["shape"]] = failed.get(pl["peg"]["shape"], 0) + 1
        print("\nRESULT: verified in their holes:", done or "nothing", "| not in hole after the move:", lost or "none",
              "| failed grasps:", failed or "none")
        mv.tuck(base_raw=calib["scan_pose_raw"][0])
    finally:
        rec.close()
        cam.release()
        arm.disconnect(disable_torque=False)
        print("run folder:", run_dir)


if __name__ == "__main__":
    main()
