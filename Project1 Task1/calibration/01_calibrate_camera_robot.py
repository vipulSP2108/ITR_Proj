#!/usr/bin/env python3
"""
STEP 1 - Camera <-> robot calibration (eye-to-hand, automatic, model-free "two-plane" method).

Idea (same as "detect a point with the camera, move the end-effector there, store both
coordinates, repeat, then average"):
  1. (only with --probe) the fingertip is lowered slowly on the mat until it touches -> table height.
  2. collect : the closed fingertip is placed on a grid of KNOWN robot positions 45 mm above
               the table (plane LOW) and again 125 mm above it (plane HIGH) - it never touches anything. At every position
               the gripper opens/closes; the image difference shows exactly where the closed
               fingers are, and their far end is the fingertip pixel.
  3. solve   : for each plane a homography pixel -> robot XY is fitted through all points
               (least squares = the "average"). A pixel is a ray through both planes, so the
               robot XY of anything at any height z follows by interpolation. No camera
               intrinsics, no hand-measured camera position needed.
  4. check   : the ArUco markers on the table (38 mm squares) are measured through the new
               calibration; their size must come out as 38 mm.
Result: config/camera_robot_calib.yaml   (used by src/vision/camera_model.py)

Usage (real terminal, inside "Project1 Task1"):
    python calibration/01_calibrate_camera_robot.py            # probe + collect + solve (~7 min)
    python calibration/01_calibrate_camera_robot.py --solve    # re-solve from the saved images
Before running: mat clear of loose objects (the flat board and paper markers may stay),
hands away. The arm stays below 250 mm.
"""
import os, sys, json, math, argparse, glob, time
import numpy as np
import cv2
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hw_common import T1_DIR, CAM_W, CAM_H, GRIP_OPEN, GRIP_CLOSED, FOLD_RAW, SCAN_RAW
from src.planning import omx_kinematics as K

DATA = os.environ.get("CAL_DATA_DIR") or os.path.join(T1_DIR, "calibration", "output", "calib_data")
OUT = os.path.join(T1_DIR, "calibration", "output")
CALIB_YAML = os.environ.get("CAL_OUT_YAML") or os.path.join(T1_DIR, "config", "camera_robot_calib.yaml")
with open(os.path.join(T1_DIR, "config", "calibration_setup.yaml")) as _f:
    SETUP = yaml.safe_load(_f)                      # every number below comes from that file
MARKER_MM = float(SETUP["marker_side_mm"])
ARUCO_DICT = cv2.aruco.DICT_4X4_50
SCAN_POSES = {"scan": SCAN_RAW}       # arm folded up, turned away from the camera stand (config/robot_safety.yaml)

TIP_LEN0 = float(SETUP["nominal_fingertip_len_mm"])
PLANE_LOW, PLANE_HIGH = SETUP["fingertip_planes_above_table_mm"]
TABLE_Z = float(SETUP["table_z_mm"])
GRID_X, GRID_Y = tuple(SETUP["grid_x_mm"]), tuple(SETUP["grid_y_mm"])
TILTS = tuple(SETUP["tool_tilts_deg"])
PROBE_XY = tuple(SETUP["probe_xy_mm"])


def tip_ik(x, y, z, down):
    """Nominal IK for the fingertip; falls back to flatter tilts when out of reach."""
    d = down
    while d >= 15:
        raw = K.ik_raw(x, y, z, d, TIP_LEN0)
        if raw is not None:
            return raw, d
        d -= 5
    return None, None


# -------------------------------------------------------------------- probe --
def probe_table(arm):
    """Lower the fingertip in 2 mm steps until the joints stop following -> table height (commanded z)."""
    from hw_common import move_joints
    x, y = PROBE_XY
    raw, dn = tip_ik(x, y, 70, 60)
    move_joints(arm, raw)
    errs, z = [], 70.0
    while z > -25:
        z -= 2.0
        raw, _ = tip_ik(x, y, z, dn)
        if raw is None:
            break
        got = move_joints(arm, raw, max_step=6, settle=0.25, tol=3)
        err = sum(abs(a - b) for a, b in zip(got[1:], raw[1:]))
        if z > 46:
            errs.append(err)
            continue
        base_err = float(np.median(errs))
        if err - base_err > 14:
            table = z + 2.0
            print(f"  touch at commanded z = {z:.0f} mm (joint error {err} vs {base_err:.0f}) -> table z = {table:.0f} mm")
            raw, _ = tip_ik(x, y, table + 40, dn)
            move_joints(arm, raw)
            return table
    raise RuntimeError("table not found while probing - is the arm above the mat?")


# ---------------------------------------------------------------- checks --
def check_framing(img):
    """Warn if the mat or the row of holes comes close to the edge of the camera picture
    (pegs standing there cannot be seen whole). Uses the same mat / hole detectors as pick_place.py."""
    import importlib
    pp = importlib.import_module("pick_place")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m_ = int(SETUP.get("frame_margin_px", 25))
    h, w = img.shape[:2]
    mat, _ = pp.mat_mask(hsv)
    mat = cv2.morphologyEx(mat, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(mat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = {}
    if cnts:
        boxes["mat"] = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    red = cv2.inRange(hsv, (0, 90, 90), (12, 255, 255)) | cv2.inRange(hsv, (168, 90, 90), (179, 255, 255))
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    rc, hier = cv2.findContours(red, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    holes = [c for c, hh in zip(rc, hier[0]) if hh[3] >= 0 and 1500 < cv2.contourArea(c) < 20000] if hier is not None else []
    if holes:
        boxes[f"row of {len(holes)} holes"] = cv2.boundingRect(np.vstack(holes))
    for name in ("mat", "row of holes"):
        if not any(k.startswith(name.split()[0]) for k in boxes):
            print(f"FRAMING: cannot see the {name} in the picture")
    for name, (x, y, bw, bh) in boxes.items():
        edges = [e for e, bad in (("top", y < m_), ("bottom", y + bh > h - m_), ("left", x < m_), ("right", x + bw > w - m_)) if bad]
        if edges:
            print(f"FRAMING WARNING: the {name} reaches the {' and '.join(edges)} edge of the camera picture - "
                  "pegs standing there will be cut off. Better: aim the camera so it sits fully inside the picture, then recalibrate.")
        else:
            print(f"framing: the {name} is fully inside the camera picture - good")


def camera_drift_px():
    """How far the ArUco markers moved in the picture between the start and the end of the run (pixels)."""
    a, b = os.path.join(DATA, "scan.png"), os.path.join(DATA, "scan_end.png")
    if not (os.path.exists(a) and os.path.exists(b)):
        return None
    ma, mb = detect_markers(cv2.imread(a)), detect_markers(cv2.imread(b))
    common = sorted(set(ma) & set(mb))
    if not common:
        return None
    return float(max(np.linalg.norm(ma[i] - mb[i], axis=1).max() for i in common))


# ------------------------------------------------------------------ collect --
QUICK = [False]
NPOINTS = [0]


def collect(probe=False):
    from hw_common import LiveCamera, connect_arm, move_joints, read_joints, gripper_goto
    os.makedirs(DATA, exist_ok=True)
    for f in glob.glob(os.path.join(DATA, "*")):
        os.remove(f)
    cam = LiveCamera()
    arm = connect_arm()
    records, meta = [], {}
    try:
        cur = read_joints(arm)
        move_joints(arm, [cur[0]] + FOLD_RAW)           # fold up where it stands first
        gripper_goto(arm, GRIP_OPEN)
        for name, pose in SCAN_POSES.items():           # clear views of the table for ArUco
            move_joints(arm, pose)
            time.sleep(0.5)
            cv2.imwrite(os.path.join(DATA, f"{name}.png"), cam.read_avg(4))
        check_framing(cv2.imread(os.path.join(DATA, "scan.png")))
        gripper_goto(arm, GRIP_CLOSED)
        meta["table_z"] = TABLE_Z
        if probe:
            print("probing table height (fingertip touches the mat once, gently)...")
            meta["table_z"] = probe_table(arm)
            SETUP["table_z_mm"] = float(meta["table_z"])
            print(f"NOTE: measured table height {meta['table_z']:.0f} mm - put this as table_z_mm in config/calibration_setup.yaml")
        targets = []
        for plane, h in (("low", PLANE_LOW), ("high", PLANE_HIGH)):
            k = 0
            for x in GRID_X:
                for y in (GRID_Y if GRID_X.index(x) % 2 == 0 else GRID_Y[::-1]):   # snake path
                    targets.append((plane, x, y, meta["table_z"] + h, TILTS[k % 2]))
                    k += 1
        if QUICK[0]:                                       # try-out: N well spread poses over both heights
            n = NPOINTS[0] or 8
            pts3 = np.array([[t[1], t[2], (t[3] - meta["table_z"]) * 1.5] for t in targets], float)
            chosen = [int(np.argmin(pts3[:, 0] + pts3[:, 1]))]
            while len(chosen) < min(n, len(targets)):    # farthest-point sampling: spread over x, y and height
                dmin = np.min([np.linalg.norm(pts3 - pts3[c], axis=1) for c in chosen], axis=0)
                chosen.append(int(np.argmax(dmin)))
            chosen.sort()                                  # keep the snake order (short, smooth moves)
            targets = [(targets[c][0], targets[c][1], targets[c][2], targets[c][3], TILTS[k_ % 2]) for k_, c in enumerate(chosen)]
        for i, (plane, x, y, z, tilt) in enumerate(targets):
            raw, dn = tip_ik(x, y, z, tilt)
            if raw is None:
                print(f"[{i + 1}/{len(targets)}] ({x},{y},{z:.0f}) unreachable - skipped")
                continue
            hop, _ = tip_ik(x, y, z + 30, dn)            # approach from above
            if hop is not None:
                move_joints(arm, hop)
            move_joints(arm, raw, max_step=12)
            gripper_goto(arm, GRIP_OPEN); time.sleep(0.4)
            img_open = cam.read_avg(3)
            gripper_goto(arm, GRIP_CLOSED); time.sleep(0.4)
            img_closed = cam.read_avg(3)
            cv2.imwrite(os.path.join(DATA, f"p{i:02d}_open.png"), img_open)
            cv2.imwrite(os.path.join(DATA, f"p{i:02d}_closed.png"), img_closed)
            records.append({"i": i, "plane": plane, "xyz": [x, y, z], "down": dn, "raw_cmd": raw,
                            "raw_meas": read_joints(arm)})
            print(f"[{i + 1}/{len(targets)}] {plane} tip=({x},{y},{z:.0f}) tilt={dn:.0f}", flush=True)
            if hop is not None:
                move_joints(arm, hop)
        move_joints(arm, SCAN_POSES["scan"])              # same view as at the start: did anything move?
        time.sleep(0.5)
        cv2.imwrite(os.path.join(DATA, "scan_end.png"), cam.read_avg(4))
        gripper_goto(arm, GRIP_OPEN)
    finally:
        meta["records"] = records
        with open(os.path.join(DATA, "records.json"), "w") as f:
            json.dump(meta, f, indent=1)
        cam.release()
        arm.disconnect(disable_torque=False)


# ------------------------------------------------------------------- vision --
def finger_blobs(img_open, img_closed):
    """Candidate blobs of the CLOSED finger pair: pixels that turn from bright to black when the gripper closes."""
    go = cv2.GaussianBlur(cv2.cvtColor(img_open, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.int16)
    gc = cv2.GaussianBlur(cv2.cvtColor(img_closed, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.int16)
    m = (((go - gc) > 28) & (gc < 75)).astype(np.uint8) * 255      # fingers are black plastic
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    return [((lab == j).astype(np.uint8) * 255, cent[j], stats[j, cv2.CC_STAT_AREA])
            for j in range(1, n) if stats[j, cv2.CC_STAT_AREA] >= 250]


def merge_fingers(mask, centre, blobs, reach_px=90):
    """Both fingers belong to the fingertip: add every other finger blob lying right next to the chosen
    one (the two fingers can show up as two blobs when a small gap stays between them)."""
    out = mask.copy()
    for m_, c_, _ in blobs:
        if m_ is not mask and np.linalg.norm(np.asarray(c_) - np.asarray(centre)) < reach_px:
            out = cv2.bitwise_or(out, m_)
    return out


def tip_from_mask(mask, direction):
    """Fingertip mid-point of the closed finger pair.

    The blob is a two-pronged fork pointing along `direction` (tool axis in the image). In its far
    part the two fingers are separate: the tip of each prong is found and their middle returned.
    """
    ys, xs = np.nonzero(mask)
    pts = np.stack([xs, ys], 1).astype(np.float64)
    c = pts.mean(0)
    u = np.asarray(direction, float)
    u = u / np.linalg.norm(u)
    v = np.array([-u[1], u[0]])
    t, w = (pts - c) @ u, (pts - c) @ v
    tmax, tmin = np.percentile(t, 99.7), np.percentile(t, 0.3)
    far = t >= tmax - 0.4 * (tmax - tmin)
    mid = 0.5 * (w[far].min() + w[far].max())                 # between the outer edges of the two fingers
    tips = []
    for side in (far & (w < mid), far & (w >= mid)):
        if side.sum() > 0.08 * far.sum():
            tt = np.percentile(t[side], 99.5)
            tips.append((tt, w[side & (t >= tt - 4)].mean()))
    if len(tips) == 2:
        tt, ww = np.mean([a_[0] for a_ in tips]), np.mean([a_[1] for a_ in tips])
    else:
        tt, ww = tmax, mid
    return c + tt * u + ww * v


def detect_markers(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT))
    corners, ids, _ = det.detectMarkers(gray)
    out = {}
    if ids is not None:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01)
        for c, i in zip(corners, ids.flatten()):
            c = cv2.cornerSubPix(gray, c.reshape(-1, 1, 2).astype(np.float32), (4, 4), (-1, -1), crit)
            out[int(i)] = c.reshape(4, 2).astype(np.float64)
    return out


# ------------------------------------------------------------- charuco board --
BOARD_DIR = os.path.join(T1_DIR, "calibration", "output")
BOARD_SNAPSHOT = os.path.join(BOARD_DIR, "board_snapshot.png")     # first placement; more: board_snapshot_2.png, _3 ...


def charuco_points(img):
    """Board corners seen in img: (local board coordinates in mm (N x 2), pixels (N x 2)), or None."""
    cb = SETUP.get("charuco_board")
    if not cb or img is None:
        return None
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cb["dictionary"]))
    board = cv2.aruco.CharucoBoard(tuple(cb["squares"]), float(cb["square_mm"]), float(cb["marker_mm"]), d)
    board.setLegacyPattern(bool(cb.get("legacy_pattern", False)))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cc, ci, mc, mi = cv2.aruco.CharucoDetector(board).detectBoard(gray)
    loc, px = [], []
    if ci is not None:
        chess = board.getChessboardCorners()
        for c, i in zip(cc.reshape(-1, 2), ci.ravel()):
            loc.append(chess[int(i)][:2]); px.append(c)
    if mi is not None:
        obj, ids = board.getObjPoints(), list(board.getIds().ravel())
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01)
        for c, i in zip(mc, mi.ravel()):
            c = cv2.cornerSubPix(gray, c.reshape(-1, 1, 2).astype(np.float32), (3, 3), (-1, -1), crit).reshape(4, 2)
            for k in range(4):
                loc.append(np.asarray(obj[ids.index(int(i))])[k][:2]); px.append(c[k])
    if len(loc) < 12:
        return None
    return np.array(loc, float), np.array(px, float)


def board_snapshot():
    """Arm to the scan pose, take a picture of the ChArUco board (python ... --board-snapshot)."""
    from hw_common import LiveCamera, connect_arm, move_joints, read_joints
    cam, arm = LiveCamera(), connect_arm()
    try:
        cur = read_joints(arm)
        move_joints(arm, [cur[0]] + FOLD_RAW)
        move_joints(arm, SCAN_POSES["scan"]); time.sleep(0.8)
        img = cam.read_avg(6)
    finally:
        cam.release()
        arm.disconnect(disable_torque=False)
    pts = charuco_points(img)
    if pts is None:
        print("board NOT found in the picture - is it flat on the mat and fully visible?")
        return
    import glob
    old = sorted(glob.glob(os.path.join(BOARD_DIR, "board_snapshot*.png")))
    fresh = [f for f in old if time.time() - os.path.getmtime(f) < 3600 * SETUP.get("charuco_board", {}).get("snapshot_max_age_h", 3.0)]
    path = BOARD_SNAPSHOT if not fresh else os.path.join(BOARD_DIR, f"board_snapshot_{len(fresh) + 1}.png")
    if not fresh:
        for f in old:                                    # stale pictures from an earlier session
            os.remove(f)
    cv2.imwrite(path, img)
    print(f"board picture {len(fresh) + 1} saved ({len(pts[0])} corner points): {path}")
    print("Move the board to another spot and run --board-snapshot again for more coverage, or take it OFF the mat and calibrate.")


def camera_shift_px(img_a, img_b):
    """How far the CAMERA moved between two pictures (px), from the static background.

    Matches features over the whole picture and fits one shift+rotation with RANSAC: the floor,
    table edges and rail agree on it, while moved things (a chair, the arm, a person) are ignored.
    """
    orb = cv2.ORB_create(4000)
    ga, gb = [cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) for x in (img_a, img_b)]
    ka, da = orb.detectAndCompute(ga, None)
    kb, db = orb.detectAndCompute(gb, None)
    if da is None or db is None:
        return 99.0
    m = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
    if len(m) < 30:
        return 99.0
    pa = np.float32([ka[x.queryIdx].pt for x in m]); pb = np.float32([kb[x.trainIdx].pt for x in m])
    M, inl = cv2.estimateAffinePartial2D(pa, pb, method=cv2.RANSAC, ransacReprojThreshold=1.5)
    if M is None or inl.sum() < 20:
        return 99.0
    corners = np.float32([[0, 0], [CAM_W, 0], [CAM_W, CAM_H], [0, CAM_H]])
    moved = corners @ M[:, :2].T + M[:, 2] - corners
    return float(np.linalg.norm(moved, axis=1).max())


def board_for_calibration(scan_img):
    """List of board corner sets for the solver: the board seen in the scan picture itself, plus every
    recent board snapshot taken from the SAME camera position (checked by comparing the pictures)."""
    import glob
    out = []
    pts = charuco_points(scan_img)
    if pts is not None:
        print(f"ChArUco board seen in the scan picture: {len(pts[0])} corner points")
        out.append(pts)
    w = scan_img.shape[1] // 3
    for f in sorted(glob.glob(os.path.join(BOARD_DIR, "board_snapshot*.png"))):
        age_h = (time.time() - os.path.getmtime(f)) / 3600.0
        if age_h > SETUP.get("charuco_board", {}).get("snapshot_max_age_h", 3.0):
            print(f"{os.path.basename(f)} is {age_h:.1f} h old - not used")
            continue
        snap = cv2.imread(f)
        shift = camera_shift_px(snap, scan_img)           # the camera must not have moved since
        if shift > 1.5:
            print(f"{os.path.basename(f)} NOT used: the camera, or the mat/robot, moved {shift:.1f} px since it was taken")
            continue
        p = charuco_points(snap)
        if p is not None:
            print(f"using {os.path.basename(f)} ({len(p[0])} corner points, taken {age_h * 60:.0f} min ago, camera unchanged)")
            out.append(p)
    return out


# -------------------------------------------------------------------- solve --
F0 = float(SETUP["nominal_focal_px"])
FOCAL_PRIOR_SIGMA = float(SETUP.get("focal_prior_sigma_px", 30.0))   # how firmly the focal length is held near nominal
LENS_CENTRE_SIGMA = float(os.environ.get("CAL_CENTRE_SIGMA", SETUP.get("lens_centre_sigma_px", 15.0)))
DISTORTION_SIGMA = float(os.environ.get("CAL_DIST_SIGMA", SETUP.get("distortion_sigma", 0.05)))
LATERAL_SIGMA = float(os.environ.get("CAL_LAT_SIGMA", SETUP.get("gripper_sideways_sigma_mm", 1.0)))
MARKER_LOCAL = MARKER_MM / 2 * np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]], float)   # TL TR BR BL


def _lens(cam):
    """cam = focal (px)  or  (focal, centre shift x, centre shift y, k1, k2) -> f, cx, cy, k1, k2."""
    c = np.atleast_1d(np.asarray(cam, float))
    c = np.concatenate([c, np.zeros(5 - len(c))])
    return c[0], CAM_W / 2 + c[1], CAM_H / 2 + c[2], c[3], c[4]


def project(P, rvec, tvec, cam):
    f, cx, cy, k1, k2 = _lens(cam)
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    pc = np.atleast_2d(P) @ R.T + tvec
    xn, yn = pc[:, 0] / pc[:, 2], pc[:, 1] / pc[:, 2]
    r2 = xn * xn + yn * yn
    d = 1 + k1 * r2 + k2 * r2 * r2                      # radial lens distortion
    return np.stack([f * xn * d + cx, f * yn * d + cy], 1)


def undistort_px(px, cam):
    """Pixel -> ideal normalised image coordinates (inverse of the distortion above)."""
    f, cx, cy, k1, k2 = _lens(cam)
    out = []
    for u, v in np.atleast_2d(px):
        xd, yd = (u - cx) / f, (v - cy) / f
        xn, yn = xd, yd
        for _ in range(8):
            r2 = xn * xn + yn * yn
            d = 1 + k1 * r2 + k2 * r2 * r2
            xn, yn = xd / d, yd / d
        out.append((xn, yn))
    return np.array(out)


def backproject(px, rvec, tvec, cam, z):
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    c0 = -R.T @ tvec
    out = []
    for xn, yn in undistort_px(px, cam):
        d = R.T @ np.array([xn, yn, 1.0])
        out.append(c0 + (z - c0[2]) / d[2] * d)
    return np.array(out)


def marker_corners3d(mx, my, th, z):
    c, s = math.cos(th), math.sin(th)
    xy = MARKER_LOCAL @ np.array([[c, s], [-s, c]]) + [mx, my]
    return np.column_stack([xy, np.full(4, z)])


QUALITY_OK = [True]


def robust(r, c=3.0):
    """Residual transform whose square is the soft-L1 cost (big errors count less) - applied to measurements only."""
    r = np.asarray(r, float)
    return np.sign(r) * c * np.sqrt(2.0 * (np.sqrt(1.0 + (r / c) ** 2) - 1.0))


def init_from_board(board_sets, rows, tip3d, table_z):
    """Starting camera pose from the ChArUco board (no previous calibration needed).

    1) The board picture gives the camera pose relative to the BOARD (plane + known size).
    2) Each fingertip is a point at a known height above the table: its ray hits that height at a
       known spot in board coordinates; its robot coordinates come from the arm. A 2-D fit of those
       pairs gives where the board lies in the robot frame -> camera pose in the robot frame.
    """
    loc, px = board_sets[0]
    K = np.array([[F0, 0, CAM_W / 2], [0, F0, CAM_H / 2], [0, 0, 1]])
    obj = np.column_stack([loc, np.zeros(len(loc))]).astype(np.float32)
    ok, rb, tb = cv2.solvePnP(obj, px.astype(np.float32), K, None)
    if not ok:
        return None
    Rcb, _ = cv2.Rodrigues(rb)
    tb = tb.ravel()
    C = -Rcb.T @ tb                                   # camera centre in board coordinates
    s_up = -1.0 if C[2] < 0 else 1.0                  # board z axis points away from (s=-1) or towards the camera
    Pb, Pr = [], []
    for row, p3 in zip(rows, tip3d):
        h = p3[2] - table_z                           # height above the table (mm)
        u, v = row["c0"]                              # blob centre: good enough for a first guess
        d = Rcb.T @ np.array([(u - CAM_W / 2) / F0, (v - CAM_H / 2) / F0, 1.0])
        zt = -s_up * h if s_up < 0 else h
        zt = h * np.sign(C[2])
        q = C + (zt - C[2]) / d[2] * d
        Pb.append(q[:2]); Pr.append(p3[:2])
    Pb, Pr = np.array(Pb), np.array(Pr)
    want = -1.0 if C[2] < 0 else 1.0                  # in-plane mirror needed when board z points down
    want = 1.0 if C[2] > 0 else -1.0
    cb, cr = Pb.mean(0), Pr.mean(0)
    U, S_, Vt = np.linalg.svd((Pr - cr).T @ (Pb - cb))
    D = np.diag([1.0, want * np.sign(np.linalg.det(U @ Vt))])
    A = U @ D @ Vt                                    # board XY -> robot XY (rotation, or rotation+mirror)
    t2 = cr - A @ cb
    zsign = 1.0 if C[2] > 0 else -1.0                 # robot z = table_z + zsign * board z
    Rrb = np.array([[A[0, 0], A[0, 1], 0], [A[1, 0], A[1, 1], 0], [0, 0, zsign]])
    if np.linalg.det(Rrb) < 0:
        return None
    trb = np.array([t2[0], t2[1], table_z])
    Rcr = Rcb @ Rrb.T
    tcr = tb - Rcr @ trb
    err = np.linalg.norm((Pb - cb) @ A.T + cr - Pr, axis=1)
    print(f"starting pose from the board: fingertips agree within {np.median(err):.0f} mm (median, rough first guess)")
    return cv2.Rodrigues(Rcr)[0].ravel(), tcr


def solve():
    from scipy.optimize import least_squares
    meta = json.load(open(os.path.join(DATA, "records.json")))
    recs, table_z = meta["records"], meta["table_z"]
    scan = {n: cv2.imread(os.path.join(DATA, f"{n}.png")) for n in SCAN_POSES}
    best = max(scan, key=lambda n: len(detect_markers(scan[n])))
    markers = detect_markers(scan[best])
    mids = sorted(markers)
    print(f"ArUco markers seen in '{best}': {mids}")
    board_raw = board_for_calibration(scan[best])
    # OpenCV board coordinates have z pointing INTO the table; flip y so the board frame is right-handed
    # with z up like the robot's - then a plain rotation + shift places it on the table
    board = [(loc * np.array([1.0, -1.0]), px) for loc, px in board_raw]

    # rough robot-base pixel, only used to tell the fingertip end of the finger blob in the first pass
    prev = os.path.join(T1_DIR, "config", "camera_robot_calib.yaml")
    base_px = np.array([CAM_W * 0.8, CAM_H * 0.4])
    if os.path.exists(prev):
        with open(prev) as fh:
            base_px = np.array(yaml.safe_load(fh).get("base_pixel", base_px), float)

    rows = []
    for r in recs:
        io = cv2.imread(os.path.join(DATA, f"p{r['i']:02d}_open.png"))
        ic = cv2.imread(os.path.join(DATA, f"p{r['i']:02d}_closed.png"))
        blobs = finger_blobs(io, ic)
        if blobs:
            rows.append({"r": r, "blobs": blobs, "img": ic, "ang": K.raw_to_angles(r.get("raw_meas") or r["raw_cmd"])})
    print(f"finger candidates found in {len(rows)}/{len(recs)} poses")

    def pts3d(rows, L, sag, tool=None):
        out = []
        for row in rows:
            a = row["ang"]
            x, y, z, _ = K.fk([a[0], a[1] + sag[0], a[2], a[3] + sag[1]], tool_len=L if tool is None else tool)
            lat = sag[2] if (len(sag) > 2 and LATERAL_SIGMA > 0) else 0.0   # gripper sideways of the arm's plane (mm, + = left); sigma 0 = not fitted
            out.append([x - lat * math.sin(a[0]), y + lat * math.cos(a[0]), z])
        return np.array(out)

    # ---- pass 0: biggest blob, direction away from the rough base pixel
    for row in rows:
        m, c, _ = max(row["blobs"], key=lambda b: b[2])
        row["mask"], row["px"], row["c0"] = m, tip_from_mask(m, c - base_px), c
    Kmat = np.array([[F0, 0, CAM_W / 2], [0, F0, CAM_H / 2], [0, 0, 1]])
    obj, img = pts3d(rows, TIP_LEN0, (0.0, 0.0, 0.0)), np.array([row["px"] for row in rows])
    ok, inl = False, None
    if board:
        init = init_from_board(board_raw, rows, obj, table_z)
        if init is not None:
            rvec, tvec = init
            ok = True
    if not ok:
        ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, Kmat, None, reprojectionError=14.0, iterationsCount=3000,
                                                 flags=cv2.SOLVEPNP_EPNP)
    prev = os.path.join(T1_DIR, "config", "camera_robot_calib.yaml")
    if not ok and os.path.exists(prev):
        with open(prev) as fh:                       # few points: start from the last full calibration instead
            pc = yaml.safe_load(fh)
        rvec, tvec = np.array(pc["rvec_robot_to_cam"], float), np.array(pc["tvec_robot_to_cam_mm"], float)
        print("starting from the last full calibration (too few points for a fresh start)")
    elif not ok:
        raise RuntimeError("could not get an initial camera pose - look at calibration/output/calib_data images")
    rvec, tvec = np.asarray(rvec, float).ravel(), np.asarray(tvec, float).ravel()
    m0 = []
    for i in mids:
        P = backproject(markers[i], rvec, tvec, F0, table_z)
        ctr, top = P.mean(0), (P[0] + P[1]) / 2 - P.mean(0)
        m0 += [ctr[0], ctr[1], math.atan2(top[1], top[0]) - math.pi / 2]
    b0 = []
    for bd in board:                               # initial pose of each board placement: rigid fit on the table
        Pb = backproject(bd[1], rvec, tvec, F0, table_z)[:, :2]
        L = bd[0]
        cl, cp = L.mean(0), Pb.mean(0)
        U_, _, Vt_ = np.linalg.svd((Pb - cp).T @ (L - cl))
        Rb = U_ @ np.diag([1, np.linalg.det(U_ @ Vt_)]) @ Vt_
        tb = cp - Rb @ cl
        b0 += [tb[0], tb[1], math.atan2(Rb[1, 0], Rb[0, 0])]
    NB = len(b0)
    x = np.concatenate([rvec, tvec, [F0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], m0, b0])  # f dL | shoulder wrist sideways | lens | markers | board

    def unpack(x):
        cam = np.array([x[6], x[11], x[12], x[13], x[14]])        # focal + lens centre shift + distortion
        return x[0:3], x[3:6], cam, x[7], x[8:11], x[15:len(x) - NB].reshape(-1, 3)

    def board_robot(x, k):
        bx, by, bth = x[len(x) - NB + 3 * k: len(x) - NB + 3 * k + 3]
        c, s_ = math.cos(bth), math.sin(bth)
        xy = board[k][0] @ np.array([[c, s_], [-s_, c]]) + [bx, by]
        return np.column_stack([xy, np.full(len(xy), table_z)])

    def reselect(rows, x, radius=90):
        """Per pose: measure each finger blob's TIP along the predicted tool axis, keep the blob whose tip
        is nearest to the predicted fingertip (the blob's middle sits 4-6 cm behind the tip, so it is
        not used for this), and drop the pose if even that tip is further than `radius` px away."""
        rv, tv, f, dL, sag, _ = unpack(x)
        tipp = project(pts3d(rows, TIP_LEN0 + dL, sag), rv, tv, f)
        wrist = project(pts3d(rows, 0, sag, tool=40.0), rv, tv, f)
        keep = []
        for row, tp, wp in zip(rows, tipp, wrist):
            cands = [(np.linalg.norm(tip_from_mask(m_, tp - wp) - tp), m_, c_) for m_, c_, _ in row["blobs"]]
            dist, m, cm_ = min(cands, key=lambda c_: c_[0])
            m = merge_fingers(m, cm_, row["blobs"])
            dist = np.linalg.norm(tip_from_mask(m, tp - wp) - tp)
            if dist > radius:
                continue                                     # nothing near the gripper (e.g. a person walked by)
            row["mask"], row["px"] = m, tip_from_mask(m, tp - wp)
            keep.append(row)
        return keep

    def make_resid(rows):
        img = np.array([row["px"] for row in rows])

        def resid(x, split=False):
            rv, tv, f, dL, sag, mk = unpack(x)
            r_tip = (project(pts3d(rows, TIP_LEN0 + dL, sag), rv, tv, f) - img).ravel()
            r_mk = np.concatenate([(project(marker_corners3d(mx, my, th, table_z), rv, tv, f) - markers[i]).ravel()
                                   for i, (mx, my, th) in zip(mids, mk)]) if mids else np.zeros(0)
            for k_ in range(len(board)):
                r_mk = np.concatenate([r_mk, (project(board_robot(x, k_), rv, tv, f) - board[k_][1]).ravel()])
            if split:
                return r_tip, r_mk
            prior = np.array([(f[0] - F0) / FOCAL_PRIOR_SIGMA, dL / 25.0, sag[0] / math.radians(4.0), sag[1] / math.radians(6.0),
                              sag[2] / max(LATERAL_SIGMA, 1e-3), f[1] / LENS_CENTRE_SIGMA, f[2] / LENS_CENTRE_SIGMA,
                              f[3] / DISTORTION_SIGMA, f[4] / DISTORTION_SIGMA])
            # outlier-tolerant (soft-L1) treatment for the MEASUREMENTS only; the priors stay full-strength
            return np.concatenate([robust(r_tip), 3.0 * robust(r_mk), prior])
        return resid

    used = rows
    def fit(x, used, free):
        """Least squares over the parameters in `free` only (others held at their current value)."""
        free = np.asarray(free)
        def f_(xf):
            xx = x.copy(); xx[free] = xf
            return make_resid(used)(xx)
        out = x.copy()
        out[free] = least_squares(f_, x[free]).x
        return out

    pose_only = list(range(0, 6)) + list(range(15, len(x)))          # camera pose + marker/board poses
    for it in range(6):
        used = reselect(rows, x, radius=(200 if it == 0 else 140 if it == 1 else 90))
        # first lock the camera onto the fingertips with the arm model fixed, then free everything
        x = fit(x, used, pose_only if it < 3 else range(len(x)))
        if os.environ.get("CAL_TRACE"):
            rv_, tv_, f_, dL_, sag_, _ = unpack(x)
            pr_ = project(pts3d(rows, TIP_LEN0 + dL_, sag_), rv_, tv_, f_)
            print(f"   round {it}: kept {[r_['r']['i'] for r_ in used]}  focal {f_[0]:.0f}  predicted-to-detected tip px",
                  [int(np.linalg.norm(r_["px"] - p_)) for r_, p_ in zip(rows, pr_)])
    resid = make_resid(used)
    e = np.linalg.norm(resid(x, True)[0].reshape(-1, 2), axis=1)
    lim = max(6.0, 2.5 * np.median(e))                 # points far worse than the typical one: hand in view, cut off, ...
    if (e > lim).any():
        print("dropping outlier poses:", [row["r"]["i"] for row, ee in zip(used, e) if ee > lim])
        used = [row for row, ee in zip(used, e) if ee <= lim]
        resid = make_resid(used)
        x = least_squares(resid, x).x
    rv, tv, f, dL, sag, mk = unpack(x)
    r_tip, r_mk = resid(x, True)
    e_tip = np.linalg.norm(r_tip.reshape(-1, 2), axis=1)
    e_mk = np.linalg.norm(r_mk.reshape(-1, 2), axis=1) if len(r_mk) else np.zeros(1)
    R, _ = cv2.Rodrigues(rv)
    cam_pos = -R.T @ tv
    lens = f
    f = float(lens[0])
    mm_px = (cam_pos[2] - table_z) / f
    base_px = project([0.0, 0.0, table_z], rv, tv, lens)[0]

    print("\n================ CALIBRATION RESULT ================")
    print(f"poses used                    : {len(used)} of {len(recs)}")
    print(f"fingertip reprojection RMS    : {np.sqrt((e_tip ** 2).mean()):.1f} px = {np.sqrt((e_tip ** 2).mean()) * mm_px:.1f} mm (max {e_tip.max() * mm_px:.1f} mm)")
    print(f"ArUco corner reprojection RMS : {np.sqrt((e_mk ** 2).mean()):.2f} px")
    print(f"focal length                  : {f:.0f} px ; lens centre shift ({lens[1]:+.0f},{lens[2]:+.0f}) px ; distortion k1 {lens[3]:+.3f} k2 {lens[4]:+.3f}")
    print(f"camera position (robot frame) : {np.round(cam_pos, 0)} mm")
    print(f"table height (robot z)        : {table_z:.1f} mm")
    print(f"robot base axis in the image  : ({base_px[0]:.0f}, {base_px[1]:.0f}) px")
    print(f"scale on the mat              : {mm_px:.3f} mm/px")
    print(f"fingertip length from wrist   : {TIP_LEN0 + dL:.1f} mm ; joint zero offsets: shoulder {math.degrees(sag[0]):+.1f}, wrist {math.degrees(sag[1]):+.1f} deg")
    print(f"gripper sideways offset       : {sag[2]:+.1f} mm (+ = to the left of the arm's plane; >3 mm = gripper twisted or a finger bent)")
    print("rotation matrix robot -> camera:\n", np.round(R, 4))
    mk_out = {}
    for i, (mx, my, th) in zip(mids, mk):
        mk_out[int(i)] = {"robot_xy_mm": [float(mx), float(my)], "corners_px": markers[i].tolist()}
        print(f"ArUco {i}: centre robot XY = ({mx:.1f}, {my:.1f}) mm")
    if os.environ.get("CAL_DEBUG"):
        P3 = pts3d(used, TIP_LEN0 + dL, sag)
        for row, p3 in zip(used, P3):
            q = backproject(row["px"], rv, tv, lens, p3[2])[0]
            rh = p3[:2] / np.linalg.norm(p3[:2])
            dxy = q[:2] - p3[:2]
            print(f"   {row['r']['i']:2d} {row['r']['plane']:4s} xy=({row['r']['xyz'][0]:3.0f},{row['r']['xyz'][1]:4.0f}) tilt={row['r']['down']:2.0f} "
                  f"radial={dxy @ rh:+6.1f} tangential={dxy @ np.array([-rh[1], rh[0]]):+6.1f} mm")
    drift = camera_drift_px()
    if drift is not None:
        print(f"camera stability check     : markers moved {drift:.1f} px between start and end of the run "
              f"({'OK' if drift <= SETUP.get('max_camera_drift_px', 2.0) else 'SOMETHING MOVED: the camera, or the robot base / mat sliding on the table'})")
    problems = []
    if np.sqrt((e_tip ** 2).mean()) * mm_px >= 3.5:
        problems.append("fingertip fit worse than 3.5 mm")
    if len(used) < (max(5, int(0.6 * (NPOINTS[0] or 8))) if QUICK[0] else 20):
        problems.append("too few usable poses (something moved in the picture?)")
    if drift is not None and drift > SETUP.get("max_camera_drift_px", 2.0):
        problems.append("the marker moved during the run - clamp the robot base and the mat (and check the camera mount)")
    quality = "GOOD" if not problems else "POOR (" + "; ".join(problems) + ") - check calib_tips.jpg and run again"
    print("calibration quality:", quality)
    QUALITY_OK[0] = not problems

    out = {
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "method": "pinhole camera fitted to robot fingertip positions + ArUco markers",
        "image_size": [CAM_W, CAM_H],
        "focal_px": float(f), "cx": float(CAM_W / 2 + lens[1]), "cy": float(CAM_H / 2 + lens[2]),
        "dist_k1": float(lens[3]), "dist_k2": float(lens[4]),
        "rvec_robot_to_cam": [float(v) for v in rv],
        "tvec_robot_to_cam_mm": [float(v) for v in tv],
        "rotation_matrix_robot_to_cam": [[float(v) for v in row] for row in R],
        "camera_position_robot_mm": [float(v) for v in cam_pos],
        "table_z_mm": float(table_z),
        "base_pixel": [float(base_px[0]), float(base_px[1])],
        "mm_per_px_on_table": float(mm_px),
        "fingertip_len_mm": float(TIP_LEN0 + dL),
        "joint_offsets_rad": [float(sag[0]), 0.0, float(sag[1])],   # shoulder, elbow, wrist zero offsets
        "tool_lateral_mm": float(sag[2]),   # gripper sits this far to the LEFT of the arm's plane (twisted gripper / bent finger)
        "scan_pose_raw": SCAN_RAW,
        "xy_correction_mm": [0.0, 0.0],
        "fine_tune_affine": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],   # reset; refined by pick_place.py --tune-calibration
        "fit_rms_mm": float(np.sqrt((e_tip ** 2).mean()) * mm_px),
        "n_poses": len(used),
        "marker_mm": MARKER_MM,
        "markers": mk_out,
    }
    with open(CALIB_YAML, "w") as fh:
        yaml.safe_dump(out, fh, sort_keys=False)
    print("saved", CALIB_YAML)

    # --- debug pictures
    pred = project(pts3d(used, TIP_LEN0 + dL, sag), rv, tv, lens)
    tiles = []
    for row, pp, ee in zip(used, pred, e_tip):
        vis = row["img"].copy()
        vis[row["mask"] > 0] = (0.5 * vis[row["mask"] > 0] + np.array([0, 0, 127])).astype(np.uint8)
        q = row["px"]
        cv2.circle(vis, (int(q[0]), int(q[1])), 8, (0, 255, 255), 2)
        cv2.drawMarker(vis, (int(pp[0]), int(pp[1])), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        x0, y0 = int(np.clip(q[0] - 160, 0, CAM_W - 320)), int(np.clip(q[1] - 120, 0, CAM_H - 240))
        vis = vis[y0:y0 + 240, x0:x0 + 320].copy()
        cv2.putText(vis, f"{row['r']['i']} err {ee * mm_px:.1f}mm", (6, 18), 0, 0.55, (255, 255, 255), 2)
        tiles.append(vis)
    while len(tiles) % 6:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(os.path.join(OUT, "calib_tips.jpg"), np.vstack([np.hstack(tiles[i:i + 6]) for i in range(0, len(tiles), 6)]))

    ov = scan[best].copy()

    def tab(x_, y_):
        p = project([float(x_), float(y_), table_z], rv, tv, lens)[0]
        return int(p[0]), int(p[1])
    cv2.arrowedLine(ov, tab(0, 0), tab(100, 0), (0, 0, 255), 3); cv2.putText(ov, "+X 100mm", tab(100, 0), 0, 0.7, (0, 0, 255), 2)
    cv2.arrowedLine(ov, tab(0, 0), tab(0, 100), (0, 255, 0), 3); cv2.putText(ov, "+Y 100mm", tab(0, 100), 0, 0.7, (0, 255, 0), 2)
    for gx in range(50, 351, 50):
        for gy in range(-150, 151, 50):
            cv2.circle(ov, tab(gx, gy), 3, (255, 255, 0), -1)
            if gy == -150:
                cv2.putText(ov, str(gx), tab(gx, gy), 0, 0.45, (255, 255, 0), 1)
    for i, (mx, my, th) in zip(mids, mk):
        c = project(marker_corners3d(mx, my, th, table_z), rv, tv, lens).astype(np.int32)
        cv2.polylines(ov, [c], True, (255, 0, 255), 2)
        cv2.putText(ov, f"id{i} ({mx:.0f},{my:.0f})", tuple(int(v) for v in c[0]), 0, 0.55, (255, 0, 255), 2)
    cv2.imwrite(os.path.join(OUT, "calib_overlay.jpg"), ov)
    print("debug images: calibration/output/calib_tips.jpg, calib_overlay.jpg")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--solve", action="store_true", help="only re-solve from saved images")
    ap.add_argument("--probe", action="store_true", help="re-measure the table height by touching the mat once")
    ap.add_argument("--board-snapshot", action="store_true", help="only take a picture of the ChArUco board (then remove it)")
    ap.add_argument("--points", type=int, default=0, help="use only this many well-spread poses (e.g. 8, 15) to try things out")
    ap.add_argument("--quick", action="store_true",
                    help="8 poses only (~2 min) to try out changes; saves to config/camera_robot_calib_quick.yaml, not the real calibration")
    a = ap.parse_args()
    if a.board_snapshot:
        board_snapshot()
        sys.exit(0)
    if a.quick and not a.points:
        a.points = 8
    if a.points:
        a.quick = True
        NPOINTS[0] = a.points
    if a.quick:
        QUICK[0] = True
        DATA = os.path.join(T1_DIR, "calibration", "output", "calib_quick")
        CALIB_YAML = os.path.join(T1_DIR, "config", "camera_robot_calib_quick.yaml")
    if not a.solve:
        collect(a.probe)
    solve()
    sys.exit(0 if QUALITY_OK[0] else 2)
