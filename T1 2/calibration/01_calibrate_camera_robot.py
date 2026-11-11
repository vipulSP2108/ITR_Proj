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

Usage (real terminal, inside "T1 2"):
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
from hw_common import T1_DIR, CAM_W, CAM_H, GRIP_OPEN, GRIP_CLOSED
from src.planning import omx_kinematics as K

DATA = os.path.join(T1_DIR, "calibration", "output", "calib_data")
OUT = os.path.join(T1_DIR, "calibration", "output")
CALIB_YAML = os.path.join(T1_DIR, "config", "camera_robot_calib.yaml")
with open(os.path.join(T1_DIR, "config", "calibration_setup.yaml")) as _f:
    SETUP = yaml.safe_load(_f)                      # every number below comes from that file
MARKER_MM = float(SETUP["marker_side_mm"])
ARUCO_DICT = cv2.aruco.DICT_4X4_50
HOME = [0, 2048, 2048, 2048]
SCAN_POSES = {"scan_pos": [1024, 2048, 2048, 2048], "scan_neg": [-1024, 2048, 2048, 2048]}

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


# ------------------------------------------------------------------ collect --
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
        move_joints(arm, [cur[0]] + HOME[1:])           # lift straight up into the L pose first
        gripper_goto(arm, GRIP_OPEN)
        for name, pose in SCAN_POSES.items():           # clear views of the table for ArUco
            move_joints(arm, pose)
            time.sleep(0.5)
            cv2.imwrite(os.path.join(DATA, f"{name}.png"), cam.read_avg(4))
        move_joints(arm, HOME)
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
        move_joints(arm, HOME)
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


# -------------------------------------------------------------------- solve --
F0 = float(SETUP["nominal_focal_px"])
MARKER_LOCAL = MARKER_MM / 2 * np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]], float)   # TL TR BR BL


def project(P, rvec, tvec, f):
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    pc = np.atleast_2d(P) @ R.T + tvec
    return np.stack([f * pc[:, 0] / pc[:, 2] + CAM_W / 2, f * pc[:, 1] / pc[:, 2] + CAM_H / 2], 1)


def backproject(px, rvec, tvec, f, z):
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    cam = -R.T @ tvec
    out = []
    for u, v in np.atleast_2d(px):
        d = R.T @ np.array([(u - CAM_W / 2) / f, (v - CAM_H / 2) / f, 1.0])
        out.append(cam + (z - cam[2]) / d[2] * d)
    return np.array(out)


def marker_corners3d(mx, my, th, z):
    c, s = math.cos(th), math.sin(th)
    xy = MARKER_LOCAL @ np.array([[c, s], [-s, c]]) + [mx, my]
    return np.column_stack([xy, np.full(4, z)])


def solve():
    from scipy.optimize import least_squares
    meta = json.load(open(os.path.join(DATA, "records.json")))
    recs, table_z = meta["records"], meta["table_z"]
    scan = {n: cv2.imread(os.path.join(DATA, f"{n}.png")) for n in SCAN_POSES}
    best = max(scan, key=lambda n: len(detect_markers(scan[n])))
    markers = detect_markers(scan[best])
    mids = sorted(markers)
    print(f"ArUco markers seen in '{best}': {mids}")

    d = (cv2.absdiff(scan["scan_pos"], scan["scan_neg"]).max(axis=2) > 40).astype(np.uint8)
    d = cv2.morphologyEx(d, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    ys, xs = np.nonzero(d)
    base_px = np.array([np.median(xs), np.median(ys)])       # rough: the arm swings about the base

    rows = []
    for r in recs:
        io = cv2.imread(os.path.join(DATA, f"p{r['i']:02d}_open.png"))
        ic = cv2.imread(os.path.join(DATA, f"p{r['i']:02d}_closed.png"))
        blobs = finger_blobs(io, ic)
        if blobs:
            rows.append({"r": r, "blobs": blobs, "img": ic, "ang": K.raw_to_angles(r["raw_cmd"])})
    print(f"finger candidates found in {len(rows)}/{len(recs)} poses")

    def pts3d(rows, L, sag, tool=None):
        out = []
        for row in rows:
            a = row["ang"]
            x, y, z, _ = K.fk([a[0], a[1] + sag[0], a[2], a[3] + sag[1]], tool_len=L if tool is None else tool)
            out.append([x, y, z])
        return np.array(out)

    # ---- pass 0: biggest blob, direction away from the rough base pixel, RANSAC PnP
    for row in rows:
        m, c, _ = max(row["blobs"], key=lambda b: b[2])
        row["mask"], row["px"] = m, tip_from_mask(m, c - base_px)
    Kmat = np.array([[F0, 0, CAM_W / 2], [0, F0, CAM_H / 2], [0, 0, 1]])
    obj, img = pts3d(rows, TIP_LEN0, (0.0, 0.0)), np.array([row["px"] for row in rows])
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, Kmat, None, reprojectionError=14.0, iterationsCount=3000,
                                             flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        raise RuntimeError("could not get an initial camera pose - look at calibration/output/calib_data images")
    rvec, tvec = rvec.ravel(), tvec.ravel()
    m0 = []
    for i in mids:
        P = backproject(markers[i], rvec, tvec, F0, table_z)
        ctr, top = P.mean(0), (P[0] + P[1]) / 2 - P.mean(0)
        m0 += [ctr[0], ctr[1], math.atan2(top[1], top[0]) - math.pi / 2]
    x = np.concatenate([rvec, tvec, [F0, 0.0, 0.0, 0.0], m0])  # ..., f, dL, shoulder offset, wrist offset, markers

    def unpack(x):
        return x[0:3], x[3:6], x[6], x[7], x[8:10], x[10:].reshape(-1, 3)

    def reselect(rows, x):
        """Pick, per pose, the blob nearest to the predicted fingertip and measure its tip along the predicted tool axis."""
        rv, tv, f, dL, sag, _ = unpack(x)
        tipp = project(pts3d(rows, TIP_LEN0 + dL, sag), rv, tv, f)
        wrist = project(pts3d(rows, 0, sag, tool=40.0), rv, tv, f)
        keep = []
        for row, tp, wp in zip(rows, tipp, wrist):
            m, c, _ = min(row["blobs"], key=lambda b: np.linalg.norm(b[1] - tp))
            if np.linalg.norm(c - tp) > 90:
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
            if split:
                return r_tip, r_mk
            prior = np.array([(f - F0) / 30.0, dL / 25.0, sag[0] / math.radians(4.0), sag[1] / math.radians(6.0)])
            return np.concatenate([r_tip, 3.0 * r_mk, prior])
        return resid

    used = rows
    for it in range(5):
        used = reselect(rows, x)
        x = least_squares(make_resid(used), x, loss="soft_l1", f_scale=3.0).x
    resid = make_resid(used)
    e = np.linalg.norm(resid(x, True)[0].reshape(-1, 2), axis=1)
    lim = max(8.0, 3.0 * np.median(e))
    if (e > lim).any():
        print("dropping outlier poses:", [row["r"]["i"] for row, ee in zip(used, e) if ee > lim])
        used = [row for row, ee in zip(used, e) if ee <= lim]
        resid = make_resid(used)
        x = least_squares(resid, x, loss="soft_l1", f_scale=3.0).x
    rv, tv, f, dL, sag, mk = unpack(x)
    r_tip, r_mk = resid(x, True)
    e_tip = np.linalg.norm(r_tip.reshape(-1, 2), axis=1)
    e_mk = np.linalg.norm(r_mk.reshape(-1, 2), axis=1) if len(r_mk) else np.zeros(1)
    R, _ = cv2.Rodrigues(rv)
    cam_pos = -R.T @ tv
    mm_px = (cam_pos[2] - table_z) / f
    base_px = project([0.0, 0.0, table_z], rv, tv, f)[0]

    print("\n================ CALIBRATION RESULT ================")
    print(f"poses used                    : {len(used)} of {len(recs)}")
    print(f"fingertip reprojection RMS    : {np.sqrt((e_tip ** 2).mean()):.1f} px = {np.sqrt((e_tip ** 2).mean()) * mm_px:.1f} mm (max {e_tip.max() * mm_px:.1f} mm)")
    print(f"ArUco corner reprojection RMS : {np.sqrt((e_mk ** 2).mean()):.2f} px")
    print(f"focal length                  : {f:.0f} px")
    print(f"camera position (robot frame) : {np.round(cam_pos, 0)} mm")
    print(f"table height (robot z)        : {table_z:.1f} mm")
    print(f"robot base axis in the image  : ({base_px[0]:.0f}, {base_px[1]:.0f}) px")
    print(f"scale on the mat              : {mm_px:.3f} mm/px")
    print(f"fingertip length from wrist   : {TIP_LEN0 + dL:.1f} mm ; joint zero offsets: shoulder {math.degrees(sag[0]):+.1f}, wrist {math.degrees(sag[1]):+.1f} deg")
    print("rotation matrix robot -> camera:\n", np.round(R, 4))
    mk_out = {}
    for i, (mx, my, th) in zip(mids, mk):
        mk_out[int(i)] = {"robot_xy_mm": [float(mx), float(my)], "corners_px": markers[i].tolist()}
        print(f"ArUco {i}: centre robot XY = ({mx:.1f}, {my:.1f}) mm")
    if os.environ.get("CAL_DEBUG"):
        P3 = pts3d(used, TIP_LEN0 + dL, sag)
        for row, p3 in zip(used, P3):
            q = backproject(row["px"], rv, tv, f, p3[2])[0]
            rh = p3[:2] / np.linalg.norm(p3[:2])
            dxy = q[:2] - p3[:2]
            print(f"   {row['r']['i']:2d} {row['r']['plane']:4s} xy=({row['r']['xyz'][0]:3.0f},{row['r']['xyz'][1]:4.0f}) tilt={row['r']['down']:2.0f} "
                  f"radial={dxy @ rh:+6.1f} tangential={dxy @ np.array([-rh[1], rh[0]]):+6.1f} mm")
    quality = "GOOD" if np.sqrt((e_tip ** 2).mean()) * mm_px < 3.5 and len(used) >= 20 else "POOR - check calib_tips.jpg and run again"
    print("calibration quality:", quality)

    out = {
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "method": "pinhole camera fitted to robot fingertip positions + ArUco markers",
        "image_size": [CAM_W, CAM_H],
        "focal_px": float(f), "cx": CAM_W / 2, "cy": CAM_H / 2,
        "rvec_robot_to_cam": [float(v) for v in rv],
        "tvec_robot_to_cam_mm": [float(v) for v in tv],
        "rotation_matrix_robot_to_cam": [[float(v) for v in row] for row in R],
        "camera_position_robot_mm": [float(v) for v in cam_pos],
        "table_z_mm": float(table_z),
        "base_pixel": [float(base_px[0]), float(base_px[1])],
        "mm_per_px_on_table": float(mm_px),
        "fingertip_len_mm": float(TIP_LEN0 + dL),
        "joint_offsets_rad": [float(sag[0]), 0.0, float(sag[1])],   # shoulder, elbow, wrist zero offsets
        "scan_pose_raw": SCAN_POSES[best],
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
    pred = project(pts3d(used, TIP_LEN0 + dL, sag), rv, tv, f)
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
        p = project([float(x_), float(y_), table_z], rv, tv, f)[0]
        return int(p[0]), int(p[1])
    cv2.arrowedLine(ov, tab(0, 0), tab(100, 0), (0, 0, 255), 3); cv2.putText(ov, "+X 100mm", tab(100, 0), 0, 0.7, (0, 0, 255), 2)
    cv2.arrowedLine(ov, tab(0, 0), tab(0, 100), (0, 255, 0), 3); cv2.putText(ov, "+Y 100mm", tab(0, 100), 0, 0.7, (0, 255, 0), 2)
    for gx in range(50, 351, 50):
        for gy in range(-150, 151, 50):
            cv2.circle(ov, tab(gx, gy), 3, (255, 255, 0), -1)
            if gy == -150:
                cv2.putText(ov, str(gx), tab(gx, gy), 0, 0.45, (255, 255, 0), 1)
    for i, (mx, my, th) in zip(mids, mk):
        c = project(marker_corners3d(mx, my, th, table_z), rv, tv, f).astype(np.int32)
        cv2.polylines(ov, [c], True, (255, 0, 255), 2)
        cv2.putText(ov, f"id{i} ({mx:.0f},{my:.0f})", tuple(int(v) for v in c[0]), 0, 0.55, (255, 0, 255), 2)
    cv2.imwrite(os.path.join(OUT, "calib_overlay.jpg"), ov)
    print("debug images: calibration/output/calib_tips.jpg, calib_overlay.jpg")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--solve", action="store_true", help="only re-solve from saved images")
    ap.add_argument("--probe", action="store_true", help="re-measure the table height by touching the mat once")
    a = ap.parse_args()
    if not a.solve:
        collect(a.probe)
    solve()
