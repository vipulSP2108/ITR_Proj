#!/usr/bin/env python3
"""
Integrated hardware pipeline that uses ffmpeg for camera capture
(works around macOS camera permission issues with OpenCV).

Run from terminal:
    cd "T1 2" && python hw_run.py
"""

import os, sys, cv2, time, math, json, subprocess, tempfile
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from src.control.dynamixel_wrapper import OpenManipulatorController
from src.control.gripper_guard import GripperGuard, GraspState
from src.planning.kinematics import OpenManipulatorKinematics
from src.planning.workspace_analyzer import WorkspaceAnalyzer, ReachabilityStatus
from src.vision.color_segmentation import ColorSegmenter
from src.vision.shape_detection import ShapeDetector, DetectedObject

# ─── Constants ────────────────────────────────────────────────────────
HOME_POSE = [0, 2048, 2048, 2048, 3130]
SIDE_PARK = [-1024, 2400, 1500, 2500, 3130]  # Swing away from board, folded compact
GRIPPER_OPEN = 1700
GRIPPER_CLOSE = 2200

OUTPUT_DIR = os.path.join(SCRIPT_DIR, "hw_test_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

import yaml
with open(os.path.join(SCRIPT_DIR, 'config', 'workspace_dimensions.yaml')) as f:
    WS = yaml.safe_load(f)

CAM_HEIGHT = WS['camera']['height_above_board_mm']
FOCAL_PX = WS['camera']['focal_length_px']
SCALE = CAM_HEIGHT / FOCAL_PX
BASE_U = WS['robot_mounting']['base_center_pixel_estimate']['u']
BASE_V = WS['robot_mounting']['base_center_pixel_estimate']['v']
Z_HOVER = WS['manipulation_heights']['z_hover_mm']
Z_GRASP = WS['manipulation_heights']['z_grasp_default_mm']
Z_TRANSIT = WS['manipulation_heights']['z_transit_mm']
Z_SOCK_HOVER = WS['manipulation_heights']['z_sock_hover_mm']
Z_INSERT = WS['manipulation_heights']['z_insert_default_mm']


# ─── Camera via ffmpeg ────────────────────────────────────────────────

def capture_frame(device="Intel", width=640, height=480, warmup_frames=5):
    """Capture a frame using ffmpeg (bypasses macOS OpenCV permission issue)."""
    tmp = os.path.join(OUTPUT_DIR, "_capture.jpg")
    cmd = [
        "ffmpeg", "-y", "-f", "avfoundation",
        "-framerate", "30", "-video_size", f"{width}x{height}",
        "-i", device,
        "-frames:v", str(warmup_frames),
        "-update", "1", tmp,
    ]
    result = subprocess.run(cmd, capture_output=True, timeout=15)
    if result.returncode == 0 and os.path.exists(tmp):
        frame = cv2.imread(tmp)
        return frame
    return None


# ─── Pixel ↔ Robot ────────────────────────────────────────────────────

def pixel_to_robot(u, v):
    x = (BASE_U - u) * SCALE
    y = (BASE_V - v) * SCALE
    return x, y

def robot_to_pixel(x, y):
    u = BASE_U - x / SCALE
    v = BASE_V - y / SCALE
    return u, v


# ─── Vision ──────────────────────────────────────────────────────────

def detect_pegs(frame):
    """Detect colored pegs using color segmentation + shape classification."""
    seg = ColorSegmenter(os.path.join(SCRIPT_DIR, 'config', 'colors.yaml'))
    det = ShapeDetector(min_area=800, max_area=50000)

    color_shape_map = {"green": "circle", "blue": "square", "red": "hexagon", "yellow": "triangle"}
    masks = seg.segment_all(frame)
    pegs = []

    for color_name, mask in masks.items():
        objects = det.detect_objects_in_mask(mask, color_name)
        for obj in objects:
            u, v = obj.centroid_2d
            x, y = pixel_to_robot(u, v)
            r = math.sqrt(x**2 + y**2)
            # Filter: must be within reachable workspace
            if r < 80 or r > 380:
                continue
            expected_shape = color_shape_map.get(color_name, obj.shape)
            pegs.append({
                "shape": expected_shape, "color": color_name,
                "pixel": (u, v), "robot": (x, y),
                "yaw": obj.yaw_deg, "symmetry": obj.symmetry_order,
                "confidence": obj.confidence, "area": cv2.contourArea(obj.contour),
                "contour": obj.contour,
            })
    return pegs


def detect_board_sockets(frame):
    """Detect socket cutouts on the wooden board.

    Strategy: find the board via wood + orange border colors combined,
    then find holes within it. Also try direct teal-hole detection as fallback.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    kernel_sm = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    # Wood color (beige/tan board surface)
    wood_mask = cv2.inRange(hsv, np.array([5, 20, 70]), np.array([40, 220, 255]))
    # Orange/red socket outlines on the board
    orange_mask = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([15, 255, 255]))
    orange_mask2 = cv2.inRange(hsv, np.array([165, 80, 80]), np.array([179, 255, 255]))
    # Combine: board = wood + orange borders
    board_mask = cv2.bitwise_or(wood_mask, orange_mask)
    board_mask = cv2.bitwise_or(board_mask, orange_mask2)
    board_mask = cv2.morphologyEx(board_mask, cv2.MORPH_CLOSE, kernel, iterations=4)
    board_mask = cv2.morphologyEx(board_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    cv2.imwrite(os.path.join(OUTPUT_DIR, "dbg_board_mask.jpg"), board_mask)

    # Find the board contour (largest combined region)
    contours, _ = cv2.findContours(board_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("    [board] No wood/orange regions found")
        return [], None

    # Filter to substantial regions only
    contours = [c for c in contours if cv2.contourArea(c) > 5000]
    if not contours:
        print("    [board] No large enough board region")
        return [], None

    board_cnt = max(contours, key=cv2.contourArea)
    bx, by, bw, bh = cv2.boundingRect(board_cnt)
    board_roi = (bx, by, bw, bh)
    board_area = cv2.contourArea(board_cnt)
    print(f"    [board] Found at ({bx},{by}) {bw}x{bh}, area={board_area}")

    if bw < 40 or bh < 40:
        return [], board_roi

    # Within board ROI, find holes (non-board areas = socket cutouts)
    board_region = board_mask[by:by+bh, bx:bx+bw]
    holes = cv2.bitwise_not(board_region)

    # Remove border artifacts more aggressively
    border = max(10, bw // 20)
    holes[:border, :] = 0
    holes[-border:, :] = 0
    holes[:, :border] = 0
    holes[:, -border:] = 0

    holes = cv2.morphologyEx(holes, cv2.MORPH_OPEN, kernel_sm, iterations=2)
    holes = cv2.morphologyEx(holes, cv2.MORPH_CLOSE, kernel, iterations=2)

    cv2.imwrite(os.path.join(OUTPUT_DIR, "dbg_holes_mask.jpg"), holes)

    cutout_contours, _ = cv2.findContours(holes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"    [board] Found {len(cutout_contours)} raw cutout contours")

    det = ShapeDetector(min_area=400, max_area=30000)
    sockets = []

    for cnt in cutout_contours:
        area = cv2.contourArea(cnt)
        if area < 400 or area > 30000:
            continue

        # Use enclosing circle ratio for shape classification
        (cx_e, cy_e), radius = cv2.minEnclosingCircle(cnt)
        circle_area = math.pi * radius * radius
        ratio = area / circle_area if circle_area > 0 else 0

        # Approximate polygon
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        verts = len(approx)

        # Classify shape from ratio + vertices
        if ratio > 0.88:
            shape = "circle"
        elif verts == 3 or ratio < 0.55:
            shape = "triangle"
        elif verts == 4 or (0.55 <= ratio <= 0.78):
            shape = "square"
        elif verts == 6 or (0.78 < ratio <= 0.88):
            shape = "hexagon"
        else:
            shape = "circle" if ratio > 0.85 else "square"

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        local_cx = M["m10"] / M["m00"]
        local_cy = M["m01"] / M["m00"]

        # Convert to global pixel coords
        gx = local_cx + bx
        gy = local_cy + by
        rx, ry = pixel_to_robot(gx, gy)

        yaw = 0.0
        try:
            yaw = det._estimate_orientation(cnt, shape, approx, local_cx, local_cy)
        except Exception:
            pass

        conf = min(1.0, ratio + 0.1) if shape == "circle" else 0.7
        print(f"    [socket] {shape:10s} area={area:.0f} ratio={ratio:.3f} verts={verts} px=({gx:.0f},{gy:.0f})")

        sockets.append({
            "shape": shape, "pixel": (gx, gy), "robot": (rx, ry),
            "yaw": yaw, "area": area, "confidence": conf,
            "contour": cnt + np.array([[bx, by]]),  # offset to global coords
        })

    return sockets, board_roi


# ─── Arm Control ─────────────────────────────────────────────────────

def smooth_move(arm, targets, duration=2.5, steps=10):
    """Interpolate joint positions smoothly."""
    joint_ids = [11, 12, 13, 14, 15]
    current = []
    for jid in joint_ids:
        pos = arm.read_position(jid)
        if pos is None:
            print(f"    Cannot read joint {jid}")
            return False
        current.append(pos)

    dt = duration / steps
    for step in range(1, steps + 1):
        alpha = step / steps
        for i, jid in enumerate(joint_ids):
            if jid == 11:
                diff = targets[i] - current[i]
                goal = current[i] + int(round(diff * alpha))
            else:
                goal = int(round(current[i] + (targets[i] - current[i]) * alpha))
            arm.write_position(jid, goal)
        time.sleep(dt)

    # Wait for settling
    time.sleep(0.3)
    return True


def move_to_xyz(arm, kin, wa, x, y, z, gripper=GRIPPER_OPEN, duration=2.0):
    """Move arm to Cartesian position using IK."""
    status, info = wa.classify_target(x, y, z)

    if status == ReachabilityStatus.OUT_OF_BOUNDS:
        print(f"    OUT OF BOUNDS: ({x:.1f},{y:.1f},{z:.1f}), r={info['radius']:.0f}mm")
        return False

    q = info.get('ik_solution')
    if q is None:
        print(f"    IK FAILED for ({x:.1f},{y:.1f},{z:.1f})")
        return False

    raw = kin.radians_to_raw(q)
    targets = list(raw) + [gripper]
    print(f"    IK: ({x:.1f},{y:.1f},{z:.1f}) pitch={info.get('pitch_deg', -90):.0f}° -> raw={targets}")
    return smooth_move(arm, targets, duration)


# ─── Annotation ──────────────────────────────────────────────────────

def annotate(frame, pegs, sockets, board_roi, step_text=""):
    vis = frame.copy()

    # Base marker
    bu, bv = int(BASE_U), int(BASE_V)
    cv2.circle(vis, (bu, bv), 6, (0, 0, 255), -1)
    cv2.putText(vis, "BASE", (bu+8, bv), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    # Board
    if board_roi:
        bx, by, bw, bh = board_roi
        cv2.rectangle(vis, (bx, by), (bx+bw, by+bh), (0, 200, 200), 2)

    color_bgr = {"green": (0,255,0), "blue": (255,0,0), "red": (0,0,255), "yellow": (0,255,255)}

    for p in pegs:
        u, v = int(p["pixel"][0]), int(p["pixel"][1])
        clr = color_bgr.get(p["color"], (255,255,255))
        cv2.circle(vis, (u, v), 10, clr, 2)
        cv2.putText(vis, f'{p["shape"]}', (u+12, v-5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, clr, 1)
        rx, ry = p["robot"]
        cv2.putText(vis, f'({rx:.0f},{ry:.0f})', (u+12, v+12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, clr, 1)

    for s in sockets:
        u, v = int(s["pixel"][0]), int(s["pixel"][1])
        cv2.drawContours(vis, [s["contour"]], -1, (180,180,180), 2)
        cv2.putText(vis, f'[{s["shape"]}]', (u+8, v), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)

    if step_text:
        cv2.rectangle(vis, (0, 0), (640, 28), (0, 0, 0), -1)
        cv2.putText(vis, step_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    return vis


# ─── Main Pipeline ───────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  HARDWARE PIPELINE — Vision-Guided Peg Sorting")
    print("=" * 60)

    kin = OpenManipulatorKinematics()
    wa = WorkspaceAnalyzer(kin)

    # ─── 1. Connect Arm ──────────────────────────────────
    print("\n[1] Connecting to arm...")
    arm = OpenManipulatorController()
    if not arm.connect():
        print("FAILED: No arm connection")
        return
    arm.initialize_all_joints(profile_vel=50, profile_acc=15)
    gripper = GripperGuard(arm)

    # Read current positions
    print("  Current positions:")
    for jid in [11, 12, 13, 14, 15]:
        pos = arm.read_position(jid)
        print(f"    Joint {jid}: {pos}")

    # ─── 2. Go HOME ─────────────────────────────────────
    print("\n[2] Moving to HOME...")
    smooth_move(arm, HOME_POSE, duration=4.0, steps=15)
    time.sleep(1.0)

    # Verify home
    print("  After HOME:")
    for jid in [11, 12, 13, 14, 15]:
        pos = arm.read_position(jid)
        print(f"    Joint {jid}: {pos}")

    # ─── 3. Park arm to side for camera ──────────────────
    print("\n[3] Parking arm to side...")
    # Move joint 11 (base) to +90° (3072 counts) to clear camera view
    # Keep other joints at home
    side_targets = [-1024, 2400, 1500, 2500, GRIPPER_OPEN]
    smooth_move(arm, side_targets, duration=3.0, steps=12)
    time.sleep(1.0)

    # ─── 4. Capture and Scan ─────────────────────────────
    print("\n[4] Capturing camera frame...")
    frame = capture_frame(warmup_frames=12)
    if frame is None:
        print("  FAILED: No camera frame!")
        arm.disconnect()
        return
    cv2.imwrite(os.path.join(OUTPUT_DIR, "scan_raw.jpg"), frame)
    print(f"  Frame: {frame.shape}, brightness={frame.mean():.1f}")

    # Enhance for better detection
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    l = clahe.apply(l)
    enhanced = cv2.merge([l, a, b])
    enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

    print("\n[5] Detecting pegs...")
    pegs = detect_pegs(enhanced)
    print(f"  Found {len(pegs)} pegs:")
    for p in pegs:
        rx, ry = p["robot"]
        r = math.sqrt(rx**2 + ry**2)
        print(f"    {p['shape']:10s} ({p['color']:6s}) px=({p['pixel'][0]:.0f},{p['pixel'][1]:.0f}) robot=({rx:.1f},{ry:.1f}) r={r:.0f}mm conf={p['confidence']:.2f} area={p['area']:.0f}")

    print("\n[6] Detecting board & sockets...")
    sockets, board_roi = detect_board_sockets(enhanced)
    print(f"  Board ROI: {board_roi}")
    print(f"  Found {len(sockets)} sockets:")
    for s in sockets:
        rx, ry = s["robot"]
        print(f"    [{s['shape']:10s}] px=({s['pixel'][0]:.0f},{s['pixel'][1]:.0f}) robot=({rx:.1f},{ry:.1f}) conf={s['confidence']:.2f} area={s['area']:.0f}")

    # Save annotated frame
    ann = annotate(enhanced, pegs, sockets, board_roi, "Workspace Scan")
    cv2.imwrite(os.path.join(OUTPUT_DIR, "scan_annotated.jpg"), ann)

    # ─── 7. Match pegs to sockets ────────────────────────
    assignments = {}
    socket_by_shape = {}
    for s in sockets:
        socket_by_shape.setdefault(s["shape"], []).append(s)

    for peg in pegs:
        candidates = socket_by_shape.get(peg["shape"], [])
        if candidates:
            sock = candidates.pop(0)
            assignments[peg["shape"]] = (peg, sock)

    print(f"\n[7] Assignments: {list(assignments.keys())}")
    for shape, (p, s) in assignments.items():
        print(f"    {shape}: peg({p['robot'][0]:.1f},{p['robot'][1]:.1f}) -> socket({s['robot'][0]:.1f},{s['robot'][1]:.1f})")

    if not assignments:
        print("\n  *** NO PEG-SOCKET ASSIGNMENTS ***")
        print("  Make sure colored pegs are placed on the mat within camera view.")
        print("  Detected pegs:", [p["shape"] for p in pegs])
        print("  Detected sockets:", [s["shape"] for s in sockets])

        # Save diagnostic info
        summary = {
            "pegs": [{"shape": p["shape"], "color": p["color"], "pixel": list(p["pixel"]), "robot": list(p["robot"])} for p in pegs],
            "sockets": [{"shape": s["shape"], "pixel": list(s["pixel"]), "robot": list(s["robot"])} for s in sockets],
            "board_roi": list(board_roi) if board_roi else None,
        }
        with open(os.path.join(OUTPUT_DIR, "scan_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

        print("\n  Returning to HOME...")
        smooth_move(arm, HOME_POSE, duration=3.0)
        arm.disconnect()
        return

    # ─── 8. Pick and Place each peg ──────────────────────
    sort_order = ["circle", "square", "hexagon", "triangle"]
    sorted_shapes = [s for s in sort_order if s in assignments]

    for shape_name in sorted_shapes:
        peg, socket = assignments[shape_name]
        px, py = peg["robot"]
        sx, sy = socket["robot"]

        print(f"\n{'='*50}")
        print(f"  PICKING: {shape_name} ({peg['color']})")
        print(f"  Pick: ({px:.1f}, {py:.1f})  Place: ({sx:.1f}, {sy:.1f})")
        print(f"{'='*50}")

        # a) Move to pre-pick hover
        print("\n  [a] Pre-pick hover...")
        ok = move_to_xyz(arm, kin, wa, px, py, Z_HOVER, gripper=GRIPPER_OPEN, duration=2.5)
        if not ok:
            print("  SKIP: Can't reach pre-pick hover")
            continue

        # Capture and verify peg position
        print("  [a+] Vision check...")
        check_frame = capture_frame(warmup_frames=3)
        if check_frame is not None:
            cv2.imwrite(os.path.join(OUTPUT_DIR, f"check_{shape_name}_prepick.jpg"), check_frame)

        # b) Descend to grasp
        print("  [b] Descending to grasp...")
        ok = move_to_xyz(arm, kin, wa, px, py, Z_GRASP, gripper=GRIPPER_OPEN, duration=2.0)
        if not ok:
            print("  SKIP: Can't reach grasp height")
            continue

        # c) Close gripper
        print("  [c] Closing gripper...")
        arm.write_position(15, GRIPPER_CLOSE)
        time.sleep(1.0)

        # Check grasp
        state, info = gripper.evaluate_grasp()
        print(f"  Grasp: {state.value} (pos={info.get('position','?')}, cur={info.get('current_ma',0):.1f}mA)")

        if state == GraspState.EMPTY_AIR:
            print("  MISSED! Trying tighter close...")
            arm.write_position(15, GRIPPER_CLOSE + 100)
            time.sleep(0.5)
            state, info = gripper.evaluate_grasp()
            print(f"  Retry: {state.value}")

        # d) Lift
        print("  [d] Lifting...")
        ok = move_to_xyz(arm, kin, wa, px, py, Z_TRANSIT, gripper=GRIPPER_CLOSE, duration=1.5)

        # e) Transit to socket
        print("  [e] Transit to socket...")
        ok = move_to_xyz(arm, kin, wa, sx, sy, Z_TRANSIT, gripper=GRIPPER_CLOSE, duration=2.5)
        if not ok:
            print("  SKIP: Can't reach socket transit")
            # Open gripper and retreat
            arm.write_position(15, GRIPPER_OPEN)
            time.sleep(0.5)
            smooth_move(arm, HOME_POSE, duration=3.0)
            continue

        # f) Socket hover
        print("  [f] Socket hover...")
        ok = move_to_xyz(arm, kin, wa, sx, sy, Z_SOCK_HOVER, gripper=GRIPPER_CLOSE, duration=1.5)

        # g) Insert
        print("  [g] Inserting...")
        ok = move_to_xyz(arm, kin, wa, sx, sy, Z_INSERT, gripper=GRIPPER_CLOSE, duration=1.5)

        # h) Release
        print("  [h] Releasing...")
        arm.write_position(15, GRIPPER_OPEN)
        time.sleep(0.8)

        # i) Retreat
        print("  [i] Retreating up...")
        ok = move_to_xyz(arm, kin, wa, sx, sy, Z_HOVER, gripper=GRIPPER_OPEN, duration=1.5)

        # Capture after placement
        print("  [i+] Post-place check...")
        check_frame = capture_frame(warmup_frames=3)
        if check_frame is not None:
            cv2.imwrite(os.path.join(OUTPUT_DIR, f"check_{shape_name}_postplace.jpg"), check_frame)

        print(f"  DONE: {shape_name}")

    # ─── 9. Final verification ───────────────────────────
    print("\n\n[9] Final verification...")
    smooth_move(arm, side_targets, duration=2.5)
    time.sleep(1.0)
    final_frame = capture_frame(warmup_frames=5)
    if final_frame is not None:
        cv2.imwrite(os.path.join(OUTPUT_DIR, "final_verification.jpg"), final_frame)
        final_pegs = detect_pegs(final_frame)
        print(f"  Remaining pegs: {[p['shape'] for p in final_pegs] if final_pegs else 'NONE'}")

    # ─── 10. Return home ─────────────────────────────────
    print("\n[10] Returning home...")
    smooth_move(arm, HOME_POSE, duration=3.0)
    arm.disconnect()
    print("\nDone!")


if __name__ == "__main__":
    main()
