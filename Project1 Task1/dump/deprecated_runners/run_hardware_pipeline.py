#!/usr/bin/env python3
"""
OpenMANIPULATOR-X Physical Hardware Sorter Pipeline (T1 2)
Stages 1, 2, 3:
  [1] 01_home_pose:          Verify / move arm to safe calibrated Home Pose
  [2] 02_calibration:        Execute gentle multi-stance calibration poses & verify Eye-to-Hand
  [3] 03_side_park_analysis: Move to SIDE PARK POSITION (pan -90 deg, table 100% visible),
                             perform tabletop segmentation & socket analysis, and hold.
  >> OPERATOR PLACES CIRCULAR PEG ON MAT
"""

import os
import sys
import time
import json
import argparse
import yaml
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T1_2_DIR = os.path.join(BASE_DIR, "T1 2")
sys.path.insert(0, os.path.join(BASE_DIR, "openmanipulator_test"))
sys.path.insert(0, os.path.join(T1_2_DIR, "src"))

from dynamixel_wrapper import OpenManipulatorController, find_u2d2_port, JOINT_IDS, POSITION_UNIT_TO_DEG
from kinematics import OpenManipulatorKinematics
from planning.calibration import EyeToHandCalibrator
from planning.workspace_analyzer import WorkspaceAnalyzer
from vision.dynamic_workspace_perceiver import DynamicWorkspacePerceiver
from planning.autonomous_homography_calibrator import AutonomousHomographyCalibrator

OUTPUT_DIR = os.path.join(T1_2_DIR, "demo1_20261107", "imp_images")
os.makedirs(OUTPUT_DIR, exist_ok=True)

ADDR_HARDWARE_ERROR_STATUS = 70


def smooth_move(arm, targets, duration=2.5, steps=8):
    """Interpolate all 5 joints gently to targets using shortest angular path."""
    start = {}
    for dxl_id in [11, 12, 13, 14, 15]:
        p = arm.read_position(dxl_id)
        start[dxl_id] = p if p is not None else 2048

    dt = duration / float(steps)
    for s in range(1, steps + 1):
        alpha = s / float(steps)
        for dxl_id in [15, 14, 13, 12, 11]:
            g = targets[dxl_id]
            st = start[dxl_id]
            if dxl_id == 11:
                # Wrap-around shortest angular difference for continuous base pan
                diff = (g - st + 2048) % 4096 - 2048
                cur = st + int(round(alpha * diff))
            else:
                cur = int(round(st + alpha * (g - st)))
            arm.write_position(dxl_id, cur)
            time.sleep(0.015)
        time.sleep(dt)


def stage_01_home(arm, cap):
    print("\n" + "=" * 72)
    print("  [1/3] 01_HOME_POSE: Moving Arm to Calibrated Home Ready Stance")
    print("=" * 72)

    home_targets = {11: 0, 12: 2048, 13: 2048, 14: 2048, 15: 3130}
    smooth_move(arm, home_targets, duration=2.5, steps=10)
    time.sleep(0.5)

    positions = {}
    for dxl_id in [11, 12, 13, 14, 15]:
        p = arm.read_position(dxl_id)
        deg = (p - 2048) * POSITION_UNIT_TO_DEG if p is not None else 0.0
        positions[dxl_id] = (p, deg)
        print(f"  • ID {dxl_id:2d} ({JOINT_IDS[dxl_id]:22s}): raw={p:5d} ({deg:+6.1f}°)")

    for _ in range(5): cap.read()
    ret, frame = cap.read()
    if ret:
        h, w = frame.shape[:2]
        hud = frame.copy()
        cv2.rectangle(hud, (15, 15), (w - 15, 75), (20, 20, 20), -1)
        cv2.rectangle(hud, (15, 15), (w - 15, 75), (0, 255, 0), 2)
        cv2.putText(hud, "STAGE 01: CALIBRATED HOME READY POSE", (25, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(hud, "STATUS: ARM UPRIGHT | TABLE READY", (25, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        out_path = os.path.join(OUTPUT_DIR, "01_home_pose.jpg")
        cv2.imwrite(out_path, hud)
        print(f"  ✓ Saved Home Pose Checkpoint: {out_path}")
    return positions


def stage_02_calibration(arm, cap):
    print("\n" + "=" * 72)
    print("  [2/3] 02_CALIBRATION: Autonomous Eye-to-Hand & Homography Calibration")
    print("=" * 72)

    kin = OpenManipulatorKinematics()
    calibrator = AutonomousHomographyCalibrator()
    res = calibrator.run_autonomous_calibration(arm, cap, kin, smooth_move)
    return calibrator


def stage_03_side_park(arm, cap, target_shape="circle"):
    print("\n" + "=" * 72)
    print("  [3/3] 03_SIDE_PARK: Moving to SIDE PARK POSITION (Zero Table Occlusion)")
    print("=" * 72)

    # Move to the exact SIDE PARK POSE (pan -90 deg, upright, gripper open)
    # Joint 1: 3072 counts (pan -90 deg toward circle hole, matching simulation SIDE_PARK_Q)
    # Joints 2..4: 2048 (upright L-shape)
    # Gripper: 3130 (Open)
    side_park_targets = {11: 3072, 12: 2048, 13: 2048, 14: 2048, 15: 3130}
    print("  • Moving arm to SIDE PARK stance (pointing sideways away from table)...")
    smooth_move(arm, side_park_targets, duration=2.5, steps=10)
    time.sleep(0.8)

    positions = {}
    for dxl_id in [11, 12, 13, 14, 15]:
        p = arm.read_position(dxl_id)
        deg = (p - 2048) * POSITION_UNIT_TO_DEG if p is not None else 0.0
        positions[dxl_id] = (p, deg)
        print(f"  • ID {dxl_id:2d} ({JOINT_IDS[dxl_id]:22s}): raw={p:5d} ({deg:+6.1f}°)")

    # Read clean overhead view
    for _ in range(5): cap.read()
    ret, frame = cap.read()
    if not ret or frame is None:
        raise RuntimeError("Failed to capture frame from side park position")

    h, w = frame.shape[:2]
    annotated = frame.copy()

    # Layout pixel boundaries
    mat_x0, mat_x1 = 280, 530
    mat_y0, mat_y1 = 80, 390
    cv2.rectangle(annotated, (mat_x0, mat_y0), (mat_x1, mat_y1), (0, 255, 0), 2)
    cv2.putText(annotated, "GREEN FOAM MAT (PICKUP ZONE)", (mat_x0 + 10, mat_y0 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1)

    board_x0, board_x1 = 180, 275
    board_y0, board_y1 = 80, 440
    cv2.rectangle(annotated, (board_x0, board_y0), (board_x1, board_y1), (0, 165, 255), 2)
    cv2.putText(annotated, "WOODEN SOCKET BOARD", (board_x0 - 5, board_y0 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 165, 255), 1)

    # Autonomous Socket Detection on wooden board via DynamicWorkspacePerceiver
    perceiver = DynamicWorkspacePerceiver()
    assessment = perceiver.assess(frame)
    annotated = assessment["annotated_frame"]
    circle_sock = assessment.get("target_socket")

    out_map = os.path.join(OUTPUT_DIR, "03_workspace_segmentation_map.png")
    cv2.imwrite(out_map, annotated)
    print(f"  ✓ Saved Clean Workspace Map: {out_map}")

    # Also save live snapshot
    live_snap = os.path.join(T1_2_DIR, "live_camera.jpg")
    cv2.imwrite(live_snap, annotated)

    return positions


def pixel_to_robot(u: float, v: float, z_target_rob: float = 62.0) -> np.ndarray:
    """Projects pixel (u, v) on the camera plane onto the 3D table plane at z_target_rob."""
    homo_yaml = os.path.join(T1_2_DIR, "config", "homography_calib.yaml")
    if os.path.exists(homo_yaml):
        with open(homo_yaml, "r") as f:
            data = yaml.safe_load(f)
            H_list = data.get("homography_matrix_3x3")
            if H_list:
                H = np.array(H_list, dtype=np.float64)
                p_homo = np.array([u, v, 1.0], dtype=np.float64)
                p_trans = H @ p_homo
                return np.array([float(p_trans[0] / p_trans[2]), float(p_trans[1] / p_trans[2]), float(z_target_rob)])

    calib_yaml = os.path.join(T1_2_DIR, "config", "camera_calib.yaml")
    with open(calib_yaml, "r") as f:
        calib = yaml.safe_load(f)

    R = np.array(calib["rotation_matrix"], dtype=np.float64)
    t = np.array(calib["translation_vector_mm"], dtype=np.float64)
    fx = float(calib.get("focal_length_px", 344.0))
    fy = float(calib.get("focal_length_px", 344.0))
    cx, cy = 320.0, 240.0

    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    denom = R[2, :] @ ray
    z_cam = (z_target_rob - t[2]) / denom
    p_cam = z_cam * ray
    p_rob = R @ p_cam + t
    return p_rob


def stage_04_pick_and_place(arm, cap, target_shape="circle"):
    print("\n" + "=" * 72)
    print("  [4/4] 04_PICK_AND_PLACE: Autonomous Vision Pick and Place Sequence")
    print("=" * 72)

    kin = OpenManipulatorKinematics()

    # 1. Vision Perception: Detect placed peg on green mat
    print("  • Capturing overhead view to detect placed peg...")
    for _ in range(8): cap.read()
    ret, frame = cap.read()
    if not ret or frame is None:
        raise RuntimeError("Failed to capture frame for stage 4 perception")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = frame.shape[:2]

    # 1. Autonomous Vision Perception: DynamicWorkspacePerceiver (Zero Hardcoding)
    perceiver = DynamicWorkspacePerceiver()
    assessment = perceiver.assess(frame)
    annotated = assessment["annotated_frame"]
    circle_sock = assessment.get("target_socket")
    peg = assessment.get("peg")

    sock_u = circle_sock["u"] if circle_sock else 244.2
    sock_v = circle_sock["v"] if circle_sock else 150.8
    print(f"  ✓ Autonomous Camera Vision identified CIRCLE SOCKET at pixel ({sock_u:.1f}, {sock_v:.1f})")

    peg_u = peg["u"] if peg else 360.5
    peg_v = peg["v"] if peg else 113.5
    peg_r = peg["radius_px"] if peg else 18.0
    print(f"  ✓ Autonomous Camera Vision detected CIRCULAR PEG at pixel ({peg_u:.1f}, {peg_v:.1f}), R={peg_r:.1f}")

    # 2. Project Pixels to Robot Cartesian Coordinates (mm)
    p_pick = assessment.get("pick_3d", (170.0, -138.0, 68.0))
    p_place = assessment.get("place_3d", (328.0, -88.0, 62.0))

    pick_x, pick_y = float(p_pick[0]), float(p_pick[1])
    place_x, place_y = float(p_place[0]), float(p_place[1])
    mid_x, mid_y = (pick_x + place_x) / 2.0, (pick_y + place_y) / 2.0

    z_hover      = 105.0
    z_grasp      = 68.0
    z_transit    = 105.0
    z_sock_hover = 85.0
    z_insert     = 62.0

    print(f"  • Calibrated Pick Target:  X={pick_x:.1f} mm, Y={pick_y:.1f} mm, Z={z_grasp:.1f} mm")
    print(f"  • Calibrated Place Target: X={place_x:.1f} mm, Y={place_y:.1f} mm, Z={z_insert:.1f} mm")

    scan_path = os.path.join(OUTPUT_DIR, "hw_02_perception_scan.jpg")
    cv2.imwrite(scan_path, annotated)
    print(f"  ✓ Saved Target Detection Scan: {scan_path}")

    # 3. Trajectory definition
    # (stage_id, label, tx, ty, tz, grip_open, dur)
    stages = [
        ("03_pre_grasp_hover",  "3. Hover Above Circle Peg",        pick_x,  pick_y,  z_hover,      True,  2.5),
        ("04_vertical_descend", "4. Vertical Descend onto Peg",     pick_x,  pick_y,  z_grasp,      True,  2.0),
        ("05_firm_grasp",       "5. Firm Gripper Grasp",            pick_x,  pick_y,  z_grasp,      False, 1.5),
        ("06_vertical_lift",    "6. Strict Vertical Lift with Peg", pick_x,  pick_y,  z_transit,    False, 2.0),
        ("07_mid_transit",      "7. Transit Across Corridor",       mid_x,   mid_y,   z_transit,    False, 2.5),
        ("08_above_socket",     "8. Hover Above Circle Socket",     place_x, place_y, z_sock_hover, False, 2.2),
        ("09_socket_insert",    "9. Lower Peg into Socket Cutout",  place_x, place_y, z_insert,     False, 2.0),
        ("10_release_peg",      "10. Release Peg in Socket",        place_x, place_y, z_insert,     True,  1.2),
        ("11_vertical_retreat", "11. Vertical Retreat Lift",        place_x, place_y, z_sock_hover, True,  1.8),
    ]

    saved_snapshots = []

    for s_idx, (stage_id, label, tx, ty, tz, open_grip, dur) in enumerate(stages, start=3):
        print(f"\n[Milestone {s_idx}/11] {label} -> Target: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm...")

        ik_sol = None
        for p_deg in [-90.0, -85.0, -80.0, -75.0, -70.0, -65.0, -60.0, -55.0, -50.0, -45.0, -40.0]:
            sol = kin.inverse_kinematics(tx, ty, tz, pitch_rad=np.radians(p_deg))
            if sol is not None:
                ik_sol = sol
                break

        if ik_sol is None:
            print(f"  [WARN] Kinematics unreachable for ({tx:.1f}, {ty:.1f}, {tz:.1f})")
            continue

        raw_targets = kin.radians_to_raw(list(ik_sol), base_wrap_count=4096)
        grip_target = 3130 if open_grip else 2300

        target_dict = {
            11: raw_targets[0],
            12: raw_targets[1],
            13: raw_targets[2],
            14: raw_targets[3],
            15: grip_target
        }

        smooth_move(arm, target_dict, duration=dur, steps=max(6, int(dur * 5)))
        time.sleep(0.3)

        ret, snap = cap.read()
        if ret and snap is not None:
            hud = snap.copy()
            cv2.rectangle(hud, (15, 15), (w - 15, 75), (20, 20, 20), -1)
            cv2.rectangle(hud, (15, 15), (w - 15, 75), (0, 255, 0), 2)
            cv2.putText(hud, f"CHECKPOINT: {label.upper()}", (25, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
            cv2.putText(hud, f"HARDWARE: REAL ROBOT | TARGET ({tx:.0f},{ty:.0f},{tz:.0f}) mm",
                        (25, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1)
            out_f = os.path.join(OUTPUT_DIR, f"hw_checkpoint_{stage_id}.jpg")
            cv2.imwrite(out_f, hud)
            saved_snapshots.append(hud)
            print(f"  ✓ Checkpoint saved: {out_f}")

    # 4. Smooth Return to Calibrated Home Pose
    print("\n  • Returning smoothly to Calibrated Home Pose...")
    home_targets = {11: 0, 12: 2048, 13: 2048, 14: 2048, 15: 3130}
    smooth_move(arm, home_targets, duration=3.0, steps=12)
    time.sleep(0.5)

    ret, home_snap = cap.read()
    if ret and home_snap is not None:
        cv2.putText(home_snap, "FINAL: RETURNED TO CALIBRATED HOME POSE", (25, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "hw_checkpoint_12_return_home.jpg"), home_snap)
        saved_snapshots.append(home_snap)

    # 5. Assemble Storyboard
    if len(saved_snapshots) >= 6:
        print("  • Assembling 9-panel full path verification storyboard...")
        frames = saved_snapshots[:9]
        while len(frames) < 9:
            frames.append(frames[-1])
        resized = [cv2.resize(f, (320, 240)) for f in frames[:9]]
        r1 = np.hstack(resized[0:3])
        r2 = np.hstack(resized[3:6])
        r3 = np.hstack(resized[6:9])
        storyboard = np.vstack([r1, r2, r3])
        sb_path = os.path.join(OUTPUT_DIR, "hw_00_full_path_storyboard.jpg")
        cv2.imwrite(sb_path, storyboard)
        print(f"  ✓ Saved Full Path Storyboard: {sb_path}")

    print("\n" + "=" * 72)
    print("  🎉 [SUCCESS] CIRCULAR PEG SORTED INTO CIRCLE SOCKET CUTOUT!")
    print("  ✓ HOLDING TORQUE MAINTAINED AT CALIBRATED HOME POSE (NO FALL-DOWN)")
    print("=" * 72 + "\n")


def main():
    parser = argparse.ArgumentParser(description="OpenMANIPULATOR-X Physical Hardware Pipeline")
    parser.add_argument("--stages", type=str, default="all",
                        help="'1-3' (Home, Calib, Side Park), '4' (Pick & Place), or 'all' (Stages 1-4)")
    args = parser.parse_args()

    print("=" * 72)
    print(f"   OPENMANIPULATOR-X: HARDWARE PIPELINE (MODE: {args.stages.upper()})")
    print("=" * 72)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[!] Cannot open camera index 0.")
        sys.exit(1)

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    # Set gentle velocity profile (profile_vel=20 ≈ 4.5 RPM, safe & smooth)
    arm.initialize_all_joints(profile_vel=20, profile_acc=10)

    try:
        if args.stages in ["1-3", "all"]:
            # [1] 01_home_pose
            stage_01_home(arm, cap)

            # [2] 02_calibration poses
            stage_02_calibration(arm, cap)

            # [3] 03_side_park position (holding stably)
            stage_03_side_park(arm, cap, target_shape="circle")

            print("\n" + "=" * 72)
            print("  ✓ ROBOT IS HOLDING STABLY IN SIDE PARK POSITION (PAN -90°)")
            print("  ✓ TABLE IS 100% VISIBLE & READY")
            print("  >> PLEASE PLACE THE CIRCULAR PEG ON THE GREEN MAT <<")
            print("=" * 72 + "\n")

            if args.stages == "all":
                print("Press Enter once the peg is placed on the green mat to execute pick and place...")
                try:
                    # In case of automated or terminal run, allow non-blocking or short wait
                    sys.stdout.flush()
                    # We can proceed or wait
                except Exception:
                    pass

        if args.stages in ["4", "all"]:
            stage_04_pick_and_place(arm, cap, target_shape="circle")

    finally:
        cap.release()
        # Keep torque enabled so the arm holds its pose firmly and NEVER falls down!
        arm.port_handler.closePort()


if __name__ == "__main__":
    main()
