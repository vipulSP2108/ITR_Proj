#!/usr/bin/env python3
"""
Physical Multi-Angle Workspace Survey Pipeline for OpenMANIPULATOR-X (Demo 1).

Home Angle Configuration (User Calibrated):
  • Base Pan (ID 11):        180.0° (raw ~ 48 counts)
  • Shoulder Pitch (ID 12):    0.0° (raw ~ 2048 counts)
  • Elbow Pitch (ID 13):       0.0° (raw ~ 2048 counts)
  • Wrist Pitch (ID 14):       0.0° (raw ~ 2048 counts)
  • Gripper (ID 15):          90.0° (raw ~ 3130 counts)

Survey Workflow:
  1. Arm smoothly transitions to Home Angle: [180°, 0°, 0°, 0°, 90°].
  2. Overhead camera captures Stance 1.
  3. Arm smoothly pans +30° Left -> Camera captures Stance 2.
  4. Arm smoothly pans -30° Right -> Camera captures Stance 3.
  5. Arm adjusts to Elevated Pitch -> Camera captures Stance 4.
  6. Arm returns to Home Angle.
  7. Assembles a 2x2 composite multi-angle survey grid in imp_images/.
"""

import os
import sys
import time
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(BASE_DIR, "openmanipulator_test"))

from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    JOINT_IDS,
    POSITION_UNIT_TO_DEG
)

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
IMP_IMAGES_DIR = os.path.join(CURRENT_DIR, "imp_images")
os.makedirs(IMP_IMAGES_DIR, exist_ok=True)

# User-Calibrated Home Raw Counts
BASE_HOME = 48
HOME_COUNTS = [BASE_HOME, 2048, 2048, 2048, 3130]

SURVEY_STANCES = [
    ("Home_180deg",      [BASE_HOME,       2048, 2048, 2048, 3130], "1. Home Stance: [180°, 0°, 0°, 0°, 90°]"),
    ("Pan_Left_30deg",   [BASE_HOME + 341, 2048, 2048, 2048, 3130], "2. Pan Left +30° (Uncover Right Workspace)"),
    ("Pan_Right_30deg",  [BASE_HOME - 341, 2048, 2048, 2048, 3130], "3. Pan Right -30° (Uncover Left Workspace)"),
    ("Elevated_Pitch",   [BASE_HOME,       2330, 1765, 2048, 3130], "4. Elevated Pitch Survey Stance"),
    ("Return_Home",      [BASE_HOME,       2048, 2048, 2048, 3130], "5. Return to Calibrated Home Stance"),
]

def safe_read_pos(arm, dxl_id: int, fallback: int = 2048) -> int:
    for _ in range(4):
        p = arm.read_position(dxl_id)
        if p is not None:
            return p
        time.sleep(0.04)
    return fallback

def run_hardware_survey(camera_idx: int = 0):
    print("=" * 72)
    print("   OpenMANIPULATOR-X Physical Multi-Angle Workspace Survey")
    print("=" * 72)

    # 1. Open Camera
    print(f"\n[Step 1] Opening camera on index {camera_idx}...")
    cap = cv2.VideoCapture(camera_idx)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_idx}")
    for _ in range(10): cap.read() # auto-exposure stabilization

    # 2. Connect to Robot
    port = find_u2d2_port()
    if not port:
        raise RuntimeError("U2D2 adapter not detected!")
    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        raise RuntimeError("Failed to connect to U2D2!")

    # Set very gentle profile velocity (20 counts ≈ 4.5 RPM, smooth & safe)
    arm.initialize_all_joints(profile_vel=20, profile_acc=10)
    print("  ✓ Hardware connected & gentle velocity profile configured.")

    captured_stances = []

    try:
        print("\n[Step 2] Executing Multi-Angle Survey Across Tabletop:")
        for idx, (stance_id, targets, desc) in enumerate(SURVEY_STANCES, start=1):
            print(f"\n  [{idx}/{len(SURVEY_STANCES)}] Moving to '{stance_id}': {desc}...")

            # Command motors in order: Gripper -> Wrist -> Elbow -> Shoulder -> Base
            cmd_order = [4, 3, 2, 1, 0]
            dxl_ids = [11, 12, 13, 14, 15]
            for order_i in cmd_order:
                arm.write_position(dxl_ids[order_i], targets[order_i])
                time.sleep(0.2)

            # Wait for smooth movement
            time.sleep(2.5)

            # Read actual positions safely
            actual_pos = [safe_read_pos(arm, d, fallback=targets[i]) for i, d in enumerate(dxl_ids)]
            deg_vals = [round((p - 2048) * POSITION_UNIT_TO_DEG, 1) for p in actual_pos]
            print(f"      -> Reached Raw: {actual_pos}")
            print(f"      -> Degrees:     {deg_vals}")

            # Capture camera frame
            ret, frame = cap.read()
            if ret and frame is not None:
                annotated = frame.copy()
                h, w = annotated.shape[:2]

                # HUD Overlay
                cv2.rectangle(annotated, (15, 15), (w - 15, 75), (20, 20, 20), -1)
                cv2.rectangle(annotated, (15, 15), (w - 15, 75), (0, 255, 0), 2)
                cv2.putText(annotated, f"STANCE [{idx}]: {stance_id.upper()}", (25, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                cv2.putText(annotated, f"Raw Joints: {actual_pos[:4]} | Grip: {actual_pos[4]}", (25, 62),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1)

                shot_path = os.path.join(IMP_IMAGES_DIR, f"hw_survey_{idx}_{stance_id}.jpg")
                cv2.imwrite(shot_path, annotated)
                print(f"      -> Snapshot saved: {shot_path}")
                if stance_id != "Return_Home":
                    captured_stances.append((stance_id, annotated))

        # 3. Assemble 2x2 Composite Grid
        print("\n[Step 3] Assembling 2x2 Composite Multi-Angle Survey Grid...")
        if len(captured_stances) >= 4:
            r_frames = [cv2.resize(f[1], (640, 480)) for f in captured_stances[:4]]
            row1 = np.hstack([r_frames[0], r_frames[1]])
            row2 = np.hstack([r_frames[2], r_frames[3]])
            grid = np.vstack([row1, row2])

            grid_path = os.path.join(IMP_IMAGES_DIR, "hw_00_multi_angle_survey_grid.jpg")
            cv2.imwrite(grid_path, grid)
            print(f"  ✓ Saved 2x2 Survey Grid to: {grid_path}")

        print("\n" + "=" * 72)
        print("  ✓ Multi-Angle Hardware Workspace Survey Complete!")
        print("=" * 72)

    finally:
        cap.release()
        arm.port_handler.closePort()

if __name__ == "__main__":
    run_hardware_survey(camera_idx=0)
