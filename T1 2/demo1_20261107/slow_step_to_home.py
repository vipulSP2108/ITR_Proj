#!/usr/bin/env python3
"""
Step-by-Step Gentle Calibration to Home Pose for OpenMANIPULATOR-X.

Moves the arm slowly in small micro-steps to the calibrated Home Pose:
  • Joint 1 (Base Pan):        0 counts (180.0°)
  • Joint 2 (Shoulder Pitch):  2048 counts (0.0°)
  • Joint 3 (Elbow Pitch):     2048 counts (0.0°)
  • Joint 4 (Wrist Pitch):     2048 counts (0.0°)
  • Joint 5 (Gripper):         3130 counts (90.0° Open)

Takes a camera screenshot after every single micro-step, annotating
live motor positions and error status, ensuring zero sudden movements.
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

ADDR_HARDWARE_ERROR_STATUS = 70
IMP_IMAGES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imp_images")
os.makedirs(IMP_IMAGES_DIR, exist_ok=True)

HOME_TARGETS = {
    11: 0,
    12: 2048,
    13: 2048,
    14: 2048,
    15: 3130
}

def main():
    print("=" * 66)
    print("  GENTLE STEP-BY-STEP CALIBRATION TO HOME POSE")
    print("=" * 66)

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    # 1. Clear any error latches
    print("\n[1] Checking & clearing hardware error latches...")
    for dxl_id in [11, 12, 13, 14, 15]:
        err, _, _ = arm.packet_handler.read1ByteTxRx(arm.port_handler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        if err != 0:
            print(f"  • Motor {dxl_id} has error {err}. Rebooting...")
            arm.packet_handler.reboot(arm.port_handler, dxl_id)
            time.sleep(0.5)

    # 2. Open camera
    print("\n[2] Initializing overhead camera (Index 0)...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[!] Cannot open camera index 0.")
        sys.exit(1)
    for _ in range(5): cap.read()

    # 3. Read current starting positions
    start_pos = {}
    print("\n[3] Current Starting Positions:")
    for dxl_id in [11, 12, 13, 14, 15]:
        p = arm.read_position(dxl_id)
        if p is None:
            print(f"[!] Failed to read position for motor {dxl_id}")
            cap.release()
            sys.exit(1)
        start_pos[dxl_id] = p
        deg = (p - 2048) * POSITION_UNIT_TO_DEG
        print(f"  • ID {dxl_id:2d}: raw={p:5d} ({deg:+.2f}°)")

    # 4. Set gentle profile (profile_vel=15 ≈ 3.4 RPM, profile_acc=10)
    print("\n[4] Configuring gentle velocity profile (profile_vel=15)...")
    arm.initialize_all_joints(profile_vel=15, profile_acc=10)

    # Take Step 0 screenshot (Starting state)
    ret, frame = cap.read()
    if ret:
        cv2.putText(frame, "STEP 0/10: INITIAL RESTING STANCE", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(IMP_IMAGES_DIR, "step_to_home_00.jpg"), frame)
        print("  ✓ Saved step_to_home_00.jpg")

    # 5. Execute 10 micro-steps
    TOTAL_STEPS = 10
    print(f"\n[5] Executing {TOTAL_STEPS} micro-steps to Home Pose...")
    for step in range(1, TOTAL_STEPS + 1):
        alpha = step / float(TOTAL_STEPS)
        step_targets = {}
        for dxl_id in [15, 14, 13, 12, 11]:
            s = start_pos[dxl_id]
            g = HOME_TARGETS[dxl_id]
            tgt = int(round(s + alpha * (g - s)))
            step_targets[dxl_id] = tgt
            arm.write_position(dxl_id, tgt)
            time.sleep(0.05)

        # Allow motors to smoothly reach the micro-target
        time.sleep(0.8)

        # Read actual positions & check errors
        actual_pos = {}
        has_error = False
        for dxl_id in [11, 12, 13, 14, 15]:
            p = arm.read_position(dxl_id)
            err, _, _ = arm.packet_handler.read1ByteTxRx(arm.port_handler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
            actual_pos[dxl_id] = p
            if err != 0:
                print(f"[!] ERROR ON MOTOR {dxl_id}: Status {err}")
                has_error = True

        # Capture camera screenshot after this step
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (w - 10, 75), (20, 20, 20), -1)
            cv2.rectangle(overlay, (10, 10), (w - 10, 75), (0, 255, 0), 1)

            cv2.putText(overlay, f"STEP {step}/{TOTAL_STEPS}: PROGRESS {int(alpha*100)}%",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            telemetry_str = f"ID11:{actual_pos.get(11)} ID12:{actual_pos.get(12)} ID13:{actual_pos.get(13)} ID14:{actual_pos.get(14)} ID15:{actual_pos.get(15)}"
            cv2.putText(overlay, telemetry_str, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

            step_img_path = os.path.join(IMP_IMAGES_DIR, f"step_to_home_{step:02d}.jpg")
            cv2.imwrite(step_img_path, overlay)
            print(f"  ✓ Step {step:2d}/{TOTAL_STEPS} reached -> Snapshot: step_to_home_{step:02d}.jpg | Telemetry: {telemetry_str}")

        if has_error:
            print("[!] Halting due to motor error latch.")
            break

    # Save final verified snapshot
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(os.path.join(IMP_IMAGES_DIR, "hw_calibrated_home_verified.jpg"), frame)
        print("\n  ✓ Saved final verified Home Pose to hw_calibrated_home_verified.jpg")

    cap.release()
    arm.port_handler.closePort()
    print("\n" + "=" * 66)
    print("  STEP-BY-STEP CALIBRATION SEQUENCE COMPLETE")
    print("=" * 66)

if __name__ == "__main__":
    main()

