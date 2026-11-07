#!/usr/bin/env python3
"""
Teach-Mode Home Pose Calibration for OpenMANIPULATOR-X.

Workflow:
  1. All motors have torque DISABLED so you can freely position the arm by hand.
  2. Hold the arm in the exact True 90-degree Default / Ready stance.
  3. This script continuously prints the live joint angles and encoder counts.
  4. Press Ctrl+C (or wait for automatic capture) to save the exact calibrated counts.
  5. Automatically saves the parameters to config/calibrated_home.yaml and captures
     a camera snapshot to verify visual alignment.
"""

import os
import sys
import time
import yaml
import cv2

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    JOINT_IDS,
    POSITION_UNIT_TO_DEG
)

OUTPUT_YAML_PATH = os.path.join(BASE_DIR, "calibrated_home.yaml")
T1_YAML_PATH = os.path.join(os.path.dirname(BASE_DIR), "T1", "config", "calibrated_home.yaml")
IMAGE_PATH = os.path.join(os.path.dirname(BASE_DIR), "T1", "demo1_20261107", "imp_images", "hw_calibrated_home_camera.jpg")

def main():
    print("=" * 66)
    print("   OpenMANIPULATOR-X: TEACH-MODE HOME POSE CALIBRATION")
    print("=" * 66)
    print("Motors are UNLOCKED. Please hold the arm by hand in the desired")
    print("True 90° Upright Ready Position.\n")

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    arm.disable_all_torque()

    last_positions = {}
    try:
        print("Reading live joint positions (stabilize arm in position for 3 seconds)...")
        print("-" * 66)
        stable_count = 0
        prev_p = None

        while True:
            positions = {}
            degrees = {}
            for dxl_id in [11, 12, 13, 14, 15]:
                pos = arm.read_position(dxl_id)
                if pos is not None:
                    positions[dxl_id] = pos
                    degrees[dxl_id] = round((pos - 2048) * POSITION_UNIT_TO_DEG, 1)

            last_positions = positions

            status_line = " | ".join([f"ID{i}: {positions.get(i, '---'):4d} ({degrees.get(i, 0.0):+5.1f}°)" for i in [11, 12, 13, 14, 15]])
            sys.stdout.write(f"\r  {status_line}")
            sys.stdout.flush()

            # Check if arm is holding steady
            cur_tuple = tuple(positions.get(i, 0) for i in [11, 12, 13, 14, 15])
            if prev_p is not None and all(abs(c - p) <= 5 for c, p in zip(cur_tuple, prev_p)):
                stable_count += 1
            else:
                stable_count = 0
            prev_p = cur_tuple

            if stable_count >= 6: # ~3 seconds steady
                print("\n\n✓ Arm is stable! Recording calibrated positions...")
                break

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nCaptured current position on interrupt!")

    finally:
        arm.port_handler.closePort()

    if not last_positions or len(last_positions) < 5:
        print("[!] Failed to read all 5 motor positions.")
        return

    # Save to YAML
    calib_data = {
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "home_encoder_counts": {
            "joint1_base_pan": last_positions[11],
            "joint2_shoulder_pitch": last_positions[12],
            "joint3_elbow_pitch": last_positions[13],
            "joint4_wrist_pitch": last_positions[14],
            "joint5_gripper": last_positions[15],
        },
        "home_degrees": {
            "joint1_base_pan": round((last_positions[11] - 2048) * POSITION_UNIT_TO_DEG, 2),
            "joint2_shoulder_pitch": round((last_positions[12] - 2048) * POSITION_UNIT_TO_DEG, 2),
            "joint3_elbow_pitch": round((last_positions[13] - 2048) * POSITION_UNIT_TO_DEG, 2),
            "joint4_wrist_pitch": round((last_positions[14] - 2048) * POSITION_UNIT_TO_DEG, 2),
            "joint5_gripper": round((last_positions[15] - 2048) * POSITION_UNIT_TO_DEG, 2),
        }
    }

    with open(OUTPUT_YAML_PATH, "w") as f:
        yaml.dump(calib_data, f, sort_keys=False)
    with open(T1_YAML_PATH, "w") as f:
        yaml.dump(calib_data, f, sort_keys=False)

    print(f"✓ Saved calibration config to: {OUTPUT_YAML_PATH}")
    print(f"✓ Saved calibration config to: {T1_YAML_PATH}")

    # Capture Camera Snapshot
    cap = cv2.VideoCapture(0)
    for _ in range(5): cap.read()
    ret, frame = cap.read()
    if ret:
        cv2.putText(frame, "CALIBRATED HOME POSE", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(IMAGE_PATH, frame)
        print(f"✓ Saved camera reference shot to: {IMAGE_PATH}")
    cap.release()

    print("\n" + "=" * 66)
    print("  CALIBRATED HOME POSE SUMMARY:")
    for j_id, name in JOINT_IDS.items():
        raw_val = last_positions[j_id]
        deg_val = round((raw_val - 2048) * POSITION_UNIT_TO_DEG, 1)
        print(f"  • ID {j_id:2d} ({name:20s}): {raw_val:5d} counts ({deg_val:+.1f}°)")
    print("=" * 66)

if __name__ == "__main__":
    main()

