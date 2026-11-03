#!/usr/bin/env python3
"""
Step 5: Move OpenMANIPULATOR-X to Default Ready / Home Pose
Moves smoothly to the reference L-pose (as shown in the CAD image):
  • Base (Joint 1):    Centered Forward
  • Shoulder (Joint 2): Vertical (Straight Up)
  • Elbow (Joint 3):    Horizontal Forward (90° relative to shoulder)
  • Wrist (Joint 4):    Level Forward (Collinear with forearm)
  • Gripper (Joint 5):  Neutral / Open
"""

import sys
import time
from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    JOINT_IDS,
    POSITION_UNIT_TO_DEG
)

# Reference Ready / Home Pose (Encoder counts 0 ~ 4095)
# Note: Base (ID 11) center forward is 2048 counts (0 degrees)
HOME_TARGETS = {
    11: 2048,  # Base Pan: Centered Forward (2048 = 0 deg)
    12: 2048,  # Shoulder Pitch: Vertical Upright (2048 = 0 deg / 90 deg to table)
    13: 1024,  # Elbow Pitch: ~90° Forward Bend (Forearm horizontal)
    14: 2048,  # Wrist Pitch: Level Forward (Collinear with Forearm)
    15: 1800   # Gripper: Neutral Open (~1800 counts)
}

def move_to_home_pose(arm: OpenManipulatorController, speed_rpm: float = 3.0):
    """
    Safely and smoothly moves all joints to the default 90-degree ready posture.
    Uses low Profile Velocity to ensure smooth, slow, elegant motion.
    """
    print("\n" + "=" * 60)
    print("  Moving OpenMANIPULATOR-X to True 90° Home Ready Pose")
    print("=" * 60)

    # 1. Initialize all joints into Position Control mode with safe profiles
    profile_vel = int(speed_rpm / 0.229)
    arm.initialize_all_joints(profile_vel=profile_vel, profile_acc=10)

    # 2. Determine best base target (closest equivalent of 2048 to avoid unnecessary spins)
    current_base = arm.read_position(11)
    if current_base is not None:
        # Wrap to closest multiple of 4096 relative to center 2048
        target_base = 2048 + int(round((current_base - 2048) / 4096.0)) * 4096
        HOME_TARGETS[11] = target_base
        print(f"  • Base Joint 1 Target: {target_base} (current: {current_base})")

    # 3. Read starting positions
    print("\n[1] Current Joint Positions:")
    for dxl_id, name in JOINT_IDS.items():
        pos = arm.read_position(dxl_id)
        deg = (pos - 2048) * POSITION_UNIT_TO_DEG if pos is not None else 0
        print(f"  • ID {dxl_id:2d} ({name:20s}): {pos:5d} ({deg:+.1f}°)")

    # 4. Command the target home pose
    print("\n[2] Moving smoothly to Home Pose...")
    for dxl_id in [15, 14, 13, 12, 11]: # Move gripper and upper joints first
        target = HOME_TARGETS[dxl_id]
        arm.write_position(dxl_id, target)
        print(f"  -> Commanding ID {dxl_id:2d} to target: {target}")
        time.sleep(0.3)

    # 5. Wait for motion to settle smoothly
    print("\n[3] Waiting for motion to complete...")
    time.sleep(3.0)

    # 6. Verify final positions
    print("\n[4] Final Positions Reached:")
    for dxl_id, name in JOINT_IDS.items():
        final_pos = arm.read_position(dxl_id)
        final_deg = (final_pos - 2048) * POSITION_UNIT_TO_DEG if final_pos is not None else 0
        print(f"  ✓ ID {dxl_id:2d} ({name:20s}): {final_pos:5d} ({final_deg:+.1f}°)")

    print("\n" + "=" * 60)
    print("  Arm is now in Default Ready Pose!")
    print("=" * 60)

def main():
    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected on Mac.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    try:
        move_to_home_pose(arm, speed_rpm=3.5)
    finally:
        # Keep torque enabled so the arm holds its pose, or ask user
        print("\nNote: Torque remains ENABLED to hold the ready pose.")
        print("To release torque at any time, run 04_keyboard_teleop.py and press 'T' or 'ESC'.")
        arm.port_handler.closePort()

if __name__ == "__main__":
    main()

