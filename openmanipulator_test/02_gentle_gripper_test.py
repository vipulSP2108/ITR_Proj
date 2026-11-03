#!/usr/bin/env python3
"""
Step 2: Gentle Gripper Movement Test
Safe physical test: Gently opens the gripper (ID 15) slightly, pauses,
returns it to the original position, and releases torque.
Moving the gripper carries ZERO risk of arm collisions.
"""

import time
import sys
from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    POSITION_UNIT_TO_DEG
)

GRIPPER_ID = 15

def main():
    print("=" * 60)
    print("  OpenMANIPULATOR-X: Safe Gripper Test (ID 15)")
    print("=" * 60)

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected on Mac. Plug it in and retry.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    try:
        # 1. Ping Gripper
        print(f"\n[1] Pinging Gripper (Motor ID {GRIPPER_ID})...")
        ok, model = arm.ping_motor(GRIPPER_ID)
        if not ok:
            print(f"[!] Gripper (ID {GRIPPER_ID}) did not respond. Check 12V power and cable.")
            return

        # 2. Read starting position
        start_pos = arm.read_position(GRIPPER_ID)
        if start_pos is None:
            print("[!] Could not read gripper position.")
            return
        start_deg = (start_pos - 2048) * POSITION_UNIT_TO_DEG
        print(f"  ✓ Gripper detected (Model #{model})")
        print(f"  • Current Position: {start_pos} counts ({start_deg:+.1f}°)")

        # 3. Configure Operating Mode = 3 (Position Control) and gentle velocity profile
        arm.disable_torque(GRIPPER_ID)
        arm.set_operating_mode(GRIPPER_ID, 3) # Mode 3 = Position Control Mode
        arm.set_speed_and_accel(GRIPPER_ID, profile_velocity=40, profile_accel=12)
        arm.enable_torque(GRIPPER_ID)

        # 4. Move by small safe increment (+150 encoder units ~ 13 degrees)
        delta_units = 150
        target_pos = start_pos + delta_units
        if target_pos > 4000:
            target_pos = start_pos - delta_units # Reverse direction if near upper limit

        print(f"\n[2] Gently moving gripper by {delta_units} counts...")
        arm.write_position(GRIPPER_ID, target_pos)
        time.sleep(1.5) # Wait for gentle motion to complete

        # 5. Return to starting position
        print(f"[3] Returning gripper to original starting position ({start_pos})...")
        arm.write_position(GRIPPER_ID, start_pos)
        time.sleep(1.5)

        print("\n✓ Gripper test completed successfully!")

    finally:
        # Always release torque and disconnect safely
        print("\n[4] Releasing gripper motor torque...")
        arm.disable_torque(GRIPPER_ID)
        arm.disconnect()
        print("  All done.")

if __name__ == "__main__":
    main()

