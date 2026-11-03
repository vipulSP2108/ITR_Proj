#!/usr/bin/env python3
"""
Step 3: Gentle Base Joint Nudge (Joint 1, ID 11)
Rotates the base pan joint (ID 11) by only +5 degrees at a very slow,
gentle speed, pauses, and returns back to the exact initial angle.
Base rotation moves horizontally and does not drop the arm links.
"""

import time
import sys
from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    POSITION_UNIT_TO_DEG
)

JOINT1_ID = 11

def main():
    print("=" * 60)
    print("  OpenMANIPULATOR-X: Gentle Base Nudge Test (ID 11)")
    print("=" * 60)

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected on Mac. Plug it in and retry.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    try:
        print(f"\n[1] Checking Base Pan Joint (Motor ID {JOINT1_ID})...")
        ok, model = arm.ping_motor(JOINT1_ID)
        if not ok:
            print(f"[!] Motor ID {JOINT1_ID} not responding. Check cable and 12V power.")
            return

        start_deg = arm.read_position_deg(JOINT1_ID)
        start_raw = arm.read_position(JOINT1_ID)
        print(f"  ✓ Joint 1 ready (Model #{model})")
        print(f"  • Current Angle: {start_deg:+.2f}° (Raw: {start_raw})")

        # Configure Operating Mode = 4 (Extended Multi-turn Position) and gentle profile
        arm.disable_torque(JOINT1_ID)
        arm.set_operating_mode(JOINT1_ID, 4)
        arm.set_speed_and_accel(JOINT1_ID, profile_velocity=30, profile_accel=10)
        arm.enable_torque(JOINT1_ID)

        # Nudge angle: 5 degrees
        nudge_deg = 5.0
        print(f"\n[2] Nudging Base Joint by +{nudge_deg}° at ultra-low velocity...")
        arm.gentle_move_relative_deg(JOINT1_ID, delta_deg=+nudge_deg, profile_vel=20)
        time.sleep(2.0)

        mid_deg = arm.read_position_deg(JOINT1_ID)
        print(f"  • Reached Angle: {mid_deg:+.2f}° (Nudged by {mid_deg - start_deg:+.2f}°)")

        print(f"\n[3] Returning smoothly back to original position ({start_deg:+.2f}°)...")
        arm.gentle_move_relative_deg(JOINT1_ID, delta_deg=-nudge_deg, profile_vel=20)
        time.sleep(2.0)

        final_deg = arm.read_position_deg(JOINT1_ID)
        print(f"  • Returned to: {final_deg:+.2f}°")
        print("\n✓ Joint nudge completed smoothly with zero jerking!")

    finally:
        print("\n[4] Releasing torque on Joint 1...")
        arm.disable_torque(JOINT1_ID)
        arm.disconnect()
        print("  All done.")

if __name__ == "__main__":
    main()

