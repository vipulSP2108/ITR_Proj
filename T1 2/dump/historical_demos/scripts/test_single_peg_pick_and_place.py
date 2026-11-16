#!/usr/bin/env python3
"""
Test 1 Peg Pick-and-Place State Machine for OpenMANIPULATOR-X on macOS.
Executes an end-to-end autonomous cycle for a single peg (e.g. Blue Square):
  1. Ready Pose -> 2. Pre-Grasp -> 3. Vertical Descent -> 4. Grasp
  -> 5. Vertical Lift -> 6. Transit to Socket -> 7. Lower into Socket
  -> 8. Release Gripper -> 9. Vertical Retreat -> 10. Return Home.

Supports:
  • --dry-run (Default: Runs sequence elevated in mid-air for zero-risk verification)
  • --live (Tabletop pick-and-place)
"""

import os
import sys
import time
import argparse
import numpy as np

try:
    from src.control.dynamixel_wrapper import OpenManipulatorController, find_u2d2_port, JOINT_IDS
    from src.planning.kinematics import OpenManipulatorKinematics
except ImportError:
    from dynamixel_wrapper import OpenManipulatorController, find_u2d2_port, JOINT_IDS
    from kinematics import OpenManipulatorKinematics

def check_hardware_power(arm: OpenManipulatorController) -> bool:
    """Verifies that the motors actually have 12V power and respond to pings."""
    ok_base, _ = arm.ping_motor(11)
    ok_grip, _ = arm.ping_motor(15)
    if not ok_base or not ok_grip:
        print("\n" + "!" * 66)
        print(" [!] COMMUNICATION ERROR: Motors are NOT responding (RX Timeout)!")
        print(" Please check:")
        print("   1. Is the 12V DC power brick plugged into the robot power hub?")
        print("   2. Is the power switch on the power hub turned ON?")
        print("   3. Is the black cable from U2D2 to Joint 11 firmly plugged in?")
        print("!" * 66 + "\n")
        return False
    return True

def execute_cartesian_waypoint(arm: OpenManipulatorController, kin: OpenManipulatorKinematics,
                               x: float, y: float, z: float, pitch_deg: float = -90.0,
                               duration_sec: float = 2.0, base_wrap: int = 4096) -> bool:
    """Solves IK for (x, y, z, pitch_deg) and commands all 4 arm joints smoothly."""
    pitch_rad = np.radians(pitch_deg)
    ik_sol = kin.inverse_kinematics(x, y, z, pitch_rad=pitch_rad)
    if ik_sol is None:
        print(f"  [!] IK Error: Target Cartesian pose ({x:.1f}, {y:.1f}, {z:.1f}) is UNREACHABLE!")
        return False

    raw_targets = kin.radians_to_raw(list(ik_sol), base_wrap_count=base_wrap)
    print(f"  -> Moving to ({x:5.1f}, {y:5.1f}, {z:5.1f})mm | Joint Counts: {raw_targets}")

    # Command Joints 1 to 4
    for i, dxl_id in enumerate([11, 12, 13, 14]):
        arm.write_position(dxl_id, raw_targets[i])

    time.sleep(duration_sec)
    return True

def set_gripper(arm: OpenManipulatorController, open_state: bool, wait_sec: float = 1.0):
    """Opens (~1700 counts) or closes (~2350 counts) the gripper."""
    target = 1700 if open_state else 2350
    state_str = "OPENING" if open_state else "CLOSING"
    print(f"  -> Gripper {state_str} (Target: {target})...")
    arm.write_position(15, target)
    time.sleep(wait_sec)

def run_pick_and_place(arm: OpenManipulatorController, kin: OpenManipulatorKinematics, dry_run: bool = True):
    print("=" * 66)
    mode_str = "DRY RUN (Elevated in Air - Zero Risk)" if dry_run else "LIVE TABLETOP RUN"
    print(f"  OpenMANIPULATOR-X 1-Peg Pick-and-Place: {mode_str}")
    print("=" * 66)

    # 1. Hardware Power & Ping Pre-flight Check
    if not check_hardware_power(arm):
        sys.exit(1)

    # Safe reachable Cartesian coordinates (in mm from robot base)
    pick_x, pick_y = 160.0, 60.0    # Left side (pickup)
    place_x, place_y = 160.0, -60.0 # Right side (placement)

    if dry_run:
        z_transit = 85.0
        z_pre = 80.0
        z_grasp = 55.0  # Elevated in mid-air
    else:
        z_transit = 85.0
        z_pre = 80.0
        z_grasp = 40.0  # Tabletop grasp height

    # Configure gentle speed profile
    arm.initialize_all_joints(profile_vel=35, profile_acc=10)

    # Determine base wrap count from current position
    cur_base = arm.read_position(11) or 4096
    base_wrap = round(cur_base / 4096.0) * 4096

    t0 = time.time()
    try:
        # Step 1: Open Gripper at Start
        print("\n[Stage 1/8] Opening Gripper...")
        set_gripper(arm, open_state=True, wait_sec=1.0)

        # Step 2: Pre-Grasp Pose (Directly above peg)
        print(f"\n[Stage 2/8] Moving to PRE-GRASP above Peg at ({pick_x}, {pick_y}, {z_pre})...")
        execute_cartesian_waypoint(arm, kin, pick_x, pick_y, z_pre, duration_sec=2.5, base_wrap=base_wrap)

        # Step 3: Vertical Descent to Grasp Height
        print(f"\n[Stage 3/8] Descending vertically to GRASP height ({pick_x}, {pick_y}, {z_grasp})...")
        execute_cartesian_waypoint(arm, kin, pick_x, pick_y, z_grasp, duration_sec=2.0, base_wrap=base_wrap)

        # Step 4: Close Gripper (Grasp the Peg)
        print("\n[Stage 4/8] Closing Gripper to grasp peg...")
        set_gripper(arm, open_state=False, wait_sec=1.2)

        # Step 5: Strict Vertical Lift to Transit Height
        print(f"\n[Stage 5/8] Lifting peg vertically to transit height ({pick_x}, {pick_y}, {z_transit})...")
        execute_cartesian_waypoint(arm, kin, pick_x, pick_y, z_transit, duration_sec=2.0, base_wrap=base_wrap)

        # Step 6: Transit to Pre-Place Pose above Target Socket
        print(f"\n[Stage 6/8] Transiting to PRE-PLACE above Socket at ({place_x}, {place_y}, {z_pre})...")
        execute_cartesian_waypoint(arm, kin, place_x, place_y, z_pre, duration_sec=3.0, base_wrap=base_wrap)

        # Step 7: Vertical Descent into Socket
        print(f"\n[Stage 7/8] Lowering peg into socket ({place_x}, {place_y}, {z_grasp})...")
        execute_cartesian_waypoint(arm, kin, place_x, place_y, z_grasp, duration_sec=2.0, base_wrap=base_wrap)

        # Step 8: Release Peg & Retreat Vertically
        print("\n[Stage 8/8] Releasing peg and retreating...")
        set_gripper(arm, open_state=True, wait_sec=1.0)
        execute_cartesian_waypoint(arm, kin, place_x, place_y, z_transit, duration_sec=2.0, base_wrap=base_wrap)

        total_time = time.time() - t0
        print("\n" + "=" * 66)
        print(f"  ✓ Pick-and-Place sequence completed successfully in {total_time:.1f} seconds!")
        print("=" * 66)

    finally:
        # Move back to safe ready pose
        print("\nReturning to default Home Ready Pose...")
        home_counts = [base_wrap, 2048, 1100, 2048]
        for i, dxl_id in enumerate([11, 12, 13, 14]):
            arm.write_position(dxl_id, home_counts[i])
        time.sleep(2.0)
        arm.port_handler.closePort()

def main():
    parser = argparse.ArgumentParser(description="OpenMANIPULATOR-X 1-Peg Pick-and-Place Test")
    parser.add_argument("--live", action="store_true", help="Execute live tabletop grasp (default is safe dry-run in air)")
    args = parser.parse_args()

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected on Mac.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    kin = OpenManipulatorKinematics()
    dry_run = not args.live

    run_pick_and_place(arm, kin, dry_run=dry_run)

if __name__ == "__main__":
    main()
