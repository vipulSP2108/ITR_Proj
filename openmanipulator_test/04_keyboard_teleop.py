#!/usr/bin/env python3
"""
Step 4: Interactive Keyboard Teleoperation for OpenMANIPULATOR-X on macOS.
Directly control joints using single key presses:
  • Joint 1 (Base Pan):       A (Left)  / D (Right)
  • Joint 2 (Shoulder Pitch): W (Up)    / S (Down)
  • Joint 3 (Elbow Pitch):    I (Up)    / K (Down)
  • Joint 4 (Wrist Pitch):    U (Up)    / J (Down)
  • Joint 5 (Gripper):        O (Open)  / P (Close)  or SPACE (Toggle)
  • Step Size:                1 (1.0°)  / 2 (2.5°)   / 3 (5.0°)
  • Emergency Stop:           ESC or X (Cut Torque Instantly)
  • Quit:                     Q (Safely disable torque and exit)
"""

import sys
import os
import tty
import termios
import select
from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    JOINT_IDS,
    POSITION_UNIT_TO_DEG,
    DEG_TO_POSITION_UNIT
)

# Safe software boundaries for OpenMANIPULATOR-X (in raw encoder units 0 ~ 4095)
JOINT_LIMITS = {
    11: (200, 3900),   # Base: ~ -160° to +160°
    12: (1000, 3100),  # Shoulder: ~ -90° to +90°
    13: (900, 3200),   # Elbow: ~ -100° to +100°
    14: (900, 3200),   # Wrist: ~ -100° to +100°
    15: (1400, 2700),  # Gripper: Open (~1500) to Closed (~2600)
}

def get_key_nonblocking(timeout=0.05):
    """Reads a single keypress from standard input without blocking."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Handle escape sequences for arrow keys or ESC
            if key == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if rlist2:
                    seq = sys.stdin.read(2)
                    if seq == '[A': return 'UP'
                    if seq == '[B': return 'DOWN'
                    if seq == '[C': return 'RIGHT'
                    if seq == '[D': return 'LEFT'
                return 'ESC'
            return key
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def print_dashboard(current_positions, step_deg, torque_enabled, last_action="Ready"):
    # Clear terminal screen
    sys.stdout.write("\033[H\033[J")
    print("=" * 66)
    print("      OpenMANIPULATOR-X Keyboard Teleoperation (macOS)")
    print("=" * 66)
    print(" [Joint 1: Base Pan]       A : Turn Left  (-Yaw) | D : Turn Right (+Yaw)")
    print(" [Joint 2: Shoulder]       W : Move Up    (-Pitch)| S : Move Down  (+Pitch)")
    print(" [Joint 3: Elbow]          I : Move Up    (-Pitch)| K : Move Down  (+Pitch)")
    print(" [Joint 4: Wrist]          U : Tilt Up    (-Pitch)| J : Tilt Down  (+Pitch)")
    print(" [Joint 5: Gripper]        O : Open               | P : Close")
    print("                           SPACE : Toggle Gripper Open/Close")
    print(" [Step Size]               1 : 1.0° | 2 : 2.5° | 3 : 5.0°")
    print(" [Preset Posture]          H : Move to Default Ready Pose (L-Shape)")
    print(" [Safety / Torque]         T : Toggle Torque | X or ESC : E-STOP")
    print(" [Exit]                    Q : Quit & Disable Torque")
    print("-" * 66)
    status_str = "\033[92mTORQUE ENABLED (ACTIVE)\033[0m" if torque_enabled else "\033[91mTORQUE DISABLED (IDLE)\033[0m"
    print(f" Status: {status_str} | Step: {step_deg:.1f}° | Action: {last_action}")
    print("-" * 66)
    print(" Current Positions:")
    for dxl_id in sorted(JOINT_IDS.keys()):
        raw = current_positions.get(dxl_id, 0)
        deg = (raw - 2048) * POSITION_UNIT_TO_DEG
        name = JOINT_IDS[dxl_id]
        print(f"  • ID {dxl_id:2d} | {name:24s} : {deg:6.1f}° (Raw: {raw:4d})")
    print("=" * 66)
    print(" Press a key to move... (Press 'Q' to exit)")

def main():
    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected on Mac.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    step_deg = 2.5
    torque_enabled = False
    current_positions = {}
    gripper_open = False

    try:
        # Initialize operating mode (Position Control) and torque on all 5 motors
        arm.initialize_all_joints(profile_vel=45, profile_acc=12)
        torque_enabled = True

        # Read actual positions of all 5 joints
        for dxl_id in JOINT_IDS.keys():
            raw = arm.read_position(dxl_id)
            current_positions[dxl_id] = raw if raw is not None else 2048

        last_action = "Connected & Initialized in Position Mode"
        print_dashboard(current_positions, step_deg, torque_enabled, last_action)

        while True:
            key = get_key_nonblocking(timeout=0.08)
            if not key:
                continue

            action_taken = False
            raw_step = int(step_deg * DEG_TO_POSITION_UNIT)

            # --- QUIT ---
            if key in ['q', 'Q']:
                last_action = "Exiting teleop..."
                break

            # --- EMERGENCY STOP ---
            elif key in ['ESC', 'x', 'X']:
                arm.disable_all_torque()
                torque_enabled = False
                last_action = "EMERGENCY STOP (Torque Disabled)!"
                action_taken = True

            # --- TORQUE TOGGLE ---
            elif key in ['t', 'T']:
                if torque_enabled:
                    arm.disable_all_torque()
                    torque_enabled = False
                    last_action = "Torque Disabled (Motors free)"
                else:
                    # Refresh positions before re-enabling torque
                    for dxl_id in JOINT_IDS.keys():
                        pos = arm.read_position(dxl_id)
                        if pos is not None:
                            current_positions[dxl_id] = pos
                        arm.enable_torque(dxl_id)
                    torque_enabled = True
                    last_action = "Torque Re-Enabled"
                action_taken = True

            # --- PRESET READY / HOME POSE (H) ---
            elif key in ['h', 'H']:
                last_action = "Moving smoothly to Default Ready Pose..."
                print_dashboard(current_positions, step_deg, torque_enabled, last_action)
                # Target L-Pose (Image reference)
                home_targets = {
                    11: round(current_positions[11] / 4096.0) * 4096, # Forward
                    12: 2048, # Shoulder Vertical Up
                    13: 1100, # Elbow ~90 deg Forward
                    14: 2048, # Wrist Level Forward
                    15: 1800  # Gripper Open
                }
                for dxl_id in [15, 14, 13, 12, 11]:
                    t_pos = home_targets[dxl_id]
                    arm.write_position(dxl_id, t_pos)
                    current_positions[dxl_id] = t_pos
                last_action = "Default Ready Pose Reached!"
                action_taken = True

            # --- STEP SIZE ADJUSTMENT ---
            elif key == '1':
                step_deg = 1.0
                last_action = "Step size set to 1.0°"
                action_taken = True
            elif key == '2':
                step_deg = 2.5
                last_action = "Step size set to 2.5°"
                action_taken = True
            elif key == '3':
                step_deg = 5.0
                last_action = "Step size set to 5.0°"
                action_taken = True

            # If torque is disabled, do not process joint movements
            elif not torque_enabled:
                last_action = "Torque is OFF! Press 'T' to enable torque."
                action_taken = True

            # --- JOINT 1: BASE PAN (A / D) ---
            elif key in ['a', 'A', 'LEFT']:
                target = current_positions[11] - raw_step
                min_lim, max_lim = JOINT_LIMITS[11]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(11, target)
                current_positions[11] = target
                last_action = f"Base Left -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            elif key in ['d', 'D', 'RIGHT']:
                target = current_positions[11] + raw_step
                min_lim, max_lim = JOINT_LIMITS[11]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(11, target)
                current_positions[11] = target
                last_action = f"Base Right -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            # --- JOINT 2: SHOULDER PITCH (W / S) ---
            elif key in ['w', 'W', 'UP']:
                target = current_positions[12] - raw_step
                min_lim, max_lim = JOINT_LIMITS[12]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(12, target)
                current_positions[12] = target
                last_action = f"Shoulder Up -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            elif key in ['s', 'S', 'DOWN']:
                target = current_positions[12] + raw_step
                min_lim, max_lim = JOINT_LIMITS[12]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(12, target)
                current_positions[12] = target
                last_action = f"Shoulder Down -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            # --- JOINT 3: ELBOW PITCH (I / K) ---
            elif key in ['i', 'I']:
                target = current_positions[13] - raw_step
                min_lim, max_lim = JOINT_LIMITS[13]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(13, target)
                current_positions[13] = target
                last_action = f"Elbow Up -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            elif key in ['k', 'K']:
                target = current_positions[13] + raw_step
                min_lim, max_lim = JOINT_LIMITS[13]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(13, target)
                current_positions[13] = target
                last_action = f"Elbow Down -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            # --- JOINT 4: WRIST PITCH (U / J) ---
            elif key in ['u', 'U']:
                target = current_positions[14] - raw_step
                min_lim, max_lim = JOINT_LIMITS[14]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(14, target)
                current_positions[14] = target
                last_action = f"Wrist Up -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            elif key in ['j', 'J']:
                target = current_positions[14] + raw_step
                min_lim, max_lim = JOINT_LIMITS[14]
                target = max(min_lim, min(max_lim, target))
                arm.write_position(14, target)
                current_positions[14] = target
                last_action = f"Wrist Down -> {(target-2048)*POSITION_UNIT_TO_DEG:+.1f}°"
                action_taken = True

            # --- JOINT 5: GRIPPER (O / P / SPACE) ---
            elif key in ['o', 'O']:
                # Open gripper (approx 1600 counts)
                target = 1600
                arm.write_position(15, target)
                current_positions[15] = target
                gripper_open = True
                last_action = "Gripper Opened"
                action_taken = True

            elif key in ['p', 'P']:
                # Close gripper (approx 2400 counts)
                target = 2400
                arm.write_position(15, target)
                current_positions[15] = target
                gripper_open = False
                last_action = "Gripper Closed"
                action_taken = True

            elif key == ' ': # SPACE toggle
                if gripper_open:
                    target = 2400
                    gripper_open = False
                    last_action = "Gripper Closed (Toggled)"
                else:
                    target = 1600
                    gripper_open = True
                    last_action = "Gripper Opened (Toggled)"
                arm.write_position(15, target)
                current_positions[15] = target
                action_taken = True

            if action_taken:
                # Update dashboard
                print_dashboard(current_positions, step_deg, torque_enabled, last_action)

    finally:
        print("\n\n[Shutdown] Disabling all motor torque and closing port...")
        arm.disable_all_torque()
        arm.disconnect()
        print("Done. Goodbye!")

if __name__ == "__main__":
    main()

