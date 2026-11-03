#!/usr/bin/env python3
"""
Step 1: Scan & Ping Test
Detects U2D2 on macOS, connects, pings all 5 OpenMANIPULATOR-X motors (IDs 11-15),
and prints their present positions and degrees.
"""

import sys
import serial.tools.list_ports
from dynamixel_wrapper import (
    OpenManipulatorController,
    JOINT_IDS,
    POSITION_UNIT_TO_DEG,
    find_u2d2_port
)

def main():
    print("=" * 60)
    print("  OpenMANIPULATOR-X macOS Diagnostic: Scan & Ping")
    print("=" * 60)

    # 1. List all detected serial ports on macOS
    print("\n[Step 1] Scanning Mac serial ports...")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  [!] No serial ports found at all.")
    else:
        for p in ports:
            print(f"  • Device: {p.device:32s} | Description: {p.description}")

    detected_port = find_u2d2_port()
    if not detected_port:
        print("\n[!] ROBOTIS U2D2 not detected on your Mac.")
        print("    -> Plug the U2D2 USB cable into your Mac.")
        print("    -> Ensure the 12V power supply is plugged into the robot power hub.")
        sys.exit(1)

    print(f"\n[Step 2] Detected U2D2 on: {detected_port}")
    arm = OpenManipulatorController(port_name=detected_port)

    if not arm.connect():
        print("\n[!] Connection failed.")
        print("    -> Check if DYNAMIXEL Wizard is currently open and holding the port.")
        sys.exit(1)

    try:
        print("\n[Step 3] Pinging OpenMANIPULATOR-X Servos (Protocol 2.0)...")
        found_motors = 0
        for dxl_id, joint_name in JOINT_IDS.items():
            success, model_num = arm.ping_motor(dxl_id)
            if success:
                raw_pos = arm.read_position(dxl_id)
                deg_pos = (raw_pos - 2048) * POSITION_UNIT_TO_DEG if raw_pos is not None else 0.0
                print(f"  ✓ ID {dxl_id:2d} | {joint_name:25s} | Model #{model_num} | Raw: {raw_pos} | Deg: {deg_pos:+.1f}°")
                found_motors += 1
            else:
                print(f"  ✗ ID {dxl_id:2d} | {joint_name:25s} | [NOT RESPONDING - Check cable/power]")

        print("\n" + "-" * 60)
        if found_motors == 5:
            print(f"  [SUCCESS] All 5 actuators successfully detected and responding on macOS!")
        elif found_motors > 0:
            print(f"  [PARTIAL] Detected {found_motors}/5 actuators. Check cables between non-responsive joints.")
        else:
            print(f"  [FAILED] 0 actuators detected. Check 12V power switch on robot.")
        print("-" * 60)

    finally:
        arm.disconnect()

if __name__ == "__main__":
    main()

