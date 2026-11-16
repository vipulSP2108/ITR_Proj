#!/usr/bin/env python3
"""
Hardware Connection & Diagnostic Monitor
Checks:
  1. U2D2 Serial Interface (/dev/cu.usbserial-*)
  2. Dynamixel XM430-W350 Actuators (IDs 11-15) + Input Voltage
  3. RealSense / UVC Camera Stream + Live Image Capture with HUD
Overwrites in-place:
  - hardware_status.txt  (zero size accumulation)
  - live_camera.jpg      (real-time annotated image)
"""

import os
import sys
import time
import argparse
import datetime
import cv2
import serial.tools.list_ports

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("[Error] dynamixel_sdk not found. Run: pip install dynamixel-sdk")
    sys.exit(1)

# Hardware Constants
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
ADDR_PRESENT_POSITION = 132    # 4 bytes
ADDR_PRESENT_INPUT_VOLTAGE = 144 # 2 bytes (0.1V units)
POSITION_UNIT_TO_DEG = 360.0 / 4096.0

JOINTS = {
    11: "Joint 1 (Base Pan)",
    12: "Joint 2 (Shoulder Pitch)",
    13: "Joint 3 (Elbow Pitch)",
    14: "Joint 4 (Wrist Pitch)",
    15: "Joint 5 (Gripper)"
}

STATUS_TXT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "hardware_status.txt"))
LIVE_IMG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "live_camera.jpg"))


def find_u2d2_port():
    """Locate U2D2 adapter on macOS."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        dev = p.device
        if ("usbserial" in dev or "FTDI" in (p.description or "")) and dev.startswith("/dev/cu."):
            return dev
    for p in ports:
        if "usbserial" in p.device and p.device.startswith("/dev/tty."):
            return p.device
    return None


def scan_dynamixel(port_name):
    """Scan all 5 OpenMANIPULATOR-X motors."""
    if not port_name:
        return {"connected": False, "port": None, "motors": {}, "power_detected": False}

    portHandler = PortHandler(port_name)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        return {"connected": False, "port": port_name, "motors": {}, "power_detected": False}

    results = {}
    found_count = 0
    try:
        for dxl_id, joint_name in JOINTS.items():
            model, comm_result, dxl_err = packetHandler.ping(portHandler, dxl_id)
            if comm_result == COMM_SUCCESS:
                # Read position
                raw_pos, comm_res_pos, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
                deg_pos = (raw_pos - 2048) * POSITION_UNIT_TO_DEG if comm_res_pos == COMM_SUCCESS else None

                # Read voltage
                raw_volt, comm_res_v, _ = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE)
                voltage_v = raw_volt / 10.0 if comm_res_v == COMM_SUCCESS else None

                results[dxl_id] = {
                    "name": joint_name,
                    "model": model,
                    "online": True,
                    "raw_pos": raw_pos if comm_res_pos == COMM_SUCCESS else None,
                    "deg_pos": deg_pos,
                    "voltage_v": voltage_v
                }
                found_count += 1
            else:
                results[dxl_id] = {
                    "name": joint_name,
                    "model": None,
                    "online": False,
                    "raw_pos": None,
                    "deg_pos": None,
                    "voltage_v": None
                }
    finally:
        portHandler.closePort()

    return {
        "connected": True,
        "port": port_name,
        "motors": results,
        "power_detected": (found_count > 0)
    }


def scan_camera():
    """Detect and capture frame from the camera."""
    # Try index 0 first (RealSense D435), fallback to 1
    for idx in [0, 1]:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                return {"index": idx, "available": True, "frame": frame, "width": frame.shape[1], "height": frame.shape[0]}
    return {"index": None, "available": False, "frame": None, "width": 0, "height": 0}


def generate_status_report(u2d2_info, cam_info):
    """Generate in-place formatted text report."""
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "================================================================================",
        f"        ROBOTIS OPENMANIPULATOR-X & REALSENSE DIAGNOSTIC STATUS",
        f"        Timestamp: {now}",
        "================================================================================",
        "",
        "--- [1] SERIAL ADAPTER (U2D2) ---",
        f"  Status        : {'CONNECTED' if u2d2_info['connected'] else 'NOT DETECTED'}",
        f"  Port Device   : {u2d2_info['port'] if u2d2_info['port'] else 'None'}",
        f"  Baud Rate     : {BAUDRATE:,} bps",
        f"  Protocol      : {PROTOCOL_VERSION}",
        "",
        "--- [2] DYNAMIXEL SERVOS (XM430-W350) ---",
    ]

    all_online = True
    if not u2d2_info['connected']:
        lines.append("  [!] U2D2 Serial interface not detected. Check USB connection to Mac.")
        all_online = False
    elif not u2d2_info['power_detected']:
        lines.append("  [!] U2D2 is connected, but NO motors responded to ping.")
        lines.append("      -> 12V Power Supply: Check if 12V wall adapter is connected.")
        lines.append("      -> Power Hub Switch: Check if robot power switch is toggled ON.")
        lines.append("      -> TTL Cable: Check 3-pin cable between U2D2 and Motor ID 11.")
        all_online = False
    else:
        for dxl_id, data in u2d2_info['motors'].items():
            if data['online']:
                deg_str = f"{data['deg_pos']:+6.1f} deg" if data['deg_pos'] is not None else "N/A"
                volt_str = f"{data['voltage_v']:.1f} V" if data['voltage_v'] is not None else "N/A"
                lines.append(f"  [OK] ID {dxl_id:2d} | {data['name']:25s} | Pos: {deg_str} | Volt: {volt_str}")
            else:
                lines.append(f"  [!!] ID {dxl_id:2d} | {data['name']:25s} | OFFLINE (Check daisy-chain cable)")
                all_online = False

    lines.extend([
        "",
        "--- [3] VISION CAMERA (INTEL REALSENSE D435) ---",
        f"  Status        : {'ACTIVE & STREAMING' if cam_info['available'] else 'NOT DETECTED'}",
        f"  Camera Index  : {cam_info['index'] if cam_info['index'] is not None else 'None'}",
        f"  Resolution    : {cam_info['width']} x {cam_info['height']} px" if cam_info['available'] else "  Resolution    : N/A",
        f"  Live Snapshot : {LIVE_IMG_PATH if cam_info['available'] else 'None'}",
        "",
        "================================================================================",
        f"SYSTEM READINESS: {'READY FOR HARDWARE EXECUTION (ALL PASS)' if (all_online and cam_info['available']) else 'WAITING FOR HARDWARE CONNECTION'}",
        "================================================================================",
        ""
    ])
    return "\n".join(lines), all_online


def annotate_frame(frame, u2d2_info, cam_info, all_online):
    """Draw professional HUD diagnostic overlay on frame."""
    hud = frame.copy()
    h, w = hud.shape[:2]

    # Semi-transparent top bar
    cv2.rectangle(hud, (0, 0), (w, 42), (25, 25, 28), -1)
    # Semi-transparent side panel
    cv2.rectangle(hud, (10, 52), (290, 260), (25, 25, 28), -1)

    # Blend overlay
    cv2.addWeighted(hud, 0.85, frame, 0.15, 0, frame)

    # Top Bar Text
    stamp = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(frame, f"OPENMANIPULATOR-X LIVE MONITOR | {stamp}", (16, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

    status_color = (60, 220, 80) if all_online else (40, 80, 240)
    status_text = "HARDWARE READY" if all_online else "ATTENTION NEEDED"
    cv2.putText(frame, status_text, (w - 180, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2, cv2.LINE_AA)

    # Side Panel - Components
    cv2.putText(frame, "DIAGNOSTIC STATUS:", (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

    # U2D2
    u2d2_ok = u2d2_info['connected']
    cv2.circle(frame, (28, 98), 5, (60, 220, 80) if u2d2_ok else (40, 60, 240), -1)
    cv2.putText(frame, f"U2D2: {'OK' if u2d2_ok else 'MISSING'}", (42, 102),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)

    # Motors 11-15
    y_offset = 124
    if not u2d2_info['power_detected']:
        cv2.circle(frame, (28, y_offset), 5, (40, 60, 240), -1)
        cv2.putText(frame, "12V Power: OFF / DISCONNECTED", (42, y_offset + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 100, 255), 1, cv2.LINE_AA)
        y_offset += 24
        cv2.putText(frame, "-> Turn ON robot power switch", (42, y_offset + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 180, 255), 1, cv2.LINE_AA)
        y_offset += 22
        cv2.putText(frame, "-> Check 12V DC power plug", (42, y_offset + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 180, 255), 1, cv2.LINE_AA)
    else:
        for dxl_id in range(11, 16):
            m_data = u2d2_info['motors'].get(dxl_id, {})
            m_ok = m_data.get('online', False)
            cv2.circle(frame, (28, y_offset), 5, (60, 220, 80) if m_ok else (40, 60, 240), -1)
            deg_txt = f"{m_data.get('deg_pos', 0.0):+.1f} deg" if m_ok and m_data.get('deg_pos') is not None else "OFFLINE"
            cv2.putText(frame, f"ID {dxl_id}: {deg_txt}", (42, y_offset + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 240, 240) if m_ok else (60, 100, 255), 1, cv2.LINE_AA)
            y_offset += 22

    # Camera status
    cv2.circle(frame, (28, 240), 5, (60, 220, 80) if cam_info['available'] else (40, 60, 240), -1)
    cv2.putText(frame, f"RealSense: {cam_info['width']}x{cam_info['height']} px", (42, 244),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 240, 240), 1, cv2.LINE_AA)

    return frame


def run_once(gui=False):
    """Execute a single diagnostic scan and overwrite files."""
    port = find_u2d2_port()
    u2d2_info = scan_dynamixel(port)
    cam_info = scan_camera()

    # Generate status report
    report_text, all_online = generate_status_report(u2d2_info, cam_info)

    # Overwrite hardware_status.txt in-place (atomic write, zero size accumulation)
    with open(STATUS_TXT_PATH, "w") as f:
        f.write(report_text)

    # Save live camera snapshot with HUD
    if cam_info['available'] and cam_info['frame'] is not None:
        annotated = annotate_frame(cam_info['frame'], u2d2_info, cam_info, all_online)
        cv2.imwrite(LIVE_IMG_PATH, annotated)
        if gui:
            cv2.imshow("OpenMANIPULATOR-X Hardware Check", annotated)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:
                return False

    # Print to stdout
    print(report_text)
    print(f"-> Overwritten in-place: {STATUS_TXT_PATH}")
    if cam_info['available']:
        print(f"-> Overwritten in-place: {LIVE_IMG_PATH}")
    return True


def main():
    parser = argparse.ArgumentParser(description="OpenMANIPULATOR-X & RealSense Connection Monitor")
    parser.add_argument("--watch", "-w", action="store_true", help="Continuously poll and update status every 2 seconds")
    parser.add_argument("--gui", "-g", action="store_true", help="Display interactive camera window with HUD")
    parser.add_argument("--interval", "-i", type=float, default=2.0, help="Poll interval in seconds (default: 2.0)")
    args = parser.parse_args()

    print("[Diagnostic Monitor Initialized]")
    if args.watch or args.gui:
        print(f"Watching connections every {args.interval:.1f}s. Press Ctrl+C or 'q' to stop.\n")
        try:
            while True:
                # Clear terminal screen for clean live dashboard
                os.system('cls' if os.name == 'nt' else 'clear')
                cont = run_once(gui=args.gui)
                if not cont:
                    break
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n[Monitor Stopped]")
        finally:
            if args.gui:
                cv2.destroyAllWindows()
    else:
        run_once(gui=args.gui)


if __name__ == "__main__":
    main()

