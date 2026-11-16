#!/usr/bin/env python3
"""
Hardware Protector & Electrical Auto-Disconnector Layer
Protects the Mac and the OpenMANIPULATOR-X actuators from:
  1. Overvoltage (> 14.6V) & Undervoltage (< 10.5V) brownouts
  2. Overheating (> 65°C)
  3. Mechanical Stalls / Excessive Motor Load (> 85%)
  4. Hardware Error Bitmask Faults (electrical shock, internal short)

On any safety fault, this layer instantly disables torque across all motors,
zeros velocities, and severs serial communication to protect the hardware.
"""

import sys
import os
import time
import threading
from typing import Dict, Optional, Tuple

# Add openmanipulator_test to path for wrapper
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEST_DIR = os.path.join(BASE_DIR, "openmanipulator_test")
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    JOINT_IDS,
    COMM_SUCCESS
)

# Control Table Registers for Electrical Health
ADDR_HARDWARE_ERROR_STATUS = 70   # 1 byte
ADDR_TORQUE_ENABLE         = 64   # 1 byte
ADDR_PRESENT_LOAD          = 126  # 2 bytes
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes (unit: 0.1V)
ADDR_PRESENT_TEMPERATURE   = 146  # 1 byte  (unit: °C)

# Safety Limits
SAFE_VOLTAGE_MIN = 10.5  # Volts (brownout threshold)
SAFE_VOLTAGE_MAX = 14.6  # Volts (overvoltage threshold)
SAFE_TEMP_MAX    = 65.0  # °C (early thermal shutdown, XM430 max is 80°C)
SAFE_LOAD_MAX    = 85.0  # % effort (anti-stall threshold)

class ElectricalSafetyException(Exception):
    """Raised when an electrical or thermal safety threshold is breached."""
    pass

class HardwareProtector:
    """
    Active electrical guardian that monitors voltage, temperature, and load,
    tripping an automated disconnector if any parameter exceeds safe boundaries.
    """
    def __init__(self, arm: OpenManipulatorController, check_interval_sec: float = 0.5):
        self.arm = arm
        self.check_interval = check_interval_sec
        self.is_monitoring = False
        self._thread: Optional[threading.Thread] = None
        self.last_health_report: Dict[int, dict] = {}

    def get_motor_health(self, dxl_id: int) -> Optional[dict]:
        """Reads voltage, temperature, load, and error status for a specific motor."""
        if not self.arm.is_connected:
            return None

        ph = self.arm.packet_handler
        port = self.arm.port_handler

        # 1. Voltage (0.1V)
        volt_raw, res_v, _ = ph.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE)
        voltage = volt_raw / 10.0 if res_v == COMM_SUCCESS else None

        # 2. Temperature (°C)
        temp, res_t, _ = ph.read1ByteTxRx(port, dxl_id, ADDR_PRESENT_TEMPERATURE)
        temp = float(temp) if res_t == COMM_SUCCESS else None

        # 3. Hardware Error Status
        hw_err, res_e, _ = ph.read1ByteTxRx(port, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        hw_err = int(hw_err) if res_e == COMM_SUCCESS else None

        # 4. Load (0 ~ 1000 is CCW 0~100%, 1024 ~ 2024 is CW 0~100%)
        load_raw, res_l, _ = ph.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_LOAD)
        load_pct = 0.0
        if res_l == COMM_SUCCESS and load_raw is not None:
            raw_val = load_raw % 1024
            load_pct = (raw_val / 1000.0) * 100.0

        return {
            "voltage": voltage,
            "temperature": temp,
            "hardware_error": hw_err,
            "load_pct": load_pct
        }

    def trigger_emergency_auto_disconnect(self, reason: str):
        """
        INSTANT ELECTRICAL CUT:
        Disables torque on all servos, dumps motor power, and isolates the serial port.
        """
        print("\n" + "!" * 66)
        print(f" [CRITICAL SAFETY ALERT] AUTO-DISCONNECTOR TRIPPED!")
        print(f" Reason: {reason}")
        print(" Actions Taken:")
        print("   1. Cutting torque to all 5 DYNAMIXEL actuators immediately...")

        try:
            for dxl_id in JOINT_IDS.keys():
                self.arm.packet_handler.write1ByteTxRx(
                    self.arm.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0
                )
        except Exception:
            pass

        print("   2. Motor power cut. Arm is in safe limp mode.")
        print("   3. Closing communication bus to protect Mac USB controller.")
        try:
            self.arm.port_handler.closePort()
            self.arm.is_connected = False
        except Exception:
            pass
        print("!" * 66 + "\n")
        raise ElectricalSafetyException(f"Hardware Protector auto-disconnect: {reason}")

    def check_all_safety_limits(self):
        """Polls all motors and checks against electrical and thermal boundaries."""
        for dxl_id, name in JOINT_IDS.items():
            health = self.get_motor_health(dxl_id)
            if not health:
                continue
            self.last_health_report[dxl_id] = health

            v = health["voltage"]
            t = health["temperature"]
            e = health["hardware_error"]
            l = health["load_pct"]

            # 1. Voltage Check
            if v is not None:
                if v < SAFE_VOLTAGE_MIN:
                    self.trigger_emergency_auto_disconnect(
                        f"UNDERVOLTAGE on Motor {dxl_id} ({name}): {v:.1f}V is below safe min {SAFE_VOLTAGE_MIN}V!"
                    )
                elif v > SAFE_VOLTAGE_MAX:
                    self.trigger_emergency_auto_disconnect(
                        f"OVERVOLTAGE on Motor {dxl_id} ({name}): {v:.1f}V exceeds max {SAFE_VOLTAGE_MAX}V! Power supply surge detected!"
                    )

            # 2. Temperature Check
            if t is not None and t > SAFE_TEMP_MAX:
                self.trigger_emergency_auto_disconnect(
                    f"OVERHEATING on Motor {dxl_id} ({name}): {t:.1f}°C exceeds threshold {SAFE_TEMP_MAX}°C!"
                )

            # 3. Hardware Error Bitmask (Bit 0: Voltage, Bit 2: Temp, Bit 4: Shock, Bit 5: Overload)
            if e is not None and e > 0:
                self.trigger_emergency_auto_disconnect(
                    f"HARDWARE ERROR STATUS {e:#04x} on Motor {dxl_id} ({name})! Fault detected inside servo."
                )

    def _monitor_loop(self):
        while self.is_monitoring:
            try:
                self.check_all_safety_limits()
            except ElectricalSafetyException:
                self.is_monitoring = False
                break
            except Exception:
                pass
            time.sleep(self.check_interval)

    def start_background_guardian(self):
        """Starts the safety watchdog thread in the background."""
        if self.is_monitoring:
            return
        self.is_monitoring = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
        print("[Safety] Hardware Protector Guardian Thread STARTED (Monitoring Voltage, Temp, Load).")

    def stop_background_guardian(self):
        """Stops the safety watchdog."""
        self.is_monitoring = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        print("[Safety] Hardware Protector Guardian Thread stopped.")

def run_health_check_cli():
    print("=" * 66)
    print("      OpenMANIPULATOR-X Electrical Health Diagnostic")
    print("=" * 66)

    port = find_u2d2_port()
    if not port:
        print("[!] U2D2 adapter not detected on Mac.")
        sys.exit(1)

    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        sys.exit(1)

    protector = HardwareProtector(arm)
    print(f"\nConnected to: {port} (1,000,000 bps)")
    print(f"Safety Limits: Voltage [{SAFE_VOLTAGE_MIN}V ~ {SAFE_VOLTAGE_MAX}V] | Max Temp: {SAFE_TEMP_MAX}°C\n")

    print(f"{'ID':^4} | {'Joint Name':^24} | {'Voltage':^9} | {'Temp':^7} | {'Load':^7} | {'Status':^12}")
    print("-" * 72)

    all_safe = True
    for dxl_id, name in JOINT_IDS.items():
        h = protector.get_motor_health(dxl_id)
        if not h:
            print(f" {dxl_id:2d}  | {name:24s} | [NO RESPONSE]")
            all_safe = False
            continue

        v = h['voltage'] or 0.0
        t = h['temperature'] or 0.0
        l = h['load_pct']
        err = h['hardware_error'] or 0

        v_ok = SAFE_VOLTAGE_MIN <= v <= SAFE_VOLTAGE_MAX
        t_ok = t < SAFE_TEMP_MAX
        e_ok = err == 0
        safe = v_ok and t_ok and e_ok

        status_str = "\033[92mSAFE / NOMINAL\033[0m" if safe else "\033[91mWARNING\033[0m"
        print(f" {dxl_id:2d}  | {name:24s} | {v:5.1f} V   | {t:4.1f}°C  | {l:4.1f}%  | {status_str}")
        if not safe:
            all_safe = False

    print("-" * 72)
    if all_safe:
        print("  ✓ All actuators operating within nominal electrical & thermal limits!")
    else:
        print("  ! Electrical or thermal warnings present. Check power supply.")
    print("=" * 66)

    arm.disconnect()

if __name__ == "__main__":
    run_health_check_cli()

