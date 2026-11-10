import sys
import time
import serial.tools.list_ports
from typing import List, Optional, Tuple
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)

# OpenMANIPULATOR-X uses DYNAMIXEL XM430-W350-T actuators (Protocol 2.0)
PROTOCOL_VERSION = 2.0
DEFAULT_BAUDRATE = 1000000 # 1 Mbps default for OpenMANIPULATOR-X

# Control Table Addresses (XM430-W350-T / X-Series)
ADDR_OPERATING_MODE       = 11   # 1 byte  (3: Position Control, 4: Extended Multi-turn Position)
ADDR_TORQUE_ENABLE        = 64   # 1 byte  (1: Enable, 0: Disable)
ADDR_PROFILE_ACCELERATION = 108  # 4 bytes (Controls motion acceleration)
ADDR_PROFILE_VELOCITY     = 112  # 4 bytes (Controls max motion speed)
ADDR_GOAL_POSITION        = 116  # 4 bytes (Target position: 0 ~ 4095)
ADDR_PRESENT_POSITION     = 132  # 4 bytes (Current position: 0 ~ 4095)
ADDR_PRESENT_LOAD         = 126  # 2 bytes (Current motor effort/load)

# Standard OpenMANIPULATOR-X Joint IDs
JOINT_IDS = {
    11: "Joint 1 (Base Pan)",
    12: "Joint 2 (Shoulder Pitch)",
    13: "Joint 3 (Elbow Pitch)",
    14: "Joint 4 (Wrist Pitch)",
    15: "Joint 5 (Gripper)"
}

# Unit conversions for XM430 (4096 counts = 360 degrees = 0.08789 deg/count)
POSITION_UNIT_TO_DEG = 360.0 / 4096.0
DEG_TO_POSITION_UNIT = 4096.0 / 360.0

def find_u2d2_port() -> Optional[str]:
    """
    Auto-detects the ROBOTIS U2D2 / FTDI USB serial adapter on macOS.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # On macOS, FTDI devices show up as /dev/cu.usbserial-* or mention FTDI / U2D2
        if "usbserial" in port.device.lower() or "ftdi" in (port.description or "").lower():
            return port.device
    # If no obvious usbserial device, return the first valid USB port if any
    for port in ports:
        if "cu.usb" in port.device:
            return port.device
    return None

class OpenManipulatorController:
    """
    Safe and gentle controller interface for OpenMANIPULATOR-X using DYNAMIXEL SDK on macOS.
    """
    def __init__(self, port_name: Optional[str] = None, baudrate: int = DEFAULT_BAUDRATE):
        self.port_name = port_name or find_u2d2_port()
        self.baudrate = baudrate
        self.is_connected = False

        if not self.port_name:
            print("[Warning] No U2D2 USB adapter detected on macOS yet.")
            print("  Make sure the U2D2 USB cable is plugged into your Mac.")
        else:
            print(f"[Info] Found U2D2 on port: {self.port_name}")

        self.port_handler = PortHandler(self.port_name) if self.port_name else None
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

    def connect(self) -> bool:
        """Opens serial port and sets baudrate."""
        if not self.port_name or not self.port_handler:
            self.port_name = find_u2d2_port()
            if not self.port_name:
                print("[Error] Cannot connect: U2D2 adapter not detected.")
                return False
            self.port_handler = PortHandler(self.port_name)

        if not self.port_handler.openPort():
            print(f"[Error] Failed to open port: {self.port_name}")
            print("  Check Mac permissions or close any other app using this port (e.g. DYNAMIXEL Wizard).")
            return False

        if not self.port_handler.setBaudRate(self.baudrate):
            print(f"[Error] Failed to set baudrate to {self.baudrate}")
            self.port_handler.closePort()
            return False

        self.is_connected = True
        print(f"[Success] Connected to U2D2 at {self.port_name} ({self.baudrate} bps)")
        return True

    def disconnect(self, disable_torque: bool = False):
        """Closes the serial port. Keeps torque enabled by default to prevent arm falling."""
        if self.is_connected and self.port_handler:
            if disable_torque:
                self.disable_all_torque()
            self.port_handler.closePort()
            self.is_connected = False
            print("[Info] Disconnected from U2D2 (holding torque maintained).")

    def ping_motor(self, dxl_id: int) -> Tuple[bool, Optional[int]]:
        """Pings a specific motor ID. Returns (success, model_number)."""
        if not self.is_connected:
            return False, None
        model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, dxl_id)
        if dxl_comm_result == COMM_SUCCESS:
            return True, model_number
        return False, None

    def read_position(self, dxl_id: int) -> Optional[int]:
        """Reads current position in raw encoder units."""
        if not self.is_connected:
            return None
        pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION
        )
        if dxl_comm_result == COMM_SUCCESS:
            if dxl_id == 11 and pos >= 0x80000000:
                pos -= 0x100000000
            return pos
        return None

    def read_position_deg(self, dxl_id: int) -> Optional[float]:
        """Reads current position converted to degrees (relative to center 2048 = 0 deg)."""
        raw = self.read_position(dxl_id)
        if raw is not None:
            # 2048 is centered (0 deg), -180 deg to +180 deg
            return (raw - 2048) * POSITION_UNIT_TO_DEG
        return None

    def read_load(self, dxl_id: int) -> Optional[float]:
        """Reads current load as signed percentage (-100.0% to +100.0%)."""
        if not self.is_connected:
            return None
        load_raw, dxl_comm_result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_LOAD
        )
        if dxl_comm_result == COMM_SUCCESS:
            if load_raw >= 0x8000:
                load_raw -= 0x10000
            return round(load_raw * 0.1, 1)  # 0.1% unit
        return None

    def set_operating_mode(self, dxl_id: int, mode: int) -> bool:
        """
        Sets operating mode (3: Position Control, 4: Extended Multi-turn Position).
        Note: Torque must be disabled before changing operating mode.
        """
        if not self.is_connected:
            return False
        res, err = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, mode)
        return res == COMM_SUCCESS and err == 0

    def initialize_all_joints(self, profile_vel: int = 50, profile_acc: int = 15) -> bool:
        """
        Ensures all 5 joints are set to Position Control Mode, configures safe motion profiles,
        and enables motor torque.
        """
        if not self.is_connected:
            return False

        print("[Init] Configuring joint operating modes and velocity profiles...")
        for dxl_id in JOINT_IDS.keys():
            target_mode = 4 if dxl_id == 11 else 3
            cur_mode, res, _ = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE)
            if res != COMM_SUCCESS or cur_mode != target_mode:
                self.disable_torque(dxl_id)
                self.set_operating_mode(dxl_id, target_mode)
            self.set_speed_and_accel(dxl_id, profile_velocity=profile_vel, profile_accel=profile_acc)
            self.enable_torque(dxl_id)

        print("[Init] All 5 joints initialized in Position Control Mode with Torque ENABLED.")
        return True

    def set_speed_and_accel(self, dxl_id: int, profile_velocity: int = 40, profile_accel: int = 10) -> bool:
        """
        Sets smooth, gentle motion profile parameters so the motor never jerks or snaps.
        Default velocity 40 is very gentle and slow (~2.3 rpm).
        """
        if not self.is_connected:
            return False
        # Set acceleration
        self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_PROFILE_ACCELERATION, profile_accel)
        # Set velocity
        res, err = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, profile_velocity)
        return res == COMM_SUCCESS

    def enable_torque(self, dxl_id: int) -> bool:
        """Enables motor torque for a specific motor."""
        if not self.is_connected:
            return False
        res, err = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        return res == COMM_SUCCESS

    def disable_torque(self, dxl_id: int) -> bool:
        """Disables motor torque for a specific motor (becomes freely movable by hand)."""
        if not self.is_connected:
            return False
        res, err = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        return res == COMM_SUCCESS

    def disable_all_torque(self):
        """Safely disables torque on all 5 motors."""
        for dxl_id in JOINT_IDS.keys():
            self.disable_torque(dxl_id)

    def write_position(self, dxl_id: int, goal_position: int) -> bool:
        """Writes target raw position with hardware safety boundary check."""
        if not self.is_connected:
            return False
        if dxl_id == 11:
            # Allow signed 32-bit position for multi-turn mode 4, bounded to safe +/- 3000 counts (+/- 263 deg)
            goal_position = max(-3000, min(3000, int(round(goal_position))))
            val_to_write = goal_position & 0xFFFFFFFF
        else:
            goal_position = max(0, min(4095, int(round(goal_position))))
            val_to_write = goal_position
        res, err = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_POSITION, val_to_write
        )
        return res == COMM_SUCCESS

    def gentle_move_relative_deg(self, dxl_id: int, delta_deg: float, profile_vel: int = 30) -> bool:
        """
        Safely moves a joint by a small degree delta at a very slow, gentle speed.
        """
        current_pos = self.read_position(dxl_id)
        if current_pos is None:
            print(f"[Error] Could not read current position for motor {dxl_id}")
            return False

        delta_units = int(delta_deg * DEG_TO_POSITION_UNIT)
        target_pos = current_pos + delta_units

        # Set gentle velocity profile first
        self.set_speed_and_accel(dxl_id, profile_velocity=profile_vel, profile_accel=10)
        # Enable torque
        self.enable_torque(dxl_id)
        # Command position
        success = self.write_position(dxl_id, target_pos)
        return success

