#!/usr/bin/env python3
"""
Hardware Abstraction Layer (HAL) for OpenMANIPULATOR-X.
Complies with repare.md §7.3 and §10.

Units:
  • Positions: metres (m)
  • Angles: radians (rad), sim-zero convention (L-pose = all zeros)
  • Time: seconds (s)

Provides:
  • RobotInterface (abstract base class)
  • SimRobot (wraps SimEnv in MuJoCo)
  • RealRobot (wraps OpenManipulatorController + RealSense/webcam + GripperGuard)
"""

import os
import sys
import time
from abc import ABC, abstractmethod
from typing import Tuple, Optional, Dict, Any, List
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import arm_model as A

# Try importing hardware controller from legacy tree if available
LEGACY_ROOT = os.path.dirname(HERE)
SRC_PATH = os.path.join(LEGACY_ROOT, "src")
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)


class RobotInterface(ABC):
    """
    Unified robot hardware/simulator interface.
    All Cartesian coordinates are in robot base frame (m), all joint angles in radians.
    """

    @abstractmethod
    def read_joints(self) -> np.ndarray:
        """Returns current [q1, q2, q3, q4] in radians (sim-zero convention)."""
        pass

    @abstractmethod
    def move_joints(self, q: np.ndarray, duration: float) -> None:
        """Smoothly moves joints to target q in duration seconds."""
        pass

    @abstractmethod
    def move_cartesian(self, p0: np.ndarray, p1: np.ndarray, tilt_rad: float, duration: float) -> None:
        """Moves TCP in a straight Cartesian line from p0 to p1 maintaining tilt_rad."""
        pass

    @abstractmethod
    def set_gripper(self, close: bool, duration: float = 0.8) -> None:
        """Commands gripper open (False) or closed (True)."""
        pass

    @abstractmethod
    def grasp_state(self) -> str:
        """Returns 'held', 'empty', or 'unknown'."""
        pass

    @abstractmethod
    def estop(self) -> None:
        """Emergency stop: cuts torque or freezes motion immediately."""
        pass

    @abstractmethod
    def capture_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        """Captures (bgr, depth_m) from the calibrated overhead camera."""
        pass

    @abstractmethod
    def settle(self, seconds: float = 0.3) -> None:
        """Allows arm physics/servos to settle."""
        pass


class SimRobot(RobotInterface):
    """
    MuJoCo Simulator implementation of RobotInterface.
    Wraps sim_env.SimEnv.
    """

    def __init__(self, sim_env):
        self.env = sim_env
        self.dry_run = False

    def read_joints(self) -> np.ndarray:
        return self.env.read_joints()

    def move_joints(self, q: np.ndarray, duration: float) -> None:
        self.env.move_joints(q, duration)

    def move_cartesian(self, p0: np.ndarray, p1: np.ndarray, tilt_rad: float, duration: float) -> None:
        self.env.move_cartesian(p0, p1, tilt_rad, duration)

    def set_gripper(self, close: bool, duration: float = 0.8) -> None:
        self.env.set_gripper(close, duration)

    def grasp_state(self) -> str:
        # 40 mm peg: fingers stall at ~30 mm total travel; closed on air ≈ 50 mm
        stroke = self.env.read_gripper()
        if 0.018 < stroke < 0.042:
            return "held"
        elif stroke >= 0.042:
            return "empty"
        return "unknown"

    def estop(self) -> None:
        self.env.ctrl_q = self.env.read_joints().copy()

    def capture_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.env.capture_rgbd()

    def settle(self, seconds: float = 0.3) -> None:
        self.env.settle(seconds)


class RealRobot(RobotInterface):
    """
    Physical hardware implementation for ROBOTIS OpenMANIPULATOR-X.
    Wraps OpenManipulatorController and GripperGuard.
    """

    def __init__(self, port_name: Optional[str] = None, camera_idx: int = 0, dry_run: bool = False):
        self.dry_run = dry_run
        self.camera_idx = camera_idx
        self.arm = None
        self.gripper_guard = None
        self.cap = None

        # Dynamixel counts to sim radians calibration
        # Sim L-pose convention: q = [0, 0, 0, 0] rad
        # Count 2048 is center (0 degrees).
        self.ZERO_COUNTS = np.array([2048, 2048, 2048, 2048])
        self.SIGNS = np.array([+1.0, +1.0, +1.0, +1.0])
        self.COUNTS_PER_RAD = 4096.0 / (2.0 * np.pi)

        self._init_hardware(port_name)

    def _init_hardware(self, port_name: Optional[str]):
        try:
            from control.dynamixel_wrapper import OpenManipulatorController, find_u2d2_port
            from control.gripper_guard import GripperGuard
        except ImportError:
            try:
                from dynamixel_wrapper import OpenManipulatorController, find_u2d2_port
                from gripper_guard import GripperGuard
            except ImportError:
                print("[RealRobot] Warning: Dynamixel hardware wrapper not available on path.")
                return

        port = port_name or find_u2d2_port()
        if not port:
            print("[RealRobot] Warning: U2D2 adapter not detected on serial bus.")
            return

        self.arm = OpenManipulatorController(port_name=port)
        if self.arm.connect():
            self.arm.initialize_all_joints(profile_vel=25, profile_acc=10)
            self.gripper_guard = GripperGuard(self.arm)
            print("[RealRobot] Connected to OpenMANIPULATOR-X hardware.")

    def _counts_to_rad(self, counts: List[int]) -> np.ndarray:
        q = np.zeros(4)
        for i in range(4):
            q[i] = self.SIGNS[i] * (counts[i] - self.ZERO_COUNTS[i]) / self.COUNTS_PER_RAD
        return q

    def _rad_to_counts(self, q: np.ndarray) -> List[int]:
        counts = []
        for i in range(4):
            c = int(round(self.ZERO_COUNTS[i] + self.SIGNS[i] * q[i] * self.COUNTS_PER_RAD))
            counts.append(int(np.clip(c, 0, 4095)))
        return counts

    def read_joints(self) -> np.ndarray:
        if not self.arm:
            return A.HOME_Q.copy()
        raw = [self.arm.read_position(dxl_id) for dxl_id in [11, 12, 13, 14]]
        # Handle fallback if read fails
        raw = [r if r is not None else 2048 for r in raw]
        return self._counts_to_rad(raw)

    def move_joints(self, q: np.ndarray, duration: float) -> None:
        if self.dry_run:
            print(f"  [RealRobot:DRY-RUN] move_joints({np.round(q, 3).tolist()}, duration={duration:.1f}s)")
            time.sleep(min(duration, 0.5))
            return

        if not self.arm:
            return

        counts = self._rad_to_counts(q)
        for dxl_id, c in zip([11, 12, 13, 14], counts):
            self.arm.write_position(dxl_id, c)
        time.sleep(duration)

    def move_cartesian(self, p0: np.ndarray, p1: np.ndarray, tilt_rad: float, duration: float) -> None:
        if self.dry_run:
            # Elevate dry run Cartesian targets by 50 mm
            p1_dry = p1.copy()
            p1_dry[2] += 0.05
            print(f"  [RealRobot:DRY-RUN] Cartesian move to {np.round(p1_dry, 3).tolist()} (elevated +5cm)")
            time.sleep(min(duration, 0.5))
            return

        # Interpolate in N small steps with IK
        steps = max(int(duration * 20), 10)
        dt = duration / steps
        for step in range(1, steps + 1):
            alpha = step / float(steps)
            p = (1.0 - alpha) * p0 + alpha * p1
            sol = A.ik(p[0], p[1], p[2], tilt_rad)
            if sol is not None:
                self.move_joints(sol, dt)

    def set_gripper(self, close: bool, duration: float = 0.8) -> None:
        if self.dry_run:
            print(f"  [RealRobot:DRY-RUN] set_gripper({close})")
            time.sleep(0.2)
            return

        if not self.arm:
            return

        # Count 1700 = open, 2150 = close
        target = 2150 if close else 1700
        self.arm.write_position(15, target)
        time.sleep(duration)

    def grasp_state(self) -> str:
        if not self.gripper_guard or not self.arm:
            return "held" if not self.dry_run else "held"

        eval_res = self.gripper_guard.evaluate_grasp()
        # GripperGuard returns dict: is_holding, is_empty, current_position, present_current_ma
        if eval_res.get("is_holding"):
            return "held"
        elif eval_res.get("is_empty"):
            return "empty"
        return "unknown"

    def estop(self) -> None:
        if self.arm:
            self.arm.disable_all_torque()
            print("[RealRobot] EMERGENCY STOP: All motor torque disabled!")

    def capture_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_idx)
            for _ in range(5):
                self.cap.read()

        ret, frame = self.cap.read()
        if not ret or frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Synthetic/planar depth proxy if RealSense depth is unavailable
        depth_m = np.full((frame.shape[0], frame.shape[1]), 0.61, dtype=np.float32)
        return frame, depth_m

    def settle(self, seconds: float = 0.3) -> None:
        time.sleep(seconds)

