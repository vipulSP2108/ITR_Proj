#!/usr/bin/env python3
"""
Gripper Guard & Slip/Drop Detection Engine for OpenMANIPULATOR-X.
Addresses Edge Cases:
  • EC-05: Object Slip / Drop during Transfer (User Case 1)
  • EC-08: Empty / False Grasp (Air Grasp)
  • EC-09: Socket Jamming / Edge Resistance during Insertion

Monitors proprioceptive motor feedback (position, current/load) in real-time.
"""

import os
import sys
import time
import math
from enum import Enum
from typing import Optional, Tuple, List, Callable, Dict

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
T1_DIR = os.path.dirname(os.path.dirname(CUR_DIR))
SRC_DIR = os.path.dirname(CUR_DIR)
for p in [T1_DIR, SRC_DIR, CUR_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

# DYNAMIXEL XM430-W350 Control Table & Gripper Characteristics
GRIPPER_ID = 15
ADDR_PRESENT_CURRENT = 126  # 2 bytes, signed, unit: ~2.69 mA
ADDR_PRESENT_POSITION = 132 # 4 bytes
ADDR_PRESENT_LOAD = 126     # Alias for current effort

# Encoder Count Thresholds for Gripper ID 15
OPEN_COUNT_NOMINAL   = 1700 # Fully open (~60 mm finger gap)
GRASP_MIN_COUNT      = 1920 # Minimum count where a peg can be engaged
GRASP_MAX_COUNT      = 2240 # Maximum count when holding a 40-50 mm peg
EMPTY_CLOSED_LIMIT   = 2310 # Counts above this indicate fingers closed on empty air

# Electrical Load Thresholds
CURRENT_UNIT_MA      = 2.69 # 1 raw unit = 2.69 mA on XM430-W350
MIN_HOLDING_CURRENT_MA = 25.0 # Minimum current drawn when holding a peg securely
MAX_IDLE_CURRENT_MA    = 15.0 # Current drawn when motor is unloaded / stationary in air

class GraspState(Enum):
    OPEN            = "open"             # Fingers open, no object engaged
    SECURE_HOLD     = "secure_hold"      # Peg firmly grasped between fingers
    EMPTY_AIR       = "empty_air"        # Grasp closed on thin air (missed peg)
    SLIPPED_DROPPED = "slipped_dropped"  # Peg was held, but slipped/dropped in transit

class PegDroppedException(Exception):
    """Raised immediately when an in-transit peg slip or drop is detected."""
    def __init__(self, position: int, current_ma: float, message: str = "Peg slipped or dropped from gripper!"):
        self.position = position
        self.current_ma = current_ma
        super().__init__(f"{message} (Position: {position}, Current: {current_ma:.1f} mA)")

class GripperGuard:
    def __init__(self, controller = None):
        self.controller = controller
        self._last_state: GraspState = GraspState.OPEN
        self._holding_active: bool = False

    def read_gripper_feedback(self) -> Tuple[Optional[int], Optional[float]]:
        """
        Reads raw position and present current (in mA) from Gripper motor ID 15.
        Returns: (position_raw, current_ma)
        """
        if not self.controller or not self.controller.is_connected:
            return None, None

        pos = self.controller.read_position(GRIPPER_ID)
        if pos is None:
            return None, None

        # Read 2-byte signed current
        raw_cur, res, err = self.controller.packet_handler.read2ByteTxRx(
            self.controller.port_handler, GRIPPER_ID, ADDR_PRESENT_CURRENT
        )
        if res == 0:
            # Convert unsigned 16-bit to signed
            if raw_cur > 32767:
                raw_cur -= 65536
            current_ma = abs(raw_cur) * CURRENT_UNIT_MA
        else:
            current_ma = 0.0

        return pos, current_ma

    def evaluate_grasp(self, verify_cycles: int = 3) -> Tuple[GraspState, Dict]:
        """
        Evaluates current grasp state with multi-sample verification.
        """
        positions = []
        currents = []

        for _ in range(verify_cycles):
            p, c = self.read_gripper_feedback()
            if p is not None:
                positions.append(p)
                currents.append(c)
            time.sleep(0.02)

        if not positions:
            # Fallback if hardware not connected (e.g. mock test)
            return self._last_state, {"mock": True}

        avg_pos = int(sum(positions) / len(positions))
        avg_cur = float(sum(currents) / len(currents))

        # Decision Logic:
        if avg_pos < (OPEN_COUNT_NOMINAL + 120):
            state = GraspState.OPEN
            self._holding_active = False
        elif avg_pos >= EMPTY_CLOSED_LIMIT:
            # Fingers closed past peg boundary
            if self._holding_active:
                state = GraspState.SLIPPED_DROPPED
            else:
                state = GraspState.EMPTY_AIR
            self._holding_active = False
        elif GRASP_MIN_COUNT <= avg_pos < EMPTY_CLOSED_LIMIT:
            # Fingers stopped on physical obstacle
            if avg_cur >= MIN_HOLDING_CURRENT_MA or avg_pos < 2220:
                state = GraspState.SECURE_HOLD
                self._holding_active = True
            else:
                state = GraspState.EMPTY_AIR
        else:
            state = GraspState.EMPTY_AIR

        self._last_state = state
        return state, {
            "position": avg_pos,
            "current_ma": avg_cur,
            "holding_active": self._holding_active
        }

    def monitor_transit_trajectory(self, duration_sec: float, poll_hz: float = 20.0,
                                   on_drop_callback: Optional[Callable] = None):
        """
        Monitors gripper state during arm transit.
        If a drop occurs mid-flight, raises PegDroppedException and fires callback.
        """
        if not self._holding_active:
            # Not holding anything to monitor
            time.sleep(duration_sec)
            return

        interval = 1.0 / poll_hz
        steps = int(duration_sec * poll_hz)

        for _ in range(steps):
            pos, cur = self.read_gripper_feedback()
            if pos is not None and cur is not None:
                # Check for drop condition: fingers suddenly snapped closed
                if pos >= EMPTY_CLOSED_LIMIT and cur < MIN_HOLDING_CURRENT_MA:
                    self._holding_active = False
                    self._last_state = GraspState.SLIPPED_DROPPED
                    if on_drop_callback:
                        on_drop_callback(pos, cur)
                    raise PegDroppedException(pos, cur)
            time.sleep(interval)

    @staticmethod
    def generate_spiral_search_offsets(max_radius: float = 2.0, num_steps: int = 12) -> List[Tuple[float, float]]:
        """
        Generates an Archimedean micro-spiral search trajectory (dx, dy in mm)
        to compliantly seat a jammed peg into a socket cutout.
        """
        offsets = [(0.0, 0.0)]
        # Archimedean spiral: r = a * theta
        theta_max = 3.0 * 2.0 * math.pi # 3 full turns
        d_theta = theta_max / float(num_steps)
        b = max_radius / theta_max

        for i in range(1, num_steps + 1):
            theta = i * d_theta
            r = b * theta
            dx = r * math.cos(theta)
            dy = r * math.sin(theta)
            offsets.append((round(dx, 2), round(dy, 2)))

        return offsets
