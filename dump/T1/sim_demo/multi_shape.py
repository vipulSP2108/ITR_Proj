#!/usr/bin/env python3
"""
Multi-Shape Sorting & Orientation Alignment Planner for OpenMANIPULATOR-X.
Addresses repare.md §11 (items 9 & 10) and Task 1 requirements for all 4 shapes:
  • Circle, Hexagon, Square, Triangle
  • 4-DOF kinematics constraint: OpenMANIPULATOR-X lacks wrist roll; end-effector
    orientation in the horizontal plane is determined by Joint 1 (Base Pan).
  • Implements symmetry-aware minimal rotation alignment and multi-peg sequencing.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass

import arm_model as A
from planner import Waypoint, Plan, plan_pick_place, GRASP_DEPTH


SYMMETRY_ORDERS = {
    "circle": 0,    # Continuous symmetry (no yaw constraint)
    "square": 4,    # 90° rotational symmetry
    "hexagon": 6,   # 60° rotational symmetry
    "triangle": 3   # 120° rotational symmetry
}

SYMMETRY_INTERVAL_RAD = {
    "circle": 0.0,
    "square": np.pi / 2.0,
    "hexagon": np.pi / 3.0,
    "triangle": 2.0 * np.pi / 3.0
}


def minimal_symmetry_angle_diff(angle1_rad: float, angle2_rad: float, shape: str) -> float:
    """
    Computes the minimum angular difference considering the shape's rotational symmetry.
    Returns error in [-interval/2, interval/2].
    """
    if shape == "circle":
        return 0.0

    interval = SYMMETRY_INTERVAL_RAD.get(shape, np.pi)
    diff = (angle1_rad - angle2_rad) % interval
    if diff > interval / 2.0:
        diff -= interval
    return diff


@dataclass
class MultiPegPlan:
    shape: str
    plan: Plan
    peg_info: Dict[str, Any]
    socket_info: Dict[str, Any]
    yaw_alignment_error_deg: float
    is_yaw_compatible: bool


class MultiShapePlanner:
    """
    Plans sequential pick-and-place for multiple shaped pegs, validating
    geometric reachability and 4-DOF horizontal orientation alignment.
    """

    def __init__(self, board_z: float = 0.008):
        self.board_z = board_z

    def evaluate_shape_alignment(
        self,
        shape: str,
        peg_xy: Tuple[float, float],
        socket_xy: Tuple[float, float],
        peg_yaw_rad: float = 0.0,
        socket_yaw_rad: float = 0.0
    ) -> Tuple[bool, float]:
        """
        Since OMX-X gripper orientation in the horizontal plane is locked to q1 = atan2(y, x),
        evaluates whether the socket yaw relative to q1_place matches peg yaw relative to q1_pick.
        """
        if shape == "circle":
            return True, 0.0

        q1_pick = np.arctan2(peg_xy[1], peg_xy[0])
        q1_place = np.arctan2(socket_xy[1], socket_xy[0])

        # Gripper rotates by Delta_q1 = q1_place - q1_pick during transit
        delta_q1 = q1_place - q1_pick
        peg_arrival_yaw = peg_yaw_rad + delta_q1

        # Error between arrival yaw and socket orientation modulo shape symmetry
        err_rad = minimal_symmetry_angle_diff(peg_arrival_yaw, socket_yaw_rad, shape)
        err_deg = float(np.degrees(abs(err_rad)))

        # Allow small compliance window (±12 degrees for chamfered sockets)
        is_compatible = err_deg <= 12.0
        return is_compatible, err_deg

    def plan_all_pegs(
        self,
        pegs: List[Dict[str, Any]],
        sockets: List[Dict[str, Any]]
    ) -> List[MultiPegPlan]:
        """
        Generates an ordered sequence of pick-and-place plans for all available pegs.
        """
        socket_by_shape = {s["shape"]: s for s in sockets}
        plans = []

        # Default order: circle first (clears space), then hexagon, square, triangle
        ORDER = ["circle", "hexagon", "square", "triangle"]
        sorted_pegs = sorted(pegs, key=lambda p: ORDER.index(p["shape"]) if p["shape"] in ORDER else 99)

        for peg in sorted_pegs:
            shape = peg["shape"]
            if shape not in socket_by_shape:
                continue

            socket = socket_by_shape[shape]
            base_plan = plan_pick_place(peg, socket, self.board_z)

            if base_plan is None:
                continue

            compat, err_deg = self.evaluate_shape_alignment(
                shape,
                (peg["x"], peg["y"]),
                (socket["x"], socket["y"]),
                peg.get("yaw_rad", 0.0),
                socket.get("yaw_rad", 0.0)
            )

            plans.append(MultiPegPlan(
                shape=shape,
                plan=base_plan,
                peg_info=peg,
                socket_info=socket,
                yaw_alignment_error_deg=err_deg,
                is_yaw_compatible=compat
            ))

        return plans
