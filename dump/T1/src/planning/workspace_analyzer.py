#!/usr/bin/env python3
"""
Workspace Analyzer & Reachability Recovery Engine for OpenMANIPULATOR-X.
Addresses Edge Case EC-06: Object Out of Reachable Workspace.

Provides:
  1. Instant reachability classification (Vertical, Angled, Sweep, Out-of-Bounds).
  2. Angled grasp trajectory solver (extends reach up to 290 mm).
  3. Inward rake/sweep trajectory generator (brings pegs from 290-340 mm into prime workspace).
  4. Precise reach deficit calculations for operator feedback.
"""

import math
import numpy as np
from enum import Enum
from typing import Tuple, Optional, List, Dict

import os
import sys

# Ensure T1 and T1/src are accessible on sys.path
CUR_DIR = os.path.dirname(os.path.abspath(__file__))
T1_DIR = os.path.dirname(os.path.dirname(CUR_DIR))
SRC_DIR = os.path.dirname(CUR_DIR)
for p in [T1_DIR, SRC_DIR, CUR_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from src.planning.kinematics import OpenManipulatorKinematics
except ImportError:
    try:
        from kinematics import OpenManipulatorKinematics
    except ImportError:
        from T1.src.planning.kinematics import OpenManipulatorKinematics

class ReachabilityStatus(Enum):
    REACHABLE_VERTICAL = "reachable_vertical"   # Prime workspace: strict vertical top-grasp (-90 deg)
    REACHABLE_ANGLED   = "reachable_angled"     # Extended workspace: angled pitch grasp (-85 to -45 deg)
    REACHABLE_SWEEP    = "reachable_sweep"      # Sweepable zone: rake/drag inward to prime workspace
    OUT_OF_BOUNDS      = "out_of_bounds"        # Beyond physical hardware arm limits

class WorkspaceAnalyzer:
    # Kinematic Radii (in mm from center of base joint 11)
    RADIUS_MIN = 95.0            # Inner collision boundary with robot body
    RADIUS_PRIME_MAX = 215.0     # Maximum radius for strictly vertical top-down grasp (-90 deg)
    RADIUS_ANGLED_MAX = 290.0    # Maximum radius for angled diagonal grasp
    RADIUS_SWEEP_MAX = 340.0     # Maximum radius where closed gripper can reach behind to sweep
    RADIUS_HARD_LIMIT = 380.0    # Theoretical full geometric arm extension (L1 + L2 + L3 + L4)

    # Safe Height Envelopes (in mm from mounting table surface)
    TABLE_Z_CLEARANCE = 15.0     # Minimum Z to avoid scraping table
    TRANSIT_Z_DEFAULT = 85.0     # Safe travel height
    PEG_TOP_Z_NOMINAL = 60.0     # Nominal top surface of 10 cm peg

    def __init__(self, kinematics: Optional[OpenManipulatorKinematics] = None):
        self.kin = kinematics or OpenManipulatorKinematics()

    def get_radius(self, x: float, y: float) -> float:
        """Returns horizontal cylindrical radius r = sqrt(x^2 + y^2)."""
        return float(np.sqrt(x**2 + y**2))

    def classify_target(self, x: float, y: float, z: float) -> Tuple[ReachabilityStatus, Dict]:
        """
        Classifies target coordinate into reachability category and computes kinematic telemetry.
        """
        r = self.get_radius(x, y)
        theta_deg = float(np.degrees(np.arctan2(y, x)))

        # 1. Test vertical IK
        ik_vert = self.kin.inverse_kinematics(x, y, z, pitch_rad=-np.pi / 2.0)
        if ik_vert is not None:
            return ReachabilityStatus.REACHABLE_VERTICAL, {
                "radius": r,
                "yaw_deg": theta_deg,
                "pitch_deg": -90.0,
                "ik_solution": ik_vert,
                "margin_mm": self.RADIUS_PRIME_MAX - r
            }

        # 2. Test angled pitch IK
        angled_sol = self.solve_optimal_angled_grasp(x, y, z)
        if angled_sol is not None:
            pitch_deg, q_sol = angled_sol
            return ReachabilityStatus.REACHABLE_ANGLED, {
                "radius": r,
                "yaw_deg": theta_deg,
                "pitch_deg": pitch_deg,
                "ik_solution": q_sol,
                "margin_mm": self.RADIUS_ANGLED_MAX - r
            }

        # 3. Test sweep feasibility (closed gripper reaching behind peg)
        if self.RADIUS_ANGLED_MAX < r <= self.RADIUS_SWEEP_MAX:
            return ReachabilityStatus.REACHABLE_SWEEP, {
                "radius": r,
                "yaw_deg": theta_deg,
                "deficit_mm": r - self.RADIUS_PRIME_MAX,
                "recommended_sweep_distance_mm": r - 160.0
            }

        # 4. Out of bounds
        return ReachabilityStatus.OUT_OF_BOUNDS, {
            "radius": r,
            "yaw_deg": theta_deg,
            "deficit_mm": max(0.0, r - self.RADIUS_SWEEP_MAX),
            "reason": "Target exceeds maximum mechanical reach envelope" if r > self.RADIUS_SWEEP_MAX else "Target within inner base blindspot"
        }

    def solve_optimal_angled_grasp(self, x: float, y: float, z: float,
                                   min_pitch_deg: float = -60.0) -> Optional[Tuple[float, Tuple[float, float, float, float]]]:
        """
        Searches candidate pitch angles from -85 deg to min_pitch_deg (default: -60 deg)
        to find the most vertical reachable grasp pose. Restricting pitch to >= -60 deg
        guarantees the gripper securely captures the 10 cm vertical peg without toppling it.
        Returns: (pitch_deg, (q1, q2, q3, q4)) or None.
        """
        # Test candidate pitches from steep to shallower in 2.5 deg increments
        for pitch_deg in np.arange(-85.0, min_pitch_deg + 0.1, 2.5):
            pitch_rad = np.radians(pitch_deg)
            sol = self.kin.inverse_kinematics(x, y, z, pitch_rad=pitch_rad)
            if sol is not None:
                return float(pitch_deg), sol
        return None

    def generate_inward_sweep_trajectory(self, x: float, y: float, table_z: float = 25.0,
                                         target_radius: float = 160.0) -> Optional[List[Dict]]:
        """
        Generates a 4-step inward rake/sweep trajectory for pegs sitting in the sweepable zone:
          1. Pre-sweep: Position closed gripper behind peg at elevated clearance.
          2. Descend: Lower closed gripper behind peg to table clearance.
          3. Sweep: Smoothly drag peg radially inward toward prime target_radius (e.g. 160 mm).
          4. Retract: Elevate gripper and retreat to safe transit height.
        """
        r = self.get_radius(x, y)
        if r <= self.RADIUS_PRIME_MAX or r > self.RADIUS_SWEEP_MAX:
            return None

        # Unit vector along radial line from base to peg
        u_x = x / r
        u_y = y / r

        # Point 1: 30 mm behind peg
        r_behind = min(r + 30.0, self.RADIUS_HARD_LIMIT - 15.0)
        x_behind = r_behind * u_x
        y_behind = r_behind * u_y

        # Point 2: Target inward position
        x_inward = target_radius * u_x
        y_inward = target_radius * u_y

        # Dynamically solve optimal pitch for each waypoint (rake probe can use shallower pitch up to -35 deg)
        sol_behind_high = self.solve_optimal_angled_grasp(x_behind, y_behind, table_z + 40.0, min_pitch_deg=-35.0)
        sol_behind_low  = self.solve_optimal_angled_grasp(x_behind, y_behind, table_z, min_pitch_deg=-35.0)
        sol_inward_low  = self.solve_optimal_angled_grasp(x_inward, y_inward, table_z, min_pitch_deg=-75.0)
        sol_inward_high = self.solve_optimal_angled_grasp(x_inward, y_inward, table_z + 40.0, min_pitch_deg=-75.0)

        if not (sol_behind_high and sol_behind_low and sol_inward_low and sol_inward_high):
            return None

        return [
            {"name": "1. Position Behind Peg", "cartesian": (x_behind, y_behind, table_z + 40.0), "joints": sol_behind_high[1], "gripper_open": False, "duration": 2.5},
            {"name": "2. Lower Behind Peg",    "cartesian": (x_behind, y_behind, table_z),        "joints": sol_behind_low[1],  "gripper_open": False, "duration": 1.5},
            {"name": "3. Drag Peg Inward",      "cartesian": (x_inward, y_inward, table_z),        "joints": sol_inward_low[1],  "gripper_open": False, "duration": 3.0},
            {"name": "4. Elevate Above Table", "cartesian": (x_inward, y_inward, table_z + 40.0), "joints": sol_inward_high[1], "gripper_open": False, "duration": 1.5},
        ]
