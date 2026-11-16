#!/usr/bin/env python3
"""
Camera Drift Monitor & Online Extrinsics Recalibrator for sim_demo.
Complies with repare.md §7.2, §11 (item 4), and EDGE_CASES EC-07.

Monitors the detected 3D socket positions on the board across snapshots.
If camera bumps, mechanical vibration, or thermal drift shift the camera extrinsics:
  1. Detects translational drift (mm) and rotational drift (deg).
  2. Runs closed-form Kabsch alignment on the socket centroids to update T_robot_cam online (< 1 ms).
  3. Prevents failed grasps due to extrinsic calibration drift.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from calibration import kabsch


class SimCameraDriftMonitor:
    """
    Monitors socket board landmarks to detect and correct camera drift.
    Units: metres for positions, degrees for angular drift.
    """

    def __init__(self, tol_trans_mm: float = 4.0, tol_rot_deg: float = 1.5):
        self.tol_trans_m = tol_trans_mm / 1000.0
        self.tol_rot_deg = tol_rot_deg
        self.baseline_landmarks: Optional[Dict[str, np.ndarray]] = None

    def set_baseline_from_holes(self, holes: List[Dict[str, Any]]) -> None:
        """
        Stores baseline 3D positions of detected sockets in the robot frame.
        """
        self.baseline_landmarks = {}
        for h in holes:
            shape = h.get("shape")
            if shape:
                self.baseline_landmarks[shape] = np.array([h["x"], h["y"], h.get("z", 0.008)], dtype=float)

    def check_and_update(
        self,
        current_holes: List[Dict[str, Any]],
        T_robot_cam: np.ndarray,
        cam_model: Any
    ) -> Tuple[bool, float, float, np.ndarray]:
        """
        Compares currently detected holes against baseline.
        Returns:
          (drift_detected, trans_drift_mm, rot_drift_deg, updated_T_robot_cam)
        """
        if not self.baseline_landmarks or len(current_holes) < 3:
            return False, 0.0, 0.0, T_robot_cam

        common_shapes = [h["shape"] for h in current_holes if h["shape"] in self.baseline_landmarks]
        if len(common_shapes) < 3:
            return False, 0.0, 0.0, T_robot_cam

        P_base = []
        P_curr = []
        for h in current_holes:
            shape = h["shape"]
            if shape in self.baseline_landmarks:
                P_base.append(self.baseline_landmarks[shape])
                P_curr.append(np.array([h["x"], h["y"], h.get("z", 0.008)], dtype=float))

        P_base = np.array(P_base)  # (N, 3)
        P_curr = np.array(P_curr)  # (N, 3)

        # Compute translation difference
        diff = P_curr - P_base
        trans_drift_m = float(np.mean(np.linalg.norm(diff, axis=1)))
        trans_drift_mm = trans_drift_m * 1000.0

        # Estimate rotation drift using Kabsch between current and baseline
        Delta = kabsch(P_curr, P_base)
        R_delta = Delta[:3, :3]
        t_delta = Delta[:3, 3]
        # Angle from rotation trace
        tr = np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0)
        rot_drift_deg = float(np.degrees(np.arccos(tr)))

        drift_detected = (trans_drift_m > self.tol_trans_m) or (rot_drift_deg > self.tol_rot_deg)

        updated_T = T_robot_cam.copy()
        if drift_detected:
            # Apply correction: T_new = Delta * T_old
            Delta = np.eye(4)
            Delta[:3, :3] = R_delta
            Delta[:3, 3] = t_delta
            updated_T = Delta @ T_robot_cam

        return drift_detected, trans_drift_mm, rot_drift_deg, updated_T
