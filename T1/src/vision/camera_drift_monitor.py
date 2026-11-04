#!/usr/bin/env python3
"""
Camera Drift Monitor & Online Auto-Recalibration Engine.
Addresses Edge Case EC-07: Camera Extrinsic Shift / Accidental Bump (User Case 3).

Detects camera displacement relative to fixed workbench geometry (the socket board),
calculates translational and rotational drift, and performs real-time closed-form Kabsch
recalibration (< 2 ms) without requiring manual calibration wands or interrupting the run.
"""

import os
import sys
import yaml
import numpy as np
from typing import Tuple, Optional, Dict, List

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
T1_DIR = os.path.dirname(os.path.dirname(CUR_DIR))
SRC_DIR = os.path.dirname(CUR_DIR)
for p in [T1_DIR, SRC_DIR, CUR_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from src.planning.calibration import EyeToHandCalibrator
except ImportError:
    from planning.calibration import EyeToHandCalibrator

# Nominal Robot Base Coordinates (X, Y, Z in mm) of fixed board landmarks.
# Spans both longitudinal (Y) and lateral (X) dimensions to guarantee full rank-3 SVD
# and eliminate collinear rotational ambiguity:
NOMINAL_BOARD_LANDMARKS = np.array([
    # Socket Centers
    [160.0, -170.0, 38.0], # 1. Triangle Center
    [160.0, -120.0, 38.0], # 2. Square Center
    [160.0,  -70.0, 38.0], # 3. Hexagon Center
    [160.0,  -20.0, 38.0], # 4. Circle Center
    # Board Lateral Reference Edges / Corners
    [125.0, -170.0, 38.0], # 5. Triangle Socket Left Edge
    [195.0, -170.0, 38.0], # 6. Triangle Socket Right Edge
    [125.0,  -20.0, 38.0], # 7. Circle Socket Left Edge
    [195.0,  -20.0, 38.0], # 8. Circle Socket Right Edge
], dtype=np.float64)

# Backwards compatible alias
NOMINAL_SOCKET_ROBOT_COORDS = NOMINAL_BOARD_LANDMARKS

# Maximum Tolerable Drift before Re-calibration is triggered
MAX_TRANSLATION_DRIFT_MM = 4.0  # Drift beyond 4 mm triggers online recalibration
MAX_ROTATION_DRIFT_DEG   = 1.5  # Angular drift beyond 1.5 deg triggers recalibration

class CameraDriftMonitor:
    def __init__(self, calibrator: Optional[EyeToHandCalibrator] = None,
                 calib_config_path: Optional[str] = None):
        self.calibrator = calibrator or EyeToHandCalibrator()
        self.config_path = calib_config_path or os.path.join(T1_DIR, "config", "camera_calib.yaml")
        self.nominal_board_robot = NOMINAL_SOCKET_ROBOT_COORDS.copy()

    def compute_drift(self, measured_cam_pts: np.ndarray) -> Tuple[bool, float, float, np.ndarray]:
        """
        Evaluates extrinsic drift given 4 detected socket positions in camera space (N x 3).
        Returns: (drift_detected, translation_drift_mm, rotation_drift_deg, projected_robot_pts)
        """
        if len(measured_cam_pts) < 3:
            return False, 0.0, 0.0, np.array([])

        # Project measured camera points to robot space using current calibration
        projected_robot_pts = []
        for pt in measured_cam_pts:
            p_rob = self.calibrator.camera_to_robot((pt[0], pt[1], pt[2]))
            projected_robot_pts.append(p_rob)
        projected_robot_pts = np.array(projected_robot_pts, dtype=np.float64)

        # Match corresponding points to nominal board
        n_pts = min(len(projected_robot_pts), len(self.nominal_board_robot))
        diffs = projected_robot_pts[:n_pts] - self.nominal_board_robot[:n_pts]
        mean_drift_vec = np.mean(diffs, axis=0)
        trans_drift_mm = float(np.linalg.norm(mean_drift_vec))

        # Solve test rotation to measure angular drift
        p_cam = measured_cam_pts[:n_pts]
        p_rob = self.nominal_board_robot[:n_pts]

        p_cam_c = p_cam - np.mean(p_cam, axis=0)
        p_rob_c = p_rob - np.mean(p_rob, axis=0)

        H = p_cam_c.T @ p_rob_c
        U, S, Vt = np.linalg.svd(H)
        R_new = Vt.T @ U.T
        if np.linalg.det(R_new) < 0:
            Vt[-1, :] *= -1
            R_new = Vt.T @ U.T

        # Relative rotation against current calibration
        R_rel = R_new @ self.calibrator.R.T
        trace_val = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
        rot_drift_deg = float(np.degrees(np.arccos(trace_val)))

        drift_detected = (trans_drift_mm > MAX_TRANSLATION_DRIFT_MM) or (rot_drift_deg > MAX_ROTATION_DRIFT_DEG)
        return drift_detected, trans_drift_mm, rot_drift_deg, projected_robot_pts

    def recalibrate_online(self, measured_cam_pts: np.ndarray,
                           save_to_disk: bool = True) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Solves closed-form Kabsch SVD algorithm to update (R, t) in < 2 ms.
        Returns: (R_new, t_new, residual_rmse_mm)
        """
        n_pts = min(len(measured_cam_pts), len(self.nominal_board_robot))
        A = measured_cam_pts[:n_pts].copy()
        B = self.nominal_board_robot[:n_pts].copy()

        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        AA = A - centroid_A
        BB = B - centroid_B

        H = AA.T @ BB
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Reflection handling
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_B - (R @ centroid_A)

        # Calculate residual RMSE
        transformed = (R @ A.T).T + t
        rmse = float(np.sqrt(np.mean(np.sum((transformed - B)**2, axis=1))))

        # Update calibrator memory
        self.calibrator.R = R
        self.calibrator.t = t

        # Optionally persist updated calibration to YAML
        if save_to_disk and os.path.exists(os.path.dirname(self.config_path)):
            calib_data = {
                "rotation_matrix": R.tolist(),
                "translation_vector": t.tolist(),
                "rmse_mm": rmse,
                "auto_recalibrated": True
            }
            try:
                with open(self.config_path, "w") as f:
                    yaml.dump(calib_data, f, default_flow_style=False)
            except Exception as e:
                print(f"[Warning] Could not persist recalibrated YAML: {e}")

        return R, t, rmse
