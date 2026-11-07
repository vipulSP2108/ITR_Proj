#!/usr/bin/env python3
"""
Visual Drop & Slip Detection Engine (Camera Ground Truth) for OpenMANIPULATOR-X.
Addresses Edge Case EC-05: Vision-Based Object Drop & Slip Verification.

Complements proprioceptive motor load/current sensing with optical verification:
  1. Post-Lift Pickup Verification: Confirms peg was successfully lifted from mat.
  2. Tabletop Drop Detector: Locates newly fallen pegs on the table/mat surface.
  3. In-Transit In-Hand Tracking: Monitors target color blob within the gripper ROI.
  4. Post-Placement Socket Occupancy: Optical confirmation that socket is filled.
  5. In-Hand Tilt / Partial Slip Inspector: Detects peg shifting/slipping before total drop.
"""

import os
import sys
import cv2
import numpy as np
from enum import Enum
from typing import Optional, Tuple, Dict, List

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
T1_DIR = os.path.dirname(os.path.dirname(CUR_DIR))
SRC_DIR = os.path.dirname(CUR_DIR)
for p in [T1_DIR, SRC_DIR, CUR_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

from src.vision.single_camera_pipeline import SingleCameraPipeline, TargetPose
from src.vision.color_segmentation import ColorSegmenter
from src.vision.shape_detection import ShapeDetector
from src.planning.calibration import EyeToHandCalibrator

# Shape to Expected Color mapping
SHAPE_COLOR_MAP = {
    "square": "blue",
    "triangle": "yellow",
    "hexagon": "orange_red",
    "circle": "green"
}

class VisualDropStatus(Enum):
    SECURE_IN_HAND       = "secure_in_hand"        # Peg visually verified elevated in gripper
    DROPPED_ON_TABLE     = "dropped_on_table"      # Peg detected resting on tabletop/mat
    MISSED_AT_PICKUP     = "missed_at_pickup"      # Peg never lifted; still sitting at pickup spot
    PARTIAL_SLIP_TILTED  = "partial_slip_tilted"   # Peg still in hand but tilted/slipping (>15 deg)
    SOCKET_EMPTY         = "socket_empty"          # Target socket verified empty post-placement
    SOCKET_FILLED        = "socket_filled"         # Target socket successfully populated

class VisualDropDetector:
    def __init__(self, pipeline: Optional[SingleCameraPipeline] = None,
                 calibrator: Optional[EyeToHandCalibrator] = None):
        self.pipeline = pipeline or SingleCameraPipeline()
        self.calibrator = calibrator or self.pipeline.calibrator
        self.segmenter = self.pipeline.segmenter
        self.detector = self.pipeline.detector
        self.intrinsics = self.detector.intrinsics

    def verify_post_lift(self, rgb_image: np.ndarray, depth_map: Optional[np.ndarray],
                         shape_name: str, original_pickup_robot_xy: Tuple[float, float],
                         ee_robot_xyz: Tuple[float, float, float]) -> Tuple[VisualDropStatus, Dict]:
        """
        Visual Checkpoint 1: Evaluated right after the vertical lift to transit height.
        Verifies:
          1. Original pickup zone on the green mat is now CLEAR (no peg remaining).
          2. Gripper 3D ROI contains the elevated colored peg blob.
        """
        color_target = SHAPE_COLOR_MAP.get(shape_name.lower(), "blue")
        color_masks = self.segmenter.segment_all(rgb_image)
        mask = color_masks.get(color_target)

        if mask is None or cv2.countNonZero(mask) < 20:
            # Color not visible at all
            return VisualDropStatus.DROPPED_ON_TABLE, {
                "reason": f"No {color_target} pixels detected in scene.",
                "color_pixel_count": 0
            }

        # Find contours of target color
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return VisualDropStatus.DROPPED_ON_TABLE, {"reason": "No color contours found."}

        # Analyze each contour's 3D robot base location
        found_at_pickup = False
        found_in_hand = False
        found_dropped = False
        in_hand_contour = None

        pick_x, pick_y = original_pickup_robot_xy
        ee_x, ee_y, ee_z = ee_robot_xyz

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            u = float(M["m10"] / M["m00"])
            v = float(M["m01"] / M["m00"])

            # Deproject to 3D camera and robot coordinates
            if depth_map is not None:
                # Sample median depth from 5x5 ROI
                u_i, v_i = int(round(u)), int(round(v))
                h, w = depth_map.shape
                u0, u1 = max(0, u_i - 2), min(w, u_i + 3)
                v0, v1 = max(0, v_i - 2), min(h, v_i + 3)
                roi = depth_map[v0:v1, u0:u1]
                val = roi[roi > 0]
                z_cam = float(np.median(val)) if len(val) > 0 else 400.0
            else:
                z_cam = 400.0

            # Transform to robot base
            p_cam = self.intrinsics.deproject_pixel_to_point(u, v, z_cam)
            p_rob = self.calibrator.camera_to_robot(p_cam)
            rx, ry, rz = p_rob

            # Check 1: Is this blob still at the pickup spot on the table?
            dist_to_pickup = np.hypot(rx - pick_x, ry - pick_y)
            if dist_to_pickup < 45.0 and rz < (ee_z - 30.0):
                found_at_pickup = True

            # Check 2: Is this blob near the elevated end-effector?
            dist_to_ee = np.hypot(rx - ee_x, ry - ee_y)
            if dist_to_ee < 60.0 and (rz >= (ee_z - 45.0)):
                found_in_hand = True
                in_hand_contour = cnt

            # Check 3: Is it elsewhere on the table surface?
            if dist_to_ee >= 60.0 and dist_to_pickup >= 45.0 and rz < 75.0:
                found_dropped = True

        if found_at_pickup and not found_in_hand:
            return VisualDropStatus.MISSED_AT_PICKUP, {
                "reason": "Peg remained at original pickup coordinates on mat."
            }

        if found_dropped and not found_in_hand:
            return VisualDropStatus.DROPPED_ON_TABLE, {
                "reason": "Peg detached and is resting on table surface."
            }

        if found_in_hand:
            # Check for partial slip / tilt
            if in_hand_contour is not None:
                rect = cv2.minAreaRect(in_hand_contour)
                rw, rh = rect[1]
                if rw > 0 and rh > 0:
                    aspect = max(rw, rh) / min(rw, rh)
                    if aspect > 1.55: # Tilted sideways in fingers
                        return VisualDropStatus.PARTIAL_SLIP_TILTED, {
                            "aspect_ratio": aspect,
                            "tilt_angle_deg": rect[2]
                        }

            return VisualDropStatus.SECURE_IN_HAND, {"reason": "Peg confirmed held in gripper."}

        # Fallback if ambiguous
        return VisualDropStatus.DROPPED_ON_TABLE, {"reason": "Peg not detected in gripper."}

    def detect_tabletop_dropped_peg(self, rgb_image: np.ndarray, depth_map: Optional[np.ndarray],
                                    shape_name: str) -> Tuple[bool, Optional[TargetPose]]:
        """
        Visual Checkpoint 2: Scans tabletop / green mat to locate a dropped peg.
        Returns: (is_dropped, TargetPose of the fallen peg with updated (X, Y, Z))
        """
        color_target = SHAPE_COLOR_MAP.get(shape_name.lower(), "blue")
        color_masks = self.segmenter.segment_all(rgb_image)
        mask = color_masks.get(color_target)

        if mask is None or cv2.countNonZero(mask) < 25:
            return False, None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and area > 60:
                max_area = area
                best_cnt = cnt

        if best_cnt is None:
            return False, None

        M = cv2.moments(best_cnt)
        if M["m00"] == 0:
            return False, None

        u = float(M["m10"] / M["m00"])
        v = float(M["m01"] / M["m00"])

        # Sample depth
        if depth_map is not None:
            u_i, v_i = int(round(u)), int(round(v))
            h, w = depth_map.shape
            u0, u1 = max(0, u_i - 2), min(w, u_i + 3)
            v0, v1 = max(0, v_i - 2), min(h, v_i + 3)
            roi = depth_map[v0:v1, u0:u1]
            val = roi[roi > 0]
            z_cam = float(np.median(val)) if len(val) > 0 else 400.0
        else:
            z_cam = 400.0

        p_cam = self.intrinsics.deproject_pixel_to_point(u, v, z_cam)
        p_rob = self.calibrator.camera_to_robot(p_cam)

        # Orientation from minimum area rectangle
        rect = cv2.minAreaRect(best_cnt)
        yaw_deg = float(rect[2])

        target_pose = TargetPose(
            shape=shape_name,
            color=color_target,
            centroid_2d=(u, v),
            camera_3d=p_cam,
            robot_3d=p_rob,
            yaw_deg=yaw_deg,
            is_socket=False
        )
        return True, target_pose

    def verify_socket_placement(self, rgb_image: np.ndarray, depth_map: Optional[np.ndarray],
                                shape_name: str, expected_socket_robot_xy: Tuple[float, float],
                                tolerance_mm: float = 40.0) -> Tuple[VisualDropStatus, Dict]:
        """
        Visual Checkpoint 3: Post-placement socket occupancy check.
        Inspects the socket cutout to verify whether the matching colored peg is seated inside.
        """
        color_target = SHAPE_COLOR_MAP.get(shape_name.lower(), "blue")
        color_masks = self.segmenter.segment_all(rgb_image)
        mask = color_masks.get(color_target)

        if mask is None or cv2.countNonZero(mask) < 20:
            return VisualDropStatus.SOCKET_EMPTY, {
                "reason": f"No {color_target} pixels detected at target socket."
            }

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sock_x, sock_y = expected_socket_robot_xy

        for cnt in contours:
            if cv2.contourArea(cnt) < 40:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            u = float(M["m10"] / M["m00"])
            v = float(M["m01"] / M["m00"])

            if depth_map is not None:
                u_i, v_i = int(round(u)), int(round(v))
                h, w = depth_map.shape
                roi = depth_map[max(0, v_i-2):min(h, v_i+3), max(0, u_i-2):min(w, u_i+3)]
                val = roi[roi > 0]
                z_cam = float(np.median(val)) if len(val) > 0 else 400.0
            else:
                z_cam = 400.0

            p_cam = self.intrinsics.deproject_pixel_to_point(u, v, z_cam)
            p_rob = self.calibrator.camera_to_robot(p_cam)
            dist = np.hypot(p_rob[0] - sock_x, p_rob[1] - sock_y)

            if dist <= tolerance_mm:
                return VisualDropStatus.SOCKET_FILLED, {
                    "distance_to_center_mm": float(dist),
                    "measured_robot_3d": p_rob
                }

        return VisualDropStatus.SOCKET_EMPTY, {
            "reason": f"No {color_target} peg detected within {tolerance_mm} mm of socket ({sock_x}, {sock_y})."
        }

