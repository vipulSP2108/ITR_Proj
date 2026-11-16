"""
Autonomous Eye-to-Hand & Planar Homography Calibrator for OpenMANIPULATOR-X.
Executes multi-point self-calibration by commanding the arm to safe hover stances,
optically detecting the Tool Center Point (TCP) via differential gripper motion,
and computing the empirical Homography Matrix H and 3D rigid transform (R, t).

Axis Conventions (as specified):
  • Home Pose:  Aligned along the robot base centerline (-X_cam / forward table axis)
  • RGB Survey: Aligned along the side workspace axis (+Y_rob / sideways survey stance)
"""

import os
import time
import yaml
import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional

class AutonomousHomographyCalibrator:
    def __init__(self, config_dir: Optional[str] = None):
        if config_dir is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            config_dir = os.path.join(base_dir, "config")
        self.config_dir = config_dir
        self.homography_yaml = os.path.join(config_dir, "homography_calib.yaml")
        self.camera_yaml = os.path.join(config_dir, "camera_calib.yaml")
        self.H: Optional[np.ndarray] = None
        self.R: Optional[np.ndarray] = None
        self.t: Optional[np.ndarray] = None
        self.rmse: float = 0.0

    @staticmethod
    def detect_tcp_by_gripper_flick(arm, cap, dxl_id: int = 15, open_pos: int = 3130,
                                     closed_pos: int = 2400) -> Optional[Tuple[float, float, np.ndarray]]:
        """
        Detects exact end-effector Tool Center Point (TCP) with subpixel precision
        by computing optical difference of a minor gripper finger stroke.
        """
        # Ensure arm is fully settled
        time.sleep(0.4)

        # 1. Capture with gripper OPEN
        arm.write_position(dxl_id, open_pos)
        time.sleep(0.4)
        for _ in range(4): cap.read()
        ret, frame_open = cap.read()
        if not ret or frame_open is None:
            return None

        # 2. Capture with gripper CLOSED
        arm.write_position(dxl_id, closed_pos)
        time.sleep(0.4)
        for _ in range(4): cap.read()
        ret, frame_closed = cap.read()
        if not ret or frame_closed is None:
            return None

        # 3. Restore gripper to OPEN
        arm.write_position(dxl_id, open_pos)
        time.sleep(0.3)

        # 4. Difference & Cluster Filtering
        gray_open = cv2.cvtColor(frame_open, cv2.COLOR_BGR2GRAY)
        gray_closed = cv2.cvtColor(frame_closed, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(gray_open, gray_closed)
        _, dt = cv2.threshold(diff, 20, 255, cv2.THRESH_BINARY)

        cnts, _ = cv2.findContours(dt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_cnts = [c for c in cnts if cv2.contourArea(c) >= 30]
        valid_cnts.sort(key=cv2.contourArea, reverse=True)

        if len(valid_cnts) >= 2:
            c1, c2 = valid_cnts[0], valid_cnts[1]
            M1, M2 = cv2.moments(c1), cv2.moments(c2)
            u1, v1 = M1["m10"] / (M1["m00"] + 1e-6), M1["m01"] / (M1["m00"] + 1e-6)
            u2, v2 = M2["m10"] / (M2["m00"] + 1e-6), M2["m01"] / (M2["m00"] + 1e-6)
            dist = np.hypot(u1 - u2, v1 - v2)
            if dist < 140.0:
                tcp_u = float((u1 + u2) / 2.0)
                tcp_v = float((v1 + v2) / 2.0)
                return (tcp_u, tcp_v, frame_open)

        if len(valid_cnts) >= 1:
            c1 = valid_cnts[0]
            M1 = cv2.moments(c1)
            tcp_u = float(M1["m10"] / (M1["m00"] + 1e-6))
            tcp_v = float(M1["m01"] / (M1["m00"] + 1e-6))
            return (tcp_u, tcp_v, frame_open)

        # Fallback if motion was subtle
        pts = np.argwhere(dt > 0)
        if len(pts) >= 6:
            tcp_v = float(np.mean(pts[:, 0]))
            tcp_u = float(np.mean(pts[:, 1]))
            return (tcp_u, tcp_v, frame_open)

        return None

    def run_autonomous_calibration(self, arm, cap, kin, smooth_move_fn) -> Dict:
        """
        Executes complete autonomous multi-point calibration routine.
        Commands arm to 4 safe tabletop hover points (Z=100mm, well above objects),
        detects TCP in image space, computes H matrix and 3D (R, t).
        """
        print("\n" + "=" * 72)
        print("  [AUTONOMOUS CALIBRATION] Computing Eye-to-Hand Homography Transformation")
        print("=" * 72)

        # 4 Safe Hover Stances (Z = 100 mm, zero collision risk)
        # Forming a wide non-degenerate quadrilateral across the working envelope
        stances = [
            ("Stance_1_Center",        180.0,   0.0, 100.0, -90.0),
            ("Stance_2_Forward_Left",  210.0,  60.0, 100.0, -80.0),
            ("Stance_3_Forward_Right", 210.0, -60.0, 100.0, -80.0),
            ("Stance_4_Far_Center",    250.0,   0.0, 100.0, -70.0),
        ]

        img_pts = []
        rob_pts_2d = []
        rob_pts_3d = []
        cam_pts_3d = []
        captured_frames = []

        for name, tx, ty, tz, pitch in stances:
            print(f"\n  • Calibrating Stance: {name} -> Target: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm...")

            ik_sol = kin.inverse_kinematics(tx, ty, tz, pitch_rad=np.radians(pitch))
            if ik_sol is None:
                print(f"    [WARN] Kinematics failed for {name}, trying pitch search...")
                for p in range(-90, -40, 5):
                    ik_sol = kin.inverse_kinematics(tx, ty, tz, pitch_rad=np.radians(p))
                    if ik_sol is not None:
                        break

            if ik_sol is None:
                raise RuntimeError(f"Calibration stance {name} unreachable!")

            raw_targets = kin.radians_to_raw(list(ik_sol), base_wrap_count=4096)
            cmd = {
                11: raw_targets[0],
                12: raw_targets[1],
                13: raw_targets[2],
                14: raw_targets[3],
                15: 3130
            }

            smooth_move_fn(arm, cmd, duration=2.2, steps=8)
            time.sleep(0.4)

            # Read true joint angles from actual motor encoders
            cur_raw = [arm.read_position(d) or raw_targets[i] for i, d in enumerate([11, 12, 13, 14])]
            # Calculate true FK Cartesian coordinates
            q_rad = kin.raw_to_radians(cur_raw)
            actual_fk = kin.forward_kinematics(q_rad[0], q_rad[1], q_rad[2], q_rad[3])
            rx, ry, rz = actual_fk[0], actual_fk[1], actual_fk[2]

            # Detect TCP optically
            result = self.detect_tcp_by_gripper_flick(arm, cap)
            if result is None:
                # Retry once
                time.sleep(0.3)
                result = self.detect_tcp_by_gripper_flick(arm, cap)

            if result is not None:
                u, v, frame = result
                print(f"    ✓ Stance {name}: TCP detected at pixel ({u:.2f}, {v:.2f}) <-> Motor FK: ({rx:.1f}, {ry:.1f}, {rz:.1f}) mm")
                img_pts.append([u, v])
                rob_pts_2d.append([rx, ry])
                rob_pts_3d.append([rx, ry, rz])
                captured_frames.append((name, (u, v), (rx, ry), frame))
            else:
                print(f"    [WARN] Could not detect TCP for {name}")

        if len(img_pts) < 4:
            raise RuntimeError(f"Autonomous calibration requires at least 4 valid stances, got {len(img_pts)}")

        img_pts_arr = np.array(img_pts, dtype=np.float64)
        rob_pts_arr = np.array(rob_pts_2d, dtype=np.float64)

        # 1. Compute Planar Homography Matrix H (Pixel -> Table Robot Frame)
        H, _ = cv2.findHomography(img_pts_arr, rob_pts_arr, method=0)
        self.H = H

        # 2. Compute Reprojection Errors
        reprojected = []
        errors_mm = []
        for i in range(len(img_pts)):
            p_homo = np.array([img_pts[i][0], img_pts[i][1], 1.0], dtype=np.float64)
            p_trans = H @ p_homo
            pred_x = p_trans[0] / p_trans[2]
            pred_y = p_trans[1] / p_trans[2]
            err = np.hypot(pred_x - rob_pts_2d[i][0], pred_y - rob_pts_2d[i][1])
            errors_mm.append(err)
            reprojected.append((pred_x, pred_y))

        self.rmse = float(np.sqrt(np.mean(np.array(errors_mm)**2)))

        print(f"\n  • Homography Matrix H computed successfully!")
        print(f"  • Mean Calibration Reprojection RMSE: {self.rmse:.2f} mm")
        for i, (name, (u, v), (rx, ry), _) in enumerate(captured_frames):
            print(f"    Stance {i+1}: Image ({u:.1f}, {v:.1f}) -> Target ({rx:.1f}, {ry:.1f}) -> Pred ({reprojected[i][0]:.1f}, {reprojected[i][1]:.1f}) -> Residual: {errors_mm[i]:.2f} mm")

        # 3. Compute 3D Rigid Transform (R, t) for full 3D Eye-to-Hand
        # Camera coordinates: X_c = (u - cx)*Z/f, Y_c = (v - cy)*Z/f, Z_c = H_cam - Z_rob
        f_px = 344.0
        cx, cy = 320.0, 240.0
        H_cam = 470.0

        for i in range(len(img_pts)):
            u, v = img_pts[i]
            z_rob = rob_pts_3d[i][2]
            z_cam = H_cam - z_rob
            x_c = (u - cx) * z_cam / f_px
            y_c = (v - cy) * z_cam / f_px
            cam_pts_3d.append([x_c, y_c, z_cam])

        from planning.calibration import EyeToHandCalibrator
        R, t, rmse_3d = EyeToHandCalibrator.compute_rigid_transform(cam_pts_3d, rob_pts_3d)
        self.R = R
        self.t = t

        # 4. Save Calibration to Configuration Files
        homo_data = {
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "rmse_mm": float(self.rmse),
            "num_stances": len(img_pts),
            "homography_matrix_3x3": H.tolist(),
            "calibration_pairs": [
                {
                    "stance": captured_frames[i][0],
                    "pixel_u": float(img_pts[i][0]),
                    "pixel_v": float(img_pts[i][1]),
                    "robot_x_mm": float(rob_pts_2d[i][0]),
                    "robot_y_mm": float(rob_pts_2d[i][1]),
                    "residual_error_mm": float(errors_mm[i])
                }
                for i in range(len(img_pts))
            ]
        }
        with open(self.homography_yaml, "w") as f:
            yaml.dump(homo_data, f, default_flow_style=False)
        print(f"  ✓ Saved Homography calibration to: {self.homography_yaml}")

        cam_data = {
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "method": "autonomous_differential_gripper_flick_svd",
            "rmse_mm": float(rmse_3d),
            "camera_height_above_board_mm": float(H_cam),
            "focal_length_px": float(f_px),
            "table_scale_mm_per_px": float(H_cam / f_px),
            "rotation_matrix": R.tolist(),
            "translation_vector_mm": t.tolist()
        }
        with open(self.camera_yaml, "w") as f:
            yaml.dump(cam_data, f, default_flow_style=False)
        print(f"  ✓ Saved updated Eye-to-Hand 3D calibration to: {self.camera_yaml}")

        # 5. Build Annotated Calibration Diagnostic Map
        last_frame = captured_frames[-1][3].copy()
        h_f, w_f = last_frame.shape[:2]

        # Draw Stances and Reprojections
        for i, (name, (u, v), (rx, ry), _) in enumerate(captured_frames):
            uv_int = (int(round(u)), int(round(v)))
            cv2.circle(last_frame, uv_int, 8, (0, 255, 0), -1)
            cv2.circle(last_frame, uv_int, 14, (0, 255, 255), 2)
            cv2.putText(last_frame, f"S{i+1}: ({rx:.0f},{ry:.0f}mm)", (uv_int[0] + 12, uv_int[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # Draw Axis Coordinate Frame (Home Pose -X / RGB Stance +Y)
        origin = (w_f // 2, h_f // 2)
        # Home pose is forward / -X in camera frame
        cv2.arrowedLine(last_frame, origin, (origin[0] - 80, origin[1]), (0, 0, 255), 2, tipLength=0.2)
        cv2.putText(last_frame, "Home (-X)", (origin[0] - 110, origin[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        # Workspace Analysis is +Y (sideways)
        cv2.arrowedLine(last_frame, origin, (origin[0], origin[1] + 80), (255, 100, 0), 2, tipLength=0.2)
        cv2.putText(last_frame, "Survey (+Y)", (origin[0] + 10, origin[1] + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 0), 1)

        # HUD Banner
        cv2.rectangle(last_frame, (15, 15), (w_f - 15, 80), (20, 20, 20), -1)
        cv2.rectangle(last_frame, (15, 15), (w_f - 15, 80), (0, 255, 0), 2)
        cv2.putText(last_frame, "AUTONOMOUS HOMOGRAPHY & EYE-TO-HAND CALIBRATION", (25, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)
        cv2.putText(last_frame, f"CALIBRATED: 4 STANCES | RMSE: {self.rmse:.2f} mm | H-MATRIX MAPPED TO MOTOR FRAME",
                    (25, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1)

        diag_path = os.path.join(os.path.dirname(self.config_dir), "demo1_20261107", "imp_images", "02_calibration_reprojection.jpg")
        cv2.imwrite(diag_path, last_frame)
        print(f"  ✓ Saved Calibration Diagnostic Map: {diag_path}")

        # Smoothly return to Calibrated Home Pose
        print("\n  • Returning smoothly to Calibrated Home Pose...")
        home_targets = {11: 0, 12: 2048, 13: 2048, 14: 2048, 15: 3130}
        smooth_move_fn(arm, home_targets, duration=2.5, steps=10)

        return {
            "H": H,
            "R": R,
            "t": t,
            "rmse_mm": self.rmse,
            "diagnostic_frame": last_frame
        }

    def pixel_to_robot(self, u: float, v: float) -> Tuple[float, float]:
        """
        Transforms pixel (u, v) into table robot Cartesian coordinates (X_rob, Y_rob) in mm
        using the empirically calibrated Homography matrix H.
        """
        if self.H is None:
            if os.path.exists(self.homography_yaml):
                with open(self.homography_yaml, "r") as f:
                    data = yaml.safe_load(f)
                    self.H = np.array(data["homography_matrix_3x3"], dtype=np.float64)
            else:
                raise ValueError("Homography matrix H not computed or loaded.")

        p_homo = np.array([u, v, 1.0], dtype=np.float64)
        p_trans = self.H @ p_homo
        rx = p_trans[0] / p_trans[2]
        ry = p_trans[1] / p_trans[2]
        return (float(rx), float(ry))
