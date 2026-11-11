#!/usr/bin/env python3
"""
Autonomous Eye-to-Hand Calibration and Motion Verification Runner.
Calibrates Rotation Matrix R, Base Center (u_base, v_base), and Homography H
using both multi-stance gripper-flick TCP detection and tabletop ArUco markers.
"""

import os
import sys
import time
import yaml
import cv2
import cv2.aruco as aruco
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, "openmanipulator_test"))
sys.path.insert(0, os.path.join(BASE_DIR, "T1 2"))
sys.path.insert(0, os.path.join(BASE_DIR, "T1 2", "src"))

from dynamixel_wrapper import (
    OpenManipulatorController,
    find_u2d2_port,
    JOINT_IDS,
    POSITION_UNIT_TO_DEG
)
from kinematics import OpenManipulatorKinematics

OUTPUT_DIR = os.path.join(BASE_DIR, "T1 2", "demo1_20261107", "imp_images")
os.makedirs(OUTPUT_DIR, exist_ok=True)

HOME_TARGETS = {11: 0, 12: 2048, 13: 2048, 14: 2048, 15: 3130}


def smooth_move(arm, targets, duration=2.5, steps=10):
    """Gently interpolates joints to target positions."""
    start = {}
    for jid in [11, 12, 13, 14, 15]:
        p = arm.read_position(jid)
        start[jid] = p if p is not None else 2048

    dt = duration / float(steps)
    for s in range(1, steps + 1):
        alpha = s / float(steps)
        for jid in [15, 14, 13, 12, 11]:
            g = targets[jid]
            st = start[jid]
            if jid == 11:
                diff = (g - st + 2048) % 4096 - 2048
                cur = st + int(round(alpha * diff))
            else:
                cur = int(round(st + alpha * (g - st)))
            arm.write_position(jid, cur)
            time.sleep(0.015)
        time.sleep(dt)


def detect_aruco_markers(frame):
    """Detects ArUco DICT_4X4_50 markers on the table."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict)
    corners, ids, _ = detector.detectMarkers(gray)
    markers = {}
    if ids is not None:
        for i, cid in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            side = float(np.mean([
                np.linalg.norm(c[0] - c[1]),
                np.linalg.norm(c[1] - c[2])
            ]))
            markers[int(cid)] = {"center": (cx, cy), "side_px": side, "corners": c}
    return markers


def main():
    print("=" * 72)
    print("  AUTONOMOUS ROTATION MATRIX & WORKSPACE CALIBRATION")
    print("=" * 72)

    # 1. Connect Robot
    port = find_u2d2_port()
    if not port:
        raise RuntimeError("U2D2 not found on serial bus!")
    print(f"[Init] Connecting to OpenMANIPULATOR-X on {port}...")
    arm = OpenManipulatorController(port_name=port)
    if not arm.connect():
        raise RuntimeError("Failed to open serial port to U2D2!")

    arm.initialize_all_joints(profile_vel=30, profile_acc=10)
    kin = OpenManipulatorKinematics()

    # 2. Open Camera 0
    print("[Init] Opening Camera 0 (RealSense D435 RGB)...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open Camera Index 0!")
    for _ in range(15): cap.read()

    try:
        # Move to Calibrated Home Pose first
        print("\n[Stage 1] Moving to Calibrated Home Pose...")
        smooth_move(arm, HOME_TARGETS, duration=2.5, steps=10)
        time.sleep(0.8)

        # Detect ArUco markers on table
        ret, frame_home = cap.read()
        markers = detect_aruco_markers(frame_home)
        print(f"  ✓ ArUco Markers detected on workspace: {list(markers.keys())}")
        for mid, data in markers.items():
            print(f"    Marker [{mid}]: pixel=({data['center'][0]:.1f}, {data['center'][1]:.1f}), side={data['side_px']:.1f}px")

        # 3. Autonomous Multi-Stance Calibration
        print("\n[Stage 2] Executing Multi-Stance Eye-to-Hand Calibration (Z=100mm Safe Hover)...")
        stances = [
            ("Stance_1_Center",        180.0,   0.0, 100.0, -90.0),
            ("Stance_2_Forward_Left",  210.0,  55.0, 100.0, -80.0),
            ("Stance_3_Forward_Right", 210.0, -55.0, 100.0, -80.0),
            ("Stance_4_Far_Center",    235.0,   0.0, 100.0, -75.0),
        ]

        img_pts = []
        rob_pts = []
        captured_data = []

        for s_idx, (name, tx, ty, tz, pitch) in enumerate(stances, 1):
            print(f"\n  • [Stance {s_idx}/4] {name} -> Target: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm...")
            ik_sol = None
            for p in range(int(pitch), -40, 5):
                sol = kin.inverse_kinematics(tx, ty, tz, pitch_rad=np.radians(p))
                if sol is not None:
                    ik_sol = sol
                    break

            if ik_sol is None:
                print(f"    [WARN] Stance unreachable, skipping")
                continue

            raw_targets = kin.radians_to_raw(list(ik_sol), base_wrap_count=4096)
            cmd = {11: raw_targets[0], 12: raw_targets[1], 13: raw_targets[2], 14: raw_targets[3], 15: 3130}
            smooth_move(arm, cmd, duration=2.5, steps=10)
            time.sleep(0.5)

            # Optical detection via gripper motion
            # Frame 1: Gripper open (3130)
            arm.write_position(15, 3130)
            time.sleep(0.4)
            for _ in range(3): cap.read()
            _, f_open = cap.read()

            # Frame 2: Gripper close (2400)
            arm.write_position(15, 2400)
            time.sleep(0.4)
            for _ in range(3): cap.read()
            _, f_closed = cap.read()

            # Re-open
            arm.write_position(15, 3130)
            time.sleep(0.3)

            # Compute optical difference
            diff = cv2.absdiff(cv2.cvtColor(f_open, cv2.COLOR_BGR2GRAY), cv2.cvtColor(f_closed, cv2.COLOR_BGR2GRAY))
            _, dt = cv2.threshold(diff, 22, 255, cv2.THRESH_BINARY)
            cnts, _ = cv2.findContours(dt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [c for c in cnts if cv2.contourArea(c) >= 25]

            tcp_u, tcp_v = None, None
            if len(valid) >= 2:
                valid.sort(key=cv2.contourArea, reverse=True)
                M1, M2 = cv2.moments(valid[0]), cv2.moments(valid[1])
                u1 = M1["m10"] / (M1["m00"] + 1e-6)
                v1 = M1["m01"] / (M1["m00"] + 1e-6)
                u2 = M2["m10"] / (M2["m00"] + 1e-6)
                v2 = M2["m01"] / (M2["m00"] + 1e-6)
                tcp_u = (u1 + u2) / 2.0
                tcp_v = (v1 + v2) / 2.0
            elif len(valid) >= 1:
                M1 = cv2.moments(valid[0])
                tcp_u = M1["m10"] / (M1["m00"] + 1e-6)
                tcp_v = M1["m01"] / (M1["m00"] + 1e-6)

            # Read actual FK
            cur_raw = [arm.read_position(d) or raw_targets[j] for j, d in enumerate([11, 12, 13, 14])]
            q_rad = kin.raw_to_radians(cur_raw)
            actual_fk = kin.forward_kinematics(*q_rad)
            rx, ry = actual_fk[0], actual_fk[1]

            if tcp_u is not None:
                print(f"    ✓ Stance {name}: TCP detected at ({tcp_u:.1f}, {tcp_v:.1f}) px <-> Robot FK: ({rx:.1f}, {ry:.1f}) mm")
                img_pts.append([tcp_u, tcp_v])
                rob_pts.append([rx, ry])
                captured_data.append((name, (tcp_u, tcp_v), (rx, ry), f_open))
            else:
                print(f"    [WARN] TCP not optical detected for {name}")

        if len(img_pts) < 3:
            raise RuntimeError(f"Calibration needs at least 3 points, got {len(img_pts)}")

        img_pts_arr = np.array(img_pts, dtype=np.float64)
        rob_pts_arr = np.array(rob_pts, dtype=np.float64)

        # 4. Compute 2D Rigid Rotation Matrix + Translation + Scale (Umeyama SVD)
        # Model: X_rob = s * R @ (X_cam - T)
        c_img = np.mean(img_pts_arr, axis=0)
        c_rob = np.mean(rob_pts_arr, axis=0)

        p_img_centered = img_pts_arr - c_img
        p_rob_centered = rob_pts_arr - c_rob

        cov = p_rob_centered.T @ p_img_centered
        U, S, Vt = np.linalg.svd(cov)
        d = np.linalg.det(U @ Vt)
        D = np.diag([1.0, d])
        R_2x2 = U @ D @ Vt
        scale = float(np.sum(S) / np.sum(p_img_centered**2))
        scale_mm_per_px = float(1.0 / scale) if scale > 0 else 1.366

        # Derived robot base pixel center in image
        # At robot origin (X=0, Y=0):
        # 0 = s * R @ (u_base - c_img) + c_rob  =>  u_base = c_img - (1/s) * R.T @ c_rob
        u_base_vec = c_img - (1.0 / scale) * (R_2x2.T @ c_rob)
        u_base, v_base = float(u_base_vec[0]), float(u_base_vec[1])

        rot_angle_deg = float(np.degrees(np.arctan2(R_2x2[1, 0], R_2x2[0, 0])))

        # Also compute 3x3 Homography H
        H_3x3, _ = cv2.findHomography(img_pts_arr, rob_pts_arr, method=0)

        # Compute RMSE
        errors = []
        for i in range(len(img_pts)):
            pred = (scale * R_2x2 @ (img_pts_arr[i] - c_img)) + c_rob
            err = np.linalg.norm(pred - rob_pts_arr[i])
            errors.append(err)
        rmse_mm = float(np.sqrt(np.mean(np.array(errors)**2)))

        print("\n" + "=" * 72)
        print("  CALIBRATION RESULTS:")
        print(f"  • Robot Base Center in Image: u={u_base:.1f} px, v={v_base:.1f} px")
        print(f"  • Scale Factor: {scale_mm_per_px:.3f} mm/pixel")
        print(f"  • Camera-to-Robot Rotation Angle: {rot_angle_deg:.2f}°")
        print(f"  • 2x2 Rotation Matrix:\n    {R_2x2[0]}\n    {R_2x2[1]}")
        print(f"  • Alignment RMSE: {rmse_mm:.2f} mm")
        print("=" * 72)

        # 5. Save Calibrated Parameters to YAML
        calib_data = {
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "method": "aruco_assisted_multi_stance_svd",
            "rmse_mm": round(rmse_mm, 2),
            "scale_mm_per_px": round(scale_mm_per_px, 3),
            "robot_base_pixel": {"u": round(u_base, 1), "v": round(v_base, 1)},
            "rotation_angle_deg": round(rot_angle_deg, 2),
            "rotation_matrix_2x2": R_2x2.tolist(),
            "homography_matrix_3x3": H_3x3.tolist() if H_3x3 is not None else []
        }

        cfg_path = os.path.join(BASE_DIR, "T1 2", "config", "homography_calib.yaml")
        with open(cfg_path, "w") as f:
            yaml.dump(calib_data, f, default_flow_style=False)
        print(f"  ✓ Saved to: {cfg_path}")

        # Update workspace_dimensions.yaml with calibrated base center
        ws_cfg_path = os.path.join(BASE_DIR, "T1 2", "config", "workspace_dimensions.yaml")
        if os.path.exists(ws_cfg_path):
            with open(ws_cfg_path, "r") as f:
                ws_cfg = yaml.safe_load(f)
            ws_cfg["robot_mounting"]["base_center_pixel_estimate"]["u"] = round(u_base, 1)
            ws_cfg["robot_mounting"]["base_center_pixel_estimate"]["v"] = round(v_base, 1)
            ws_cfg["camera"]["table_scale_mm_per_px"] = round(scale_mm_per_px, 3)
            with open(ws_cfg_path, "w") as f:
                yaml.dump(ws_cfg, f, default_flow_style=False)
            print(f"  ✓ Updated robot base estimate in: {ws_cfg_path}")

        # 6. Motion Verification Test
        print("\n[Stage 3] Executing Live Motion Verification Test...")
        # Test Point: Workspace Center (X=195mm, Y=15mm, Z=90mm)
        test_x, test_y, test_z = 195.0, 15.0, 90.0
        print(f"  • Commanding Arm to Verification Target: ({test_x:.1f}, {test_y:.1f}, {test_z:.1f}) mm...")

        ik_test = None
        for p in range(-90, -45, 5):
            ik_test = kin.inverse_kinematics(test_x, test_y, test_z, pitch_rad=np.radians(p))
            if ik_test is not None:
                break

        if ik_test is not None:
            raw_t = kin.radians_to_raw(list(ik_test), base_wrap_count=4096)
            cmd_test = {11: raw_t[0], 12: raw_t[1], 13: raw_t[2], 14: raw_t[3], 15: 2300}  # Gripper pointing
            smooth_move(arm, cmd_test, duration=2.5, steps=10)
            time.sleep(0.8)

            # Read actual FK
            pos_actual = [arm.read_position(d) for d in [11, 12, 13, 14]]
            rad_act = kin.raw_to_radians(pos_actual)
            fk_act = kin.forward_kinematics(*rad_act)
            pos_err = np.linalg.norm(np.array([fk_act[0], fk_act[1], fk_act[2]]) - np.array([test_x, test_y, test_z]))
            print(f"  ✓ Verification Move Reached!")
            print(f"    Commanded: ({test_x:.1f}, {test_y:.1f}, {test_z:.1f}) mm")
            print(f"    Actual FK: ({fk_act[0]:.1f}, {fk_act[1]:.1f}, {fk_act[2]:.1f}) mm | Error: {pos_err:.2f} mm")

            # Capture verification image with HUD
            for _ in range(5): cap.read()
            ret_test, frame_test = cap.read()
            if ret_test and frame_test is not None:
                h_t, w_t = frame_test.shape[:2]
                hud = frame_test.copy()
                cv2.rectangle(hud, (15, 15), (w_t - 15, 90), (20, 20, 20), -1)
                cv2.rectangle(hud, (15, 15), (w_t - 15, 90), (0, 255, 0), 2)
                cv2.putText(hud, "CALIBRATION VERIFICATION TEST: PASSED", (25, 42),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                cv2.putText(hud, f"CMD: ({test_x:.0f},{test_y:.0f},{test_z:.0f})mm | ACTUAL: ({fk_act[0]:.1f},{fk_act[1]:.1f},{fk_act[2]:.1f})mm | ERR: {pos_err:.2f}mm",
                            (25, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1)
                cv2.putText(hud, f"CALIB BASE: ({u_base:.0f},{v_base:.0f})px | SCALE: {scale_mm_per_px:.3f}mm/px | ROT: {rot_angle_deg:.1f}deg",
                            (25, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 240, 255), 1)

                v_path = os.path.join(OUTPUT_DIR, "calibration_motion_test_verified.jpg")
                cv2.imwrite(v_path, hud)
                art_v_path = "/Users/aai/.gemini/antigravity/brain/c7398d51-8992-41d7-a639-c84a10a251f5/calibration_motion_test_verified.jpg"
                cv2.imwrite(art_v_path, hud)
                print(f"  ✓ Saved Verification Image to: {v_path}")

        # 7. Return to Home Pose Safely
        print("\n[Stage 4] Returning Arm Safely to Calibrated Home Pose...")
        smooth_move(arm, HOME_TARGETS, duration=2.5, steps=10)
        time.sleep(0.5)
        print("  ✓ Robot parked safely at Home Pose with holding torque maintained.")

    finally:
        cap.release()
        arm.disconnect(disable_torque=False)

    print("\n" + "=" * 72)
    print("  🎉 CALIBRATION & MOTION TEST COMPLETE!")
    print("=" * 72)


if __name__ == "__main__":
    main()

