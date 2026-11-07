#!/usr/bin/env python3
"""
Unit and Integration Test Suite for Peg Sorting Edge Cases & Autonomous Recovery.
Tests:
  1. EC-05: Gripper slip & in-transit drop detection (PegDroppedException)
  2. EC-06: Workspace reachability classification, angled grasp solver & inward sweep
  3. EC-07: Camera extrinsic drift detection & closed-form Kabsch auto-recalibration (< 2 ms)
  4. EC-09: Archimedean micro-spiral compliant search generation
  5. RecoveryManager orchestration state transitions
"""

import os
import sys
import cv2
import unittest
import numpy as np

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
T1_DIR = os.path.dirname(CUR_DIR)
for p in [T1_DIR, os.path.join(T1_DIR, "src")]:
    if p not in sys.path:
        sys.path.insert(0, p)

from src.planning.kinematics import OpenManipulatorKinematics
from src.planning.workspace_analyzer import WorkspaceAnalyzer, ReachabilityStatus
from src.control.gripper_guard import GripperGuard, GraspState, PegDroppedException, EMPTY_CLOSED_LIMIT
from src.vision.camera_drift_monitor import CameraDriftMonitor, NOMINAL_BOARD_LANDMARKS
from src.vision.visual_drop_detector import VisualDropDetector, VisualDropStatus
from src.vision.synthetic_scene import SyntheticSceneGenerator
from src.planning.calibration import EyeToHandCalibrator
from src.planning.recovery_manager import RecoveryManager

class MockDynamixelArm:
    """Mock controller simulating DYNAMIXEL motor feedback for testing."""
    def __init__(self, simulate_peg_present: bool = True):
        self.is_connected = True
        self.positions = {11: 4096, 12: 2048, 13: 2048, 14: 2048, 15: 2100}
        self.current_raw = 40 # ~107 mA
        self.simulate_peg = simulate_peg_present
        self.packet_handler = self
        self.port_handler = self

    def read_position(self, dxl_id: int):
        return self.positions.get(dxl_id, 2048)

    def write_position(self, dxl_id: int, pos: int):
        if dxl_id == 15 and pos > 2200 and self.simulate_peg:
            # Physical peg stops gripper fingers at ~2080 counts with holding load
            self.positions[15] = 2080
            self.current_raw = 35 # ~94 mA holding current
        else:
            self.positions[dxl_id] = pos
            if dxl_id == 15 and pos < 1800:
                self.current_raw = 0 # Idle current when open
        return True

    def read2ByteTxRx(self, port, dxl_id, addr):
        return self.current_raw, 0, 0

class TestEdgeCaseRecovery(unittest.TestCase):
    def setUp(self):
        self.kin = OpenManipulatorKinematics()
        self.workspace = WorkspaceAnalyzer(kinematics=self.kin)

    # ---------------------------------------------------------
    # Test 1: Workspace Analysis & Out-of-Reach Recovery (EC-06)
    # ---------------------------------------------------------
    def test_workspace_classification_prime(self):
        # 160 mm radius is in the prime vertical workspace
        x, y, z = 140.0, 80.0, 60.0
        status, info = self.workspace.classify_target(x, y, z)
        self.assertEqual(status, ReachabilityStatus.REACHABLE_VERTICAL)
        self.assertAlmostEqual(info["pitch_deg"], -90.0)
        self.assertGreater(info["margin_mm"], 0.0)

    def test_workspace_classification_angled(self):
        # 240 mm radius cannot be reached strictly vertically (-90 deg), but is reachable angled
        x, y, z = 220.0, 95.0, 60.0 # r ~= 239.6 mm
        status, info = self.workspace.classify_target(x, y, z)
        self.assertEqual(status, ReachabilityStatus.REACHABLE_ANGLED)
        self.assertLess(info["pitch_deg"], -40.0)
        self.assertIsNotNone(info["ik_solution"])

    def test_workspace_sweep_generation(self):
        # 310 mm radius is in the sweepable zone
        x, y = 260.0, 170.0 # r ~= 310.6 mm
        status, info = self.workspace.classify_target(x, y, 60.0)
        self.assertEqual(status, ReachabilityStatus.REACHABLE_SWEEP)

        # Generate inward rake trajectory
        sweep_traj = self.workspace.generate_inward_sweep_trajectory(x, y)
        self.assertIsNotNone(sweep_traj)
        self.assertEqual(len(sweep_traj), 4)
        for step in sweep_traj:
            self.assertEqual(len(step["joints"]), 4)

    def test_workspace_out_of_bounds(self):
        # 420 mm is beyond hardware limits
        x, y, z = 350.0, 240.0, 60.0 # r ~= 424 mm
        status, info = self.workspace.classify_target(x, y, z)
        self.assertEqual(status, ReachabilityStatus.OUT_OF_BOUNDS)
        self.assertGreater(info["deficit_mm"], 0.0)

    # ---------------------------------------------------------
    # Test 2: Gripper Slip & Drop Detection (EC-05 & EC-08)
    # ---------------------------------------------------------
    def test_gripper_secure_hold(self):
        mock_arm = MockDynamixelArm()
        mock_arm.positions[15] = 2100 # Holding peg
        mock_arm.current_raw = 30     # Drawing 80 mA
        guard = GripperGuard(controller=mock_arm)

        state, telemetry = guard.evaluate_grasp()
        self.assertEqual(state, GraspState.SECURE_HOLD)
        self.assertTrue(telemetry["holding_active"])

    def test_gripper_empty_air(self):
        mock_arm = MockDynamixelArm()
        mock_arm.positions[15] = 2380 # Closed all the way on empty air
        mock_arm.current_raw = 4      # Drawing ~10 mA
        guard = GripperGuard(controller=mock_arm)

        state, telemetry = guard.evaluate_grasp()
        self.assertEqual(state, GraspState.EMPTY_AIR)
        self.assertFalse(telemetry["holding_active"])

    def test_in_transit_drop_exception(self):
        mock_arm = MockDynamixelArm()
        guard = GripperGuard(controller=mock_arm)
        guard._holding_active = True # Arm is in transit holding a peg

        # Simulate drop event: position jumps to empty closed and current collapses
        mock_arm.positions[15] = 2360
        mock_arm.current_raw = 2

        with self.assertRaises(PegDroppedException):
            guard.monitor_transit_trajectory(duration_sec=0.1, poll_hz=20.0)

    # ---------------------------------------------------------
    # Test 3: Camera Drift & Online Auto-Recalibration (EC-07)
    # ---------------------------------------------------------
    def test_camera_drift_detection_and_recalibration(self):
        calib = EyeToHandCalibrator()
        calib.load_calibration()
        R_nom = calib.R.copy()
        t_nom = calib.t.copy()

        monitor = CameraDriftMonitor(calibrator=calib)

        # Baseline: No drift (below 0.5 mm threshold)
        P_cam_baseline = (R_nom.T @ (NOMINAL_BOARD_LANDMARKS - t_nom).T).T
        drift_det, dt, dr, _ = monitor.compute_drift(P_cam_baseline)
        self.assertFalse(drift_det)
        self.assertLess(dt, 0.5)
        self.assertLess(dr, 0.5)

        # Simulate camera bumped by +16 mm translation and +3.5 deg rotation
        t_bump = t_nom + np.array([12.0, -6.0, 9.0])
        theta = np.radians(3.5)
        c, s = np.cos(theta), np.sin(theta)
        R_bump = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R_nom
        P_cam_bumped = (R_bump.T @ (NOMINAL_BOARD_LANDMARKS - t_bump).T).T

        # Drift MUST be detected
        drift_det, dt, dr, _ = monitor.compute_drift(P_cam_bumped)
        self.assertTrue(drift_det)
        self.assertGreater(dt, 10.0)
        self.assertGreater(dr, 3.0)

        # Online Recalibration via Kabsch
        R_new, t_new, rmse = monitor.recalibrate_online(P_cam_bumped, save_to_disk=False)
        self.assertLess(rmse, 0.5) # Sub-millimeter residual error

        # Post-recalibration drift check MUST pass
        drift_after, dt_after, dr_after, _ = monitor.compute_drift(P_cam_bumped)
        self.assertFalse(drift_after)
        self.assertLess(dt_after, 0.5)
        self.assertLess(dr_after, 0.5)

    # ---------------------------------------------------------
    # Test 4: Archimedean Micro-Spiral Search (EC-09)
    # ---------------------------------------------------------
    def test_spiral_search_offsets(self):
        offsets = GripperGuard.generate_spiral_search_offsets(max_radius=2.0, num_steps=12)
        self.assertEqual(len(offsets), 13) # Origin + 12 steps
        self.assertEqual(offsets[0], (0.0, 0.0))

        # Ensure radius monotonically increases to max_radius
        radii = [np.sqrt(dx**2 + dy**2) for dx, dy in offsets]
        self.assertAlmostEqual(radii[-1], 2.0, delta=0.05)
        for i in range(len(radii) - 1):
            self.assertLessEqual(radii[i], radii[i+1] + 1e-4)

    # ---------------------------------------------------------
    # Test 5: RecoveryManager Orchestration
    # ---------------------------------------------------------
    def test_recovery_manager_drop_sequence(self):
        mock_arm = MockDynamixelArm()
        rm = RecoveryManager(arm_controller=mock_arm)

        # Trigger drop recovery for square peg
        success = rm.recover_from_drop(shape_name="square", target_socket_xy=(160.0, -120.0))
        self.assertTrue(success)

    # ---------------------------------------------------------
    # Test 6: Visual Drop & Slip Camera Detection (EC-05 Vision)
    # ---------------------------------------------------------
    def test_visual_drop_secure_in_hand(self):
        detector = VisualDropDetector()
        # Create synthetic scene with elevated blue square peg in hand
        # End effector at (160.0, 60.0, 85.0) in robot frame
        # Camera is calibrated; convert robot (160, 60, 85) to camera (X, Y, Z)
        p_rob = (160.0, 60.0, 85.0)
        p_cam = detector.calibrator.R.T @ (np.array(p_rob) - detector.calibrator.t)
        u, v = detector.detector.intrinsics.project_point_to_pixel(p_cam[0], p_cam[1], p_cam[2])

        # Draw blue square at (u, v)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        u_int, v_int = int(round(u)), int(round(v))
        cv2.rectangle(rgb, (u_int - 20, v_int - 20), (u_int + 20, v_int + 20), (220, 50, 40), -1)

        depth = np.full((480, 640), float(p_cam[2]), dtype=np.float32)

        status, info = detector.verify_post_lift(
            rgb_image=rgb,
            depth_map=depth,
            shape_name="square",
            original_pickup_robot_xy=(160.0, 60.0),
            ee_robot_xyz=(160.0, 60.0, 85.0)
        )
        self.assertEqual(status, VisualDropStatus.SECURE_IN_HAND)

    def test_visual_drop_missed_at_pickup(self):
        detector = VisualDropDetector()
        # Peg remains on table at pickup spot (160, 60, 25) while EE lifted to 85mm
        p_rob_pickup = (160.0, 60.0, 25.0)
        p_cam_pickup = detector.calibrator.R.T @ (np.array(p_rob_pickup) - detector.calibrator.t)
        u, v = detector.detector.intrinsics.project_point_to_pixel(p_cam_pickup[0], p_cam_pickup[1], p_cam_pickup[2])

        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        u_int, v_int = int(round(u)), int(round(v))
        cv2.rectangle(rgb, (u_int - 20, v_int - 20), (u_int + 20, v_int + 20), (220, 50, 40), -1)
        depth = np.full((480, 640), float(p_cam_pickup[2]), dtype=np.float32)

        status, info = detector.verify_post_lift(
            rgb_image=rgb,
            depth_map=depth,
            shape_name="square",
            original_pickup_robot_xy=(160.0, 60.0),
            ee_robot_xyz=(160.0, 60.0, 85.0) # EE is up at 85mm
        )
        self.assertEqual(status, VisualDropStatus.MISSED_AT_PICKUP)

    def test_visual_detect_tabletop_drop(self):
        detector = VisualDropDetector()
        # Peg dropped on tabletop at (180.0, 20.0, 25.0)
        p_rob_drop = (180.0, 20.0, 25.0)
        p_cam_drop = detector.calibrator.R.T @ (np.array(p_rob_drop) - detector.calibrator.t)
        u, v = detector.detector.intrinsics.project_point_to_pixel(p_cam_drop[0], p_cam_drop[1], p_cam_drop[2])

        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        u_int, v_int = int(round(u)), int(round(v))
        cv2.rectangle(rgb, (u_int - 20, v_int - 20), (u_int + 20, v_int + 20), (220, 50, 40), -1)
        depth = np.full((480, 640), float(p_cam_drop[2]), dtype=np.float32)

        dropped, pose = detector.detect_tabletop_dropped_peg(rgb, depth, shape_name="square")
        self.assertTrue(dropped)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.robot_3d[0], 180.0, delta=5.0)
        self.assertAlmostEqual(pose.robot_3d[1], 20.0, delta=5.0)

    def test_visual_socket_occupancy(self):
        detector = VisualDropDetector()
        # Socket at (160.0, -120.0, 38.0)
        p_rob_socket = (160.0, -120.0, 38.0)
        p_cam_socket = detector.calibrator.R.T @ (np.array(p_rob_socket) - detector.calibrator.t)
        u, v = detector.detector.intrinsics.project_point_to_pixel(p_cam_socket[0], p_cam_socket[1], p_cam_socket[2])

        # 1. Filled socket
        rgb_filled = np.zeros((480, 640, 3), dtype=np.uint8)
        u_int, v_int = int(round(u)), int(round(v))
        cv2.rectangle(rgb_filled, (u_int - 18, v_int - 18), (u_int + 18, v_int + 18), (220, 50, 40), -1)
        depth = np.full((480, 640), float(p_cam_socket[2]), dtype=np.float32)

        status, _ = detector.verify_socket_placement(rgb_filled, depth, "square", expected_socket_robot_xy=(160.0, -120.0))
        self.assertEqual(status, VisualDropStatus.SOCKET_FILLED)

        # 2. Empty socket
        rgb_empty = np.zeros((480, 640, 3), dtype=np.uint8)
        status_empty, _ = detector.verify_socket_placement(rgb_empty, depth, "square", expected_socket_robot_xy=(160.0, -120.0))
        self.assertEqual(status_empty, VisualDropStatus.SOCKET_EMPTY)

if __name__ == "__main__":
    unittest.main()
