#!/usr/bin/env python3
"""
Unit tests for the new extensions in sim_demo:
  1. RobotInterface & SimRobot HAL (robot_interface.py)
  2. Quintic Trajectory & DLS-IK (trajectory.py)
  3. Camera Drift Monitor & Online Recalibration (drift_monitor.py)
  4. Multi-Shape Symmetry & Sequencing (multi_shape.py)
"""

import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DEMO = os.path.dirname(HERE)
if SIM_DEMO not in sys.path:
    sys.path.insert(0, SIM_DEMO)

from sim_env import SimEnv
import arm_model as A
from robot_interface import SimRobot, RealRobot
from trajectory import QuinticTrajectory, DlsIkSolver, generate_smooth_cartesian_trajectory
from drift_monitor import SimCameraDriftMonitor
from multi_shape import minimal_symmetry_angle_diff, MultiShapePlanner


def test_robot_interface():
    env = SimEnv()
    bot = SimRobot(env)
    q = bot.read_joints()
    assert len(q) == 4, "read_joints must return 4 elements"

    # Test joint move
    bot.move_joints(A.HOME_Q, 0.5)
    bot.settle(0.1)
    q_new = bot.read_joints()
    assert np.allclose(q_new, A.HOME_Q, atol=0.03), "SimRobot joint move failed"

    # Test gripper
    bot.set_gripper(False, 0.2)
    state = bot.grasp_state()
    assert state in ("held", "empty", "unknown"), f"Unexpected grasp state: {state}"

    # Test RealRobot unit conversion math
    real_bot = RealRobot(dry_run=True)
    counts = [2048, 2048, 2048, 2048]
    rad = real_bot._counts_to_rad(counts)
    assert np.allclose(rad, [0, 0, 0, 0]), "Counts to rad mapping incorrect"
    back_counts = real_bot._rad_to_counts(rad)
    assert back_counts == counts, "Rad to counts round-trip failed"
    print("PASS  test_robot_interface")


def test_trajectory_quintic_and_dls():
    p0 = np.array([0.20, 0.0, 0.15])
    p1 = np.array([0.25, 0.05, 0.10])
    dur = 2.0

    # Start boundary condition: vel=0, acc=0
    pos_0, vel_0, acc_0 = QuinticTrajectory.interpolate(p0, p1, 0.0, dur)
    assert np.allclose(pos_0, p0), "Quintic start position mismatch"
    assert np.allclose(vel_0, 0.0), "Quintic start velocity not zero"
    assert np.allclose(acc_0, 0.0), "Quintic start acceleration not zero"

    # End boundary condition: vel=0, acc=0
    pos_end, vel_end, acc_end = QuinticTrajectory.interpolate(p0, p1, dur, dur)
    assert np.allclose(pos_end, p1), "Quintic end position mismatch"
    assert np.allclose(vel_end, 0.0), "Quintic end velocity not zero"
    assert np.allclose(acc_end, 0.0), "Quintic end acceleration not zero"

    # DLS-IK test
    dls = DlsIkSolver()
    target = np.array([0.22, -0.02, 0.12])
    q_sol = dls.solve(target, A.HOME_Q)
    assert q_sol is not None, "DLS-IK failed to find solution"
    actual_tcp = A.fk(q_sol)["tcp"]
    err = np.linalg.norm(target - actual_tcp)
    assert err < 0.005, f"DLS-IK error {err*1000:.1f} mm exceeds tolerance"

    # Smooth trajectory generation test
    traj = generate_smooth_cartesian_trajectory(p0, p1, tilt_rad=np.radians(45.0), duration=1.0, dt=0.1)
    assert len(traj) >= 10, "Generated trajectory too short"
    print("PASS  test_trajectory_quintic_and_dls")


def test_drift_monitor():
    monitor = SimCameraDriftMonitor(tol_trans_mm=3.0, tol_rot_deg=1.0)
    baseline_holes = [
        {"shape": "circle", "x": 0.295, "y": -0.115, "z": 0.008},
        {"shape": "hexagon", "x": 0.295, "y": -0.060, "z": 0.008},
        {"shape": "square", "x": 0.295, "y": 0.000, "z": 0.008},
        {"shape": "triangle", "x": 0.295, "y": 0.065, "z": 0.008},
    ]
    monitor.set_baseline_from_holes(baseline_holes)

    T_init = np.eye(4)
    # 1. No drift test
    drift, dt, da, T_out = monitor.check_and_update(baseline_holes, T_init, None)
    assert not drift, "False positive drift detected on baseline"
    assert dt < 0.1, "Drift distance should be ~0"

    # 2. Injected drift test: shift by 6 mm in X
    shifted_holes = [
        {"shape": h["shape"], "x": h["x"] + 0.006, "y": h["y"], "z": h["z"]}
        for h in baseline_holes
    ]
    drift, dt, da, T_updated = monitor.check_and_update(shifted_holes, T_init, None)
    assert drift, "Failed to detect 6 mm translational drift"
    assert abs(dt - 6.0) < 0.5, f"Measured drift {dt:.1f} mm differs from 6.0 mm"
    # Ensure updated transform has translation
    assert np.linalg.norm(T_updated[:3, 3]) > 0.003, "Updated transform not compensated"
    print("PASS  test_drift_monitor")


def test_multi_shape_planner():
    # Symmetry checks
    diff_circle = minimal_symmetry_angle_diff(np.radians(45), np.radians(180), "circle")
    assert diff_circle == 0.0, "Circle should have zero orientation difference"

    # Square has 90-deg symmetry: 10 deg vs 100 deg is identical (diff = 0)
    diff_square = minimal_symmetry_angle_diff(np.radians(10), np.radians(100), "square")
    assert abs(np.degrees(diff_square)) < 1e-4, "Square 90-degree symmetry failed"

    # Hexagon has 60-deg symmetry: 15 deg vs 75 deg is identical
    diff_hex = minimal_symmetry_angle_diff(np.radians(15), np.radians(75), "hexagon")
    assert abs(np.degrees(diff_hex)) < 1e-4, "Hexagon 60-degree symmetry failed"

    # Multi-peg sequencing
    planner = MultiShapePlanner(board_z=0.008)
    pegs = [
        {"shape": "square", "x": 0.16, "y": 0.05, "z_top": 0.108, "height": 0.10},
        {"shape": "circle", "x": 0.15, "y": -0.05, "z_top": 0.108, "height": 0.10},
    ]
    sockets = [
        {"shape": "circle", "x": 0.28, "y": -0.10, "z": 0.008},
        {"shape": "square", "x": 0.28, "y": 0.00, "z": 0.008},
    ]
    plans = planner.plan_all_pegs(pegs, sockets)
    assert len(plans) >= 1, "Failed to generate multi-peg plan"
    assert plans[0].shape == "circle", "Circle should be prioritized first"
    print("PASS  test_multi_shape_planner")


if __name__ == "__main__":
    print("=" * 60)
    print("  RUNNING EXTENSION TEST SUITE (T1 2 / sim_demo)")
    print("=" * 60)
    test_robot_interface()
    test_trajectory_quintic_and_dls()
    test_drift_monitor()
    test_multi_shape_planner()
    print("=" * 60)
    print("  ALL EXTENSION TESTS PASSED (4/4)")
    print("=" * 60)

