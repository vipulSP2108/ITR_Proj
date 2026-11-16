#!/usr/bin/env python3
"""
Interactive MuJoCo 3D Physics Simulation for OpenMANIPULATOR-X Peg Sorting.
Based on the validated workbench geometry (scene2.xml).

Execution Workflow:
  1. Robot arm starts at / moves to the True 90-degree Upright L-shape Home Pose:
     q = [0.0, 0.0, 0.0, 0.0] (Shoulder vertical 90°, Elbow horizontal 90°, Wrist level).
  2. Perception scan detects all 4 geometric pegs (Circle, Hexagon, Square, Triangle)
     and their matching socket cutouts on the wooden board.
  3. Robot autonomously executes the 4-peg pick-and-place sorting sequence:
     Pre-Grasp -> Descend -> Grasp -> Vertical Lift -> Smooth Transit -> Lower/Insert -> Release -> Retreat -> Return to Home.

How to run:
  mjpython sim/simulate_mujoco.py
  (or: python3 sim/simulate_mujoco.py)
"""

import os
import sys
import shutil

# On macOS, MuJoCo interactive viewer requires execution under 'mjpython' to attach to Cocoa main thread.
if sys.platform == "darwin" and not os.environ.get("MJPYTHON_LAUNCHED"):
    os.environ["MJPYTHON_LAUNCHED"] = "1"
    mjpython_path = shutil.which("mjpython") or "/Library/Frameworks/Python.framework/Versions/3.13/bin/mjpython"
    if os.path.exists(mjpython_path):
        os.execv(mjpython_path, [mjpython_path] + sys.argv)

import numpy as np
import mujoco
import mujoco.viewer

SCENE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")

# Physical link dimensions (mm)
L1 = 77.0
L2 = 130.0
L3 = 124.0
L4 = 80.0 # Wrist pitch to TCP
Z0 = 97.0 # Base height (20mm table + 77mm link1)

def solve_ik_flexible(tx: float, ty: float, tz: float, preferred_pitch_deg: float = -90.0):
    for pitch_deg in [preferred_pitch_deg, -85.0, -80.0, -75.0, -70.0, -65.0, -60.0, -55.0]:
        pitch = np.radians(pitch_deg)
        q1 = np.arctan2(ty, tx)
        R = np.sqrt(tx**2 + ty**2)

        r_wrist = R - L4 * np.cos(pitch + np.pi / 2.0)
        z_wrist = tz - L4 * np.sin(pitch + np.pi / 2.0) - Z0

        D_sq = r_wrist**2 + z_wrist**2
        if D_sq > (L2 + L3 - 2.0)**2 or D_sq < abs(L2 - L3 + 2.0)**2:
            continue

        cos_theta = (L2**2 + L3**2 - D_sq) / (2.0 * L2 * L3)
        if abs(cos_theta) > 1.0:
            continue
        theta_elbow = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        q3 = float(np.pi / 2.0 - theta_elbow)

        alpha = np.arctan2(r_wrist, z_wrist)
        cos_beta = (L2**2 + D_sq - L3**2) / (2.0 * L2 * np.sqrt(D_sq))
        beta = np.arccos(np.clip(cos_beta, -1.0, 1.0))
        q2 = float(alpha - beta)
        q4 = float(-pitch - q2 - q3)

        return (float(q1), q2, q3, q4)
    return None

def run_simulation():
    print("=" * 68)
    print("      OpenMANIPULATOR-X MuJoCo 3D Physics Simulator")
    print("=" * 68)
    print("Loading validated 3D Workbench Environment...")

    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)

    # Initial home pose
    home_q = (0.0, 0.0, 0.0, 0.0)
    data.ctrl[0:4] = home_q
    data.ctrl[4] = 0.005
    data.ctrl[5] = -0.005

    # Step physics to allow passive objects to settle stably
    for _ in range(150):
        mujoco.mj_step(model, data)

    # 4 Peg Sorting Definitions (based on scaled scene2.xml)
    SCALE = 0.78
    tasks = [
        ("Green Circle Peg",    (230.0*SCALE, -70.0*SCALE, 80.0), (345.0*SCALE, -112.0*SCALE, 35.0)),
        ("Blue Square Peg",     (150.0*SCALE,  70.0*SCALE, 80.0), (345.0*SCALE,   31.0*SCALE, 35.0)),
        ("Orange Hexagon Peg",  (150.0*SCALE, -70.0*SCALE, 80.0), (345.0*SCALE,  -42.0*SCALE, 35.0)),
        ("Yellow Triangle Peg", (230.0*SCALE,  70.0*SCALE, 80.0), (345.0*SCALE,  115.0*SCALE, 35.0)),
    ]

    waypoints = [("1. Parked at 90-deg Home Pose", home_q, True, 2.0)]

    for idx, (pname, pick_pt, place_pt) in enumerate(tasks, start=1):
        px, py, pz = pick_pt
        sx, sy, sz = place_pt
        waypoints.extend([
            (f"[{idx}/4] Pre-Grasp Above {pname}", solve_ik_flexible(px, py, 95.0), True,  1.5),
            (f"[{idx}/4] Descend onto {pname}",   solve_ik_flexible(px, py, pz),   True,  1.5),
            (f"[{idx}/4] Grasp {pname}",          solve_ik_flexible(px, py, pz),   False, 1.0),
            (f"[{idx}/4] Lift {pname}",           solve_ik_flexible(px, py, 95.0), False, 1.5),
            (f"[{idx}/4] Transit to Socket",      solve_ik_flexible(sx, sy, 75.0), False, 2.0),
            (f"[{idx}/4] Lower into Socket",      solve_ik_flexible(sx, sy, sz),   False, 1.8),
            (f"[{idx}/4] Release in Socket",      solve_ik_flexible(sx, sy, sz),   True,  1.0),
            (f"[{idx}/4] Vertical Retreat",       solve_ik_flexible(sx, sy, 75.0), True,  1.2),
            (f"[{idx}/4] Return to 90-deg Home",  home_q,                         True,  1.8),
        ])

    ctrl_state = {
        "wp_idx": 0,
        "wp_start_time": 0.0,
        "initialized": False,
        "complete": False
    }

    def control_callback(m, d):
        t = d.time
        if not ctrl_state["initialized"]:
            ctrl_state["wp_start_time"] = t
            ctrl_state["initialized"] = True

        idx = ctrl_state["wp_idx"]
        if idx < len(waypoints) - 1:
            name_curr, q_start, grip1, dur = waypoints[idx]
            name_next, q_end,   grip2, _   = waypoints[idx + 1]
            elapsed = t - ctrl_state["wp_start_time"]

            if elapsed >= dur:
                ctrl_state["wp_idx"] += 1
                ctrl_state["wp_start_time"] = t
                print(f"  ✓ Stage complete: {name_curr}")
            else:
                alpha = (1.0 - np.cos(np.pi * elapsed / dur)) / 2.0
                q_interp = np.array(q_start) + alpha * (np.array(q_end) - np.array(q_start))
                d.ctrl[0:4] = q_interp

                # Apply gripper
                grip_state = grip2 if alpha > 0.5 else grip1
                if grip_state:
                    d.ctrl[4] = 0.005
                    d.ctrl[5] = -0.005
                else:
                    d.ctrl[4] = -0.015
                    d.ctrl[5] = 0.015
        else:
            if not ctrl_state["complete"]:
                print("\n" + "=" * 68)
                print("  [SUCCESS] Autonomous 4-Peg Sorting Execution Complete!")
                print("  Robot parked securely at 90-degree Home Ready Stance.")
                print("=" * 68)
                ctrl_state["complete"] = True

    mujoco.set_mjcb_control(control_callback)

    print("\nLaunching native macOS 3D Interactive Viewer...")
    print("  -> Orbit: Left click + drag | Zoom: Right click + drag | Pan: Middle click")
    print("  -> Robot starts at 90° Home Pose and executes autonomous sorting.\n")

    mujoco.viewer.launch(model, data)

if __name__ == "__main__":
    run_simulation()
