#!/usr/bin/env python3
"""
Off-Screen MuJoCo 3D Video Renderer for OpenMANIPULATOR-X Peg Sorting.
Renders collision-free autonomous multi-peg sorting using validated scene2 geometry.

Features:
  • Starts at True 90-degree Upright L-shape Home Ready Pose: q = [0.0, 0.0, 0.0, 0.0].
  • All 4 pegs (Green Circle, Blue Square, Orange Hexagon, Yellow Triangle) sorted into matching sockets.
  • Vertical approach and high altitude transit (Z=125mm) ensuring zero collision with adjacent standing pegs.
  • Smooth 25 FPS video with real-time HUD stage tracker.

Output:
  sim/openmanipulator_pick_and_place.mp4
"""

import os
import cv2
import numpy as np
import mujoco

SCENE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
OUTPUT_VIDEO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "openmanipulator_pick_and_place.mp4")

# Kinematics dimensions (mm)
L1 = 77.0
L2 = 130.0
L3 = 124.0
L4 = 80.0
Z0 = 97.0

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

def render_simulation_video(fps: float = 25.0):
    print("=" * 68)
    print("   OpenMANIPULATOR-X 3D Simulation Video Generator (scene2)")
    print("=" * 68)

    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=720, width=1280)

    # Rail-side perspective
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 0.70
    camera.elevation = -42.0
    camera.azimuth = 75.0
    camera.lookat[:] = [0.18, -0.01, 0.04]

    SCALE = 0.78
    home_q = (0.0, 0.0, 0.0, 0.0)

    tasks = [
        ("Green Circle Peg",    (230.0*SCALE, -70.0*SCALE, 80.0), (345.0*SCALE, -112.0*SCALE, 35.0)),
        ("Blue Square Peg",     (150.0*SCALE,  70.0*SCALE, 80.0), (345.0*SCALE,   31.0*SCALE, 35.0)),
        ("Orange Hexagon Peg",  (150.0*SCALE, -70.0*SCALE, 80.0), (345.0*SCALE,  -42.0*SCALE, 35.0)),
        ("Yellow Triangle Peg", (230.0*SCALE,  70.0*SCALE, 80.0), (345.0*SCALE,  115.0*SCALE, 35.0)),
    ]

    waypoints = [("1. Parked at 90-deg Home Pose", home_q, True, 1.5)]

    for idx, (pname, pick_pt, place_pt) in enumerate(tasks, start=1):
        px, py, pz = pick_pt
        sx, sy, sz = place_pt
        waypoints.extend([
            (f"[{idx}/4] Pre-Grasp Above {pname}", solve_ik_flexible(px, py, 125.0), True,  1.5),
            (f"[{idx}/4] Descend onto {pname}",   solve_ik_flexible(px, py, pz),    True,  1.4),
            (f"[{idx}/4] Grasp {pname}",          solve_ik_flexible(px, py, pz),    False, 0.8),
            (f"[{idx}/4] Lift {pname}",           solve_ik_flexible(px, py, 125.0), False, 1.4),
            (f"[{idx}/4] Transit to Socket",      solve_ik_flexible(sx, sy, 75.0),  False, 1.8),
            (f"[{idx}/4] Lower into Socket",      solve_ik_flexible(sx, sy, sz),    False, 1.6),
            (f"[{idx}/4] Release in Socket",      solve_ik_flexible(sx, sy, sz),    True,  0.8),
            (f"[{idx}/4] Vertical Retreat",       solve_ik_flexible(sx, sy, 75.0),  True,  1.2),
            (f"[{idx}/4] Return to 90-deg Home",  home_q,                          True,  1.6),
        ])

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, fps, (1280, 720))

    frame_count = 0
    total_stages = len(waypoints) - 1

    for w_idx in range(total_stages):
        name_curr, q_start, grip1, dur = waypoints[w_idx]
        name_next, q_end,   grip2, _   = waypoints[w_idx + 1]
        print(f"  • Rendering stage [{w_idx+1}/{total_stages}]: {name_curr} -> {name_next}...")

        q_start = np.array(q_start)
        q_end   = np.array(q_end)
        n_frames = int(fps * dur)

        for step in range(n_frames):
            alpha = (1.0 - np.cos(np.pi * step / n_frames)) / 2.0
            q_interp = q_start + alpha * (q_end - q_start)
            data.ctrl[0:4] = q_interp

            grip_state = grip2 if alpha > 0.5 else grip1
            if grip_state:
                data.ctrl[4] = 0.005
                data.ctrl[5] = -0.005
            else:
                data.ctrl[4] = -0.015
                data.ctrl[5] = 0.015

            for _ in range(5):
                mujoco.mj_step(model, data)

            renderer.update_scene(data, camera=camera)
            bgr = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

            cv2.putText(bgr, f"Stage: {name_next}", (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            cv2.putText(bgr, "OpenMANIPULATOR-X 4-Peg Autonomous Sorting (MuJoCo 3D Physics)", (25, 685), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            writer.write(bgr)
            frame_count += 1

    writer.release()
    print("\n" + "=" * 68)
    print(f"✓ Video successfully rendered to: {OUTPUT_VIDEO_PATH}")
    print(f"  Total frames: {frame_count} | Duration: {frame_count/fps:.1f}s")
    print("=" * 68)

if __name__ == "__main__":
    render_simulation_video()
