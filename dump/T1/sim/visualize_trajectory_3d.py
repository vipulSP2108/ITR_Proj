#!/usr/bin/env python3
"""
Matplotlib 3D Visualizer for OpenMANIPULATOR-X Kinematics & Trajectory.
Renders an interactive 3D plot or saves an image showing:
  • Robot base and 4 link segments in 3D space
  • Tabletop, Green Mat, Wooden Socket Board
  • Peg pickup point and socket placement destination
  • Cartesian path line traced by the gripper fingertip
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from src.planning.kinematics import OpenManipulatorKinematics, L1, L2, L3, L4

def get_arm_joint_positions_3d(kin: OpenManipulatorKinematics, q1, q2, q3, q4):
    """Computes (X, Y, Z) coordinates of each joint link for 3D plotting."""
    th2 = np.pi / 2.0 - q2
    th3 = -q3
    th4 = -q4
    pitch = th2 + th3 + th4

    # Joint 0: Origin (Base bottom)
    p0 = np.array([0.0, 0.0, 0.0])
    # Joint 1: Base shoulder pivot
    p1 = np.array([0.0, 0.0, L1])
    # Joint 2: Elbow pivot
    r2 = L2 * np.cos(th2)
    p2 = np.array([r2 * np.cos(q1), r2 * np.sin(q1), L1 + L2 * np.sin(th2)])
    # Joint 3: Wrist pivot
    r3 = r2 + L3 * np.cos(th2 + th3)
    p3 = np.array([r3 * np.cos(q1), r3 * np.sin(q1), p2[2] + L3 * np.sin(th2 + th3)])
    # Joint 4: Gripper Tip (TCP)
    r4 = r3 + L4 * np.cos(pitch)
    p4 = np.array([r4 * np.cos(q1), r4 * np.sin(q1), p3[2] + L4 * np.sin(pitch)])

    return np.array([p0, p1, p2, p3, p4])

def plot_trajectory_3d(save_path: str = "trajectory_3d_preview.png"):
    kin = OpenManipulatorKinematics()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Draw tabletop
    xx, yy = np.meshgrid(np.linspace(0, 300, 10), np.linspace(-250, 250, 10))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.15, color='gray')

    # Draw Green Mat (pickup area)
    mx, my = np.meshgrid(np.linspace(80, 240, 5), np.linspace(0, 160, 5))
    mz = np.full_like(mx, 2.0)
    ax.plot_surface(mx, my, mz, alpha=0.4, color='teal')

    # Draw Socket Board
    bx, by = np.meshgrid(np.linspace(100, 220, 5), np.linspace(-240, -40, 5))
    bz = np.full_like(bx, 5.0)
    ax.plot_surface(bx, by, bz, alpha=0.4, color='peru')

    # Waypoints: Pick Square on Mat -> Place Square in Socket
    waypoints = [
        ("Home Ready",  140.0,   0.0, 180.0),
        ("Pre-Grasp",   160.0,  60.0,  80.0),
        ("Grasp Peg",   160.0,  60.0,  40.0),
        ("Lift",        160.0,  60.0,  85.0),
        ("Transit",     160.0, -60.0,  85.0),
        ("Insert",      160.0, -60.0,  40.0),
        ("Retreat",     160.0, -60.0,  85.0),
        ("Home Ready",  140.0,   0.0, 180.0),
    ]

    path_xyz = []
    colors = plt.cm.plasma(np.linspace(0, 1, len(waypoints)))

    for i, (name, tx, ty, tz) in enumerate(waypoints):
        sol = kin.inverse_kinematics(tx, ty, tz, pitch_rad=-np.pi/2.0)
        if sol:
            pts = get_arm_joint_positions_3d(kin, *sol)
            path_xyz.append(pts[-1]) # Gripper tip
            # Draw robot skeleton at this pose
            alpha = 0.8 if i in [2, 5] else 0.25 # Highlight Grasp and Insert poses
            linewidth = 4 if i in [2, 5] else 1.5
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-o', color=colors[i],
                    linewidth=linewidth, alpha=alpha, label=name if i in [0, 2, 5] else None)

    # Plot Cartesian trajectory line
    path_arr = np.array(path_xyz)
    ax.plot(path_arr[:, 0], path_arr[:, 1], path_arr[:, 2], '--k', linewidth=2, label="Gripper Path")

    ax.set_xlabel("X (Forward mm)")
    ax.set_ylabel("Y (Lateral mm)")
    ax.set_zlabel("Z (Height mm)")
    ax.set_title("OpenMANIPULATOR-X 3D Autonomous Trajectory (Square Peg)")
    ax.set_xlim([0, 280])
    ax.set_ylim([-250, 250])
    ax.set_zlim([0, 260])
    ax.legend(loc='upper right')

    out_file = os.path.join(BASE_DIR, "sim", save_path)
    plt.tight_layout()
    plt.savefig(out_file, dpi=150)
    plt.close()
    print(f"[3D Visualizer] Generated trajectory visualization at: {out_file}")

if __name__ == "__main__":
    plot_trajectory_3d()

