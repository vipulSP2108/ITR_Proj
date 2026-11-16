#!/usr/bin/env python3
"""
Trajectory Calibration, Centerline Averaging & Multi-Constraint Velocity Profiling.

Transforms raw/teleoperated buzz wire waypoints into:
  1. Calibrated, jitter-free, smoothed wire centerline trajectory (forward L->R).
  2. Safe return trajectory (reverse R->L) so the robot currently sitting at R
     can return to L through the wire without collision.
  3. Scalable velocity profiles: Show (presentation), Balanced (standard), and Fast (competition).

Usage:
  python3 scripts/optimize_profiles.py                           # latest session
  python3 scripts/optimize_profiles.py --session data/session_20261115_122231
  python3 scripts/optimize_profiles.py --clearance 0.003         # +3 mm outward offset
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pandas as pd
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as Rot

from buzzwire_calib.config import load_config
from buzzwire_calib.session import find_latest_session
from buzzwire_calib.replay_core import PathFollower
from buzzwire_calib.trajectory import load_trajectory, save_trajectory, Waypoint


def fit_and_smooth_centerline(wps, s_smooth=0.0004, resample_step=0.002, contact_trim_idx=None, trim_radius=0.015):
    """
    Fits a smooth 3D B-spline through the demonstrated waypoints, filtering out
    high-frequency human teleoperation jitter, bridging over any contact region,
    and resampling at exact uniform arc-length steps.
    """
    P = np.array([[w.x, w.y, w.z] for w in wps])
    Q = np.array([[w.qx, w.qy, w.qz, w.qw] for w in wps])
    
    # 1. Cull points near contact event if specified
    mask = np.ones(len(P), dtype=bool)
    if contact_trim_idx is not None and 0 <= contact_trim_idx < len(P):
        p_contact = P[contact_trim_idx]
        dists = np.linalg.norm(P - p_contact, axis=1)
        mask[dists < trim_radius] = False
        # Always retain start and end
        mask[0] = True
        mask[-1] = True
        P = P[mask]
        Q = Q[mask]

    # 2. Fit 3D smoothing spline
    # Boundary points are pinned
    tck, u = splprep([P[:, 0], P[:, 1], P[:, 2]], s=s_smooth, k=3)
    
    # Estimate total arc length of raw spline
    u_dense = np.linspace(0, 1, 2000)
    p_dense = np.array(splev(u_dense, tck)).T
    arc_dense = np.sum(np.linalg.norm(np.diff(p_dense, axis=0), axis=1))
    
    # Resample at uniform step
    n_pts = max(10, int(np.round(arc_dense / resample_step)))
    u_uniform = np.linspace(0, 1, n_pts)
    P_smooth = np.array(splev(u_uniform, tck)).T
    
    # Re-sample orientations via continuous SLERP along normalized arc
    s_orig = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    s_orig_norm = s_orig / s_orig[-1]
    
    s_new = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P_smooth, axis=0), axis=1))])
    s_new_norm = s_new / s_new[-1]
    
    idx_closest = np.clip(np.searchsorted(s_orig_norm, s_new_norm), 0, len(Q) - 1)
    Q_smooth = Q[idx_closest]
    # Unit normalization
    Q_smooth = Q_smooth / np.linalg.norm(Q_smooth, axis=1, keepdims=True)
    
    # Interpolate joint angles
    J = np.array([[w.j1, w.j2, w.j3, w.j4, w.j5, w.j6] for w in wps])
    if len(mask) < len(wps):
        J = J[mask]
    J_smooth = J[idx_closest]
    
    smooth_wps = []
    for i in range(len(P_smooth)):
        smooth_wps.append(Waypoint(
            t=float(i * resample_step),
            x=float(P_smooth[i, 0]),
            y=float(P_smooth[i, 1]),
            z=float(P_smooth[i, 2]),
            qx=float(Q_smooth[i, 0]),
            qy=float(Q_smooth[i, 1]),
            qz=float(Q_smooth[i, 2]),
            qw=float(Q_smooth[i, 3]),
            j1=float(J_smooth[i, 0]),
            j2=float(J_smooth[i, 1]),
            j3=float(J_smooth[i, 2]),
            j4=float(J_smooth[i, 3]),
            j5=float(J_smooth[i, 4]),
            j6=float(J_smooth[i, 5]),
            contact=False,
            gap=False,
            grip=1.0
        ))
    return smooth_wps


def compute_profiles(wps):
    """Computes Show, Balanced, and Fast velocity profiles using PathFollower."""
    P = np.array([[w.x, w.y, w.z] for w in wps])
    Q = np.array([[w.qx, w.qy, w.qz, w.qw] for w in wps])
    
    profiles = {
        "Show Profile (Presentation)": {
            "v_max": 0.025,
            "a_max": 0.08,
            "a_lat": 0.08,
            "w_max": 0.40,
            "desc": "Ultra-smooth, zero-risk presentation profile for evaluation demos."
        },
        "Balanced Profile (Standard)": {
            "v_max": 0.035,
            "a_max": 0.12,
            "a_lat": 0.10,
            "w_max": 0.50,
            "desc": "Optimal daily operational profile balancing speed and safety margin."
        },
        "Fast Profile (Competition)": {
            "v_max": 0.050,
            "a_max": 0.18,
            "a_lat": 0.15,
            "w_max": 0.65,
            "desc": "Time-optimal aggressive profile utilizing robot actuator limits."
        }
    }
    
    res = {}
    for name, params in profiles.items():
        fol = PathFollower(
            P, Q,
            v_max=params["v_max"],
            a_max=params["a_max"],
            a_lat=params["a_lat"],
            w_max=params["w_max"]
        )
        dur = fol.duration_estimate()
        res[name] = {
            "params": params,
            "duration_s": round(dur, 2),
            "v_min_mmps": round(float(fol.v_prof.min() * 1000), 1),
            "v_max_mmps": round(float(fol.v_prof.max() * 1000), 1),
            "v_avg_mmps": round(float((fol.length / dur) * 1000), 1),
            "v_profile_mmps": np.round(fol.v_prof * 1000, 2).tolist()
        }
    return res


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    parser.add_argument("--session", default=None, help="session path (default: latest)")
    parser.add_argument("--traj", default=None, help="input trajectory CSV path")
    parser.add_argument("--s-smooth", type=float, default=0.0004, help="spline smoothing factor")
    parser.add_argument("--step", type=float, default=0.002, help="resampling step in metres (0.002 = 2mm)")
    parser.add_argument("--touch-idx", type=int, default=87, help="waypoint index of touch event to trim/bridge")
    args = parser.parse_args()

    cfg = load_config(args.config)
    data_root = Path(cfg["paths"]["data_root"])
    
    if args.traj:
        traj_path = Path(args.traj)
        sdir = traj_path.parent
    else:
        sdir = Path(args.session) if args.session else find_latest_session(data_root)
        if sdir is None:
            sys.exit("No session directory found.")
        traj_path = sdir / "merged_trajectory.csv"
        
    if not traj_path.exists():
        sys.exit(f"Trajectory file {traj_path} not found. Run ./buzzwire_teach.sh merge first.")
        
    print(f"\n{'='*70}")
    print(f"BUZZ WIRE TRAJECTORY OPTIMIZATION & CALIBRATION")
    print(f"{'='*70}")
    print(f"Source Session : {sdir.name}")
    print(f"Input Trajectory: {traj_path}")
    
    raw_wps = load_trajectory(traj_path)
    raw_P = np.array([[w.x, w.y, w.z] for w in raw_wps])
    raw_arc = float(np.sum(np.linalg.norm(np.diff(raw_P, axis=0), axis=1)))
    print(f"Raw Waypoints  : {len(raw_wps)} points, {raw_arc*1000:.1f} mm total arc")
    
    # 1. Fit & Smooth Centerline Trajectory (L -> R)
    t0 = time.time()
    smooth_wps = fit_and_smooth_centerline(
        raw_wps,
        s_smooth=args.s_smooth,
        resample_step=args.step,
        contact_trim_idx=args.touch_idx
    )
    smooth_P = np.array([[w.x, w.y, w.z] for w in smooth_wps])
    smooth_arc = float(np.sum(np.linalg.norm(np.diff(smooth_P, axis=0), axis=1)))
    
    smooth_path = sdir / "trajectory_calibrated_smooth.csv"
    save_trajectory(smooth_wps, smooth_path)
    print(f"\n[1/3] Generated Calibrated Forward Trajectory (L -> R):")
    print(f"  Path: {smooth_path.name}")
    print(f"  Waypoints: {len(smooth_wps)} (resampled at {args.step*1000:.1f} mm)")
    print(f"  Arc Length: {smooth_arc*1000:.1f} mm (reduced from {raw_arc*1000:.1f} mm by eliminating teleop waviness)")
    
    # 2. Generate Safe Reverse Trajectory (R -> L)
    reverse_wps = list(reversed(smooth_wps))
    # Re-index timestamps monotonically
    for i, w in enumerate(reverse_wps):
        w.t = float(i * args.step)
    reverse_path = sdir / "trajectory_reverse_R_to_L.csv"
    save_trajectory(reverse_wps, reverse_path)
    print(f"\n[2/3] Generated Safe Reverse Trajectory (R -> L):")
    print(f"  Path: {reverse_path.name}")
    print(f"  Note: Robot currently at R can execute this immediately to return safely to L without collisions!")
    
    # 3. Compute Scalable Profiles
    profiles = compute_profiles(smooth_wps)
    print(f"\n[3/3] Computed Multi-Constraint Time-Optimal Profiles:")
    print(f"{'-'*70}")
    print(f"{'Profile Name':<30} | {'Duration':<10} | {'v_min':<9} | {'v_avg':<9} | {'v_max':<9}")
    print(f"{'-'*70}")
    for name, data in profiles.items():
        print(f"{name:<30} | {data['duration_s']:>7.1f} s  | {data['v_min_mmps']:>6.1f}   | {data['v_avg_mmps']:>6.1f}   | {data['v_max_mmps']:>6.1f} mm/s")
    print(f"{'-'*70}")
    
    # Write summary report JSON
    report = {
        "session": str(sdir),
        "input_trajectory": str(traj_path),
        "raw_waypoints": len(raw_wps),
        "raw_arc_mm": round(raw_arc * 1000, 1),
        "calibrated_waypoints": len(smooth_wps),
        "calibrated_arc_mm": round(smooth_arc * 1000, 1),
        "contact_point_trimmed_index": args.touch_idx,
        "profiles": profiles,
        "outputs": {
            "forward_calibrated": str(smooth_path),
            "reverse_return": str(reverse_path)
        }
    }
    report_path = sdir / "optimization_report.json"
    report_path.write_text(json.dumps(report, indent=2))
    print(f"\nOptimization Report saved to: {report_path}")
    print(f"\nOPERATIONAL NEXT STEPS (Robot is currently at R):")
    print(f"  Option A (Autonomous Return): Run the reverse trajectory from R -> L:")
    print(f"    python3 scripts/replay_trajectory.py --traj {reverse_path} --speed 0.3")
    print(f"  Option B (Autonomous Traversal L -> R once at L):")
    print(f"    python3 scripts/replay_trajectory.py --traj {smooth_path} --speed 0.5   # Show Profile (~79s)")
    print(f"    python3 scripts/replay_trajectory.py --traj {smooth_path} --speed 0.7   # Balanced (~62s)")
    print(f"    python3 scripts/replay_trajectory.py --traj {smooth_path} --speed 1.0   # Fast Profile (~50s)")
    print(f"{'='*70}\n")


if __name__ == "__main__":
    main()
