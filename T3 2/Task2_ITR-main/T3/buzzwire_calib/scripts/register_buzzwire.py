#!/usr/bin/env python3
"""Buzz Wire Navigation Registration & Trajectory Generator.

Maps the photogrammetrically calibrated Buzz Wire Course (31 waypoints, 4 corner fiducials)
into the Addverb HEAL Robot Base Frame {B} using hand-eye extrinsics or 3-point fiducial registration.

Usage:
  python3 scripts/register_buzzwire.py --extrinsics results/session_XXX/extrinsics.json
  python3 scripts/register_buzzwire.py --touch-fiducials FL_x,y,z FR_x,y,z RL_x,y,z
  python3 scripts/register_buzzwire.py --nominal-offset 0.40 -0.15 0.10 --yaw 0.0

Outputs:
  results/buzz_wire_robot_trajectory.json: Ready for DLS Velocity Commander or Quintic Planner.
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Add parent dir to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from buzzwire_calib.transforms import make_T, inv_T, quat_to_T, T_to_xyz_quat


def kabsch_rigid_transform(P_ground: np.ndarray, P_robot: np.ndarray):
    """Computes optimal rigid transform T_robot_ground (R, t) mapping P_ground -> P_robot.
    Both arrays are Nx3 in metres.
    """
    assert len(P_ground) >= 3 and len(P_robot) >= 3
    centroid_G = np.mean(P_ground, axis=0)
    centroid_B = np.mean(P_robot, axis=0)
    
    H = (P_ground - centroid_G).T @ (P_robot - centroid_B)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Handle reflection
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
        
    t = centroid_B - R @ centroid_G
    T = make_T(R, t)
    
    # Compute registration error
    pred = (R @ P_ground.T).T + t
    err = np.linalg.norm(pred - P_robot, axis=1)
    rmse_mm = np.sqrt(np.mean(err**2)) * 1000.0
    return T, rmse_mm, err * 1000.0


def quat_mult(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> np.ndarray:
    """Quaternion multiplication (q1 * q2) in scalar-last [x, y, z, w]."""
    x1, y1, z1, w1 = q1_xyzw
    x2, y2, z2, w2 = q2_xyzw
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    q = np.array([x, y, z, w], float)
    return q / np.linalg.norm(q)


def main():
    ap = argparse.ArgumentParser(description="Register Buzz Wire apparatus to robot base frame.")
    ap.add_argument("--spec", default="config/buzz_wire_calibration.json",
                    help="Path to buzz_wire_calibration.json")
    ap.add_argument("--extrinsics", default=None,
                    help="Path to extrinsics.json from calibrate_extrinsics.py")
    ap.add_argument("--touch-fl", nargs=3, type=float, default=None,
                    help="Touched position of FID_FL [x y z] in robot base frame (metres)")
    ap.add_argument("--touch-fr", nargs=3, type=float, default=None,
                    help="Touched position of FID_FR [x y z] in robot base frame (metres)")
    ap.add_argument("--touch-rl", nargs=3, type=float, default=None,
                    help="Touched position of FID_RL [x y z] in robot base frame (metres)")
    ap.add_argument("--nominal-offset", nargs=3, type=float, default=[0.40, -0.144, 0.05],
                    help="Nominal [x y z] offset of Buzz Wire ground origin in base frame (metres)")
    ap.add_argument("--yaw-deg", type=float, default=0.0,
                    help="Nominal rotation about base Z axis (degrees)")
    ap.add_argument("--out", default="results/buzz_wire_robot_trajectory.json",
                    help="Output trajectory JSON path")
    args = ap.parse_args()

    # Locate spec file
    spec_path = Path(args.spec)
    if not spec_path.exists():
        spec_path = Path(__file__).resolve().parent.parent / "config" / "buzz_wire_calibration.json"
    if not spec_path.exists():
        sys.exit(f"Specification file not found: {args.spec}")
    with open(spec_path) as f:
        data = json.load(f)

    print(f"Loaded apparatus geometry from: {spec_path.name}")
    print(f"Total course waypoints: {len(data['navigation_waypoints'])}")
    
    # 1. Determine T_base_ground
    if args.touch_fl and args.touch_fr and args.touch_rl:
        print("\n--- Method: 3-Point Fiducial Touch Registration ---")
        # Fiducial positions in ground frame (metres)
        # FID_FL: [0.0, -0.048, 0.0], FID_FR: [0.288, -0.048, 0.0], FID_RL: [0.0, 0.048, 0.0]
        P_ground = np.array([
            [0.0, -0.048, 0.0],
            [0.288, -0.048, 0.0],
            [0.0, 0.048, 0.0]
        ], float)
        P_robot = np.array([args.touch_fl, args.touch_fr, args.touch_rl], float)
        T_base_ground, rmse, errors = kabsch_rigid_transform(P_ground, P_robot)
        print(f"Fiducial Registration Fit RMSE: {rmse:.2f} mm")
        for fid_name, err in zip(["FID_FL", "FID_FR", "FID_RL"], errors):
            print(f"  {fid_name} error: {err:.2f} mm")
    else:
        print("\n--- Method: Nominal Base Placement (Adjust via --nominal-offset / --touch-*) ---")
        R_nom = Rot.from_euler("z", args.yaw_deg, degrees=True).as_matrix()
        t_nom = np.array(args.nominal_offset, float)
        T_base_ground = make_T(R_nom, t_nom)
        print(f"Nominal origin at: {t_nom.round(4)} m, yaw: {args.yaw_deg:.1f} deg")

    R_bg = T_base_ground[:3, :3]
    t_bg = T_base_ground[:3, 3]
    q_bg = Rot.from_matrix(R_bg).as_quat()  # xyzw

    # 2. Transform all waypoints into Robot Base Frame {B}
    robot_waypoints = []
    positions_robot = []
    
    print("\n--- Transforming 31 Navigation Waypoints ---")
    for wp in data["navigation_waypoints"]:
        # Ground position in mm -> convert to metres
        pos_raw = wp["position_mm"]
        if isinstance(pos_raw, dict):
            p_g = np.array([pos_raw["x"], pos_raw["y"], pos_raw["z"]], float) / 1000.0
        else:
            p_g = np.array(pos_raw, float) / 1000.0

        q_raw = wp.get("tcp_quaternion_xyzw") or wp.get("orientation_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        q_g = np.array(q_raw, float)
        
        # Rigid transformation
        p_b = R_bg @ p_g + t_bg
        q_b = quat_mult(q_bg, q_g)
        positions_robot.append(p_b)
        
        entry = {
            "index": wp["index"],
            "label": wp.get("label") or wp.get("id", f"WP_{wp['index']:02d}"),
            "segment_type": wp["segment_type"],
            "recommended_speed_m_s": wp.get("recommended_speed_mm_s", 10.0) / 1000.0,
            "tangent_angle_deg": wp.get("elevation_alpha_deg") or wp.get("local_tangent_angle_alpha_deg", 0.0),
            "position_base_m": [round(float(v), 5) for v in p_b],
            "quaternion_base_xyzw": [round(float(v), 5) for v in q_b],
            "ground_position_mm": pos_raw,
        }
        robot_waypoints.append(entry)

    # 3. Path envelope validation
    pos_arr = np.array(positions_robot)
    span_x = float(np.ptp(pos_arr[:, 0]))
    span_y = float(np.ptp(pos_arr[:, 1]))
    span_z = float(np.ptp(pos_arr[:, 2]))
    
    print(f"Transformed Waypoints Summary:")
    print(f"  Course length span: X={span_x*1000:.1f} mm, Y={span_y*1000:.1f} mm, Z={span_z*1000:.1f} mm")
    print(f"  Start WP_00 (Safe approach): {robot_waypoints[0]['position_base_m']}")
    print(f"  Apex WP_15 (Tallest crest P_2): {robot_waypoints[15]['position_base_m']}")
    print(f"  Goal WP_30 (Safe exit)    : {robot_waypoints[-1]['position_base_m']}")
    
    output_package = {
        "metadata": {
            "source": "Buzz Wire photogrammetric calibration",
            "robot": "Addverb HEAL 6-DOF Cobot",
            "course_total_span_mm": 288.0,
            "tube_safe_radius_mm": 9.0,
            "n_waypoints": len(robot_waypoints),
        },
        "T_base_ground": {
            "matrix": T_base_ground.tolist(),
            "translation_xyz_m": [round(float(v), 5) for v in t_bg],
            "quaternion_xyzw": [round(float(v), 5) for v in q_bg],
        },
        "waypoints": robot_waypoints
    }

    out_file = Path(args.out)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(output_package, f, indent=2)
    print(f"\nSaved robot trajectory to: {out_file} ({len(robot_waypoints)} waypoints)")


if __name__ == "__main__":
    main()
