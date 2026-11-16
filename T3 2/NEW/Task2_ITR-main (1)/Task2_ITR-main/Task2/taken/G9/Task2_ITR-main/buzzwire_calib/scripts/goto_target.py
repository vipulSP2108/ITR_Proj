#!/usr/bin/env python3
"""
Drive the robot end-effector to Post L (Start), Post R (Goal), or HOME from ANY current posture.

Usage:
  python3 scripts/goto_target.py L                   # Move to Start Post L (ready to start traversal)
  python3 scripts/goto_target.py R                   # Move to Goal Post R
  python3 scripts/goto_target.py home                # Move to all-zeros Home position
  python3 scripts/goto_target.py L --sim             # Test in simulation (no robot hardware needed)
  python3 scripts/goto_target.py L --yes             # Do not prompt for confirmation

Safety:
  - Smooth C2-continuous quintic polynomial trajectory in joint space (no jerking).
  - Works from ANY arbitrary starting posture / orientation.
  - Closed-loop error feedback (Kp = 2.0) with velocity clamp (max 0.25 rad/s).
  - Stale joint state protection (pauses immediately if controller drops frames).
  - Stall detection (aborts with zero velocity if tracking error exceeds 0.12 rad).
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from buzzwire_calib.config import load_config


# Exact joint configurations calibrated on the Addverb HEAL cobot
TARGETS = {
    "L": {
        "name": "Start Post L (Left Entry)",
        "q": np.array([-0.20704, -0.37082, -0.53853, -2.07387,  0.52603, -0.00528]),
        "cartesian_mm": [148.38, 252.04, 568.19],
        "quaternion_xyzw": [-0.0662, -0.7017, 0.1013, 0.7021]
    },
    "R": {
        "name": "Goal Post R (Right Exit)",
        "q": np.array([0.00025,  0.67414,  0.40510, -1.62820,  0.00925,  1.36498]),
        "cartesian_mm": [101.68, 548.40, 580.82],
        "quaternion_xyzw": [0.0471, 0.7042, -0.0823, -0.7036]
    },
    "HOME": {
        "name": "Robot Home (All Zeros)",
        "q": np.zeros(6),
        "cartesian_mm": None,
        "quaternion_xyzw": None
    }
}


def targets_from_session(sdir):
    """Override L/R with the first/last joint configuration of the session's calibrated LR trajectory.

    After a fresh teach + build, the posts are wherever the NEW data says they are, so the old
    hard-coded joint values above must not be used. Returns True if the session provided targets.
    """
    if not sdir:
        return False
    f = Path(sdir) / "calibrated_wire_LR.csv"
    if not f.exists():
        f = Path(sdir) / "merged_trajectory.csv"
    if not f.exists():
        print(f"[goto_target] WARNING: no calibrated_wire_LR.csv / merged_trajectory.csv in {sdir}; using built-in L/R.")
        return False
    from buzzwire_calib.trajectory import load_trajectory
    wps = load_trajectory(f)
    for key, w in (("L", wps[0]), ("R", wps[-1])):
        TARGETS[key]["q"] = np.array([w.j1, w.j2, w.j3, w.j4, w.j5, w.j6], float)
        TARGETS[key]["cartesian_mm"] = [round(w.x * 1000, 2), round(w.y * 1000, 2), round(w.z * 1000, 2)]
        TARGETS[key]["quaternion_xyzw"] = [w.qx, w.qy, w.qz, w.qw]
        TARGETS[key]["name"] += f"  [from {Path(sdir).name}/{f.name}]"
    return True


def quintic(tau):
    """Smooth quintic polynomial with zero velocity and acceleration at ends."""
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    return s, ds


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", choices=["L", "l", "R", "r", "HOME", "home", "start", "goal"],
                        help="destination target (L = Start Post, R = Goal Post, home = all zeros)")
    parser.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    parser.add_argument("--speed", type=float, default=0.25, help="maximum joint velocity clamp in rad/s (default: 0.25)")
    parser.add_argument("--min-time", type=float, default=4.0, help="minimum movement duration in seconds")
    parser.add_argument("--kp", type=float, default=2.0, help="closed-loop proportional gain")
    parser.add_argument("--tol", type=float, default=0.005, help="per-joint arrival tolerance in radians")
    parser.add_argument("--sim", action="store_true", help="run on simulated arm (no robot, no ROS)")
    parser.add_argument("--yes", action="store_true", help="skip confirmation prompt")
    parser.add_argument("--session", default=None,
                        help="session folder whose calibrated LR trajectory defines Post L (first point) and Post R (last point)")
    args = parser.parse_args()
    targets_from_session(args.session)

    cfg = load_config(args.config)
    names = cfg["robot"]["joint_names"]

    # Normalize target key
    key = args.target.upper()
    if key in ("START", "LEFT"): key = "L"
    if key in ("GOAL", "RIGHT"): key = "R"
    target_info = TARGETS[key]
    q_goal = target_info["q"]

    # Initialize backend
    if args.sim:
        from buzzwire_calib.backends import SimBackend
        be = SimBackend(cfg)
        def send(v): be.send(v)
        def get_q(): return be.state()[0].copy()
        def get_age(): return be.state()[1]
        def shutdown(): be.shutdown()
    else:
        from buzzwire_calib.ros_io import RosHub, RosRobotState
        from std_msgs.msg import Float64MultiArray
        hub = RosHub("buzzwire_goto_target")
        robot = RosRobotState(hub, cfg["robot"])
        pub = hub.node.create_publisher(Float64MultiArray, cfg["robot"]["velocity_command_topic"], 1)

        def send(v):
            msg = Float64MultiArray()
            msg.data = [float(x) for x in v]
            pub.publish(msg)

        def get_q():
            s = robot.get()
            return None if s is None else s.q.copy()

        def get_age(): return robot.age()
        def shutdown(): hub.shutdown()

    # Wait for valid joint state
    print(f"Connecting to robot joints...")
    t_wait = time.time()
    while get_q() is None:
        if time.time() - t_wait > 8.0:
            sys.exit("[goto_target] ERROR: No /joint_states received. Is ./heal_server and velocity_control running?")
        time.sleep(0.05)

    q0 = get_q().copy()
    delta = q_goal - q0
    max_delta_deg = np.degrees(np.abs(delta).max())

    print(f"\n{'='*70}")
    print(f"TARGET DESTINATION: {target_info['name']}")
    print(f"{'='*70}")
    print(f"Current Joint Angles (deg): {np.round(np.degrees(q0), 1).tolist()}")
    print(f"Goal Joint Angles    (deg): {np.round(np.degrees(q_goal), 1).tolist()}")
    print(f"Max Joint Angular Move    : {max_delta_deg:.1f} deg")
    if target_info["cartesian_mm"]:
        print(f"Goal Cartesian Target (mm): {target_info['cartesian_mm']}")

    if max_delta_deg < 0.6:
        print(f"\nArm is ALREADY at {target_info['name']} (error {max_delta_deg:.2f} deg < 0.6 deg).")
        print("Ready for wire traversal. No movement required.")
        shutdown()
        return

    # Physical safety warning if near the wire apparatus
    if not args.sim and not args.yes:
        print("\nSAFETY NOTICE:")
        print("  - Ensure the wand loop is clear of physical obstacles or the wire stand.")
        print("  - If the loop is currently threaded onto the wire, use './buzzwire_run.sh rl' instead.")
        ans = input("Press ENTER to begin smooth motion (Ctrl-C to cancel): ")
        if ans != "":
            print("Aborted.")
            shutdown()
            return

    # Compute trajectory duration based on max joint velocity clamp
    # For quintic polynomial, peak velocity = 1.875 * delta / T
    T = max(args.min_time, float(1.875 * np.abs(delta).max() / args.speed))
    print(f"\nStarting quintic move: duration = {T:.1f} s (max velocity {args.speed:.2f} rad/s)...")

    dt = 0.005
    t = 0.0
    last_t = time.time()
    last_hud = 0.0
    err_bad_since = None
    arrived_since = None

    try:
        while True:
            time.sleep(dt)
            now = time.time()
            if get_age() > 0.3:
                # Stale joint states protection: zero velocity and pause
                send(np.zeros(len(names)))
                last_t = now
                sys.stdout.write("\r\033[K[STALE JOINT STATES] Holding position...")
                sys.stdout.flush()
                continue

            t += min(now - last_t, 0.05)
            last_t = now
            q = get_q()

            tau = min(t / T, 1.0)
            s, ds = quintic(tau)

            q_ref = q0 + delta * s
            qd_ref = delta * ds / T

            err = q_ref - q
            cmd = np.clip(qd_ref + args.kp * err, -args.speed, args.speed)
            send(cmd)

            # Stall / collision detection
            if np.abs(err).max() > 0.12:
                err_bad_since = err_bad_since or now
                if now - err_bad_since > 2.0:
                    send(np.zeros(len(names)))
                    sys.exit(f"\n[ABORT] Arm is lagging (error {np.abs(err).max():.2f} rad). Joint collision or controller fault?")
            else:
                err_bad_since = None

            remaining_rad = np.abs(q_goal - q).max()
            if tau >= 1.0:
                if remaining_rad < args.tol:
                    arrived_since = arrived_since or now
                    if now - arrived_since > 0.25:
                        break
                elif t > T + 6.0:
                    send(np.zeros(len(names)))
                    print(f"\n[TIMEOUT] Stopped {np.degrees(remaining_rad):.2f} deg from goal.")
                    break

            if now - last_hud > 0.1:
                prog = min(100.0, (t / T) * 100.0)
                sys.stdout.write(f"\r\033[KMoving to {target_info['name']}: {prog:5.1f}% | remaining: {np.degrees(remaining_rad):5.2f} deg")
                sys.stdout.flush()
                last_hud = now

        print(f"\n{'='*70}")
        print(f"SUCCESS: Arm arrived at {target_info['name']}!")
        print(f"Final Joint Configuration (deg): {np.round(np.degrees(get_q()), 1).tolist()}")
        print(f"{'='*70}\n")

    except KeyboardInterrupt:
        print("\nAborted by operator.")
    finally:
        for _ in range(10):
            send(np.zeros(len(names)))
            time.sleep(0.02)
        time.sleep(0.1)
        shutdown()


if __name__ == "__main__":
    main()
