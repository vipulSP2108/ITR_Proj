#!/usr/bin/env python3
"""Recovery tool: drive the arm back to a joint configuration saved in a session (or home).

Use when the robot got stuck / was restarted and you want to continue from where you left off, e.g.
    python3 scripts/goto_pose.py --session data/session_X --last        # back to the last capture
    python3 scripts/goto_pose.py --session data/session_X --capture 7
    python3 scripts/goto_pose.py --session data/session_X --matrix 3     # last capture of matrix 3
    python3 scripts/goto_pose.py --home

NEEDS the velocity controller (./heal_server + `ros2 launch addverb_cobot_control velocity_control.launch.py`),
NOT free_drive. After arriving, stop it, restart ./free_drive and run collect_data.py again (it resumes).

Safety: smooth quintic profile, closed-loop correction, velocity clamp, pauses on stale joint states,
aborts (zero velocity) if the arm does not follow (stuck / collision / e-stop).
"""
import argparse, sys, time, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from buzzwire_calib.config import load_config
from buzzwire_calib.session import SessionStore, find_latest_session


def quintic(tau):
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    return s, ds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--session", default=None)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--last", action="store_true")
    g.add_argument("--capture", type=int)
    g.add_argument("--matrix", type=int)
    g.add_argument("--home", action="store_true")
    ap.add_argument("--max-vel", type=float, default=0.25, help="rad/s clamp per joint")
    ap.add_argument("--min-time", type=float, default=4.0)
    ap.add_argument("--kp", type=float, default=2.0)
    ap.add_argument("--tol", type=float, default=0.004, help="rad, per-joint arrival tolerance")
    ap.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    a = ap.parse_args()
    cfg = load_config(a.config)
    names = cfg["robot"]["joint_names"]

    if a.home:
        q_goal = np.zeros(len(names)); label = "HOME (all zeros)"
    else:
        sdir = Path(a.session) if a.session else find_latest_session(cfg["paths"]["data_root"])
        store = SessionStore(sdir)
        st = store.state()
        if not st["live_ids"]:
            sys.exit("session has no captures")
        if a.last:
            cid = st["last_id"]
        elif a.capture is not None:
            cid = a.capture
        else:
            ids = [i for i in st["live_ids"] if store.load_capture_meta(i)["matrix"] == a.matrix]
            if not ids:
                sys.exit(f"no captures on matrix {a.matrix}")
            cid = ids[-1]
        meta = store.load_capture_meta(cid)
        q_goal = np.array(meta["q"], float)
        label = f"capture #{cid} (matrix {meta['matrix']})"

    from buzzwire_calib.ros_io import RosHub, RosRobotState
    from std_msgs.msg import Float64MultiArray
    hub = RosHub("buzzwire_goto")
    robot = RosRobotState(hub, cfg["robot"])
    pub = hub.node.create_publisher(Float64MultiArray, cfg["robot"]["velocity_command_topic"], 1)

    def send(v):
        m = Float64MultiArray(); m.data = [float(x) for x in v]; pub.publish(m)

    t0 = time.time()
    while robot.get() is None:
        if time.time() - t0 > 10:
            sys.exit("no /joint_states - is heal_server running?")
        time.sleep(0.05)
    q0 = robot.get().q.copy()
    delta = q_goal - q0
    print(f"Target: {label}\n  current {np.round(q0,3)}\n  goal    {np.round(q_goal,3)}\n  max joint move {np.degrees(np.abs(delta).max()):.1f} deg")
    if not a.yes and input("Clear the workspace. Press ENTER to move (Ctrl-C to abort): ") != "":
        pass
    T = max(a.min_time, 1.875 * np.abs(delta).max() / a.max_vel)   # peak quintic speed = 1.875*d/T
    dt, t, last_t = 0.005, 0.0, time.time()
    err_bad_since, arrived_since = None, None
    try:
        while True:
            time.sleep(dt)
            now = time.time()
            if robot.age() > 0.3:                    # stale joint states: stop and WAIT (do not advance trajectory)
                send(np.zeros(len(names))); last_t = now
                print("\rjoint states stale - holding...", end="")
                continue
            t += min(now - last_t, 0.05); last_t = now
            q = robot.get().q
            tau = min(t / T, 1.0)
            s, ds = quintic(tau)
            q_ref = q0 + delta * s
            qd_ref = delta * ds / T
            err = q_ref - q
            cmd = np.clip(qd_ref + a.kp * err, -a.max_vel, a.max_vel)
            send(cmd)
            # stuck detection: lagging a lot behind the reference for > 2 s
            if np.abs(err).max() > 0.12:
                err_bad_since = err_bad_since or now
                if now - err_bad_since > 2.0:
                    send(np.zeros(len(names)))
                    sys.exit(f"\nABORT: arm is not following (error {np.abs(err).max():.2f} rad). Robot stuck / in fault? "
                             "Clear it, then re-run the same command - it plans from wherever the arm is now.")
            else:
                err_bad_since = None
            fin = np.abs(q_goal - q).max()
            if tau >= 1.0:
                if fin < a.tol:
                    arrived_since = arrived_since or now
                    if now - arrived_since > 0.3:
                        break
                elif t > T + 8.0:
                    send(np.zeros(len(names)))
                    sys.exit(f"\nstopped {np.degrees(fin):.2f} deg short of goal (timeout). Re-run to retry.")
            print(f"\r t={t:5.1f}/{T:.1f}s  remaining {np.degrees(fin):6.2f} deg", end="")
        print("\nArrived.")
    except KeyboardInterrupt:
        print("\nAborted by user.")
    finally:
        for _ in range(5):
            send(np.zeros(len(names))); time.sleep(0.02)
        hub.shutdown()


if __name__ == "__main__":
    main()
