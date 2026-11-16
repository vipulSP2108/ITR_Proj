#!/usr/bin/env python3
"""Replay the merged trajectory (robot in ./heal_server + velocity-control mode, NOT free_drive).

  python3 scripts/replay_trajectory.py --speed 0.3          # 30 % of the speed limit (start LOW, raise later)
  python3 scripts/replay_trajectory.py --dry-run            # no robot: prints the speed profile + duration
  python3 scripts/replay_trajectory.py --sim                # replay a trajectory recorded with teleop --sim
  python3 scripts/replay_trajectory.py --resume             # continue after an abort/stuck from where it stopped

Before the run (pre-start screen):  SPACE = gripper open/close (put the stick in, close it)   ENTER = go   q = abort
During the run:  p pause/resume   [ / ]  speed -/+ 20 %   q stop (zero velocity immediately)
Safety:  speed profile limited by curvature + tool-rotation rate + acceleration; the carrot WAITS when the arm lags;
         the run aborts (zero velocity) if tracking error stays > --abort-mm; progress is saved so --resume works.
"""
import argparse, json, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from buzzwire_calib.config import load_config
from buzzwire_calib.session import latest_teach_session
from buzzwire_calib.replay_core import PathFollower, line_path
from buzzwire_calib.teleop_core import PoseController, Limits
from buzzwire_calib.trajectory import load_trajectory


def build_follower(wps, rc, speed):
    P = np.array([[w.x, w.y, w.z] for w in wps]); Q = np.array([[w.qx, w.qy, w.qz, w.qw] for w in wps])
    return PathFollower(P, Q, v_max=rc["v_max_mps"] * speed, a_max=rc["a_max_mps2"], a_lat=rc["a_lat_mps2"],
                        w_max=rc["w_max_radps"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--session", default=None)
    ap.add_argument("--traj", default=None)
    ap.add_argument("--speed", type=float, default=0.3, help="fraction (0.05-1.0) of replay.v_max_mps")
    ap.add_argument("--start-idx", type=int, default=0, help="start at this waypoint index")
    ap.add_argument("--resume", action="store_true", help="continue from the saved progress of the last run")
    ap.add_argument("--abort-mm", type=float, default=30.0)
    ap.add_argument("--force", action="store_true", help="allow an approach longer than replay.max_approach_m")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()

    cfg = load_config(a.config); rc = cfg["replay"]
    if a.traj:
        tpath = Path(a.traj)
    else:
        sdir = Path(a.session) if a.session else latest_teach_session(cfg["paths"]["data_root"])
        if sdir is None:
            sys.exit("No session found.")
        tpath = sdir / "merged_trajectory.csv"
    if not tpath.exists():
        sys.exit(f"{tpath} not found - run ./buzzwire_teach.sh merge first.")
    state_path = tpath.parent / "replay_state.json"
    wps = load_trajectory(tpath)
    speed = float(np.clip(a.speed, 0.05, 1.0))
    fol = build_follower(wps, rc, speed)
    n_gap = sum(w.gap for w in wps)
    print(f"Trajectory {tpath}: {len(wps)} pts, {fol.length*1000:.0f} mm, est. {fol.duration_estimate():.1f} s at speed {speed:.2f}"
          f"  (v {fol.v_prof.min()*1000:.0f}-{fol.v_prof.max()*1000:.0f} mm/s)")
    if n_gap:
        print(f"  WARNING: {n_gap} waypoints are synthetic gap bridges - check them at low speed first.")
    if a.dry_run:
        for k in np.argsort(fol.v_prof)[:5]:
            print(f"  slowest spot: s={fol.s[k]*1000:6.1f} mm  v={fol.v_prof[k]*1000:5.1f} mm/s  at {np.round(fol.P[k]*1000,1)} mm")
        return

    s_start = float(fol.s[min(a.start_idx, len(fol.s) - 1)])
    if a.resume and state_path.exists():
        st = json.loads(state_path.read_text())
        if not st.get("done"):
            s_start = max(0.0, float(st["s"]) - 0.010)         # back up 10 mm for overlap
            print(f"Resuming from {s_start*1000:.0f} mm along the path (saved progress).")
    fol.pos = s_start

    from buzzwire_calib.backends import RosBackend, SimBackend
    from buzzwire_calib.keyboard import KeyInput
    be = SimBackend(cfg) if a.sim else RosBackend(cfg)
    lim = Limits(max_joint_vel=rc["max_joint_vel"], lead_lin=0.05, lead_ang=0.5, kp_lin=tc_kp(cfg, "kp_lin"),
                 kp_ang=tc_kp(cfg, "kp_ang"), joint_limits=cfg["teleop"]["joint_limits_rad"],
                 joint_margin=cfg["teleop"]["joint_limit_margin_rad"])
    ctrl = PoseController(be.fk, be.jac, lim)
    STOP = [False]
    signal.signal(signal.SIGINT, lambda *_: STOP.__setitem__(0, True))

    q, age, _ = be.state()
    T_now, T_goal = be.fk(q), fol.pose_at(s_start)
    d_app = float(np.linalg.norm(T_goal[:3, 3] - T_now[:3, 3]))
    print(f"Start pose is {d_app*1000:.0f} mm / {np.degrees(np.linalg.norm(Rot.from_matrix(T_now[:3,:3].T @ T_goal[:3,:3]).as_rotvec())):.0f} deg from the arm (a straight-line approach will be driven first).")
    if d_app > rc["max_approach_m"] and not a.force:
        be.shutdown(); sys.exit(f"Too far (> {rc['max_approach_m']*1000:.0f} mm). Move the arm near the start first (teleop) or use --force.")

    done, scale, paused, aborted = False, 1.0, False, False
    with KeyInput("terminal", 0.12) as keys:
        print("\nPRE-START: SPACE = gripper open/close, ENTER = start, q = abort")
        go = a.yes
        while not go and not STOP[0]:
            for ev in keys.events():
                if ev == "space": be.gripper.toggle()
                elif ev == "enter": go = True
                elif ev == "q": STOP[0] = True
            sys.stdout.write(f"\r\033[K gripper={be.gripper.state}  (SPACE toggle / ENTER start / q abort)"); sys.stdout.flush()
            time.sleep(0.05)
        try:
            # ---------------- phase 1: approach (slow straight line), phase 2: follow
            rot_gap = float(np.linalg.norm(Rot.from_matrix(T_now[:3, :3].T @ T_goal[:3, :3]).as_rotvec()))
            if d_app < 0.001 and rot_gap < 0.01:                # already at the start: no approach needed
                phase, fl = "follow", fol
                fol.pos = s_start
                print("Arm is already at the start pose - following directly.")
            else:
                P_a, Q_a = line_path(T_now, T_goal)
                fl = PathFollower(P_a, Q_a, v_max=rc["approach_speed_mps"], a_max=0.1, a_lat=1.0, w_max=0.3, rot_metric=0.05)
                phase = "approach"
            t_prev, t_hud, t_save, bad_since, js = time.monotonic(), 0.0, 0.0, None, 1.0
            t_run0, max_err = (time.monotonic() if phase == "follow" else None), 0.0
            while not STOP[0]:
                now = time.monotonic(); dt = max(1e-3, min(now - t_prev, 0.05)); t_prev = now
                for ev in keys.events():
                    if ev == "q": STOP[0] = True
                    elif ev == "p": paused = not paused
                    elif ev == "[": scale = max(0.2, scale * 0.8)
                    elif ev == "]": scale = min(1.5, scale * 1.25)
                q, age, _ = be.state()
                if age > 0.3 or paused:                         # stale joint states or paused: hold, carrot frozen
                    be.send(np.zeros(be.n)); time.sleep(0.01); continue
                T_cur = be.fk(q)
                err = float(np.linalg.norm(fl.pose_at(fl.pos)[:3, 3] - T_cur[:3, 3]))
                T_t, tw, fin, v = fl.advance(dt, err, scale * js)
                if phase == "follow":
                    max_err = max(max_err, err)
                qd, info = ctrl.track(q, T_t, tw)
                js = min(info["joint_scale"], js + 2.0 * dt)       # slow the carrot the instant joints saturate
                be.send(qd)
                if err > a.abort_mm / 1000:
                    bad_since = bad_since or now
                    if now - bad_since > 1.5:
                        aborted = True; STOP[0] = True
                        print(f"\nABORT: arm not following (error {err*1000:.0f} mm). Stuck / collision / fault?")
                else:
                    bad_since = None
                if fin and phase == "approach" and err < 0.003:
                    phase, fl, t_run0 = "follow", fol, time.monotonic()
                    fl.pos = s_start; print("\nApproach done - following the trajectory.")
                elif fin and phase == "follow":
                    done = True; break
                if phase == "follow" and now - t_save > 0.5:
                    state_path.write_text(json.dumps({"s": fl.pos, "done": False, "traj": str(tpath), "t": time.time()})); t_save = now
                if now - t_hud > 0.15:
                    sys.stdout.write(f"\r\033[K[{phase}] {fl.pos*1000:6.1f}/{fl.length*1000:.0f} mm  v={v*1000:4.0f}mm/s x{scale:.2f} "
                                     f"err={err*1000:4.1f}mm sigma={info['sigma_min']:.2f} {'SLOWED(singular)' if info['sing_scale']<1 else ''}")
                    sys.stdout.flush(); t_hud = now
                time.sleep(max(0.0, 0.01 - (time.monotonic() - now)))
        finally:
            be.stop()
    if done:
        state_path.write_text(json.dumps({"s": fol.length, "done": True, "max_err_mm": max_err * 1000}))
        print(f"\nDONE in {time.monotonic() - t_run0:.1f} s.  Worst tracking error {max_err*1000:.2f} mm.")
    else:
        state_path.write_text(json.dumps({"s": fol.pos, "done": False, "traj": str(tpath), "t": time.time()}))
        print(f"\nStopped at {fol.pos*1000:.0f} mm. Continue with:  python3 scripts/replay_trajectory.py --resume --speed {speed}")
    be.shutdown()


def tc_kp(cfg, key):
    return cfg["teleop"][key]


if __name__ == "__main__":
    main()
