#!/usr/bin/env python3
"""Game-style keyboard teleoperation + path recording for the Buzz Wire (all 6 DOF).

Robot mode:  ./heal_server + velocity controller launch   (NOT free_drive).
Practise with no robot at all:   python3 scripts/teleop.py --sim

  MOVE (hold)                       ROTATE (hold)                  OTHER
   w / s   forward / back  (+Y/-Y)   i / k   pitch up / down         SPACE  gripper open <-> close
   a / d   left / right    (-X/+X)   j / l   yaw  left / right       v      start / stop RECORDING
   r / f   up / down       (+Z/-Z)   u / o   roll left / right       x      mark contact (while recording)
   (combine freely: w+d = diagonal)                                  z      BACKTRACK (hold) - retrace path,
  SPEED   1 2 3 4 5  (fine -> fast)                                          also rewinds the recording = UNDO
  TAB     jog frame: world <-> tool                                  c      re-sync target to the real pose
  g       auto-tangent pitch ON/OFF (tool follows wire direction)    p / h  print pose / help
  ESC     stop + re-sync                                              q      quit (recording is saved)
"""
import argparse, math, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from buzzwire_calib.backends import RosBackend, SimBackend
from buzzwire_calib.config import load_config
from buzzwire_calib.keyboard import KeyInput
from buzzwire_calib.session import choose_teach_session, next_segment_id
from buzzwire_calib.teleop_core import PoseController, Limits
from buzzwire_calib.trajectory import SegmentWriter, Waypoint

HELP = __doc__.split("Robot mode:")[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--session", default=None)
    ap.add_argument("--new", action="store_true")
    ap.add_argument("--keys", choices=["terminal", "pynput"], default=None)
    ap.add_argument("--level", type=int, default=None, help="start speed level 1-5")
    ap.add_argument("--sim", action="store_true", help="practise with a simulated arm (no robot, no ROS)")
    ap.add_argument("--hz", type=float, default=100.0, help="control loop rate")
    ap.add_argument("--min-move", type=float, default=0.0005, help="min motion (m) between recorded samples")
    ap.add_argument("--yes", action="store_true", help="skip the 'ready?' prompt")
    a = ap.parse_args()

    cfg = load_config(a.config)
    tc = cfg["teleop"]
    ask = (lambda _: "y") if a.yes else input
    sdir = choose_teach_session(cfg["paths"]["data_root"], a.session, a.new, ask=ask)
    seg_dir = sdir / "segments"
    print(f"Session: {sdir}")

    be = SimBackend(cfg) if a.sim else RosBackend(cfg)
    lim = Limits(max_joint_vel=tc["max_joint_vel"], lin_accel=tc["lin_accel"], ang_accel=tc["ang_accel"],
                 lead_lin=tc["lead_lin_m"], lead_ang=tc["lead_ang_rad"], kp_lin=tc["kp_lin"], kp_ang=tc["kp_ang"],
                 box=tc["workspace_box_m"], joint_limits=tc["joint_limits_rad"], joint_margin=tc["joint_limit_margin_rad"])
    ctrl = PoseController(be.fk, be.jac, lim, tool_axis=tc["tool_axis"], plane_normal=tc["plane_normal_base"],
                          auto_tangent=bool(tc["auto_tangent"]))
    lin_sp = [x / 1000.0 for x in tc["lin_speeds_mmps"]]
    ang_sp = [math.radians(x) for x in tc["ang_speeds_degps"]]
    level = (a.level or tc["default_level"]) - 1
    level = int(np.clip(level, 0, len(lin_sp) - 1))
    yaw = math.radians(tc["jog_frame_yaw_deg"])
    R_jog = Rot.from_euler("z", yaw).as_matrix()
    frame = "world"

    print(HELP)
    if not a.sim and not a.yes:
        print("Robot must be in ./heal_server + velocity-control mode (NOT free_drive). Someone on the E-STOP.")
        input("Press ENTER to take control... ")

    q, age, _ = be.state()
    ctrl.sync(q)
    writer, rec_crumbs, seg_id, n_saved = None, [], None, 0
    contact_next, last_pos, was_back = False, None, False
    stale, running = False, True
    STOP = [False]
    signal.signal(signal.SIGINT, lambda *_: STOP.__setitem__(0, True))
    msg_at = None
    dt_target = 1.0 / a.hz
    t_prev, t_rec, t_hud = time.monotonic(), 0.0, 0.0
    msg = ""

    def start_recording():
        nonlocal writer, rec_crumbs, seg_id, last_pos
        q_, _, _ = be.state()
        seg_id = next_segment_id(seg_dir)
        writer = SegmentWriter(seg_dir, seg_id, be.fk(q_))
        rec_crumbs, last_pos = [], None
        ctrl.clear_history()
        return f"RECORDING seg_{seg_id:04d}  (backtrack with z = undo)"

    def stop_recording():
        nonlocal writer
        n = writer._n
        writer.close(); writer = None
        return f"saved seg_{seg_id:04d}.csv ({n} pts)"

    with KeyInput(a.keys or tc["key_backend"], tc["key_hold_s"]) as keys:
        try:
            while running and not STOP[0]:
                now = time.monotonic(); dt = max(1e-3, min(now - t_prev, 0.05)); t_prev = now

                # ---------------- discrete key events
                for ev in keys.events():
                    if ev == "q": running = False
                    elif ev == "space": be.gripper.toggle()
                    elif ev == "v":
                        msg = stop_recording() if writer else start_recording()
                    elif ev == "x":
                        contact_next = True; msg = "contact marked on next sample"
                    elif ev in "12345" and int(ev) - 1 < len(lin_sp):
                        level = int(ev) - 1
                        msg = f"speed level {level+1}: {lin_sp[level]*1000:.0f} mm/s, {math.degrees(ang_sp[level]):.0f} deg/s"
                    elif ev == "tab":
                        frame = "tool" if frame == "world" else "world"; msg = f"jog frame: {frame}"
                    elif ev == "g":
                        ctrl.auto_tangent = not ctrl.auto_tangent; msg = f"auto-tangent {'ON' if ctrl.auto_tangent else 'OFF'}"
                    elif ev == "c":
                        q, _, _ = be.state(); ctrl.sync(q); msg = "target re-synced to real pose"
                    elif ev == "esc":
                        q, _, _ = be.state(); ctrl.sync(q); be.send(np.zeros(be.n)); msg = "STOP + re-sync"
                    elif ev == "p":
                        T = be.fk(be.state()[0]); msg = f"pos {np.round(T[:3,3]*1000,1)} mm  rpy {np.round(Rot.from_matrix(T[:3,:3]).as_euler('xyz',degrees=True),1)} deg"
                    elif ev == "h": print("\n" + HELP)

                # ---------------- robot state / staleness
                q, age, _ = be.state()
                if age > 0.3:
                    be.send(np.zeros(be.n)); stale = True
                    if now - t_hud > 0.5:
                        sys.stdout.write("\r\033[K!! joint states stale - holding (robot stopped / fault?)"); t_hud = now
                    time.sleep(dt_target); continue
                if stale:
                    ctrl.sync(q); stale = False; msg = "joint states back - target re-synced"

                # ---------------- twist from held keys (game style)
                v, w = np.zeros(3), np.zeros(3)
                for k, ax, sgn in (("d", 0, 1), ("a", 0, -1), ("w", 1, 1), ("s", 1, -1), ("r", 2, 1), ("f", 2, -1)):
                    if keys.held(k): v[ax] += sgn
                for k, ax, sgn in (("u", 0, 1), ("o", 0, -1), ("i", 1, 1), ("k", 1, -1), ("j", 2, 1), ("l", 2, -1)):
                    if keys.held(k): w[ax] += sgn
                nv = np.linalg.norm(v); v = v / nv if nv > 1 else v
                nw = np.linalg.norm(w); w = w / nw if nw > 1 else w
                v, w = v * lin_sp[level], w * ang_sp[level]
                R = be.fk(q)[:3, :3] if frame == "tool" else R_jog
                twist = np.concatenate([R @ v, R @ w])
                back = keys.held("z")
                if back:                       # retrace at the selected speed level
                    twist = np.array([lin_sp[level], 0, 0, 0, 0, 0])

                qd, info = ctrl.step(q, twist, dt, backtrack=back)
                be.send(qd)

                # ---------------- stuck / collision guard
                if info["lag_s"] > 2.5:
                    be.send(np.zeros(be.n)); ctrl.sync(q)
                    msg = "!! arm NOT FOLLOWING (collision / joint limit / fault?) - stopped. Use z to back off."

                # ---------------- recording (actual pose, rec_hz)
                if writer is not None:
                    if back:
                        while rec_crumbs and rec_crumbs[-1] > info["history"]:
                            rec_crumbs.pop()
                        writer.truncate_to(len(rec_crumbs)); was_back = True
                    elif now - t_rec >= 1.0 / tc["record_hz"]:
                        t_rec = now
                        T = be.fk(q); pos = T[:3, 3]
                        if last_pos is None or np.linalg.norm(pos - last_pos) >= a.min_move or contact_next:
                            last_pos = pos.copy()
                            qt = Rot.from_matrix(T[:3, :3]).as_quat()
                            writer.write(Waypoint(time.time() - writer._t0, *pos, *qt, *q, contact=contact_next,
                                                  grip=be.gripper.value))
                            rec_crumbs.append(max(0, info["history"] - int(round(info["err_lin"] / 0.001))))
                            contact_next = False

                # ---------------- HUD
                if now - t_hud > 0.2:
                    T = be.fk(q)
                    rec = f"REC seg{seg_id:04d} {writer._n:5d}pts" if writer else "not recording"
                    flags = ("BACK " if back else "") + ("TANGENT " if ctrl.auto_tangent else "")
                    sys.stdout.write(f"\r\033[K[{rec}] L{level+1} {lin_sp[level]*1000:.0f}mm/s {frame} grip={be.gripper.state} "
                                     f"xyz=({T[0,3]*1000:6.1f},{T[1,3]*1000:6.1f},{T[2,3]*1000:6.1f}) "
                                     f"lag={info['err_lin']*1000:4.1f}mm {flags}{msg}")
                    sys.stdout.flush(); t_hud = now
                    if msg and msg_at is None:
                        msg_at = now
                    elif msg and now - msg_at > 2.5:
                        msg, msg_at = "", None
                el = time.monotonic() - now
                if el < dt_target:
                    time.sleep(dt_target - el)
        finally:
            be.stop()
            if writer is not None:
                print("\n" + stop_recording())
            be.shutdown()
    print("\nTeleop ended. Next:  ./buzzwire_teach.sh inspect   then   ./buzzwire_teach.sh merge")


if __name__ == "__main__":
    main()
