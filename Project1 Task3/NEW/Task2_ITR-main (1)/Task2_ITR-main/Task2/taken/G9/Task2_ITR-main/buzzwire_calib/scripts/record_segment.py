#!/usr/bin/env python3
"""METHOD 1 - hand-guided teaching: record ONE segment while the arm is in ./free_drive.

  SPACE   open <-> close the gripper (so the stick does not fall out)
  p       pause / resume recording
  x       mark CONTACT (you heard the buzz) - the merger trims that region
  q       stop + save this segment  (Ctrl-C also saves)

Each run writes one crash-safe file  <session>/segments/seg_NNNN.csv  (flushed on every sample).
Touched the wire? Press q, free-drive the arm back to a safe spot, run this again: the merger
(./buzzwire_teach.sh merge) joins all segments, trims contacts and bridges small gaps.
"""
import argparse, signal, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from buzzwire_calib.config import load_config
from buzzwire_calib.keyboard import KeyInput
from buzzwire_calib.session import choose_teach_session, next_segment_id
from buzzwire_calib.trajectory import SegmentWriter, Waypoint


def _gpio(pin):
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM); GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        return lambda: GPIO.input(pin) == GPIO.LOW
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--session", default=None)
    ap.add_argument("--new", action="store_true")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--min-move", type=float, default=0.001, help="drop samples that moved less than this (m)")
    ap.add_argument("--contact-pin", type=int, default=None, help="optional GPIO BCM pin wired to the buzzer")
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()

    cfg = load_config(a.config)
    ask = (lambda _: "y") if a.yes else input
    sdir = choose_teach_session(cfg["paths"]["data_root"], a.session, a.new, ask=ask)
    seg_dir = sdir / "segments"
    seg_id = next_segment_id(seg_dir)
    print(f"\nSession {sdir}\nNew segment: seg_{seg_id:04d}.csv   (robot must be in ./free_drive)")
    print(__doc__.split("Each run")[0])
    if not a.yes:
        input("Press ENTER to start (recording begins immediately)... ")

    from buzzwire_calib.ros_io import RosHub, RosRobotState
    from buzzwire_calib.gripper import GripperHelper
    hub = RosHub("record_segment")
    robot = RosRobotState(hub, cfg["robot"])
    grip = GripperHelper(cfg, hub)
    t0 = time.time()
    while not (robot.get() and robot.get().T_base_ee is not None):
        if time.time() - t0 > 20:
            hub.shutdown(); sys.exit("No /joint_states or FK in 20 s. Is the robot running / workspace sourced?")
        time.sleep(0.05)

    contact_pin = _gpio(a.contact_pin) if a.contact_pin else None
    STOP = [False]
    signal.signal(signal.SIGINT, lambda *_: STOP.__setitem__(0, True))
    paused, mark, n_saved, n_drop, n_contact, last_pos = False, False, 0, 0, 0, None
    dt = 1.0 / a.hz

    with KeyInput("terminal", 0.12) as keys, SegmentWriter(seg_dir, seg_id, robot.get().T_base_ee) as wr:
        while not STOP[0]:
            t = time.time()
            for ev in keys.events():
                if ev == "q": STOP[0] = True
                elif ev == "space": grip.toggle()
                elif ev == "p": paused = not paused
                elif ev in ("x", "d"): mark = True
            s = robot.get()
            status = ""
            if s is None or s.T_base_ee is None:
                sys.stdout.write("\r\033[K waiting for robot...")
            else:
                T = s.T_base_ee; pos = T[:3, 3]
                if robot.age() > 0.5:
                    status = "!! joint states STALE (free_drive stopped?) - waiting, nothing is lost"
                elif not paused:
                    c = mark or (contact_pin is not None and contact_pin())
                    if last_pos is not None and np.linalg.norm(pos - last_pos) < a.min_move and not c:
                        n_drop += 1
                    else:
                        last_pos = pos.copy()
                        qt = Rot.from_matrix(T[:3, :3]).as_quat()
                        wr.write(Waypoint(time.time() - wr._t0, *pos, *qt, *s.q, contact=bool(c), grip=grip.value))
                        n_saved += 1; n_contact += int(bool(c)); mark = False
                sys.stdout.write(f"\r\033[K[{'PAUSED' if paused else 'REC'} seg{seg_id:04d}] pts={n_saved} contacts={n_contact} "
                                 f"grip={grip.state} xyz=({pos[0]*1000:6.1f},{pos[1]*1000:6.1f},{pos[2]*1000:6.1f}) {status}")
            sys.stdout.flush()
            time.sleep(max(0.0, dt - (time.time() - t)))
    print(f"\n\nsaved seg_{seg_id:04d}.csv: {n_saved} points, {n_contact} contact marks, {n_drop} stationary samples skipped")
    print(f"Record another segment:  ./buzzwire_teach.sh record --session {sdir}")
    print(f"Check + merge:           ./buzzwire_teach.sh inspect --session {sdir}   /   merge --session {sdir}")
    hub.shutdown()


if __name__ == "__main__":
    main()
