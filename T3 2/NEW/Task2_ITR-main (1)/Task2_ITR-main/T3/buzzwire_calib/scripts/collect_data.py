#!/usr/bin/env python3
"""Interactive calibration-data collection (run while the robot is in ./free_drive).

Workflow
  1. Hand-guide the robot so the ChArUco board ("matrix") is clearly visible to both cameras.
  2. Press  SPACE : captures a burst (joint states + FK + N frames from each camera), averaged & saved.
     (repeat from a few different arm poses on the SAME matrix if you like)
  3. Press  n     : move on to the next matrix / board placement.
  4. ...repeat... then press  f  : finish.

Safe to kill at any moment (Ctrl-C, freeze, SSH drop): just start again, the session is resumed.
"""
import argparse, os, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2, numpy as np
from buzzwire_calib.config import load_config, save_config
from buzzwire_calib.board import BoardSpec, CharucoBoard
from buzzwire_calib.session import SessionStore, find_latest_session, new_session_dir
from buzzwire_calib.collector import Collector, KeyReader
from buzzwire_calib.cameras import OpenCVCamera


def build_sources(cfg, hub):
    cams = {}
    for name, c in cfg["cameras"].items():
        src = str(c["source"])
        if src.startswith("ros:") or src.startswith("rosc:"):
            from buzzwire_calib.ros_io import make_ros_image_source
            cams[name] = make_ros_image_source(hub, name, src.split(":", 1)[1], compressed=src.startswith("rosc:"))
        else:
            cams[name] = OpenCVCamera(name, src, c.get("width", 0), c.get("height", 0), c.get("fps", 0),
                                      c.get("fourcc", "MJPG"))
    return cams


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--data-root", default=None)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--new", action="store_true", help="force a brand-new session")
    g.add_argument("--session", default=None, help="resume this specific session dir / name")
    ap.add_argument("--no-preview", action="store_true")
    a = ap.parse_args()

    cfg = load_config(a.config)
    data_root = Path(a.data_root or cfg["paths"]["data_root"])
    data_root.mkdir(parents=True, exist_ok=True)

    # ---- choose / resume session
    if a.session:
        sdir = Path(a.session) if Path(a.session).exists() else data_root / a.session
    elif a.new:
        sdir = new_session_dir(data_root)
    else:
        last = find_latest_session(data_root)
        if last and not SessionStore(last).state()["finished"]:
            st = SessionStore(last).state()
            print(f"\nFound UNFINISHED session {last.name}: matrix #{st['matrix']}, {st['n_captures']} captures.")
            ans = input("Resume it? [Y/n] (n = start a new session): ").strip().lower()
            sdir = last if ans in ("", "y", "yes") else new_session_dir(data_root)
        else:
            sdir = new_session_dir(data_root)
    store = SessionStore(sdir)
    if store.state()["finished"]:
        print("That session was already finished - re-opening it so you can add more.")
        store.reopen()
    cfg_path = sdir / "config_used.yaml"
    if not cfg_path.exists():
        save_config(cfg, cfg_path)
    print(f"Session: {sdir}")

    # ---- hardware
    from buzzwire_calib.ros_io import RosHub, RosRobotState
    hub = RosHub()
    robot = RosRobotState(hub, cfg["robot"])
    cams = build_sources(cfg, hub)
    for c in cams.values():
        c.start()
    board = CharucoBoard(BoardSpec.from_cfg(cfg["board"]))
    col = Collector(cfg, robot, cams, store, board)
    keys = cfg["keys"]
    preview = cfg["capture"]["preview"] and not a.no_preview and bool(os.environ.get("DISPLAY"))

    print(f"""
Keys:  [{'SPACE' if keys['capture']==' ' else keys['capture']}] capture   [{keys['next_matrix']}] next matrix   [{keys['undo']}] undo last
       [{keys['status']}] status   [{keys['finish']}] FINISH   [{keys['quit']}] quit (resume later)
""")
    last_print, last_status, busy_msg = 0.0, "", ""
    try:
        with KeyReader() as kr:
            while True:
                now = time.time()
                if now - last_print > 0.5:
                    line = col.status_line()
                    sys.stdout.write("\r\033[K" + line[:200])
                    sys.stdout.flush()
                    last_print = now
                ch = kr.get()
                if preview:
                    tiles = []
                    for n, c in cams.items():
                        img, _, _ = c.latest()
                        if img is None:
                            img = np.zeros((360, 640, 3), np.uint8)
                        else:
                            cs, ids = board.detect(cv2.resize(img, None, fx=0.5, fy=0.5))
                            img = cv2.resize(img, None, fx=0.5, fy=0.5)
                            board.draw(img, cs, ids)
                            cv2.putText(img, f"{n}: {0 if ids is None else len(ids)} corners", (10, 25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        tiles.append(cv2.resize(img, (640, 360)))
                    cv2.imshow("calibration preview", np.hstack(tiles))
                    k = cv2.waitKey(1)
                    if ch is None and k != -1 and k != 255:
                        ch = chr(k & 0xFF)
                else:
                    time.sleep(0.03)
                if ch is None:
                    continue
                print()
                ch_lower = ch.lower()
                if ch == keys["capture"] or ch_lower == "c" or ch in ("\r", "\n"):
                    ok, msg = col.capture(force=False)
                    print(("OK  " if ok else "FAIL ") + msg)
                elif ch_lower == keys["next_matrix"].lower() or ch_lower == "m":
                    m = store.next_matrix()
                    print(f"-> now on matrix #{m}")
                elif ch_lower == keys["undo"].lower():
                    cid = store.undo_last()
                    print(f"undid capture #{cid}" if cid is not None else "nothing to undo")
                elif ch_lower == keys["status"].lower():
                    print(col.status_line(), "\n", store.state())
                elif ch_lower == keys["finish"].lower():
                    store.finish()
                    st = store.state()
                    print(f"\n================ SESSION FINISHED ================")
                    print(f"Total captures: {st['n_captures']} over {len(st['per_matrix'])} matrices.")
                    print(f"Distribution : {st['per_matrix']}")
                    print(f"\nTo compute calibration now, run:")
                    print(f"  python3 scripts/calibrate_extrinsics.py --session {sdir}")
                    print(f"==================================================")
                    break
                elif ch_lower == keys["quit"].lower():
                    print(f"Quit WITHOUT finishing. Resume anytime with:")
                    print(f"  python3 scripts/collect_data.py --session {sdir}")
                    break
    except KeyboardInterrupt:
        print(f"\nInterrupted - all saved data is safe. Resume with:  python3 scripts/collect_data.py --session {sdir}")
    finally:
        for c in cams.values():
            c.stop()
        if preview:
            cv2.destroyAllWindows()
        hub.shutdown()


if __name__ == "__main__":
    main()
