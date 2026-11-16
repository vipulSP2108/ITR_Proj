#!/usr/bin/env python3
"""Grab board images from both cameras for intrinsic calibration (robot NOT needed).
Hold the board at many tilts/distances and move it to every image corner. SPACE = save, q = quit."""
import argparse, os, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2, numpy as np
from buzzwire_calib.config import load_config
from buzzwire_calib.board import BoardSpec, CharucoBoard, sharpness


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--out", default="data/intrinsics")
    a = ap.parse_args()
    cfg = load_config(a.config)
    board = CharucoBoard(BoardSpec.from_cfg(cfg["board"]))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from collect_data import build_sources
    from buzzwire_calib.collector import KeyReader
    hub = None
    if any(str(c["source"]).startswith("ros") for c in cfg["cameras"].values()):
        from buzzwire_calib.ros_io import RosHub
        hub = RosHub()
    cams = build_sources(cfg, hub)
    for c in cams.values():
        c.start()
    counts = {n: len(list((Path(a.out) / n).glob("*.png"))) for n in cams}
    for n in cams:
        (Path(a.out) / n).mkdir(parents=True, exist_ok=True)
    has_display = bool(os.environ.get("DISPLAY"))
    print("Keys: [SPACE or c] save frame from camera(s) seeing board  |  [q] quit")
    if not has_display:
        print("[NOTE] Headless mode: DISPLAY not detected. Capturing from terminal keys.")
    try:
        with KeyReader() as kr:
            while True:
                tiles, det = [], {}
                ch = kr.get()
                for n, c in cams.items():
                    img, _, _ = c.latest()
                    if img is None:
                        tiles.append(np.zeros((360, 640, 3), np.uint8)); continue
                    cs, ids = board.detect(img)
                    det[n] = (img, cs, ids)
                    if has_display:
                        vis = img.copy()
                        board.draw(vis, cs, ids)
                        cv2.putText(vis, f"{n}: saved {counts[n]}  corners {0 if ids is None else len(ids)}", (10, 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                        tiles.append(cv2.resize(vis, (640, 360)))
                if has_display and tiles:
                    cv2.imshow("intrinsics capture", np.hstack(tiles))
                    k = cv2.waitKey(30) & 0xFF
                    if ch is None and k != -1 and k != 255:
                        ch = chr(k & 0xFF)
                else:
                    time.sleep(0.05)

                if ch is None:
                    continue
                ch_l = ch.lower()
                if ch_l == "q":
                    break
                if ch in (" ", "c", "\r", "\n"):
                    for n, (img, cs, ids) in det.items():
                        if ids is not None and len(ids) >= cfg["capture"]["min_corners"] + 4 and sharpness(img) > 20:
                            cv2.imwrite(str(Path(a.out) / n / f"{int(time.time()*1000)}.png"), img)
                            counts[n] += 1
                            print(f"saved {n} (total {counts[n]})")
                        else:
                            print(f"{n}: board not clearly visible - not saved")
    finally:
        for c in cams.values():
            c.stop()
        if has_display:
            cv2.destroyAllWindows()
        if hub:
            hub.shutdown()


if __name__ == "__main__":
    main()
