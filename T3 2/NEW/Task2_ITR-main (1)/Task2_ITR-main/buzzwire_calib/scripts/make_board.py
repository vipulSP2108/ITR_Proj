#!/usr/bin/env python3
"""Render the ChArUco board from the config to a PNG at exact physical size (print at 100% scale!)."""
import argparse, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2
from buzzwire_calib.config import load_config
from buzzwire_calib.board import BoardSpec, CharucoBoard

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
ap.add_argument("--dpi", type=int, default=300)
ap.add_argument("--out", default="charuco_board.png")
a = ap.parse_args()
cfg = load_config(a.config)
spec = BoardSpec.from_cfg(cfg["board"])
mm_per_px = 25.4 / a.dpi
w = int(round(spec.squares_x * spec.square_length * 1000 / mm_per_px))
h = int(round(spec.squares_y * spec.square_length * 1000 / mm_per_px))
img = CharucoBoard(spec).generate_image(w, h, margin_px=0)
cv2.imwrite(a.out, img)
print(f"wrote {a.out} ({w}x{h}px @ {a.dpi} dpi = {spec.squares_x*spec.square_length*1000:.0f} x "
      f"{spec.squares_y*spec.square_length*1000:.0f} mm). Print at 100% scale, glue to a RIGID flat plate, "
      f"then MEASURE a printed square with calipers and put the real value in config square_length.")
