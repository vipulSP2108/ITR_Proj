#!/usr/bin/env python3
"""Intrinsic calibration per camera. Sources: images from capture_intrinsics.py and/or a collected session."""
import argparse, glob, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2
from buzzwire_calib.config import load_config
from buzzwire_calib.board import BoardSpec, CharucoBoard
from buzzwire_calib.session import SessionStore
from buzzwire_calib.intrinsics import calibrate_intrinsics, save_intrinsics
from buzzwire_calib.calibration import session_images


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "config" / "calib_config.yaml"))
    ap.add_argument("--images-dir", default="data/intrinsics", help="expects <dir>/<cam>/*.png")
    ap.add_argument("--session", default=None, help="also use the images of this session")
    ap.add_argument("--out", default="results/intrinsics")
    ap.add_argument("--rational", action="store_true", help="8-coef distortion model for wide-angle lenses")
    a = ap.parse_args()
    cfg = load_config(a.config)
    board = CharucoBoard(BoardSpec.from_cfg(cfg["board"]))
    for cam in cfg["cameras"]:
        imgs = []
        for f in sorted(glob.glob(str(Path(a.images_dir) / cam / "*.png"))):
            im = cv2.imread(f)
            if im is not None:
                imgs.append(im)
        if a.session:
            imgs += session_images(SessionStore(a.session), cam)
        print(f"\n[{cam}] {len(imgs)} images")
        if not imgs:
            continue
        try:
            res = calibrate_intrinsics(board, imgs, min_corners=cfg["capture"]["min_corners"] + 2, rational=a.rational)
        except RuntimeError as e:
            print("  FAILED:", e)
            continue
        save_intrinsics(res, Path(a.out) / f"{cam}.json")
        print(f"  RMS {res['rms_px']:.3f}px over {res['n_views']} views -> {a.out}/{cam}.json")
        for w in res["warnings"]:
            print("  WARNING:", w)


if __name__ == "__main__":
    main()
