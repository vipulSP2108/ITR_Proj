#!/usr/bin/env python3
"""
Interactive 11x11 Grid Simulation Runner for OpenMANIPULATOR-X.

Workspace Grid Layout:
  • 11x11 discrete grid over the workbench table & green mat.
  • The robot base is fixed at coordinate (6, 1) [Lateral=6, Forward=1, X=0.0m, Y=0.0m].
  • Lateral Axis  (gx in 1..11): gx=1 is left (-Y ≈ -0.15m, circle socket side), gx=6 is center (Y=0.0m), gx=11 is right (+Y ≈ +0.15m).
  • Forward Axis  (gy in 1..11): gy=1 is robot base (X=0.0m), gy=7 is center mat (X=0.15m), gy=11 is board edge (X≈0.25m).

Live MuJoCo Viewer:
  • By default, launches MuJoCo's interactive 3D viewer live.
  • On macOS, automatically handles Cocoa main-thread requirements via seamless `mjpython` dispatch.

Usage:
  # Launch live simulation with peg placed at grid coordinate (4, 7):
  python3 run_grid_simulation.py --grid 4 7

  # Random peg placement across reachable workspace:
  python3 run_grid_simulation.py --random

  # Headless mode (fast, without live viewer):
  python3 run_grid_simulation.py --grid 4 7 --headless
"""

import os
import sys
import shutil

# Seamless auto-dispatch to mjpython on macOS for native Cocoa 3D viewer
if sys.platform == "darwin" and os.environ.get("MJPYTHON_LAUNCHED") != "1":
    if "--no-viewer" not in sys.argv and "--headless" not in sys.argv:
        mjpython = shutil.which("mjpython")
        if mjpython and os.path.exists(mjpython):
            os.environ["MJPYTHON_LAUNCHED"] = "1"
            os.execv(mjpython, [mjpython] + sys.argv)

import argparse
import random
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from run_pipeline import main as run_pipeline_main


def grid_to_xy(gx: int, gy: int) -> tuple:
    """
    Converts 11x11 discrete grid coordinate (gx, gy) to robot base frame (X, Y) in metres.
    Robot is at (6, 1) -> X=0.0m, Y=0.0m.
    """
    gx = int(np.clip(gx, 1, 11))
    gy = int(np.clip(gy, 1, 11))

    # gy (Forward X): gy=1 -> 0.0m, gy=11 -> 0.25m
    x = (gy - 1) * (0.25 / 10.0)

    # gx (Lateral Y): gx=1 -> -0.15m (left), gx=6 -> 0.0m, gx=11 -> +0.15m (right)
    y = (gx - 6) * (0.15 / 5.0)

    return round(x, 4), round(y, 4)


def draw_ascii_grid(peg_gx: int, peg_gy: int, x_m: float, y_m: float):
    """
    Displays an intuitive 11x11 ASCII map of the workspace.
    """
    print("\n" + "=" * 59)
    print("      OPENMANIPULATOR-X WORKSPACE: 11x11 GRID MAP")
    print("=" * 59)
    print("               [ BOARD & SOCKET CUTOUTS (X ≈ 0.25m) ]")
    print("     1   2   3   4   5   6   7   8   9  10  11  <- gx (Lateral / Y)")
    print("   +---+---+---+---+---+---+---+---+---+---+---+")

    for gy in range(11, 0, -1):
        row_str = f"{gy:2d} |"
        for gx in range(1, 12):
            if gx == 6 and gy == 1:
                cell = " R "  # Robot Base
            elif gx == peg_gx and gy == peg_gy:
                cell = " * "  # Peg Target
            elif gy >= 10 and gx in [1, 2, 3]:
                cell = " O "  # Circle Socket Cutout
            elif gy >= 10 and gx in [4, 5]:
                cell = " H "  # Hexagon Socket Cutout
            elif gy >= 10 and gx in [6, 7]:
                cell = " S "  # Square Socket Cutout
            elif gy >= 10 and gx in [8, 9, 10]:
                cell = " T "  # Triangle Socket Cutout
            elif 4 <= gy <= 9 and 3 <= gx <= 9:
                cell = " . "  # Reachable Green Mat Area
            else:
                cell = "   "  # Outer boundary
            row_str += cell + "|"
        print(row_str)
        print("   +---+---+---+---+---+---+---+---+---+---+---+")

    print("               [ ROBOT BASE AT (6, 1) (X = 0.0m) ]")
    print("  Legend: [R] Robot Base  [*] Peg Target  [O] Circle Socket Cutout")
    print(f"  Selected Grid : ({peg_gx}, {peg_gy})")
    print(f"  Robot Frame   : X = {x_m:.3f} m ({x_m*100:.1f} cm), Y = {y_m:.3f} m ({y_m*100:.1f} cm)")
    print("=" * 59 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Run OpenMANIPULATOR-X 11x11 Grid Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--grid", type=int, nargs=2, metavar=("GX", "GY"),
                       help="Grid coordinates (gx in 1..11, gy in 1..11). Robot is at (6, 1).")
    group.add_argument("--random", action="store_true",
                       help="Randomly spawn peg in reachable workspace.")
    group.add_argument("--peg-xy", type=float, nargs=2, metavar=("X", "Y"),
                       help="Direct metric coordinates in metres (e.g. 0.16 0.05).")

    parser.add_argument("--viewer", dest="viewer", action="store_true", default=True,
                        help="Launch live MuJoCo 3D interactive viewer (default: True).")
    parser.add_argument("--no-viewer", "--headless", dest="viewer", action="store_false",
                        help="Disable live viewer window for fast headless execution.")
    parser.add_argument("--opencv-gui", action="store_true", default=False,
                        help="Use OpenCV HUD viewer window instead of native MuJoCo viewer.")
    parser.add_argument("--calib-poses", type=int, default=8,
                        help="Number of calibration poses (default: 8 for fast demo, 16 for maximum precision).")
    parser.add_argument("--linger", type=float, default=3.0,
                        help="Seconds to keep viewer window open upon completion (default: 3.0s).")
    parser.add_argument("--no-video", action="store_true", help="Skip MP4 video rendering.")
    parser.add_argument("--until", choices=["home", "workspace", "pick"], default="pick")
    args = parser.parse_args()

    # Determine peg coordinate
    if args.grid:
        gx, gy = args.grid
        x, y = grid_to_xy(gx, gy)
    elif args.peg_xy:
        x, y = args.peg_xy
        # Estimate closest grid
        gy = int(round(x / 0.025)) + 1
        gx = int(round(y / 0.030)) + 6
    else:
        # Default or --random: pick reachable cell on the green mat (gx in 3..9, gy in 5..9)
        gx = random.randint(3, 9)
        gy = random.randint(5, 9)
        x, y = grid_to_xy(gx, gy)
        print(f"  [Random Grid Selection] Picked Grid Cell ({gx}, {gy})")

    # Display 11x11 ASCII Grid Map
    draw_ascii_grid(gx, gy, x, y)

    # Build command line for run_pipeline.py
    sys.argv = [
        os.path.join(HERE, "run_pipeline.py"),
        "--peg-xy", str(x), str(y),
        "--until", args.until,
        "--calib-poses", str(args.calib_poses),
        "--linger", str(args.linger)
    ]
    if args.viewer and not args.opencv_gui:
        sys.argv.append("--viewer")
    elif args.opencv_gui:
        sys.argv.append("--opencv-gui")
    if args.no_video:
        sys.argv.append("--no-video")

    # Execute simulation
    run_pipeline_main()


if __name__ == "__main__":
    main()

