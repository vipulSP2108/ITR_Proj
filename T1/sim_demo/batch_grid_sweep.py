#!/usr/bin/env python3
"""
Batch Grid Sweep Runner for OpenMANIPULATOR-X Simulation.

Iterates over discrete grid coordinates x in [0..11] and y in [0..11] (144 cells total),
invoking `run_grid_simulation.py --grid {x} {y}` for each coordinate, tracking reachability,
cycle timing, and placement precision.

Usage:
  # Fast headless sweep over full 12x12 grid (0..11 x 0..11):
  python3 "T1 2/sim_demo/batch_grid_sweep.py"

  # Quick coarse sweep with step=2 (every second cell: 6x6 = 36 cells):
  python3 "T1 2/sim_demo/batch_grid_sweep.py" --step 2

  # Focused sweep over the reachable green mat area (x: 3..9, y: 4..9):
  python3 "T1 2/sim_demo/batch_grid_sweep.py" --x-range 3 9 --y-range 4 9

  # Watch each run live in the native MuJoCo 3D viewer:
  python3 "T1 2/sim_demo/batch_grid_sweep.py" --live
"""

import os
import sys
import time
import json
import csv
import argparse
import subprocess
import datetime
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_GRID_SCRIPT = os.path.join(HERE, "run_grid_simulation.py")
OUT_DIR = os.path.join(HERE, "..", "demo_runs")


def grid_to_xy(gx: int, gy: int) -> tuple:
    gx = int(np.clip(gx, 0, 11))
    gy = int(np.clip(gy, 0, 11))
    x = (gy - 1) * (0.25 / 10.0)
    y = (gx - 6) * (0.15 / 5.0)
    return round(x, 4), round(y, 4)


def parse_output(stdout_str: str) -> dict:
    """Parses simulation output for outcome, tracking stats, and vision error."""
    res = {
        "status": "UNKNOWN",
        "error_mm": None,
        "tilt_deg": None,
        "notes": "",
    }
    if "attempt 1 -> success" in stdout_str or "result: SUCCESS" in stdout_str or ("final: peg-to-socket error" in stdout_str and "SUCCESS" in stdout_str):
        res["status"] = "SUCCESS"
    elif "no feasible pick-and-place plan" in stdout_str or "outside the reachable workspace" in stdout_str:
        res["status"] = "OUT_OF_REACH"
    elif "circle peg not found in the snapshot" in stdout_str or "peg looks toppled over" in stdout_str:
        res["status"] = "PEG_NOT_FOUND"
    elif "GraspFailed" in stdout_str or "closed on nothing" in stdout_str or "peg dropped" in stdout_str:
        res["status"] = "GRASP_FAILED"
    elif "AbortMotion" in stdout_str:
        res["status"] = "MOTION_ABORTED"
    elif "FAILED" in stdout_str or "RuntimeError" in stdout_str:
        res["status"] = "FAILED"

    # Search for peg-to-socket error
    for line in stdout_str.splitlines():
        if "peg-to-socket error (vision)" in line:
            parts = line.split("peg-to-socket error (vision)")
            if len(parts) > 1:
                try:
                    val = float(parts[1].split()[0])
                    res["error_mm"] = val
                except ValueError:
                    pass
        if "chosen constant tool tilt:" in line:
            parts = line.split("chosen constant tool tilt:")
            if len(parts) > 1:
                try:
                    val = int(parts[1].split()[0])
                    res["tilt_deg"] = val
                except ValueError:
                    pass

    return res


def print_heatmap(grid_results: dict, x_range: list, y_range: list):
    """Prints an ASCII heatmap matrix of the evaluated workspace grid."""
    print("\n" + "=" * 65)
    print("       WORKSPACE REACHABILITY & SUCCESS HEATMAP MATRIX")
    print("=" * 65)
    header = "  Y \\ X " + " ".join(f"{x:3d}" for x in x_range)
    print(header)
    print("  " + "-" * (len(header) - 2))

    for gy in reversed(y_range):
        row_str = f"   {gy:2d}  |"
        for gx in x_range:
            if gx == 6 and gy == 1:
                sym = " R "  # Robot base
            else:
                data = grid_results.get((gx, gy))
                if data is None:
                    sym = " . "
                elif data["status"] == "SUCCESS":
                    sym = " \033[92mO\033[0m "  # Green O
                elif data["status"] == "OUT_OF_REACH":
                    sym = " \033[93m-\033[0m "  # Yellow -
                elif data["status"] == "PEG_NOT_FOUND":
                    sym = "   "  # Empty / off-board
                else:
                    sym = " \033[91mX\033[0m "  # Red X
            row_str += sym
        print(row_str)

    print("  " + "-" * (len(header) - 2))
    print("  Legend: [\033[92mO\033[0m] SUCCESS  [\033[93m-\033[0m] OUT_OF_REACH  [ ] OFF_BOARD  [\033[91mX\033[0m] FAILED  [R] BASE")
    print("=" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Batch sweep runner over OpenMANIPULATOR-X 12x12 grid (x in 0..11, y in 0..11)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--x-range", type=int, nargs=2, default=[0, 11], metavar=("XMIN", "XMAX"),
                        help="Range for lateral x (gx) index (default: 0 11)")
    parser.add_argument("--y-range", type=int, nargs=2, default=[0, 11], metavar=("YMIN", "YMAX"),
                        help="Range for forward y (gy) index (default: 0 11)")
    parser.add_argument("--step", type=int, default=1,
                        help="Grid sampling step (default: 1 for all points, 2 for coarse grid)")
    parser.add_argument("--live", action="store_true",
                        help="Launch native live MuJoCo 3D viewer for each run (slower, visual)")
    parser.add_argument("--calib-poses", type=int, default=6,
                        help="Number of calibration poses per run (default: 6 for speed, min: 6)")
    parser.add_argument("--save-video", action="store_true",
                        help="Save MP4 video recordings for every run (default: False to save disk)")
    parser.add_argument("--out-dir", default=None,
                        help="Output directory for sweep results JSON/CSV")
    args = parser.parse_args()

    x_min, x_max = args.x_range
    y_min, y_max = args.y_range
    step = max(1, args.step)

    x_vals = list(range(x_min, x_max + 1, step))
    y_vals = list(range(y_min, y_max + 1, step))
    total_runs = len(x_vals) * len(y_vals)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_dir = args.out_dir or os.path.join(OUT_DIR, f"sweep_{stamp}")
    os.makedirs(sweep_dir, exist_ok=True)

    csv_path = os.path.join(sweep_dir, "grid_sweep_results.csv")
    json_path = os.path.join(sweep_dir, "grid_sweep_results.json")

    print("\n" + "=" * 65)
    print("      OPENMANIPULATOR-X BATCH GRID SWEEP RUNNER")
    print("=" * 65)
    print(f"  Grid Range X (Lateral) : {x_min} to {x_max} (step {step}) -> {len(x_vals)} values")
    print(f"  Grid Range Y (Forward) : {y_min} to {y_max} (step {step}) -> {len(y_vals)} values")
    print(f"  Total Grid Coordinates : {total_runs} combinations")
    print(f"  Live Viewer            : {'ENABLED' if args.live else 'HEADLESS (Fast)'}")
    print(f"  Calibration Poses      : {args.calib_poses}")
    print(f"  Results Directory      : {sweep_dir}")
    print("=" * 65 + "\n")

    grid_results = {}
    csv_rows = []
    run_idx = 0
    t_start = time.time()

    for gy in y_vals:
        for gx in x_vals:
            run_idx += 1
            x_m, y_m = grid_to_xy(gx, gy)
            cmd = [
                sys.executable,
                RUN_GRID_SCRIPT,
                "--grid", str(gx), str(gy),
                "--calib-poses", str(args.calib_poses),
                "--linger", "0.5"
            ]
            if not args.live:
                cmd.append("--headless")
            if not args.save_video:
                cmd.append("--no-video")

            t0 = time.time()
            try:
                proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
                stdout = proc.stdout
            except subprocess.TimeoutExpired:
                stdout = "TIMEOUT after 120s"
            duration = round(time.time() - t0, 1)

            parsed = parse_output(stdout)
            status = parsed["status"]
            err_mm = parsed["error_mm"]
            tilt = parsed["tilt_deg"]

            # Format status printout
            status_col = {
                "SUCCESS": "\033[92mSUCCESS\033[0m",
                "OUT_OF_REACH": "\033[93mOUT_OF_REACH\033[0m",
                "PEG_NOT_FOUND": "\033[90mPEG_NOT_FOUND\033[0m",
                "GRASP_FAILED": "\033[91mGRASP_FAILED\033[0m",
                "FAILED": "\033[91mFAILED\033[0m",
            }.get(status, status)

            err_str = f"{err_mm:4.1f} mm" if err_mm is not None else "  n/a  "
            tilt_str = f"{tilt:2d}°" if tilt is not None else "n/a"

            print(f"[{run_idx:3d}/{total_runs:3d}] Grid ({gx:2d}, {gy:2d}) -> X={x_m:6.3f}m, Y={y_m:6.3f}m | "
                  f"[{status_col:17s}] | Err: {err_str} | Tilt: {tilt_str} | {duration:4.1f}s")

            entry = {
                "run_idx": run_idx,
                "gx": gx,
                "gy": gy,
                "x_m": x_m,
                "y_m": y_m,
                "status": status,
                "error_mm": err_mm,
                "tilt_deg": tilt,
                "duration_s": duration
            }
            grid_results[(gx, gy)] = entry
            csv_rows.append(entry)

            # Periodically write intermediate results
            if run_idx % 5 == 0 or run_idx == total_runs:
                with open(csv_path, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=list(entry.keys()))
                    writer.writeheader()
                    writer.writerows(csv_rows)
                with open(json_path, "w") as f:
                    json.dump(csv_rows, f, indent=2)

    total_time = round(time.time() - t_start, 1)

    # Print final summary heatmap
    print_heatmap(grid_results, x_vals, y_vals)

    # Statistics
    successes = [r for r in csv_rows if r["status"] == "SUCCESS"]
    out_of_reach = [r for r in csv_rows if r["status"] == "OUT_OF_REACH"]
    not_found = [r for r in csv_rows if r["status"] == "PEG_NOT_FOUND"]
    failures = [r for r in csv_rows if r["status"] in ("FAILED", "GRASP_FAILED", "MOTION_ABORTED")]

    errors = [r["error_mm"] for r in successes if r["error_mm"] is not None]
    durations = [r["duration_s"] for r in successes]

    print("=" * 65)
    print("                    FINAL SWEEP SUMMARY")
    print("=" * 65)
    print(f"  Total Runs Completed  : {total_runs} (in {total_time:.1f} s)")
    print(f"  Successes             : {len(successes)} ({len(successes)/total_runs*100:.1f}%)")
    print(f"  Out of Reach          : {len(out_of_reach)} ({len(out_of_reach)/total_runs*100:.1f}%)")
    print(f"  Off-board / Unseen    : {len(not_found)} ({len(not_found)/total_runs*100:.1f}%)")
    print(f"  Failures              : {len(failures)} ({len(failures)/total_runs*100:.1f}%)")
    if errors:
        print(f"  Mean Placement Error  : {np.mean(errors):.2f} mm (min: {np.min(errors):.2f}, max: {np.max(errors):.2f} mm)")
    if durations:
        print(f"  Mean Cycle Duration   : {np.mean(durations):.1f} s")
    print(f"  Saved CSV Report      : {csv_path}")
    print(f"  Saved JSON Report     : {json_path}")
    print("=" * 65 + "\n")


if __name__ == "__main__":
    main()
