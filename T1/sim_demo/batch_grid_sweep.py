#!/usr/bin/env python3
"""
Batch Grid Sweep Runner for OpenMANIPULATOR-X Simulation.

Iterates over discrete grid coordinates x in [0..11] and y in [0..11] (144 cells total),
invoking `run_grid_simulation.py --grid {x} {y}` for each coordinate, tracking reachability,
cycle timing, and placement precision live in MuJoCo.

Live MuJoCo Viewer:
  • By default, launches the live interactive 3D MuJoCo viewer.
  • On macOS, automatically handles Cocoa main-thread requirements via `mjpython`.

Recordings in a Single Folder:
  • Every run's `storyboard.png` and `final_check_overhead.png` (plus `final_check_oblique.png`)
    are saved together into a single directory: `<sweep_dir>/records/`
  • Also generates a visual `index.html` gallery in that single folder.

Usage:
  # Live visual sweep across full 12x12 grid (0..11 x 0..11):
  python3 "T1 2/sim_demo/batch_grid_sweep.py"

  # Live visual sweep on a focused region (e.g. green mat x: 3..9, y: 5..9):
  python3 "T1 2/sim_demo/batch_grid_sweep.py" --x-range 3 9 --y-range 5 9

  # Specify a custom single records directory:
  python3 "T1 2/sim_demo/batch_grid_sweep.py" --record-dir "demo_runs/all_records"

  # Headless mode (fast, without live viewer window):
  python3 "T1 2/sim_demo/batch_grid_sweep.py" --headless
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


def generate_html_gallery(records_dir: str, csv_rows: list, sweep_name: str) -> str:
    """Generates a standalone responsive HTML gallery of all storyboards and final checks."""
    html_lines = [
        "<!DOCTYPE html>",
        "<html lang='en'>",
        "<head>",
        "  <meta charset='UTF-8'>",
        "  <title>MuJoCo Grid Sweep Records Gallery</title>",
        "  <style>",
        "    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #121418; color: #e0e0e0; margin: 0; padding: 24px; }",
        "    h1 { color: #58a6ff; margin-bottom: 4px; }",
        "    .stats { color: #8b949e; margin-bottom: 24px; font-size: 14px; }",
        "    .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(460px, 1fr)); gap: 20px; }",
        "    .card { background: #1c2128; border: 1px solid #30363d; border-radius: 8px; padding: 14px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }",
        "    .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; font-size: 14px; font-weight: bold; }",
        "    .badge { padding: 3px 8px; border-radius: 4px; font-size: 12px; }",
        "    .badge-SUCCESS { background: #238636; color: #fff; }",
        "    .badge-OUT_OF_REACH { background: #d29922; color: #000; }",
        "    .badge-PEG_NOT_FOUND { background: #484f58; color: #fff; }",
        "    .badge-FAILED { background: #da3633; color: #fff; }",
        "    .img-container { display: flex; gap: 10px; }",
        "    .img-box { flex: 1; text-align: center; }",
        "    .img-box img { width: 100%; border-radius: 4px; border: 1px solid #30363d; cursor: pointer; transition: transform 0.2s; }",
        "    .img-box img:hover { transform: scale(1.02); }",
        "    .caption { font-size: 11px; color: #8b949e; margin-top: 4px; }",
        "  </style>",
        "</head>",
        "<body>",
        f"  <h1>MuJoCo Grid Sweep Records - {sweep_name}</h1>",
        f"  <div class='stats'>Total Grid Runs: {len(csv_rows)} | Single Records Directory: <code>{records_dir}</code></div>",
        "  <div class='grid'>"
    ]

    for r in csv_rows:
        gx, gy = r['gx'], r['gy']
        status = r['status']
        err = f"{r['error_mm']:.1f} mm" if r.get('error_mm') is not None else "n/a"
        tilt = f"{r.get('tilt_deg', 'n/a')}°"
        dur = f"{r.get('duration_s', 0):.1f}s"
        badge_cls = f"badge-{status}" if status in ["SUCCESS", "OUT_OF_REACH", "PEG_NOT_FOUND"] else "badge-FAILED"

        sb_file = f"grid_x{gx:02d}_y{gy:02d}_storyboard.png"
        fc_file = f"grid_x{gx:02d}_y{gy:02d}_final_check_overhead.png"

        html_lines.append("    <div class='card'>")
        html_lines.append(f"      <div class='card-header'><span>Grid ({gx}, {gy}) &bull; X={r['x_m']:.3f}m, Y={r['y_m']:.3f}m</span><span class='badge {badge_cls}'>{status}</span></div>")
        html_lines.append(f"      <div style='font-size: 12px; color: #8b949e; margin-bottom: 8px;'>Error: <b style='color:#fff;'>{err}</b> | Tilt: <b style='color:#fff;'>{tilt}</b> | Time: {dur}</div>")
        html_lines.append("      <div class='img-container'>")

        if os.path.exists(os.path.join(records_dir, sb_file)):
            html_lines.append(f"        <div class='img-box'><a href='{sb_file}' target='_blank'><img src='{sb_file}' alt='Storyboard'></a><div class='caption'>Storyboard (9-steps)</div></div>")
        else:
            html_lines.append("        <div class='img-box' style='background:#222; border:1px dashed #444; border-radius:4px; display:flex; align-items:center; justify-content:center; height:120px; color:#666;'>No Storyboard</div>")

        if os.path.exists(os.path.join(records_dir, fc_file)):
            html_lines.append(f"        <div class='img-box'><a href='{fc_file}' target='_blank'><img src='{fc_file}' alt='Final Check'></a><div class='caption'>Final Check Overhead</div></div>")
        else:
            html_lines.append("        <div class='img-box' style='background:#222; border:1px dashed #444; border-radius:4px; display:flex; align-items:center; justify-content:center; height:120px; color:#666;'>No Final Check</div>")

        html_lines.append("      </div>")
        html_lines.append("    </div>")

    html_lines.extend([
        "  </div>",
        "</body>",
        "</html>"
    ])

    html_path = os.path.join(records_dir, "index.html")
    with open(html_path, "w") as f:
        f.write("\n".join(html_lines) + "\n")
    return html_path


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
    parser.add_argument("--viewer", "--live", dest="live", action="store_true", default=True,
                        help="Launch native live MuJoCo 3D viewer for simulation (default: True).")
    parser.add_argument("--no-viewer", "--headless", dest="live", action="store_false",
                        help="Disable live viewer window for fast headless execution.")
    parser.add_argument("--calib-poses", type=int, default=6,
                        help="Number of calibration poses per run (default: 6 for speed, min: 6)")
    parser.add_argument("--linger", type=float, default=0.5,
                        help="Seconds to keep 3D viewer open after each run (default: 0.5s)")
    parser.add_argument("--record-dir", default=None,
                        help="Single folder to record storyboard.png and final_check_overhead.png (default: <sweep_dir>/records)")
    parser.add_argument("--save-video", action="store_true", default=False,
                        help="Save MP4 video recordings for every run (default: False to save disk)")
    parser.add_argument("--no-video", action="store_true", default=True,
                        help="Skip MP4 video recordings (default: True)")
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

    records_dir = os.path.abspath(args.record_dir or os.path.join(sweep_dir, "records"))
    os.makedirs(records_dir, exist_ok=True)

    csv_path = os.path.join(sweep_dir, "grid_sweep_results.csv")
    json_path = os.path.join(sweep_dir, "grid_sweep_results.json")

    # Determine Python executable
    python_bin = shutil.which("mjpython") if (sys.platform == "darwin" and args.live) else sys.executable
    if not python_bin:
        python_bin = sys.executable

    print("\n" + "=" * 65)
    print("      OPENMANIPULATOR-X BATCH GRID SWEEP RUNNER")
    print("=" * 65)
    print(f"  Grid Range X (Lateral) : {x_min} to {x_max} (step {step}) -> {len(x_vals)} values")
    print(f"  Grid Range Y (Forward) : {y_min} to {y_max} (step {step}) -> {len(y_vals)} values")
    print(f"  Total Grid Coordinates : {total_runs} combinations")
    print(f"  MuJoCo 3D Viewer       : {'\033[92mLIVE (Interactive 3D Window)\033[0m' if args.live else 'HEADLESS (No Window)'}")
    print(f"  Single Records Folder  : \033[96m{records_dir}\033[0m")
    print(f"  Calibration Poses      : {args.calib_poses}")
    print(f"  Results Directory      : {sweep_dir}")
    print("=" * 65 + "\n")

    grid_results = {}
    csv_rows = []
    run_idx = 0
    t_start = time.time()

    child_env = os.environ.copy()
    child_env["MJPYTHON_LAUNCHED"] = "1"

    for gy in y_vals:
        for gx in x_vals:
            run_idx += 1
            x_m, y_m = grid_to_xy(gx, gy)
            cmd = [
                python_bin,
                RUN_GRID_SCRIPT,
                "--grid", str(gx), str(gy),
                "--calib-poses", str(args.calib_poses),
                "--linger", str(args.linger),
                "--record-dir", records_dir
            ]
            if args.live:
                cmd.append("--viewer")
            else:
                cmd.append("--headless")
            if not args.save_video:
                cmd.append("--no-video")

            t0 = time.time()
            print(f"[{run_idx:3d}/{total_runs:3d}] Starting Grid ({gx:2d}, {gy:2d}) -> X={x_m:6.3f}m, Y={y_m:6.3f}m ...")
            try:
                proc = subprocess.Popen(
                    cmd,
                    env=child_env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )
                stdout_lines = []
                for line in proc.stdout:
                    stdout_lines.append(line)
                    line_s = line.strip()
                    if any(k in line_s for k in ["[1/3]", "[2a/3]", "[2b/3]", "[3/3]", "reached |", "attempt 1 ->"]):
                        print(f"     | {line_s}")
                proc.wait(timeout=120)
                stdout = "".join(stdout_lines)
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout = "TIMEOUT after 120s"
            duration = round(time.time() - t0, 1)

            parsed = parse_output(stdout)
            status = parsed["status"]
            err_mm = parsed["error_mm"]
            tilt = parsed["tilt_deg"]

            status_col = {
                "SUCCESS": "\033[92mSUCCESS\033[0m",
                "OUT_OF_REACH": "\033[93mOUT_OF_REACH\033[0m",
                "PEG_NOT_FOUND": "\033[90mPEG_NOT_FOUND\033[0m",
                "GRASP_FAILED": "\033[91mGRASP_FAILED\033[0m",
                "FAILED": "\033[91mFAILED\033[0m",
            }.get(status, status)

            err_str = f"{err_mm:4.1f} mm" if err_mm is not None else "  n/a  "
            tilt_str = f"{tilt:2d}°" if tilt is not None else "n/a"

            # Check for saved images in records directory
            sb_name = f"grid_x{gx:02d}_y{gy:02d}_storyboard.png"
            fc_name = f"grid_x{gx:02d}_y{gy:02d}_final_check_overhead.png"
            sb_exists = os.path.exists(os.path.join(records_dir, sb_name))
            fc_exists = os.path.exists(os.path.join(records_dir, fc_name))

            rec_note = f"[Storyboard: {'YES' if sb_exists else 'NO'} | FinalCheck: {'YES' if fc_exists else 'NO'}]"
            print(f"  -> Outcome: [{status_col:17s}] | Err: {err_str} | Tilt: {tilt_str} | {rec_note} | {duration:4.1f}s\n")

            entry = {
                "run_idx": run_idx,
                "gx": gx,
                "gy": gy,
                "x_m": x_m,
                "y_m": y_m,
                "status": status,
                "error_mm": err_mm,
                "tilt_deg": tilt,
                "storyboard": sb_name if sb_exists else None,
                "final_check": fc_name if fc_exists else None,
                "duration_s": duration
            }
            grid_results[(gx, gy)] = entry
            csv_rows.append(entry)

            # Periodically write intermediate results and HTML gallery
            if run_idx % 3 == 0 or run_idx == total_runs:
                with open(csv_path, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=list(entry.keys()))
                    writer.writeheader()
                    writer.writerows(csv_rows)
                with open(json_path, "w") as f:
                    json.dump(csv_rows, f, indent=2)
                generate_html_gallery(records_dir, csv_rows, f"sweep_{stamp}")

    total_time = round(time.time() - t_start, 1)

    # Print final summary heatmap
    print_heatmap(grid_results, x_vals, y_vals)

    # Final HTML Gallery generation
    html_file = generate_html_gallery(records_dir, csv_rows, f"sweep_{stamp}")

    # Statistics
    successes = [r for r in csv_rows if r["status"] == "SUCCESS"]
    out_of_reach = [r for r in csv_rows if r["status"] == "OUT_OF_REACH"]
    not_found = [r for r in csv_rows if r["status"] == "PEG_NOT_FOUND"]
    failures = [r for r in csv_rows if r["status"] in ("FAILED", "GRASP_FAILED", "MOTION_ABORTED")]

    errors = [r["error_mm"] for r in successes if r["error_mm"] is not None]
    durations = [r["duration_s"] for r in successes]

    recorded_sb = sum(1 for r in csv_rows if r["storyboard"])
    recorded_fc = sum(1 for r in csv_rows if r["final_check"])

    print("=" * 65)
    print("                    FINAL SWEEP SUMMARY")
    print("=" * 65)
    print(f"  Total Runs Completed  : {total_runs} (in {total_time:.1f} s)")
    print(f"  Successes             : {len(successes)} ({len(successes)/total_runs*100:.1f}%)")
    print(f"  Out of Reach          : {len(out_of_reach)} ({len(out_of_reach)/total_runs*100:.1f}%)")
    print(f"  Off-board / Unseen    : {len(not_found)} ({len(not_found)/total_runs*100:.1f}%)")
    print(f"  Failures              : {len(failures)} ({len(failures)/total_runs*100:.1f}%)")
    print(f"  Storyboards Recorded  : {recorded_sb} files in single folder")
    print(f"  Final Checks Recorded : {recorded_fc} files in single folder")
    if errors:
        print(f"  Mean Placement Error  : {np.mean(errors):.2f} mm (min: {np.min(errors):.2f}, max: {np.max(errors):.2f} mm)")
    if durations:
        print(f"  Mean Cycle Duration   : {np.mean(durations):.1f} s")
    print(f"  Single Records Folder : {records_dir}")
    print(f"  HTML Visual Gallery   : {html_file}")
    print(f"  Saved CSV Report      : {csv_path}")
    print(f"  Saved JSON Report     : {json_path}")
    print("=" * 65 + "\n")


if __name__ == "__main__":
    main()
