#!/usr/bin/env python3
"""
Merge all recorded segments in a session into one clean trajectory.

Usage
-----
  python3 scripts/merge_segments.py                               # latest session
  python3 scripts/merge_segments.py --session data/run_001
  python3 scripts/merge_segments.py --session data/run_001 --only 0 1 3   # skip seg 2
  python3 scripts/merge_segments.py --session data/run_001 --reverse-seg 2  # flip direction of seg 2
  python3 scripts/merge_segments.py --session data/run_001 --clearance 0.003  # +3 mm outward offset

Algorithm summary
-----------------
  1. Load all seg_NNNN.csv files found under <session>/segments/
  2. Discard segments that are too short (< 5 points)
  3. Trim contact-event regions (8 mm radius around any marked contact)
  4. Greedy spatial ordering (sort by proximity of start to previous end)
  5. Stitch / bridge gaps between segments:
       - gap < 15 mm  : overlap-trim and direct join
       - 15–120 mm   : cubic SLERP bridge (flagged gap=True)
       - > 120 mm    : same bridge but WARNING printed (section likely missed)
  6. Savitzky-Golay smoothing (removes hand-tremor jitter)
  7. Uniform arc-length resampling at 2 mm step
  8. Optional clearance offset (+N mm outward from wire centre)
  9. Write merged trajectory to <session>/merged_trajectory.csv
 10. Write JSON report to   <session>/merge_report.json

Outputs
-------
  <session>/merged_trajectory.csv   – final waypoints (ready for replay)
  <session>/merge_report.json       – stats, warnings, segment order
  <session>/segments/seg_????.csv   – untouched originals (always kept)
"""
import argparse, json, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from buzzwire_calib.config import load_config
from buzzwire_calib.session import find_latest_session
from buzzwire_calib.trajectory import (
    list_segments, load_segment, save_trajectory, merge_segments,
    apply_clearance_offset, reverse_trajectory, MergeConfig, _arc_lengths
)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("direction",     nargs="?", default="BOTH",
                    choices=["LR", "RL", "BOTH", "1_to_2", "2_to_1", "lr", "rl", "both"],
                    help="trajectory direction to activate (default: BOTH, generates both LR and RL)")
    ap.add_argument("--dir",         dest="dir_opt", default=None,
                    choices=["LR", "RL", "BOTH", "1_to_2", "2_to_1"],
                    help="direction alias (LR or RL)")
    ap.add_argument("--config",      default=str(Path(__file__).resolve().parent.parent /
                                                 "config" / "calib_config.yaml"))
    ap.add_argument("--session",     default=None)
    ap.add_argument("--only",        type=int, nargs="*", default=None,
                    help="use only these segment indices (0-based). default = all or auto-best")
    ap.add_argument("--exclude",     type=int, nargs="*", default=[],
                    help="skip these segment indices")
    ap.add_argument("--reverse-seg", type=int, nargs="*", default=[],
                    help="reverse direction of these segment indices before merging")
    ap.add_argument("--clearance",   type=float, default=0.0,
                    help="outward clearance offset in metres (e.g. 0.003 = 3 mm)")
    ap.add_argument("--stitch-radius",    type=float, default=0.015)
    ap.add_argument("--gap-bridge-max",   type=float, default=0.120)
    ap.add_argument("--resample-step",    type=float, default=0.002)
    ap.add_argument("--smoothing-window", type=int,   default=7)
    ap.add_argument("--contact-trim",     type=float, default=0.008)
    ap.add_argument("--out",              default=None,
                    help="output CSV path (default: <session>/merged_trajectory.csv)")
    a = ap.parse_args()

    active_dir = (a.dir_opt or a.direction or "BOTH").upper()
    if active_dir == "1_TO_2": active_dir = "LR"
    elif active_dir == "2_TO_1": active_dir = "RL"

    cfg = load_config(a.config)
    data_root = Path(cfg["paths"]["data_root"])

    # --- find session
    if a.session:
        sdir = Path(a.session)
    else:
        sdir = find_latest_session(data_root)
        if sdir is None:
            sys.exit("No session found. Run record_segment.py first.")
    seg_dir = sdir / "segments"
    if not seg_dir.exists():
        sys.exit(f"No 'segments' folder in {sdir}. Run record_segment.py first.")

    # --- load segments
    csv_paths = list_segments(seg_dir)
    if not csv_paths:
        sys.exit("No segment CSV files found.")

    all_segs = []
    for i, p in enumerate(csv_paths):
        wps = load_segment(p)
        pts = [(w.x, w.y, w.z) for w in wps]
        arc = float(_arc_lengths(np.array(pts))[-1]) if pts else 0.0
        contact_n = sum(1 for w in wps if w.contact)
        print(f"  seg {i:04d}  {p.name}  {len(wps):5d} pts  arc {arc*1000:.0f} mm  contacts {contact_n}")
        all_segs.append(wps)

    # --- filter / reverse
    indices = list(range(len(all_segs)))
    if a.only is not None:
        indices = [i for i in a.only if 0 <= i < len(all_segs)]
    else:
        # Smart auto-clean: check if dropping early aborted attempts yields a zero-gap clean path
        if len(all_segs) > 2:
            try:
                mcfg_test = MergeConfig(stitch_radius=a.stitch_radius, gap_bridge_max=a.gap_bridge_max)
                _, rep_all = merge_segments([all_segs[i] for i in indices], mcfg_test, log=lambda *_: None)
                if rep_all.get("gap_bridges", 0) > 0:
                    for start_idx in range(1, len(all_segs) - 1):
                        cand_indices = [i for i in range(start_idx, len(all_segs)) if i not in (a.exclude or [])]
                        if len(cand_indices) >= 2:
                            _, rep_cand = merge_segments([all_segs[i] for i in cand_indices], mcfg_test, log=lambda *_: None)
                            if rep_cand.get("gap_bridges", 0) == 0 and rep_cand["total_arc_m"] >= 0.60 * rep_all["total_arc_m"]:
                                print(f"\n[auto-clean] Dropping initial aborted attempts. Using clean continuous segments {cand_indices} (0 gaps vs {rep_all['gap_bridges']} in full set).")
                                indices = cand_indices
                                break
            except Exception:
                pass

    indices = [i for i in indices if i not in (a.exclude or [])]
    segs = []
    for i in indices:
        s = all_segs[i]
        if i in (a.reverse_seg or []):
            s = list(reversed(s))
            print(f"  seg {i:04d}: REVERSED")
        segs.append(s)

    if not segs:
        sys.exit("No segments left after filtering.")
    print(f"\nMerging {len(segs)} segment(s): indices {indices}")

    # --- merge
    mcfg = MergeConfig(
        stitch_radius=a.stitch_radius,
        gap_bridge_max=a.gap_bridge_max,
        resample_step=a.resample_step,
        smoothing_window=a.smoothing_window,
        contact_trim_radius=a.contact_trim,
    )
    t0 = time.time()
    merged, report = merge_segments(segs, mcfg, log=print)
    elapsed = time.time() - t0

    # --- clearance offset
    if a.clearance > 0:
        merged = apply_clearance_offset(merged, a.clearance)
        report["clearance_m"] = a.clearance
        print(f"  Applied {a.clearance*1000:.1f} mm outward clearance offset")

    # --- generate both LR and RL paths
    merged_lr = merged
    merged_rl = reverse_trajectory(merged)

    lr_path = sdir / "merged_trajectory_LR.csv"
    rl_path = sdir / "merged_trajectory_RL.csv"
    default_path = Path(a.out) if a.out else sdir / "merged_trajectory.csv"

    save_trajectory(merged_lr, lr_path)
    save_trajectory(merged_rl, rl_path)

    # Active trajectory written to default_path
    if active_dir == "RL":
        save_trajectory(merged_rl, default_path)
        print(f"  Active trajectory set to: RL (End 2 [Right] -> End 1 [Left])")
    else:
        save_trajectory(merged_lr, default_path)
        print(f"  Active trajectory set to: LR (End 1 [Left] -> End 2 [Right])")

    report["active_direction"] = active_dir
    report["files"] = {
        "LR": str(lr_path),
        "RL": str(rl_path),
        "active": str(default_path)
    }
    report_path = sdir / "merge_report.json"
    report_path.write_text(json.dumps(report, indent=2))

    p_lr_start = np.array([merged_lr[0].x, merged_lr[0].y, merged_lr[0].z]) * 1000
    p_lr_end   = np.array([merged_lr[-1].x, merged_lr[-1].y, merged_lr[-1].z]) * 1000

    # --- summary
    print(f"\n{'='*60}")
    print(f"MERGE COMPLETE in {elapsed:.1f}s")
    print(f"  Active    : {default_path} [{active_dir}]")
    print(f"  LR file   : {lr_path}")
    print(f"  RL file   : {rl_path}")
    print(f"  End 1 (Left):  ({p_lr_start[0]:6.1f}, {p_lr_start[1]:6.1f}, {p_lr_start[2]:6.1f}) mm")
    print(f"  End 2 (Right): ({p_lr_end[0]:6.1f}, {p_lr_end[1]:6.1f}, {p_lr_end[2]:6.1f}) mm")
    print(f"  Waypoints : {report['n_resampled']}")
    print(f"  Arc length: {report['total_arc_m']*1000:.0f} mm")
    print(f"  Gap bridges inserted: {report['gap_bridges']}")
    gap_wps = sum(1 for w in merged if w.gap)
    if gap_wps:
        print(f"  Bridged waypoints (gap=True): {gap_wps} – review these in RViz")
    if report["warnings"]:
        print("\n  WARNINGS:")
        for w in report["warnings"]:
            print("   ", w)
    else:
        print("  No warnings.")
    print(f"\nReport: {report_path}")
    print(f"Next:   python3 scripts/replay_trajectory.py --session {sdir}")


if __name__ == "__main__":
    main()
