#!/usr/bin/env python3
"""
Inspect recorded segments: show stats, detect gaps, visualise in ASCII.

Usage
-----
  python3 scripts/inspect_segments.py                           # latest session
  python3 scripts/inspect_segments.py --session data/run_001
  python3 scripts/inspect_segments.py --session data/run_001 --plot  # matplotlib 3D plot
"""
import argparse, json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from buzzwire_calib.config import load_config
from buzzwire_calib.session import find_latest_session
from buzzwire_calib.trajectory import list_segments, load_segment, _arc_lengths


def ascii_progress(value, total, width=40, char="█"):
    filled = int(round(value / total * width)) if total else 0
    bar = char * filled + "░" * (width - filled)
    return f"[{bar}] {value/total*100:.0f}%" if total else "[" + "░" * width + "]"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config",  default=str(Path(__file__).resolve().parent.parent /
                                              "config" / "calib_config.yaml"))
    ap.add_argument("--session", default=None)
    ap.add_argument("--plot",    action="store_true", help="show 3D matplotlib plot")
    ap.add_argument("--gap-warn", type=float, default=0.030,
                    help="warn if gap between adjacent segments > this (m)")
    a = ap.parse_args()

    cfg = load_config(a.config)
    sdir = Path(a.session) if a.session else find_latest_session(Path(cfg["paths"]["data_root"]))
    if sdir is None:
        sys.exit("No session found.")
    seg_dir = sdir / "segments"
    csv_paths = list_segments(seg_dir)
    if not csv_paths:
        sys.exit("No segments found.")

    print(f"\nSession: {sdir}")
    print(f"{'='*72}")

    segs = [load_segment(p) for p in csv_paths]
    total_arc = 0.0
    contact_total = 0

    rows = []
    for i, (p, wps) in enumerate(zip(csv_paths, segs)):
        if not wps:
            rows.append((i, p.name, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            continue
        pts = np.array([[w.x, w.y, w.z] for w in wps])
        arc = float(_arc_lengths(pts)[-1])
        total_arc += arc
        contacts = sum(1 for w in wps if w.contact)
        contact_total += contacts
        dur = wps[-1].t - wps[0].t if len(wps) > 1 else 0
        speed = arc / dur * 1000 if dur > 0 else 0   # mm/s
        rows.append((i, p.name, len(wps), arc * 1000, contacts,
                     pts[0, 0]*1000, pts[0, 1]*1000, pts[0, 2]*1000,
                     pts[-1, 0]*1000, dur, speed))

    print(f"{'ID':>4}  {'File':<22}  {'Pts':>5}  {'Arc mm':>7}  "
          f"{'Cont':>4}  {'Start XYZ (mm)':>24}  {'Dur s':>5}  {'Speed mm/s':>10}")
    print("-" * 90)
    for r in rows:
        i, name, n, arc, cont, sx, sy, sz, _, dur, spd = r
        print(f"{i:4d}  {name:<22}  {n:5d}  {arc:7.1f}  "
              f"{cont:4d}  ({sx:6.1f},{sy:6.1f},{sz:6.1f})  {dur:5.1f}  {spd:10.1f}")

    print(f"\nTotal arc: {total_arc*1000:.0f} mm across {len(segs)} segments")
    print(f"Total contact events: {contact_total}")

    # --- gap analysis in SPATIAL order (the order merge will use, not recording order)
    from buzzwire_calib.trajectory import _greedy_order
    nonempty = [i for i, w in enumerate(segs) if w]
    order = [nonempty[k] for k in _greedy_order([segs[i] for i in nonempty])] if nonempty else []
    print(f"\nGap analysis in spatial order {order} (between each segment's end and the next one's start):")
    has_large_gap = False
    for i, j in zip(order[:-1], order[1:]):
        end = np.array([segs[i][-1].x, segs[i][-1].y, segs[i][-1].z])
        nxt = np.array([segs[j][0].x, segs[j][0].y, segs[j][0].z])
        gap = float(np.linalg.norm(nxt - end))
        flag = "  <-- LARGE GAP" if gap > a.gap_warn else ""
        has_large_gap |= gap > a.gap_warn
        print(f"  seg {i:04d} -> seg {j:04d} : {gap*1000:6.1f} mm{flag}")

    # --- coverage estimate (approximate: wire.expected_length_m in the config)
    WIRE_MM = float(cfg.get("wire", {}).get("expected_length_m", 1.0)) * 1000.0
    pct = min(total_arc * 1000 / WIRE_MM * 100, 100)
    print(f"\nApprox. wire coverage: {ascii_progress(pct, 100)} ({total_arc*1000:.0f} / {WIRE_MM:.0f} mm; "
          "set wire.expected_length_m in the config once you know the real length)")
    if has_large_gap:
        print("\n  ACTION NEEDED: large gaps detected. Record additional segment(s) to cover "
              "the missing section, then re-run merge.")
    else:
        print("\n  Gaps look OK. Ready to merge:")
        print(f"  ./buzzwire_teach.sh merge --session {sdir}")

    # --- merged file status
    merged = sdir / "merged_trajectory.csv"
    if merged.exists():
        from buzzwire_calib.trajectory import load_trajectory
        mwps = load_trajectory(merged)
        gap_n = sum(1 for w in mwps if w.gap)
        print(f"\nMerged trajectory already exists: {len(mwps)} pts, {gap_n} bridged")
        rp = sdir / "merge_report.json"
        if rp.exists():
            rep = json.loads(rp.read_text())
            print(f"  segment order used: {rep.get('segment_order')}")
            for w in rep.get("warnings", []):
                print(f"  WARNING: {w}")

    # --- optional 3D plot
    if a.plot:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            fig = plt.figure(figsize=(10, 7))
            ax = fig.add_subplot(111, projection="3d")
            cmap = plt.cm.tab10
            for i, wps in enumerate(segs):
                if not wps:
                    continue
                pts = np.array([[w.x, w.y, w.z] for w in wps]) * 1000
                ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                        color=cmap(i % 10), label=f"seg {i:04d}")
                ax.scatter(pts[0, 0], pts[0, 1], pts[0, 2], color=cmap(i % 10), marker="o", s=40)
                ax.scatter(pts[-1, 0], pts[-1, 1], pts[-1, 2], color=cmap(i % 10), marker="x", s=60)
            if merged.exists():
                from buzzwire_calib.trajectory import load_trajectory
                mwps = load_trajectory(merged)
                mpts = np.array([[w.x, w.y, w.z] for w in mwps]) * 1000
                ax.plot(mpts[:, 0], mpts[:, 1], mpts[:, 2], "k--", alpha=0.4,
                        linewidth=1.5, label="merged")
            ax.set_xlabel("X mm"); ax.set_ylabel("Y mm"); ax.set_zlabel("Z mm")
            ax.set_title(f"Trajectory segments – {sdir.name}")
            ax.legend()
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("matplotlib not installed – skipping plot (pip install matplotlib)")


if __name__ == "__main__":
    main()
