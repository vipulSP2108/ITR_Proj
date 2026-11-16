"""
Trajectory recording, segment management, and merging for Buzz Wire teaching.

Design principles
-----------------
* Every segment is an independent, append-only CSV file flushed after every row.
  A crash / Ctrl-C / touch-event at any moment loses at most ONE sample (~50 ms).
* A segment carries a "start_pose" (the FK pose at the moment recording began)
  which is used by the merge algorithm to spatially chain segments together even
  if they were recorded in separate terminal sessions.
* Merging is purely algorithmic:
    1. Spatial ordering  – sort segments by proximity of their start_pose to the
       end of the previous segment (greedy nearest-neighbour).
    2. Overlap trimming  – if segment i+1 starts NEAR the end of segment i
       (within `stitch_radius` metres) the overlapping tail of segment i is
       trimmed so we don't get a doubled-back section.
    3. Gap bridging      – if there is a spatial gap between two segments
       (missed section of wire), a short cubic-spline bridge is inserted and
       flagged with  gap=True  so the operator can see it.
    4. Spline smoothing  – the full merged trajectory is re-sampled through a
       3-D cubic spline at a fixed arc-length step, giving a uniform-density,
       smooth path regardless of how jerky or fast the hand-guided recording was.
    5. Wire-offset       – every point is offset outward from the local wire
       tangent direction by `clearance_m` (default 0 – caller decides).
"""
from __future__ import annotations

import csv
import json
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as Rot, Slerp


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    t: float          # wall-clock time (s) since segment start
    x: float; y: float; z: float          # end-effector position (metres, base frame)
    qx: float; qy: float; qz: float; qw: float  # orientation quaternion
    j1: float; j2: float; j3: float; j4: float; j5: float; j6: float  # joint angles (rad)
    contact: bool = False   # True if a touch / buzz was detected on this sample
    gap: bool = False       # True if this point was synthesised to bridge a segment gap
    grip: float = -1.0      # gripper state when sampled: 1 open, 0 closed, -1 unknown

    _FIELDS = ["t","x","y","z","qx","qy","qz","qw","j1","j2","j3","j4","j5","j6","contact","gap","grip"]

    def to_row(self) -> list:
        return [self.t, self.x, self.y, self.z,
                self.qx, self.qy, self.qz, self.qw,
                self.j1, self.j2, self.j3, self.j4, self.j5, self.j6,
                int(self.contact), int(self.gap), self.grip]

    @classmethod
    def from_row(cls, row: list) -> "Waypoint":
        f = [float(v) for v in row]
        grip = float(row[16]) if len(row) > 16 and row[16] != "" else -1.0
        return cls(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7],
                   f[8], f[9], f[10], f[11], f[12], f[13],
                   bool(int(float(row[14]))), bool(int(float(row[15]))), grip)


# ---------------------------------------------------------------------------
# Segment writer  (one per recording run)
# ---------------------------------------------------------------------------

class SegmentWriter:
    """
    Streams waypoints to  <seg_dir>/seg_NNNN.csv  in real time.
    Safe to kill at any moment.
    """

    HEADER = Waypoint._FIELDS

    def __init__(self, seg_dir: Path, segment_id: int, start_pose: np.ndarray):
        self.seg_dir = Path(seg_dir)
        self.seg_dir.mkdir(parents=True, exist_ok=True)
        self.segment_id = segment_id
        self.path = self.seg_dir / f"seg_{segment_id:04d}.csv"
        self.meta_path = self.seg_dir / f"seg_{segment_id:04d}.meta.json"
        self._t0 = time.time()
        self._n = 0
        self._contact_count = 0
        self._offsets = []      # byte offset of each row (lets teleop rewind = undo)
        self._contacts = []
        self._last_pose = None

        # write meta first (crash before first row still leaves meta)
        self._meta = {
            "segment_id": segment_id,
            "wall_time_start": self._t0,
            "start_pose": start_pose.tolist() if start_pose is not None else None,
            "end_pose": None,
            "n_samples": 0,
            "contact_count": 0,
            "finished": False,
        }
        self._flush_meta()

        self._fh = open(self.path, "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(self.HEADER)
        self._fh.flush()

    def write(self, wp: Waypoint):
        self._offsets.append(self._fh.tell())
        self._csv.writerow(wp.to_row())
        self._fh.flush()           # every row flushed – crash-safe
        self._n += 1
        self._contacts.append(bool(wp.contact))
        if wp.contact:
            self._contact_count += 1
        self._last_pose = np.array([wp.x, wp.y, wp.z, wp.qx, wp.qy, wp.qz, wp.qw])

    def truncate_to(self, n_rows: int):
        """Drop every row after the first n_rows (used when the operator backtracks while recording)."""
        if n_rows >= self._n:
            return
        self._fh.flush()
        off = self._offsets[n_rows]
        self._fh.truncate(off)
        self._fh.seek(off)
        del self._offsets[n_rows:], self._contacts[n_rows:]
        self._n = n_rows
        self._contact_count = sum(self._contacts)

    def close(self):
        self._fh.close()
        self._meta.update(n_samples=self._n, contact_count=self._contact_count,
                          finished=True,
                          end_pose=self._last_pose.tolist() if (self._n > 0 and self._last_pose is not None) else None)
        self._flush_meta()

    def _flush_meta(self):
        self.meta_path.write_text(json.dumps(self._meta, indent=2))

    def __enter__(self): return self
    def __exit__(self, *a): self.close()


# ---------------------------------------------------------------------------
# Segment reader
# ---------------------------------------------------------------------------

def load_segment(csv_path: Path) -> List[Waypoint]:
    wps = []
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        next(reader)   # skip header
        for row in reader:
            if len(row) < 16:
                continue
            try:
                wps.append(Waypoint.from_row(row))
            except (ValueError, IndexError):
                continue   # corrupted last line from a crash → skip
    return wps


def load_segment_meta(meta_path: Path) -> dict:
    return json.loads(meta_path.read_text())


def list_segments(seg_dir: Path) -> List[Path]:
    return sorted(Path(seg_dir).glob("seg_????.csv"))


# ---------------------------------------------------------------------------
# Spatial helpers
# ---------------------------------------------------------------------------

def _xyz(wp: Waypoint) -> np.ndarray:
    return np.array([wp.x, wp.y, wp.z])


def _arc_lengths(pts: np.ndarray) -> np.ndarray:
    """Cumulative arc-length along Nx3 point array."""
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def _resample_xyz_quat(pts: np.ndarray, quats: np.ndarray, step: float
                       ) -> tuple[np.ndarray, np.ndarray]:
    """Uniform arc-length resampling of positions + SLERP for orientations."""
    s = _arc_lengths(pts)
    if s[-1] < 1e-6:
        return pts[:1], quats[:1]
    s_new = np.arange(0.0, s[-1], step)
    if len(s_new) == 0 or s_new[-1] < s[-1]:
        s_new = np.append(s_new, s[-1])

    # position cubic spline
    cs = CubicSpline(s, pts, bc_type="not-a-knot")
    pts_new = cs(s_new)

    # orientation SLERP
    rots = Rot.from_quat(quats)          # scipy expects [x,y,z,w]
    slerp = Slerp(s, rots)
    rots_new = slerp(np.clip(s_new, s[0], s[-1]))
    return pts_new, rots_new.as_quat()


# ---------------------------------------------------------------------------
# Merge algorithm
# ---------------------------------------------------------------------------

@dataclass
class MergeConfig:
    stitch_radius: float = 0.015     # m – overlap-trimming radius at segment joints
    gap_bridge_max: float = 0.120    # m – gaps larger than this are flagged as MISSING
    resample_step: float = 0.002     # m – 2 mm arc-length step in merged trajectory
    min_segment_points: int = 5      # discard segments shorter than this
    smoothing_window: int = 7        # Savitzky-Golay window (must be odd, >= 3)
    contact_trim_radius: float = 0.008   # m – trim waypoints within 8 mm of a contact event


def _sg_smooth(arr: np.ndarray, window: int, poly: int = 3) -> np.ndarray:
    """Column-wise Savitzky-Golay smoothing (pure numpy, no scipy dependency for this)."""
    from scipy.signal import savgol_filter
    if len(arr) < window:
        return arr.copy()
    out = arr.copy()
    for col in range(arr.shape[1]):
        out[:, col] = savgol_filter(arr[:, col], window_length=window, polyorder=poly)
    return out


def _trim_contact_region(wps: List[Waypoint], radius: float) -> List[Waypoint]:
    """Remove waypoints within `radius` metres of any contact event."""
    if not any(w.contact for w in wps):
        return wps
    contact_pts = np.array([_xyz(w) for w in wps if w.contact])
    keep = []
    for w in wps:
        if np.min(np.linalg.norm(contact_pts - _xyz(w), axis=1)) > radius:
            keep.append(w)
    return keep


def _greedy_order(segments: List[List[Waypoint]]) -> List[int]:
    """
    Sort segment indices so that segment[i+1].start is closest to segment[i].end.
    Handles arbitrary recording order (you can record segment 3 before segment 2).
    """
    if len(segments) == 1:
        return [0]

    ends = [_xyz(s[-1]) for s in segments]
    starts = [_xyz(s[0]) for s in segments]

    order = [0]
    remaining = list(range(1, len(segments)))
    while remaining:
        last_end = ends[order[-1]]
        # pick the segment whose START is nearest to the current last END
        dists = [np.linalg.norm(starts[i] - last_end) for i in remaining]
        best = remaining[int(np.argmin(dists))]
        order.append(best)
        remaining.remove(best)
    return order


def merge_segments(
    segments: List[List[Waypoint]],
    cfg: MergeConfig = None,
    log=print,
) -> tuple[List[Waypoint], dict]:
    """
    Merge a list of segment waypoint lists into one clean, ordered trajectory.

    Returns
    -------
    merged_wps   : list of Waypoint  (may contain gap=True bridge points)
    report       : dict with per-segment stats and any warnings
    """
    if cfg is None:
        cfg = MergeConfig()

    # 1. discard empty / too-short segments
    valid = [(i, s) for i, s in enumerate(segments) if len(s) >= cfg.min_segment_points]
    if not valid:
        raise ValueError("No valid segments to merge (all too short or empty).")
    orig_ids, segs = zip(*valid)
    segs = list(segs)
    log(f"  {len(segs)}/{len(segments)} segments are usable (>= {cfg.min_segment_points} points)")

    # 2. trim contact regions
    trimmed = []
    for s in segs:
        t = _trim_contact_region(s, cfg.contact_trim_radius)
        trimmed.append(t if len(t) >= cfg.min_segment_points else s)
        if len(t) < len(s):
            log(f"  trimmed {len(s)-len(t)} contact-region points from a segment")
    segs = trimmed

    # 3. greedy spatial ordering
    order = _greedy_order(segs)
    log(f"  spatial order: {[orig_ids[i] for i in order]}")

    # 4. stitch
    merged: List[Waypoint] = []
    warnings = []
    gap_count = 0

    for step, idx in enumerate(order):
        seg = segs[idx]
        if not merged:
            merged.extend(seg)
            continue

        last = _xyz(merged[-1])
        first = _xyz(seg[0])
        gap = float(np.linalg.norm(first - last))

        if gap < cfg.stitch_radius:
            # overlap: trim the tail of the already-merged part that is within
            # stitch_radius of the new segment's start, then join directly
            while len(merged) > 1 and np.linalg.norm(_xyz(merged[-1]) - first) < cfg.stitch_radius:
                merged.pop()
            merged.extend(seg)
            log(f"  seg {orig_ids[idx]:04d}: direct stitch (gap {gap*1000:.1f} mm < {cfg.stitch_radius*1000:.0f} mm)")

        elif gap <= cfg.gap_bridge_max:
            # small gap: cubic bridge (3 interpolated points)
            n_bridge = max(3, int(gap / cfg.resample_step))
            ts = np.linspace(0, 1, n_bridge + 2)[1:-1]
            last_wp, next_wp = merged[-1], seg[0]
            last_q = np.array([last_wp.qx, last_wp.qy, last_wp.qz, last_wp.qw])
            next_q = np.array([next_wp.qx, next_wp.qy, next_wp.qz, next_wp.qw])
            rots = Rot.from_quat(np.vstack([last_q, next_q]))
            slerp = Slerp([0.0, 1.0], rots)
            for tau in ts:
                p = last + tau * (first - last)
                q = slerp(tau).as_quat()
                j_interp = [(a + tau * (b - a)) for a, b in
                            zip([last_wp.j1,last_wp.j2,last_wp.j3,last_wp.j4,last_wp.j5,last_wp.j6],
                                [next_wp.j1,next_wp.j2,next_wp.j3,next_wp.j4,next_wp.j5,next_wp.j6])]
                merged.append(Waypoint(last_wp.t, p[0], p[1], p[2],
                                       q[0], q[1], q[2], q[3],
                                       *j_interp, gap=True))
            merged.extend(seg)
            log(f"  seg {orig_ids[idx]:04d}: bridged gap {gap*1000:.1f} mm with {len(ts)} points")
            gap_count += 1

        else:
            # large gap: insert bridge but flag as MISSING in report
            msg = (f"  WARNING seg {orig_ids[idx]:04d}: LARGE GAP {gap*1000:.1f} mm "
                   f"(> {cfg.gap_bridge_max*1000:.0f} mm). "
                   "A section of the wire was likely missed. Consider re-recording that section.")
            log(msg)
            warnings.append(msg)
            # still bridge it so the trajectory is continuous
            n_bridge = max(5, int(gap / cfg.resample_step))
            ts = np.linspace(0, 1, n_bridge + 2)[1:-1]
            last_wp, next_wp = merged[-1], seg[0]
            last_q = np.array([last_wp.qx, last_wp.qy, last_wp.qz, last_wp.qw])
            next_q = np.array([next_wp.qx, next_wp.qy, next_wp.qz, next_wp.qw])
            rots = Rot.from_quat(np.vstack([last_q, next_q]))
            slerp = Slerp([0.0, 1.0], rots)
            for tau in ts:
                p = last + tau * (first - last)
                q = slerp(tau).as_quat()
                j_interp = [(a + tau * (b - a)) for a, b in
                            zip([last_wp.j1,last_wp.j2,last_wp.j3,last_wp.j4,last_wp.j5,last_wp.j6],
                                [next_wp.j1,next_wp.j2,next_wp.j3,next_wp.j4,next_wp.j5,next_wp.j6])]
                merged.append(Waypoint(last_wp.t, p[0], p[1], p[2],
                                       q[0], q[1], q[2], q[3],
                                       *j_interp, gap=True))
            merged.extend(seg)
            gap_count += 1

    # 5. Savitzky-Golay smoothing on positions (preserves start/end, removes jitter)
    pts = np.array([[w.x, w.y, w.z] for w in merged])
    pts_smooth = _sg_smooth(pts, cfg.smoothing_window)
    # keep first and last point exactly as recorded (don't drift off wire ends)
    pts_smooth[0] = pts[0]; pts_smooth[-1] = pts[-1]

    quats = np.array([[w.qx, w.qy, w.qz, w.qw] for w in merged])

    # 6. Uniform arc-length resampling
    pts_rs, quats_rs = _resample_xyz_quat(pts_smooth, quats, cfg.resample_step)
    log(f"  {len(merged)} raw merged points -> {len(pts_rs)} resampled at {cfg.resample_step*1000:.0f} mm step")

    # re-build waypoints from resampled arrays
    # joint angles: re-interpolate by arc-length
    joints_raw = np.array([[w.j1, w.j2, w.j3, w.j4, w.j5, w.j6] for w in merged])
    s_orig = _arc_lengths(pts)
    s_new  = _arc_lengths(pts_rs)
    joints_rs = np.zeros((len(pts_rs), 6))
    for col in range(6):
        cs_j = CubicSpline(s_orig, joints_raw[:, col], bc_type="not-a-knot")
        joints_rs[:, col] = cs_j(np.clip(s_new, s_orig[0], s_orig[-1]))

    # gripper state: inherit from nearest original sample
    grips_raw = np.array([w.grip for w in merged])
    grips_rs = np.full(len(pts_rs), -1.0)
    for idx_new, s_n in enumerate(s_new):
        nearest_orig = int(np.argmin(np.abs(s_orig - s_n)))
        grips_rs[idx_new] = grips_raw[nearest_orig]

    gap_flags = np.zeros(len(pts_rs), bool)
    s_gap = s_orig[[i for i, w in enumerate(merged) if w.gap]]
    for s_g in s_gap:
        nearest = np.argmin(np.abs(s_new - s_g))
        gap_flags[max(0, nearest-1):nearest+2] = True

    final: List[Waypoint] = []
    for i, (p, q, j, gf, gr) in enumerate(zip(pts_rs, quats_rs, joints_rs, gap_flags, grips_rs)):
        final.append(Waypoint(float(i) * cfg.resample_step, p[0], p[1], p[2],
                              q[0], q[1], q[2], q[3],
                              j[0], j[1], j[2], j[3], j[4], j[5], gap=bool(gf), grip=float(gr)))

    total_arc = float(s_new[-1]) if len(s_new) else 0.0
    report = {
        "n_input_segments": len(segments),
        "n_valid_segments": len(segs),
        "segment_order": [int(orig_ids[i]) for i in order],
        "n_raw_merged": len(merged),
        "n_resampled": len(final),
        "total_arc_m": round(total_arc, 4),
        "gap_bridges": gap_count,
        "warnings": warnings,
    }
    return final, report


# ---------------------------------------------------------------------------
# Save / load merged trajectory
# ---------------------------------------------------------------------------

def save_trajectory(wps: List[Waypoint], path: Path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(Waypoint._FIELDS)
        for wp in wps:
            w.writerow(wp.to_row())


def load_trajectory(path: Path) -> List[Waypoint]:
    return load_segment(path)   # same CSV format


def apply_clearance_offset(wps: List[Waypoint], clearance_m: float) -> List[Waypoint]:
    """
    Offset each waypoint radially outward from the wire.

    "Outward" here is defined as the direction perpendicular to the local
    tangent that is ALSO perpendicular to the tool z-axis (pointing up / away
    from the wire centre).  For a nearly-horizontal wire this is a vertical
    lift; for a looping wire it rotates with the loop.
    """
    if clearance_m == 0.0 or len(wps) < 3:
        return wps
    pts = np.array([[w.x, w.y, w.z] for w in wps], float)
    # local tangent via central differences
    tangents = np.gradient(pts, axis=0)
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    norms = np.where(norms < 1e-9, 1.0, norms)
    tangents /= norms

    # normal pointing "up" (or radially out for loops): cross(tangent, [0,0,1])
    up = np.array([0.0, 0.0, 1.0])
    normals = np.cross(tangents, up)
    n_norms = np.linalg.norm(normals, axis=1, keepdims=True)
    # fallback to [0,1,0] when tangent is parallel to up
    fallback = np.cross(tangents, np.array([0.0, 1.0, 0.0]))
    normals = np.where(n_norms < 1e-6, fallback, normals)
    n_norms = np.linalg.norm(normals, axis=1, keepdims=True)
    n_norms = np.where(n_norms < 1e-9, 1.0, n_norms)
    normals /= n_norms

    offset_pts = pts + clearance_m * normals
    out = []
    for wp, p in zip(wps, offset_pts):
        out.append(Waypoint(wp.t, p[0], p[1], p[2],
                            wp.qx, wp.qy, wp.qz, wp.qw,
                            wp.j1, wp.j2, wp.j3, wp.j4, wp.j5, wp.j6,
                            wp.contact, wp.gap, wp.grip))
    return out
