# `tracker.py` — Vision-Based Path Verification

**Module**: `sim_demo/tracker.py` (176 lines)
**Purpose**: Real-time vision-based path tracking at ~25 Hz. Detects the tool marker in camera images, estimates TCP position in robot frame, and computes tracking metrics against the planned path.

---

## Tracking Architecture

The `PathTracker` is installed as a frame hook in `SimEnv`. Every 2 control steps:

1. **Detect marker** → sphere centre in camera frame
2. **Transform to robot frame** using calibrated T_robot_cam
3. **Estimate TCP** = marker_robot + (TCP_fk − marker_fk)
4. **Compute metrics** against planned waypoint path

---

## Tracking Metrics

| Metric | Definition | Units |
|--------|-----------|-------|
| **Tracking error** | ‖TCP_estimated − TCP_planned‖ | mm |
| **Cross-track error** | Distance from estimate to start→goal line segment | mm |
| **Direction cosine** | (ΔTCP_est · ΔTCP_planned) / (‖Δest‖ · ‖Δplan‖) | dimensionless |

---

## Thresholds and Constants

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `WARN_MM` | 10.0 mm | Warning threshold |
| `ABORT_MM` | 40.0 mm | Hard abort threshold |
| `LAG_S` | 0.12 s | Lag tolerance for fast moves |
| `ABORT_CONSEC` | 4 frames | Consecutive abort-threshold breaches before abort |
| `obs_every` | 2 control steps | Observation frequency (~25 Hz) |

### Dynamic Tolerance

```
tolerance = WARN_MM + LAG_S × commanded_speed
```

Fast joint moves cause up to 25 mm apparent lag (the vision-estimated position lags behind the actual commanded position). This dynamic tolerance prevents false tracking alarms during high-speed segments.

---

## Status States

| Status | Color | Meaning |
|--------|-------|---------|
| `ON_TRACK` | Green | Within tolerance |
| `DEVIATING` | Orange | Between WARN and ABORT |
| `MARKER_LOST` | Gray | Marker not detected in frame |
| `ABORT` | Red | 4+ consecutive frames above ABORT_MM |

If `ABORT` triggers, raises `AbortMotion` exception → recovery sequence.

---

## Video Recording

The tracker records a dual-view video at 25 fps:
- **Left panel**: Overhead camera with HUD overlay (status, tracking error, peg visibility)
- **Right panel**: Oblique camera view

The `annotate()` method draws real-time status overlays on each frame.

---

## Summary Statistics

After execution, `summary()` returns:

| Statistic | Description |
|-----------|-------------|
| `mean_track_err_mm` | Mean tracking error |
| `p95_track_err_mm` | 95th percentile tracking error |
| `max_track_err_mm` | Maximum tracking error |
| `mean_cross_track_mm` | Mean cross-track error |
| `on_track_pct` | Percentage of frames ON_TRACK |
| `marker_visible_pct` | Percentage of frames with successful marker detection |

### Demo Run Results

| Run | On-Track | Mean Error | P95 | Max | Cross-Track |
|-----|----------|-----------|-----|-----|-------------|
| Circle (Nov 7) | 100% | 4.1 mm | 19.4 mm | 22.9 mm | 0.2 mm mean |
| Square (Nov 8) | 100% | 4.5 mm | 21.0 mm | 28.8 mm | — |
