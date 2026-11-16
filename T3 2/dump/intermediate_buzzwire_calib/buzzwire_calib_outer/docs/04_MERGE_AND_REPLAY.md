# 04 · Merge and replay

## Inspect (always before merging)
`./buzzwire_teach.sh inspect [--plot]` – per-segment points/length/contacts/speed, **gap analysis in spatial order** (the order merge will use), approximate wire coverage (`wire.expected_length_m`), 3D plot.

## Merge (`./buzzwire_teach.sh merge`)
Reads every `segments/seg_*.csv` (originals are never modified) and writes `merged_trajectory.csv` + `merge_report.json`.
1. Drop segments with < 5 points.
2. **Trim contacts**: remove samples within 8 mm of any `contact` mark.
3. **Spatial ordering** (greedy: next segment = the one whose start is nearest the current end) – recording order does not matter.
4. **Stitch / bridge** at each joint: gap < 15 mm → overlap trimmed and joined; 15–120 mm → straight bridge (position lerp + SLERP orientation, flagged `gap=True`); > 120 mm → bridged but a **WARNING** says a part of the wire was probably missed.
5. Savitzky–Golay smoothing of positions (window 7) – removes hand tremor, keeps the end points.
6. Uniform **2 mm arc-length resampling** (cubic spline positions, SLERP orientations).
Options: `--exclude N`, `--only N…`, `--reverse-seg N` (a segment recorded backwards), `--stitch-radius`, `--gap-bridge-max`, `--resample-step`, `--smoothing-window`, `--contact-trim`, `--clearance` (see warning).
`--clearance` shifts every point sideways by a fixed amount perpendicular to the path tangent and vertical. The wire is **not modelled**, so this is not a true "away from the wire" offset – leave it at 0 unless you tested the direction. A path taught in the middle of the loops already has clearance.
Bridged points (`gap=True`) were never driven by you – review them at low replay speed.

## Replay (`./buzzwire_teach.sh replay`)
Robot in heal_server + velocity mode. **Pre-start screen:** `SPACE` gripper (insert stick, close), `ENTER` go, `q` abort.
1. **Approach**: straight-line, slow (`approach_speed_mps` 0.03) from where the arm is to the path start; refused if > `max_approach_m` (0.30 m) unless `--force`. *You* make sure that straight line is free of the wire/posts.
2. **Follow**: a carrot moves along the path with a **speed profile** = min of `v_max·--speed`, `√(a_lat/curvature)`, `w_max/(tool rotation per metre)`, then a forward/backward pass for `a_max`. The carrot waits when the arm lags (> 4 mm slows, 10 mm stops) and slows instantly if joints saturate. Pose tracking = feed-forward + P → damped least squares.
3. Keys: `p` pause · `[`/`]` speed −/+20 % · `q` stop. Progress saved every 0.5 s.
4. **Abort conditions**: error > 30 mm for 1.5 s (stuck/collision) → zero velocity; stale joint states → hold (carrot frozen) and continue when they return.
5. **`--resume`** continues from the saved progress (backs up 10 mm). `--start-idx N` starts at waypoint N.
6. `--dry-run` prints length, estimated duration, and the 5 slowest spots – no robot needed.

## Getting faster (bonus points) safely
1. `--speed 0.2` → watch the whole path. 2. `0.4`, `0.6`… Each step, look at "worst tracking error" printed at the end: if it creeps over ~4 mm you are at the arm's limit – stop raising.
3. Speed is capped locally by bends and tool rotation: raise `replay.a_lat_mps2` / `w_max_radps` / `max_joint_vel` only with the real joint limits known.
4. A smoother, slower teaching run gives a faster replay than a jittery fast one.
