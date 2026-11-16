# Changelog / decisions log

## v2.1 – documentation overhaul & edge-case hardening (like T3)
* **Master Documentation Suite**: Created 5 comprehensive master engineering & operator guides (`01_QUICKSTART_OPERATOR_GUIDE.md` through `05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md`) and package/root `README.md` with complete mathematical formulations, ASCII diagrams, terminal cheatsheets, and recovery playbooks.
* **Inspect Bug Fix**: Fixed 10-vs-11 element unpacking `ValueError` in `scripts/inspect_segments.py` when an empty segment is inspected.
* **Gripper State Preservation**: Enhanced `buzzwire_calib/trajectory.py` (`merge_segments` and `apply_clearance_offset`) to preserve gripper state (`grip`) across resampling and offset transformations rather than resetting to -1.0.
* **Non-Blocking Auto-Resume**: Updated `scripts/teleop.py` and `scripts/record_segment.py` so that `--yes` automatically accepts session resume without blocking on interactive `input()`.
* **Simulation Support for Recovery Tool**: Added `--sim` flag to `scripts/goto_pose.py` with closed-loop quintic trajectory execution and throttled HUD updates.
* **Config Transparency**: Documented full `teleop`, `gripper`, `wire`, and `replay` configuration sections in `config/calib_config.yaml`.
* **Test Suite Verification**: Verified that `selftest_teleop.py`, `selftest_pipeline.py`, `selftest_solver.py`, `selftest.py`, and `goto_pose.py --home --sim` all pass with sub-2mm accuracy.

## v2 – teaching without cameras
* Added keyboard teleop (`teleop.py`) with 6-DOF jog, orientation hold, leash, accel limit, singularity guard, workspace box, backtrack-undo, auto-tangent, gripper SPACE, recording.
* `record_segment.py` rewritten: SPACE = gripper, `p` pause, `x` contact; stale-state tolerant.
* Replay rewritten: speed profile + carrot + joint-speed feedback + approach + resume (was stop-and-go per waypoint).
* Waypoint CSV gained `grip` column (old files still load).
* `inspect`: gap analysis in spatial order; coverage vs `wire.expected_length_m` (was a wrong 288 mm constant – the wire is ~1 m long in 3D).
* Added `--sim` (simulated arm), `simarm.py`, `backends.py`, `selftest_teleop.py`, `selftest_pipeline.py`.

### Bugs found in my own v1 code and fixed
1. `record_segment.py`: `os` used before a later local `import os` in `main()` → UnboundLocalError at start. (rewritten)
2. `buzzwire_teach.sh merge`: unset variable under `set -u` → script exited. (rewritten)
3. Replay approach crashed when the start pose was at the arm's position but with different orientation (zero-length path) → rotation-aware path metric.
4. Replay tracking error 5–7 mm in sim because joint-speed saturation was not fed back to the carrot → now 0.8–1.8 mm.
5. `inspect` coverage used 288 mm (wire X span) instead of the 3D path length.
6. Contact key `d` clashed with the new move-right key → contact is `x` (free-drive recorder also still accepts `d`).

## v1 – camera calibration toolkit (legacy, see docs/08)
