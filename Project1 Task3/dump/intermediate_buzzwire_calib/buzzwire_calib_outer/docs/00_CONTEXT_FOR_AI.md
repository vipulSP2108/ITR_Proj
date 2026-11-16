# 00 · Context for a new AI chat / teammate (read this first)

## Mission
IITGN robotics lab project: an Addverb **HEAL 6-DOF cobot** holds a stick with a ring ("wand") and must follow a
**buzz-wire** (a 3D copper wire with 4 valleys/3 peaks, ~4 mm diameter, loop inner gap ≈ 23–36 mm) without touching it
(touch = buzzer). Stages: (1) teach a path, (2) replay it accurately/smoothly, faster = better.
**Cameras are NOT allowed** for this task. Everything is done with robot joint states + forward kinematics.
The student does the teaching either by hand (free_drive) or by keyboard (teleop).

## Scope decisions (and why)
* No cameras → the earlier camera/hand-eye calibration package is kept only as legacy (`docs/08`).
* Teach-and-replay in the **robot base frame**: no wire registration is needed, because the replayed path was recorded in the same frame.
  (Registering a wire model by touching fiducials is an alternative, not implemented.)
* Both teaching methods emit the **same segment CSV** so they can be mixed and merged.
* Segments instead of one long recording: a touch / freeze / crash only costs the current segment; merge fixes the rest.

## Status (be honest in future answers)
| piece | file(s) | verified how |
|---|---|---|
| 6-DOF Cartesian controller (hold orientation, accel limit, workspace box, leash, singularity, backtrack, auto-tangent) | `buzzwire_calib/teleop_core.py` | simulation `scripts/selftest_teleop.py` (9 tests) |
| Keyboard teleop + recording + undo-by-backtrack | `scripts/teleop.py` | simulated arm via a pseudo-terminal with scripted keys |
| Free-drive recorder + gripper toggle | `scripts/record_segment.py` | pty + stubbed robot |
| Merge (order, trim contacts, stitch/bridge, smooth, resample) | `buzzwire_calib/trajectory.py`, `scripts/merge_segments.py` | synthetic segments |
| Replay (speed profile, carrot, resume, approach) | `buzzwire_calib/replay_core.py`, `scripts/replay_trajectory.py` | sim pipeline `scripts/selftest_pipeline.py` (worst error 0.8–1.8 mm) |
| Gripper helper | `buzzwire_calib/gripper.py` | **not run against the real service** |
| KDL FK/Jacobian glue | `buzzwire_calib/ros_io.py` | **compiled only; never run on the robot** |
**Nothing has touched the real robot.** Treat first hardware runs as commissioning.

## Architecture
```
keys ──> KeyInput ──> teleop.py ──> PoseController.step() ──> qdot ──> /velocity_controller/commands
                         │  (T_des integrates the keys; DLS; leash; box; backtrack)
                         └─> SegmentWriter ─> data/session_X/segments/seg_NNNN.csv ─┐
free_drive: record_segment.py ─> SegmentWriter ───────────────────────────────────┤
                                                                                    v
                                   merge_segments.py (trajectory.merge_segments) ─> merged_trajectory.csv
                                                                                    v
        replay_trajectory.py: PathFollower (speed profile + carrot) ─> PoseController.track() ─> qdot
```
* `RosHub` (one node + executor thread) → `RosRobotState` (joint states, KDL FK, Jacobian) – `ros_io.py`
* `backends.py`: `RosBackend` (real) / `SimBackend` (toy UR5-like arm, `simarm.py`) – same interface, so scripts work with `--sim`.

## Data formats
* **Segment/merged CSV** columns: `t,x,y,z,qx,qy,qz,qw,j1..j6,contact,gap,grip` (metres, base frame, quaternion **xyzw**; `grip` 1 open / 0 closed / −1 unknown; 16-column legacy files still load).
* `seg_NNNN.meta.json`: start/end pose, counts, finished flag. Rows are flushed per sample (crash-safe).
* Session folder: `data/session_YYYYmmdd_HHMMSS/{segments/, merged_trajectory.csv, merge_report.json, replay_state.json}`.
* Config: `config/calib_config.yaml` merged over `buzzwire_calib/config.py:DEFAULTS` (sections: robot, gripper, teleop, replay, wire, capture/cameras = legacy).

## Conventions
`T_a_b` = pose of frame b in frame a (4×4). Jacobian = base frame, tip point, rows [v; ω]. Quaternions xyzw (scipy).
Jog axes: +X "right/along wire", +Y "forward/depth", +Z up, rotated by `teleop.jog_frame_yaw_deg`. Pitch = rotation about +Y.
Key names: lower-case char or `space|tab|enter|esc`.

## Key design choices
1. **Carrot on a leash** (`teleop_core.py`): keys move a desired pose; controller = feed-forward + P on pose error → DLS. Orientation holds itself; if the arm can't follow, the carrot stops (`lag_s` → stop).
2. **Backtrack = undo**: breadcrumb history every 1 mm; `z` retraces and truncates the recorded rows that lie beyond (`SegmentWriter.truncate_to`).
3. **Replay speed profile** from curvature, tool rotation rate, accel, plus *joint-speed feedback*: carrot slows when joints saturate (cut tracking error 5–7 mm → ~1 mm in sim).
4. Terminal keys by default (safe: only the focused terminal reacts). Terminal cannot see key-up → a key counts as held 0.12 s after its last auto-repeat char; `--keys pynput` gives true key-up but is global.
5. Gripper calls run in a thread (service call must never stall control). Failure only prints a warning.

## Assumptions that MUST be checked on hardware
1. `joint_names`/`base_link`/`tip_link` in the config match the URDF; `ros2 topic echo /joint_states` shows 6 joints.
2. Gripper: service `/gripper_controller/command` (`addverb_cobot_msgs/srv/Gripper`, fields `position`, `grasp_force`), open = 1.0, close = 0.0; and it works while `free_drive` runs.
3. Velocity topic `/velocity_controller/commands` accepts rad/s for 6 joints; real joint speed limits ≥ `max_joint_vel`.
4. `jog_frame_yaw_deg` (see `docs/07`) so that `a/d` really move along the wire.
5. `tool_axis` (tool axis that should follow the wire tangent) before using auto-tangent (`g`).
6. Wire facts in `docs/07` come from photos/an LLM-generated keypoint script – measure before trusting.

## Known limitations / ideas
* No joint limits known → `teleop.joint_limits_rad` is empty (set it!). No self-collision / wire-collision model (the wire is not modelled).
* Replay uses a straight-line approach to the start pose (checked only by distance).
* Merge assumes the wire is traversed once, start → end; loops that revisit the same place may confuse greedy ordering – use `--only/--exclude/--reverse-seg`.
* Possible upgrades: wire model + clearance offset from the centreline, contact input from the buzzer (GPIO/ADC) to auto-mark/auto-backtrack, trajectory time-optimisation with real joint limits.

## Tests
`./buzzwire_teach.sh test` → `selftest_solver` (legacy camera math), `selftest_teleop` (controller), `selftest_pipeline` (merge+replay in sim).
For AI editing: keep modules ROS-free where possible, add a simulation test for any controller change, never claim hardware verification.
