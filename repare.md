# repare.md — Build handoff for the code editor / AI agent

> **Read this file first, before touching any code.**
> It explains what the new simulation build (`sim_demo/`) contains, how it relates to the older
> hardware-oriented code in `T1/`, how to merge the two safely, every constraint the planner enforces
> (and why), what the simulation assumes that the real robot may not satisfy, and the ordered
> checklist for bringing this up on real hardware.
>
> Companion files: `imp.md` (project knowledge base, section 11 is the short version of this file),
> `sim_demo/README.md` (user-facing quick start), `EDGE_CASES.md` (legacy fault matrix).

Date of this build: **2026-11-07**. Status: **simulation verified, hardware NOT yet tested.**

---

## 0. TL;DR

* A new self-contained package `T1/sim_demo/` runs the whole task in MuJoCo for **one circle peg**:
  **Home → camera/robot calibration → workspace analysis → camera-verified pick & place**, and stores
  every run in `T1/demo_runs/demo_<YYYYMMDD_HHMMSS>/`.
* It is **additive**: nothing in the legacy tree was changed except `imp.md` (section 11 appended).
* Everything it reports (positions, reachability, tilt, tracking error, direction check) is **computed**
  from camera images + arm geometry. Simulator ground truth is only used for scoring and is labelled
  `SIM_ONLY_` in JSON.
* **Biggest merge hazards:** (1) legacy code is in **millimetres**, new code is in **metres**;
  (2) legacy kinematics has a **different joint-zero convention** than `sim_demo/arm_model.py`;
  (3) the legacy camera model assumes an **angled** camera (45–55°), the sim camera looks **straight down**.
  See §3 and §7.
* **Biggest unknown for hardware:** where the real socket board is relative to the robot base. With the
  distance estimated from one overhead image (~32 cm) the arm needs a **60° tool tilt**; if the legacy
  assumption (socket at X≈160 mm) is right, a near-vertical grasp is enough. **Measure it with a ruler.**

---

## 1. Rules for the agent (non-negotiable)

1. **Do not overwrite legacy files.** Merge by adding adapters (§7). Legacy hardware code has been
   physically verified on the robot (IDs 11–15, U2D2, teleop, E-stop); `sim_demo` has not.
2. **Pre-hardware directive (from `imp.md` §3):** every new capability must be simulated and unit-tested
   before it is allowed to command real motors. Keep `--dry-run` style gating on anything that moves hardware.
3. **Record new insights in `imp.md`** (dynamic knowledge persistence directive). Add a dated bullet; do not
   duplicate this file there.
4. **Units/frames:** see §3. Never mix mm and m silently. New code = metres + radians. Legacy = mm + degrees/counts.
   Convert only at module boundaries and name variables with the unit when ambiguous (`x_mm`, `x_m`).
5. **Never hard-code results.** Reports, status banners ("ON_TRACK", reachability text) must be derived from data.
   (The legacy `demo1_20261107/*` scripts print fixed text — do not copy that pattern.)
6. **Entry scripts must `import sim_env` (or anything that imports it) before `import mujoco`** — `sim_env`
   selects the headless GL backend (`MUJOCO_GL=egl` on Linux without `DISPLAY`; macOS untouched).
7. Never use simulator ground truth (`env.gt_*`) in perception, planning, tracking or control. Scoring/reporting only.
8. Any change to a planner constant in §5 needs a physics re-test (`python tests/test_sim_demo.py`) and a note
   in the decision log (§13).

---

## 2. Repository state: two lineages

```
T1/
├─ src/ scripts/ sim/ config/ tests/ demo1_20261107/ ...   LEGACY (hardware-oriented, mm, older sim)
├─ sim_demo/                                                NEW  (self-contained, metres, MuJoCo pipeline)
├─ demo_runs/demo_<timestamp>/                              NEW  (one folder per run; sample included)
├─ imp.md        (section 11 appended)                      knowledge base
└─ repare.md     (this file)
```

### 2.1 Legacy (unchanged) — what it is good for
| Area | Files | Notes |
|---|---|---|
| Real motor I/O | `src/control/dynamixel_wrapper.py` (`OpenManipulatorController`) | Protocol 2.0, 1 Mbps, IDs 11–15, counts 0–4095, ports auto-detected on macOS. **Physically verified.** |
| Teleop + E-stop | `src/control/teleop_keyboard.py` | Keys in `KEY_MAPPINGS.md`. **Physically verified.** |
| Gripper safety | `src/control/gripper_guard.py` (`GripperGuard`) | Count/current thresholds: open≈1700, grasp 1920–2240, empty>2310, hold≥25 mA. |
| Kinematics | `src/planning/kinematics.py` (mm, own zero convention — see §3) | Closed-form; limits ±90/±105/±110°. |
| Calibration | `src/planning/calibration.py` (`EyeToHandCalibrator`), `config/camera_calib.yaml` | mm; the yaml is a **default guess**, `calibrated_at: default_physical_setup`. |
| Vision | `src/vision/*` (HSV, shapes, depth slicing, drift monitor, drop detector) | Tier-1 verified on synthetic scenes. `config/colors.yaml` = real HSV thresholds. |
| Recovery | `src/planning/recovery_manager.py` | Drop/jam recovery logic against the real controller. |
| Old sims | `sim/*.xml`, `demo1_20261107/*` | Older layout (smaller mat, 3.2 cm peg, socket X≈245 mm), fixed-text reports. Superseded for pipeline purposes by `sim_demo`. |

### 2.2 New (`sim_demo/`) — module map
| File | Responsibility |
|---|---|
| `run_pipeline.py` | CLI + the three stages, run-folder creation, `summary.md`, `run_log.json`, recovery loop |
| `arm_model.py` | **Analytic FK/IK from published OMX-X dimensions** (metres/radians), joint limits, `HOME_Q`, `SIDE_PARK_Q`. No simulator access. |
| `scene_builder.py` | Generates the MJCF from `config/sim_layout.yaml` (pixel layout → metres), robot, gripper, cameras, marker, pegs |
| `sim_env.py` | MuJoCo env: position-servo control, min-jerk joint moves, Cartesian line moves (IK per step), RGB-D capture with RealSense-like noise, `place_peg`, `gt_*` accessors (scoring only) |
| `calibration.py` | Eye-to-hand calibration by arm motion: marker (camera) vs FK(marker) (encoders+dimensions) → Kabsch; outlier pass; reprojection error |
| `perception.py` | HSV masks, shape classifier, RANSAC plane, ray–plane lifting, sphere-fit marker, socket/peg/surface detectors |
| `workspace.py` | Min-tool-tilt reachability grid + camera overlay |
| `planner.py` | Constant-tilt pick-and-place planner with explainable feasibility table |
| `tracker.py` | 25 Hz vision path verification (tracking/cross-track/direction/progress, lag-aware tolerance, abort) + video |
| `viz.py` | Overlays, top-down map, tracking plot, storyboard |
| `config/sim_layout.yaml`, `config/vision.yaml` | Layout in image pixels (scale **estimated**), HSV thresholds tuned for the *sim render* |
| `tests/test_sim_demo.py` | 6 checks (FK vs MuJoCo, IK round-trip, calibration, shape classification, planner, multi-position physics) |

---

## 3. Frames, units, joint conventions (CRITICAL for merging)

### 3.1 Robot/world frame (new, and compatible with legacy docstring)
Origin = centre of robot base **on the table surface**; **+X forward (toward the socket board)**, **+Y to the
arm's left**, **+Z up** (right-handed). In the mounted overhead camera view: image-left = +X, image-down = +Y.
So sockets on the right of the arm have **negative Y** (circle socket: Y ≈ −115 mm).

### 3.2 Units
| | Legacy | New |
|---|---|---|
| Length | **mm** | **m** |
| Angles | degrees / Dynamixel counts (4096/rev, 2048 centre) | **radians** |
| Camera extrinsic | `camera_calib.yaml` in mm, 3×3 + 3×1 | 4×4 `T_robot_cam` in m (OpenCV camera axes: x right, y down, z forward) |
| Depth | mm (RealSense native) | m |

### 3.3 Joint-zero conventions DIFFER — do not mix
* **`sim_demo/arm_model.py` (URDF-style):** all joints = 0 is the **"L" pose**: upper arm up (leaning 10.6° forward
  because of the 24 mm / 128 mm offset), forearm **horizontal forward**, gripper forward. Home FK = TCP **(0.274, 0, 0.205) m**.
  `q2 > 0` tilts the upper arm forward, `q3 > 0` bends the forearm downward, `q4 > 0` pitches the tool downward.
  Tool tilt convention: `tilt = π/2 − (q2+q3+q4)`; 0 = straight down, positive = tool pointing forward-down.
* **Legacy `src/planning/kinematics.py`:** `th2 = π/2 − q2`, `th3 = −q3`, `th4 = −q4`, link 2 purely vertical (no 24 mm
  offset). With `q = 0` the arm is a **straight vertical line**; the L pose is reached with roughly `q3 ≈ 1.45 rad`
  (`imp.md` §8: `[0.0, 0.0, 1.45, 0.0]`, J3 ≈ 1100 counts).
* Sign of `q2` and `q3` happens to agree between the two; the **zero of q3 differs by ≈ π/2 − 0.185 ≈ 1.386 rad**
  (legacy value 1.45 is a rounded physical measurement; difference ≈ 3.7°).
* **UNVERIFIED on hardware:** which convention the real motors follow (counts → radians offsets and signs).
  **Do not command the real robot from `arm_model.py` until the mapping in §10 step 3 is measured.**

Suggested adapter (to be filled with measured numbers, one tuple per joint):
```python
# counts_to_sim_rad:  q_sim[i] = sign[i] * (counts[i] - zero_counts[i]) * 2*pi/4096
SIGN        = np.array([+1, +1, +1, +1])          # TO BE VERIFIED by moving each joint a few degrees
ZERO_COUNTS = np.array([2048, 2048, ???, 2048])   # counts at which the arm is in the *sim-zero* L pose
```

### 3.4 Real vs sim geometry facts used by `arm_model.py`
Shoulder axis 77 mm above base; elbow = shoulder + (24 mm fwd, 128 mm up) at q2=0; forearm 124 mm;
tool (wrist → fingertip centre = TCP) 126 mm; marker = 30 mm above the wrist axis on a mast (sim only —
**must be physically added on hardware**, see §10). Limits used (URDF values, *as recalled*):
q2 ∈ [−1.79, 1.57], q3 ∈ [−0.94, 1.38], q4 ∈ [−1.79, 2.04] rad; q1 ±π. Legacy limits (±90/±105/±110°) are more
conservative on some joints and wider on others: **use the intersection until measured.**

---

## 4. Pipeline architecture (what runs, in order)

```
run_pipeline.py
 ├─ stage_home        move to HOME_Q, check FK(encoders) vs simulator, save 3 views
 ├─ stage_calibration 16 poses → detect magenta marker (sphere fit on RGB-D) → Kabsch → T_robot_cam
 │                    ── instructor now places the peg (SimEnv.place_peg) — table was CLEAR during calibration
 ├─ stage_workspace   park arm at SIDE_PARK_Q → snapshot → surfaces(plane RANSAC) / holes(shape) / peg(depth slice)
 │                    → reachability grids (grasp height, place height) → plan_pick_place() → feasibility table
 └─ stage_pick_place  PathTracker(25 Hz) + execute_plan(): move_to_home → 8 waypoints
                      → finger-travel grasp check → (recovery loop ≤ 2 attempts) → return_park → fresh snapshot → verdict
```
Data flow: `RGB-D → perception → (T_robot_cam) robot-frame objects → planner → waypoints → SimEnv (IK per step) → tracker
(camera again) → logs/plots/video`.

Key public functions (use these as integration seams): `arm_model.fk/ik`, `calibration.run_calibration`,
`perception.detect_{surfaces,holes,green_peg,marker}`, `workspace.reachability_grid`, `planner.plan_pick_place`,
`tracker.PathTracker`, `run_pipeline.{snapshot_detect,execute_plan,recovery_lift}`.

---

## 5. Constraints and parameters (waypoints and checks) — with reasons

### 5.1 Waypoint plan (`planner.py`, single peg)
All heights are TCP (fingertip-centre) heights in the robot frame, metres. `ptop` = measured peg top z, `H` = measured
peg height (≈0.10), `board_z` = measured board plane z.

| # | Waypoint | Kind | Target | Gripper after | Duration |
|---|---|---|---|---|---|
| 0 | `move_to_home` (executor) | joint | `HOME_Q` | – | 2.2 s |
| 1 | `pick_hover` | joint | (px, py, `ptop + 0.015`) | open | 2.0 s |
| 2 | `pick_descend` | line | (px, py, `ptop − 0.045`) | – | 1.6 s |
| 3 | `grasp_close` | line | same point | **close** (check) | 0.1 s |
| 4 | `pick_lift` | line | (px, py, `z_carry`) | – | 1.4 s |
| 5 | `transit` | line | (hx, hy, `z_carry`) | – | 2.4 s |
| 6 | `place_descend` | line | (hx, hy, `z_place`) | – | 1.6 s |
| 7 | `release_open` | line | same point | **open** | 0.1 s |
| 8 | `place_retreat` | line | (hx, hy, `max(z_carry, board_z + H + 0.004 + 0.022)`) | – | 1.6 s |
| – | `return_park` (after verification) | joint | `SIDE_PARK_Q` | – | 2.0 s |

Derived heights: `z_grasp = ptop − GRASP_DEPTH(0.045)`; `hold_offset = z_grasp − (ptop − H)` (TCP above peg bottom,
≈0.055); `z_place = board_z + hold_offset + PLACE_CLEARANCE(0.004)`; `z_carry = max(z_grasp, z_place) + CARRY_CLEARANCE(0.030)`.

### 5.2 Constants and the reason for each
| Constant | Value | Why (do not change casually) |
|---|---|---|
| `GRASP_DEPTH` | 0.045 m | Palm face is only **56 mm** behind the fingertips; a 10 cm peg grasped low is driven into the palm. Grasp the **upper** ~4.5 cm. Verified 0–65° tilt. |
| `HOVER_ABOVE_TOP` | 0.015 m | Higher hovers are unreachable at r≈17 cm with a vertical tool. |
| `CARRY_CLEARANCE` | 0.030 m | Mat is flat; 3 cm is enough. Larger values (we tried 5.5 cm) pushed the required tilt from 55° to 60° for no benefit. |
| `PLACE_CLEARANCE` | 0.004 m | Peg bottom 4 mm above board at release → it drops onto the board, stays upright. |
| `RETREAT_CLEAR` | 0.022 m | **After release the open fingers straddle the peg; fingertips must end above the peg top before any sideways motion**, otherwise the swing topples the peg (bug we hit twice: end-of-run park and recovery). |
| `TILTS_DEG` | 0…60 step 5 | Single **constant** tilt for the whole task: the peg stays vertical in the gripper only if tool pitch never changes while holding it. Planner picks the smallest feasible. |
| `TESTED_MAX_TILT_DEG` | 65 | Physics-verified grasp/lift/place up to 60° at several peg positions; warn above this. |
| Radial margin | ±8 mm on every waypoint except `place_retreat` | Robustness to localisation error near full arm extension. |
| Line check | 30 samples, IK must exist, `max |Δq| ≤ 0.22 rad` between samples | Catches reach loss and elbow-branch flips along straight segments. |
| IK joint-limit margin | 0.02 rad | Stay off hard stops. Elbow-up branch only. |
| Peg height sanity | 0.085 ≤ H ≤ 0.118 m | Much lower ⇒ peg lying on its side; pipeline stops (side-grasp not implemented). |

### 5.3 Reach facts that drive the strategy (OMX-X, sim geometry)
* Straight-down TCP reach at grasp height ≈ **21.8 cm**. Tilt 30° ≈ 29 cm, 45° ≈ 33 cm, 60° ≈ 35 cm (at z≈4.7 cm, no margin).
* Circle socket at r = 31.7 cm (layout scale 1.4 mm/px) ⇒ chosen tilt **60°** (50° is geometrically OK but fails the margin and
  the retreat-clearance requirement). At scale 1.2 mm/px (r = 27.2 cm) ⇒ **40°**.
* The legacy notes say socket X≈160 mm (r ≈ 17–25 cm) — if that is the real geometry, tilt ≈ 0–20° and most of this
  concern disappears. **Measure.**

### 5.4 Grasp / release checks
* Gripper closes with a commanded 25 mm per finger (position servo, stalls on the peg). **Held** if total finger travel is in
  **18–42 mm** (peg 40 mm ⇒ ≈ 30.6 mm in sim); **≈ 50 mm ⇒ closed on air**. Checked right after closing and again after
  `pick_lift` and `transit` (drop detection).
* Hardware equivalent already exists in `GripperGuard` (counts 1920–2240 = holding, > 2310 = empty, ≥ 25 mA = holding).
  **Replace the finger-travel test with `GripperGuard.evaluate_grasp()` on hardware** (see §7 adapter).
* Failure handling: `grasp_failed` → open → `recovery_lift` (straight up to ≥ `ptop + 0.04`) → `SIDE_PARK_Q` → re-scan →
  re-plan → retry (max 2 attempts). Any other failure type (`aborted_by_vision_monitor`, `motion_error`) **stops for an operator**.

### 5.5 Vision path-verification thresholds (`tracker.py`)
* TCP estimate = marker position from camera (calibrated) + (FK(TCP) − FK(marker)) from encoders.
* Tolerance = `10 mm + 0.12 s × commanded speed` (servo lag; otherwise fast joint moves raise false alarms of up to ~25 mm).
* `ABORT`: error > `40 mm + lag` for **4 consecutive frames** (arm blocked/collision) → `AbortMotion` (disabled during recovery moves).
* Direction check: cosine between measured and planned displacement between frames must be ≥ 0.5 (flag) — reported mean ≈ 1.00.
* Cross-track error = distance to the planned straight segment (line segments only). `closing` = distance-to-goal not increasing.
* Observation rate: every 2nd control step (20 ms) = **25 Hz**; video 25 fps.

### 5.6 Calibration constraints (`calibration.py`)
* 16 target poses from a grid (x ∈ {0.10, 0.15, 0.20}, y ∈ {±0.04, ±0.12}, z ∈ {0.07, 0.12, 0.17} m, tilt ∈ {0°, 35°}), IK-feasible only.
* Needs ≥ 6 marker detections; one outlier pass (reject residual > max(3×median, 6 mm)).
* **Do it with the table clear** — calibration poses sweep the pick area at 7 cm height and will knock the peg over (this happened).
* Marker position = least-squares sphere centre of known radius (9 mm) from depth points (not just a pixel centroid).

### 5.7 Detection rules (`perception.py`)
* **Snapshot with the arm parked aside** (`SIDE_PARK_Q`, pan −90°): at home the arm hides part of the mat.
* Peg centre from the **top 12 mm of the peg only** (depth slice): the visible side wall otherwise biases the centroid (perspective).
* Sockets: red-outline contour → area / min-enclosing-circle ratio: **circle ≥ 0.90** (measured 0.95), **hexagon 0.74–0.90** (0.81),
  **square 0.52–0.74** (0.65), **triangle < 0.52** (0.42). Centre by pixel → **ray–plane intersection with the board plane**
  (not by noisy depth).
* Mat/board planes by RANSAC on `z = ax + by + c` (3 mm tolerance).
* Success criterion (final snapshot): peg XY within **8 mm** of socket centre **and** `|z_top − (board_z + H)| < 10 mm`.

### 5.8 Physics/servo model (sim only)
Timestep 2 ms, control 20 ms, `implicitfast`, position actuators with `kp` 200–300 / `kv` 12–20, **gravity compensation on arm bodies**
(real Dynamixels hold position stiffly), finger friction 2.0, peg friction 1.5, peg 40 mm × 100 mm, 80 g, gripper open gap 70 mm
(real ≈ 60 mm per `GripperGuard`: **a 6 cm peg will not fit** — see §11).

---

## 6. Hard-won findings (read before debugging)

1. **Reach, not vision, decides the strategy** — see §5.3. The planner explains its choice in `workspace_report.json → verdict.feasibility`.
2. **Palm geometry** limits grasp height (§5.2). First tilted-grasp attempts failed because of it.
3. **Self-collision exclusions:** base ↔ link2/link3 must be excluded in the MJCF or the shoulder cannot swing backwards (q2<0) — the arm
   silently stopped at q2 = 0 (tracking error exactly |q2 target|).
4. **Retreat must clear the peg top** (§5.2) — appears in both the normal flow and recovery.
5. **Recovery must lift straight up before any sideways swing**; the first version swept the open fingers through the peg.
6. **Peg is placed after calibration**; the pipeline spawns it parked off-view and teleports it onto the mat (`SimEnv.place_peg`).
7. **Parallel runs started in the same second used to share one folder** — fixed (`_2`, `_3` suffix).
8. **Hole sizes are in metres in the yaml**; changing `pixel_scale_m` alone (e.g. 1.1 mm/px) makes the sockets physically touch and detection
   merges outlines. The 36 px socket must fit a 4–6 cm peg ⇒ scale ∈ [1.2, 1.8] mm/px.
9. **Parallax:** the yellow planned-path overlay is drawn at TCP height, so it appears shifted relative to table-level objects. Not a bug.
10. Home pose hides part of the table ⇒ never run detection at home.
11. `imp.md` lists the socket order top→bottom as Triangle→Square→Hexagon→Circle; the overhead frame supplied shows
    **Circle→Hexagon→Square→Triangle**. Detection is shape-based so it does not matter, but fix the doc once confirmed.

---

## 7. How to merge (step by step)

### 7.1 Safe immediate merge (no conflicts)
1. Copy `sim_demo/` and `demo_runs/` into the working copy next to `src/`. Copy `repare.md`.
2. `imp.md`: **append section 11** from the new `imp.md` to your updated `imp.md` (do not replace the file). Keep both copies of any edits.
3. `.gitignore`: add `demo_runs/*` except the sample you want to keep (`!demo_runs/demo_20261107_211140`), plus `__pycache__/`.
4. `pip install -r sim_demo/requirements.txt` (mujoco, numpy, opencv-python, pyyaml, matplotlib). Legacy deps unchanged.
5. Verify: `cd T1/sim_demo && python tests/test_sim_demo.py` → 6 PASS; `python run_pipeline.py --until workspace` → a folder appears in `T1/demo_runs/`.

```bash
# git example (both lineages updated elsewhere):
git checkout -b merge-sim-demo
cp -r /path/to/new/T1/sim_demo T1/ ; cp /path/to/new/T1/repare.md T1/
git add T1/sim_demo T1/repare.md && git commit -m "Add sim_demo pipeline + repare.md"
# imp.md: resolve by keeping YOUR version and pasting section "## 11. Sim Demo Pipeline" at the end
```

### 7.2 Concept map: legacy ↔ new (what to reuse, what to bridge)
| Concept | Legacy | New | Recommendation |
|---|---|---|---|
| Joint-space / FK / IK | `src/planning/kinematics.py` (mm, zero=straight, no 24 mm offset) | `sim_demo/arm_model.py` (m, zero=L, URDF dims) | Keep both for now. Add `src/planning/kinematics_urdf.py` = copy of `arm_model.py` **after** hardware mapping (§10.3). Long-term make `arm_model` the single source and re-implement the legacy API as a thin mm/legacy-zero wrapper; keep `tests/test_kinematics.py` green. |
| Calibration | `EyeToHandCalibrator.compute_rigid_transform` (mm) | `calibration.kabsch/run_calibration` (m, arm-motion + marker) | Reuse the legacy class for load/save; feed it points from the new routine (convert m→mm) so `camera_calib.yaml` stays the persisted format. |
| Shape/colour detection | `ShapeDetector`, `config/colors.yaml` (real HSV) | `perception.py`, `config/vision.yaml` (sim HSV) | Same enclosing-circle thresholds (0.91/0.77 legacy vs 0.90/0.74 new — pick one after real data). Use **real** `colors.yaml` on hardware; keep sim yaml for sim. |
| Depth slicing | `single_camera_pipeline.depth_slice_top_face` (±15 mm) | `detect_green_peg` (top 12 mm) | Same idea; unify the parameter. |
| Workspace analysis | `WorkspaceAnalyzer.classify_target` (radius bands 254/320 mm, fixed pitch ranges) | `workspace.reachability_grid` + `planner` (real IK, limits, margins) | New one is data-driven. Keep the legacy radius classifier as a quick pre-check only. |
| Grasp/drop check | `GripperGuard` (counts, mA) | finger-travel window in `run_pipeline.held_ok` | **Replace** with `GripperGuard` on hardware via the `RobotInterface` below. |
| Camera drift | `CameraDriftMonitor` (socket landmarks) | none | Port: run it on the **sockets** from `detect_holes` before each attempt. Cheap and valuable. |
| Recovery | `RecoveryManager` | `recovery_lift` + re-scan loop | Merge: keep new "lift clear → park → re-scan → re-plan"; call legacy jam/spiral logic for insertion jams. |
| Home/park | `HOME_READY` pose in `imp.md` §8 | `HOME_Q` (stage 1), `SIDE_PARK_Q` (snapshots) | Both needed: home for the demo, side-park for clean snapshots (legacy "arm parks out of view" idea). |
| Teleop/E-stop | `teleop_keyboard.py` | none | Keep. Add an E-stop hook to the executor (§7.3). |
| Scene | `sim/scene*.xml`, `demo1_20261107/scene_*.xml` | generated by `scene_builder.py` (static copy in each run's `config_snapshot/scene.xml`) | Prefer the generator; legacy XMLs are reference only. |

### 7.3 Recommended target architecture (do this in order)
1. **Hardware Abstraction Layer** — the executor currently talks to `SimEnv`. Introduce a small interface and two backends:
   ```python
   class RobotInterface:                      # units: metres, radians, seconds
       def read_joints(self) -> np.ndarray: ...            # q1..q4 (rad, sim-zero convention)
       def move_joints(self, q, duration): ...
       def move_cartesian(self, p0, p1, tilt, duration): ...   # IK per step via arm_model
       def set_gripper(self, close: bool, duration): ...
       def grasp_state(self) -> {"held","empty","unknown"}     # sim: finger travel; real: GripperGuard
       def estop(self): ...
       def capture_rgbd(self): ...                              # (bgr, depth_m)
   class SimRobot(RobotInterface): ...      # wrap sim_demo.sim_env.SimEnv
   class RealRobot(RobotInterface): ...     # wrap OpenManipulatorController + RealSense + counts<->rad adapter
   ```
   `execute_plan` and `PathTracker` mostly use `read_joints / move_* / set_gripper / capture_rgbd / ctrl_q / t / frame_hooks`, so they port with a
   thin shim. **Not yet portable** (touch MuJoCo/simulator directly): `stage_home` (reads `env.model/data` for the FK check), the final verdict
   (`env.gt_*`, scoring only — drop on hardware), `env.layout`, `env.cam` (take intrinsics from RealSense), `SimEnv.place_peg`. Add `--mode sim|real` (legacy roadmap item) and keep `--dry-run` (arm elevated, no table contact).
2. Move shared pure-math into one place (`arm_model`, `kabsch`, shape thresholds) and have both lineages import it.
3. Add the counts↔radians adapter (§3.3) and unit conversions at the **RealRobot** boundary only.
4. Wire `HardwareProtector`/`GripperGuard`/E-stop into the real executor loop (check every control tick; abort path = open nothing, hold torque off per legacy policy).

---

## 8. Running and acceptance criteria

```bash
cd T1/sim_demo
python run_pipeline.py                       # full run, ~100–135 s, writes ../demo_runs/demo_<ts>/
python run_pipeline.py --until home|workspace|pick
python run_pipeline.py --peg-xy 0.15 -0.05   # peg (x, y) in metres, robot frame
python run_pipeline.py --seed 3 --no-video   # random peg placement seed; skip mp4
python run_pipeline.py --inject-miss         # recovery test (first grasp misses)
python run_pipeline.py --layout my_layout.yaml --depth-noise-mm 1.5
python tests/test_sim_demo.py                # 6 checks, ~1–2 min
```
**Reference results (this build, sim):**
* Calibration: 16/16 poses, fit residual 0.10 mm, reprojection 0.36 px, error vs true camera pose 0.07 mm / 0.022° (sim has perfect intrinsics/link lengths).
* Workspace: all four sockets classified (ratios 0.95 / 0.81 / 0.65 / 0.42); circle socket within 0.1 mm of truth; peg within 1 mm, radius 20.3 mm, height 102 mm.
* Chosen tilt **60°** (layout 1.4 mm/px) — 40° at 1.2 mm/px.
* Pick & place: tracking 100 % ON_TRACK, mean 3.5–4.5 mm (lag during fast joint moves), cross-track max ≈ 1 mm, direction cosine ≈ 1.00;
  final peg-to-socket error **1.3–2.1 mm** (vision) on 4 different peg positions; physics test 6 positions ≤ 1 mm.
* `--inject-miss`: first grasp detected as air (finger travel 50 mm) → lift → re-scan (peg localised within 0.1 mm of truth) → re-plan → success on attempt 2.
* Self-tests: 6/6 pass.

**Run-folder layout** (`demo_runs/demo_<YYYYMMDD_HHMMSS>[_n]/`):
```
summary.md  run_log.json  config_snapshot/{sim_layout.yaml, vision.yaml, scene.xml}
01_home_pose/            home_overhead_camera.png, home_oblique_view.png, home_side_view.png, home_pose.json
02_calibration/          calib_pose_XX.png, calibration_reprojection.png, calibration.json
03_workspace_analysis/   01_rgb 02_depth 03_segmentation 04_detections 05/06_reachability_* 07_topdown_map, workspace_report.json
04_pick_and_place/       ckpt_XX_<waypoint>_{overhead,oblique}.png, storyboard.png, tracking_log.csv, tracking_plot.png,
                         path_topdown.png, pick_and_place.mp4, final_check_{overhead,oblique}.png, pick_place_result.json
```

---

## 9. What the simulation assumes (and the risk on hardware)

| Assumption in sim | Risk on real robot | How to find out |
|---|---|---|
| Layout scale 1.4 mm/px, base at pixel (448, 252), mat/board/hole pixel positions from **one IR frame** | Wrong distances ⇒ wrong required tilt | Measure with a ruler: base→circle-socket centre, mat size, board size, socket diameters. Update `config/sim_layout.yaml`. |
| Camera looks **straight down**, 0.61 m high, 58° vertical FOV, perfect intrinsics | `imp.md` says the camera is angled 45–55°; frames look near top-down; FOV/intrinsics differ | Read RealSense intrinsics (`rs.intrinsics`), measure camera pose; calibration is generic but parallax/occlusion differ. |
| Joint zero = URDF L pose, limits from URDF | Real counts/offsets/signs may differ (§3.3) | §10 step 3 mapping test. |
| Perfect link lengths, no backlash, gravity-compensated stiff servos | mm-level errors, droop, lag | Compare FK vs ruler at 3 poses; tune profile velocity/acceleration. |
| Gripper: 70 mm open gap, 56 mm pads, 30 mm closed-on-peg stroke, friction 2.0 | Real gap ≈ 60 mm (`GripperGuard`); 4–6 cm peg: **6 cm peg won't fit**; pad length/shape differ ⇒ `GRASP_DEPTH`, tilt limit change | Measure open gap, pad length, palm-to-tip distance; re-run physics tests with real numbers. |
| Colour marker on the wrist (magenta ball, 9 mm radius, 30 mm mast) | **Does not exist physically** | Add a small bright ball/sticker; keep it visible from above at all calibration poses. |
| Sockets are decals on a solid board; "in socket" = peg upright within 8 mm | Real board has through-holes, peg inserts/falls | Re-define success; add insertion/jam logic (legacy spiral search). |
| HSV tuned for the sim render (RGB) | Real feed shown to us is **IR with dot pattern** | Use the RealSense **RGB** stream; re-tune with `config/colors.yaml`. |
| Calibration done with table clear; peg then appears | Needs operator discipline | Document in the operator checklist. |
| Camera is rigid | Bumps shift extrinsics | Port `CameraDriftMonitor` (§7.2). |
| One peg, circle only | Other shapes need yaw alignment | §11 roadmap. |
| Depth noise 1.5 mm at 0.6 m (scaled ∝ z²) | Real noise/dropouts at edges | Median-filter depth in a 5×5 patch (legacy edge-case P2). |

---

## 10. Hardware bring-up checklist (ordered; stop at the first failure)

Mark each with a result and date; never skip ahead to motion that touches the table.

- [ ] **1. Safety first.** Torque-off posing works; E-stop key tested (`ESC`/`X`); `HardwareProtector` running (T < 65 °C, 10.5–14.6 V); power supply 12 V/5 A; USB isolator recommended.
- [ ] **2. Measure the world.** Ruler: base-axis→circle-socket centre (X, Y), mat edges, board edges, socket diameters/spacing, table edge. Pass: values entered in `sim_layout.yaml`; sim re-run gives the same chosen tilt you expect.
- [ ] **3. Joint mapping.** With torque off pose the arm in the **L pose** (upper arm up, forearm horizontal forward, gripper forward); read the 4 present-position counts → `ZERO_COUNTS`. Move each joint ≈ 5° and confirm the sign. Verify FK TCP vs ruler at ≥ 3 poses (expect ≈ (274, 0, 205) mm at L-home measured from the base origin / table surface). Pass: ≤ 3 mm error. Resolve the conflict with legacy `[0, 0, 1.45, 0]`/1100 counts.
- [ ] **4. Limits & speeds.** Confirm joint limits (take the intersection of URDF and legacy). Set profile velocity/acceleration (legacy `initialize_all_joints(profile_vel=50, profile_acc=15)`) and check the tracker lag tolerance still fits.
- [ ] **5. Gripper.** Measure open gap, pad length, palm-to-tip; map closed-on-peg and closed-on-air to **counts** (legacy: holding 1920–2240, empty > 2310) and current; plug into the `RobotInterface.grasp_state()`.
- [ ] **6. Camera.** RealSense intrinsics from the device; RGB stream; mount pose; resolution 640×480; check depth validity at the table (median 5×5). Re-tune HSV on **real** pegs/mat/board with `config/colors.yaml`.
- [ ] **7. Marker.** Attach a bright ball (known radius) on the wrist at the known offset (update `arm_model.MARKER_LOCAL`, `MARKER_R`). Verify the camera sees it at all calibration poses.
- [ ] **8. Calibration (table clear).** Run `run_calibration` on the real arm **in dry-run (arm elevated)**. Pass: ≥ 12 detections, residual ≤ 3 mm, reprojection ≤ 3 px. Save to `config/camera_calib.yaml` (convert to mm).
- [ ] **9. Static perception.** Park arm aside, capture, run `detect_surfaces/holes/peg`. Pass: 4 sockets classified correctly, circle socket and peg within 5 mm of ruler values.
- [ ] **10. Dry run.** Full pipeline with the arm held above the table (`--dry-run`): waypoints executed at +5 cm, camera tracker active. Pass: 100 % ON_TRACK, no abort.
- [ ] **11. Live, slow, one peg.** Reduced speed, hand on E-stop, circle peg near the mat centre. Pass: success ≥ 3/5.
- [ ] **12. Recovery.** Force a missed grasp and a dropped peg; confirm detection → lift → re-scan → retry; confirm `AbortMotion` stops motion.
- [ ] **13. Timing & logging.** Record cycle time per attempt; keep the run folder (`demo_<timestamp>`) for every real run.

---

## 11. Remaining work / roadmap (priority order)

**P0 – needed before the real robot moves**
1. Joint mapping + unit adapters (`RealRobot`), E-stop in the executor, `GripperGuard` for grasp state (§7.3, §10).
2. Measure real geometry (§10.2) and re-derive tilt/limits. If the real socket is at r ≤ 25 cm, re-check whether a vertical grasp now suffices.
3. Physical marker + real calibration run; replace the guessed `camera_calib.yaml`.

**P1 – robustness**
4. Port `CameraDriftMonitor` (socket landmarks) and run it before every attempt.
5. Real HSV tuning, specular/glare handling (CIELAB), depth-edge dropout handling (legacy P2 items).
6. Through-hole insertion: define success physically; jam detection (load spike) + spiral search from legacy.
7. Peg toppled on its side → side grasp (currently the pipeline just stops with a message).
8. Gripper pad geometry and stroke from measurements; re-run `tests/test_sim_demo.py` with real numbers.

**P2 – scope**
9. Hexagon / square / triangle pegs: yaw estimation (`ShapeDetector._estimate_orientation`, symmetry order 6/4/3 → `calculate_minimal_rotation`), wrist-roll is **not available** on OMX-X (4-DOF + gripper) ⇒ align by **base pan + gripper orientation relative to the peg's flats**; check feasibility per shape.
10. All four pegs sequentially (the scene builder already supports `pegs=("circle","hexagon","square","triangle")`; pipeline is circle-only).
11. Evaluation script for the README checklist (success rate and timing over N runs) — the run-folder JSON already has the data.
12. Use the angled camera if that is what is really mounted: add a tilted-camera option to `scene_builder` and a tilted-camera test.

**Nice to have:** second RGB-D camera (task allows two), speed optimisation (bonus points for fastest), YOLO-based detector behind the same detector interface.

---

## 12. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `an OpenGL platform library has not been loaded` | `mujoco` imported before `sim_env` on headless Linux | Import `sim_env` first, or `export MUJOCO_GL=egl` |
| "no circular socket detected; saw shapes [...]" | Layout scale inconsistent with metre-sized holes (outlines merge) or thresholds off | Keep `pixel_scale_m` ∈ [1.2, 1.8] mm/px or scale `hole_size_m` with it |
| "circle peg not found" | Peg occluded or knocked over (calibration with peg on table) | Place peg after calibration; snapshot only at `SIDE_PARK_Q` |
| "looks toppled over" | Measured height < 85 mm | Stand the peg up (side grasp not implemented) |
| `no feasible pick-and-place plan` | Peg or socket outside reach even at 60° | Move the peg closer; check layout numbers; inspect `verdict.feasibility` |
| Many `DEVIATING` frames on fast moves | Servo lag above tolerance | Lower speed or raise `LAG_S`; check `tracking_plot.png` |
| `AbortMotion` mid-descent | Arm blocked (peg/board) | Check `ckpt_*` images; verify peg localisation and `GRASP_DEPTH` |
| Peg toppled after release | Retreat too low / sideways swing too early | Keep `RETREAT_CLEAR`; never swing before clearing the peg top |
| Calibration residual large | Marker occluded/mis-modelled, wrong intrinsics, wrong link lengths | Inspect `calibration_reprojection.png` and per-pose residuals |

---

## 13. Decision log (why things are the way they are)

* **Constant tilt over the whole task** — the grasp holds the peg vertical *relative to the world* only if tool pitch is kept constant afterwards; changing the tilt while holding rotates the peg inside the pads (it would arrive at the socket tilted). Not stress-tested in sim beyond reasoning — keep it constant unless you re-verify.
* **Grasp high on the peg** — palm geometry (§5.2).
* **Calibrate with arm motion, not a checkerboard** — uses the arm's known dimensions as the ground truth, matches the stated idea, and
  validates link lengths at the same time (a bad FK shows up as residual).
* **Snapshot-and-freeze while parked aside** — removes self-occlusion; peg state re-verified after the placement from the same pose.
* **Lag-aware tracking tolerance** — avoids false alarms on fast joint-space moves, still catches blockage.
* **Success verified by camera, scored by simulator truth** — `vision_in_socket` drives behaviour; `SIM_ONLY_*` fields only judge it.
* **World frame = robot frame** — removes a transform from calibration scoring and from the planner.
* **Layout in pixels** — the only measured source is the camera frame; one scale parameter captures the uncertainty.
* **Scene generated by code** — all geometry comes from one yaml; each run stores the exact XML it used.

---

## 14. Quick reference

* Home FK (sim convention): TCP (0.274, 0, 0.205) m, tool tilt 90° (horizontal).
* Typical numbers (layout 1.4 mm/px): peg (0.16, 0.07) m, circle socket (0.295, −0.115) m, r = 0.317 m, chosen tilt 60°, `z_grasp ≈ 0.068`, `z_place ≈ 0.069`, `z_carry ≈ 0.099`, retreat ≈ 0.14 m.
* Camera (sim): pos (0.179, −0.017, 0.61) m, looks down, image-left = +X, image-down = +Y, fovy 58°, 640×480.
* Legacy hardware: U2D2 `/dev/cu.usbserial-FTA2W88F`, IDs 11–15, 1 Mbps, Protocol 2.0, 4096 counts/rev, E-stop `ESC`/`X`, home hotkey `H`.
* Where results live: `T1/demo_runs/demo_<timestamp>/summary.md`.
