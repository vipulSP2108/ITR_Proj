# Sim demo: camera-verified circle-peg pick & place (MuJoCo)

One command runs the whole idea end-to-end in simulation and stores everything in a new
`demo_runs/demo_<YYYYMMDD_HHMMSS>/` folder, so every run is kept.

```bash
pip install -r sim_demo/requirements.txt
cd T1/sim_demo
python run_pipeline.py                      # all stages, ~2 min, writes ../demo_runs/demo_<timestamp>/
python run_pipeline.py --until workspace    # stages 1-2 only (seconds)
python run_pipeline.py --peg-xy 0.15 -0.05  # put the peg somewhere specific (robot frame, metres)
python run_pipeline.py --inject-miss        # test recovery: first grasp misses, pipeline re-scans & retries
python run_pipeline.py --no-video           # skip the mp4
python tests/test_sim_demo.py               # 6 self-checks (~1-2 min)
```
Open `summary.md` in the run folder first.  macOS and Linux work out of the box (headless Linux uses EGL automatically).

## What it does (your three ideas, as stages)

| Stage | What happens | Key outputs |
|---|---|---|
| **1 Home** | Arm parks in the "L" home pose (the screenshot). Encoders -> forward kinematics from the published OMX-X dimensions are checked against the simulator. | `01_home_pose/` |
| **2a Calibration** | The arm moves to 16 poses; at each one the mounted RGB-D camera sees a magenta marker on the wrist. Marker position from the camera vs. from *arm dimensions + joint encoders* -> rigid transform camera->robot (Kabsch/SVD). | `02_calibration/calibration.json`, `calibration_reprojection.png` |
| **2b Workspace analysis** | Arm parks aside (clean view). Detects mat, board, all four sockets (shape-classified), the circle peg; lifts them to metric robot-frame coordinates with depth; builds a **reachability map** (minimum tool tilt per table position, from real IK + joint limits) and picks the grasp strategy. | `03_workspace_analysis/` (detections, reachability overlays, top-down map, `workspace_report.json`) |
| **3 Pick & place** | Circle peg -> circle socket. At ~25 Hz the camera tracks the tool marker and the peg; **tracking error, cross-track error, direction cosine and progress** are computed against the plan. Grasp check by finger travel; one automatic retry (lift clear -> re-scan -> re-plan). Final check from a fresh snapshot. | `04_pick_and_place/` (checkpoints, storyboard, `tracking_log.csv`, `tracking_plot.png`, `path_topdown.png`, `pick_and_place.mp4`, `final_check_*.png`) |

Nothing is hard-coded: positions, reachability verdict, tilt and every tracking number are computed from camera images + arm geometry.
Simulator ground truth is read only for *scoring* and is labelled `SIM_ONLY_` in the JSON files.

## Files

```
sim_demo/
  run_pipeline.py     CLI + the three stages      scene_builder.py  builds the MJCF from config/sim_layout.yaml
  arm_model.py        analytic FK/IK, limits      sim_env.py        MuJoCo env: control, RGB-D camera, trajectories
  calibration.py      eye-to-hand calibration     perception.py     colour/shape/plane/marker detection
  workspace.py        reachability analysis       planner.py        constant-tilt pick&place planner (+ feasibility table)
  tracker.py          vision path verification    viz.py            overlays / plots
  config/sim_layout.yaml   layout in image pixels  config/vision.yaml   HSV thresholds
  tests/test_sim_demo.py
```

## Design decisions worth knowing

* **World = robot frame**: origin at the base on the table surface, +X toward the board, +Y image-down, +Z up.
* **Layout from your real overhead frame** (`config/sim_layout.yaml`, in pixels). The scale (1.4 mm/px) is an *estimate*: a 36-px socket must fit a 4-6 cm peg, which bounds it to ~1.2-1.8 mm/px. Socket sizes in the yaml are in metres, so change `pixel_scale_m` only together with `hole_size_m`.
* **Reach drives the strategy.** With the arm's real dimensions a straight-down grasp reaches only ~22 cm at grasp height, but the circle socket is ~32 cm away. The planner therefore finds the smallest **constant tool tilt** for which every waypoint (with an 8 mm margin) and every straight segment is reachable (here 60 deg; 40 deg if the real layout is 1.2 mm/px). The peg stays vertical in the gripper, so the same tilt is needed for pick and place. The feasibility table is in `workspace_report.json`.
* **Grasp high on the peg** (pads overlap the top ~4.5 cm): the palm sits only 5.6 cm behind the fingertips and the peg is 10 cm tall - grasping low drives the peg into the palm.
* **Retreat must clear the peg top** after release (open fingers straddle the peg; a sideways swing knocks it over). The planner enforces this.
* **Occlusion**: at home the arm hides part of the table, so detection snapshots are taken with the arm parked aside (`SIDE_PARK_Q`, pan -90 deg). The peg is placed *after* calibration (the table is clear while the arm moves through calibration poses).
* **Lag-aware monitor**: tracking tolerance = 10 mm + 0.12 s x commanded speed. Fast joint moves show up to ~25 mm of servo lag in the plot; the monitor aborts only if the error stays > 40 mm (+lag) for 4 frames.
* **Projected path overlay**: the yellow path in the camera images is drawn at TCP height, so it looks shifted from table-level objects (parallax).

## Limits of this simulation (read before trusting numbers on hardware)

* Sim has perfect camera intrinsics, perfect link lengths and no backlash: calibration error here (~0.1 mm) will be larger on hardware (expect mm). Real robot: use measured RealSense intrinsics and verify link offsets.
* Socket = visual decal on a solid board: "in socket" means the peg stands upright within 8 mm of the socket centre, not that it passes through a hole.
* Colour thresholds are tuned for the sim render. The real stream you showed is IR with a dot pattern; use the RealSense **RGB** stream and re-tune `config/vision.yaml` (see `../config/colors.yaml`).
* Tilted-grasp physics were verified up to 60 deg for the circle peg at several positions. Gripper pad shape/stroke is an approximation of the real one.
* Joint limits are the open_manipulator_x URDF values (as recalled): confirm against your robot's configured limits.
* Servo model: position actuators with gravity compensation on the arm bodies (real Dynamixels hold position stiffly).
* Only the circle peg is implemented in the pipeline; the scene builder already supports the other three shapes (`pegs=` argument).
