# Project Directory Structure & Architecture Map

This document provides a comprehensive map of the entire workspace (`/Users/aai/Desktop/ITR_Proj`), detailing the roles, contents, and active statuses of all directories and files.

---

## 1. Top-Level Workspace Organization

The repository separates active, tested operational software from supporting references and buffered archives:

```
/Users/aai/Desktop/ITR_Proj/
├── README.md                      # Master repository landing page & executive summary
├── HOW_TO_RUN.md                  # Complete operator runbook & terminal command reference
├── HOW_TO_RECREATE.md             # End-to-end guide to rebuild & reproduce this project
├── DIRECTORY_STRUCTURE.md         # This architectural directory map
│
├── T1 2/                          # [ACTIVE] Task 1: OpenMANIPULATOR-X Peg Sorting & RealSense
│   ├── device/                    # Hardware specs, RealSense configs, & hardware_protector daemon
│   ├── openmanipulator_test/      # Low-level Dynamixel motor test suite & teleop
│   └── videos/                    # Submission demonstration videos for Task 1
│
├── T3 2/                          # [ACTIVE] Task 3: Addverb HEAL Cobot Buzz Wire Navigation
│   ├── buzzwire_calib/            # Quick link to active trajectory execution engine
│   ├── analysis_and_dashboards/   # Quick link to 3D dashboards & interactive reports
│   └── videos/                    # Submission demonstration videos for Task 3
│
└── dump/                          # [BUFFER] Non-destructive archive for scratch files, images, zips & legacy snapshots
    ├── T1/                        # [HISTORICAL] Early baseline snapshot for Task 1
    └── T3/                        # [HISTORICAL] Early baseline & photogrammetric calibration for Task 3
```

| Classification | Path | Role & Operational Status |
| :--- | :--- | :--- |
| **Active Working (T1)** | [`T1 2/`](file:///Users/aai/Desktop/ITR_Proj/T1%202) | **Primary tested workspace for Task 1**. Contains OpenCV perception, RealSense D435i calibration, Dynamixel pick-and-place execution stack, hardware specs (`device/`), bringup tests (`openmanipulator_test/`), and submission videos (`videos/`). |
| **Active Working (T3)** | [`T3 2/`](file:///Users/aai/Desktop/ITR_Proj/T3%202) | **Primary tested workspace for Task 3**. Contains the HEAL 6-DOF cobot Buzz Wire navigation stack, 6-DOF teleop, gravity-compensated teaching, time-optimal trajectory replay, and submission videos (`videos/`). |
| **Task 1 Hardware Specs** | [`T1 2/device/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device) | Actuator specs (XM430-W350-T), pinouts, camera specs, photographic setup gallery, and the hardware watchdog protector. |
| **Task 1 Bringup Suite** | [`T1 2/openmanipulator_test/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/openmanipulator_test) | Low-level Dynamixel SDK routines (ping, joint sweep, teleop, home stance) imported by Task 1 scripts. |
| **Buffer / Dump** | [`dump/`](file:///Users/aai/Desktop/ITR_Proj/dump) | Centralized, non-destructive buffer holding legacy task snapshots (`dump/T1`, `dump/T3`), 25 debug images, scratch test scripts, and ZIP archives. |

---

## 2. Task 1 Active Directory Breakdown (`T1 2/`)

Located at: [`/Users/aai/Desktop/ITR_Proj/T1 2/`](file:///Users/aai/Desktop/ITR_Proj/T1%202)

### 2.1 Core Executables & Scripts
* **[`pick_place.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/pick_place.py)**: The main autonomous entry point. Performs top-down RealSense RGB capture, table background segmentation, peg contour detection, hole orientation matching, and 5-DOF pick-and-place arm trajectories.
* **[`run_calibration.sh`](file:///Users/aai/Desktop/ITR_Proj/T1%202/run_calibration.sh)**: Automated shell runner to execute the full camera-to-robot hand-eye calibration routine.
* **[`RUN_COMMANDS.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/RUN_COMMANDS.md)**: Operator cheat sheet for running `pick_place.py`, checking arm voltage, and verifying calibrations.
* **[`hw_run.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/hw_run.py)**: Standalone hardware pipeline test runner.

### 2.2 Calibration Suite (`calibration/`)
* **[`00_check_arm.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/calibration/00_check_arm.py)**: Pings Dynamixel servos (IDs 11–15), checks 12V bus supply voltage, and verifies no hardware error flags.
* **[`01_calibrate_camera_robot.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/calibration/01_calibrate_camera_robot.py)**: Automated calibration routine. Moves fingertip across 40 known robot Cartesian coordinates, uses RealSense to detect fingertip visual positions and ArUco markers, and solves for the camera-to-base rigid transform $(R, \mathbf{t})$.
* **[`02_verify_calibration.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/calibration/02_verify_calibration.py)**: Verifies accuracy by moving the arm to physical target points (e.g. `--xy 150 60`) and overlaying camera projection crosshairs.
* **[`03_stability_test.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/calibration/03_stability_test.py)**: Verifies repeatability over repeated cycles.
* **[`04_gripper_sweep.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/calibration/04_gripper_sweep.py)**: Calibrates gripper open/closed encoder thresholds.
* **[`hw_common.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/calibration/hw_common.py)**: Shared Dynamixel hardware abstraction layer and kinematics helpers.

### 2.3 Configuration Files (`config/`)
* **`pick_place.yaml`**: Primary runtime parameters (clearances, approach heights, gripper force limits, camera exposure).
* **`camera_robot_calib.yaml`**: Fitted camera extrinsic transform $(R, \mathbf{t})$ and table plane height $Z_{\text{mat}}$.
* **`detected_holes.yaml`**: Stored table hole center coordinates and shapes.
* **`colors.yaml`**: Calibrated HSV color boundaries for Circle (Green), Square (Blue), Hexagon (Orange), and Triangle (Yellow).

### 2.4 Software Modules (`src/`)
* **`src/control/`**: Dynamixel protocol 2.0 communication wrapper, servo safety limits, torque management.
* **`src/vision/`**: `camera_model.py` (pixel-to-robot coordinate unprojection), color filtering, ArUco detection.
* **`src/planning/`**: `omx_kinematics.py` (forward/inverse kinematics for 4-DOF arm + gripper), Cartesian quintic trajectory generator.

### 2.5 Submission Videos (`videos/`)
* **[`videos/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/videos)**: Upload destination for demonstration and submission videos (primary: `task1_autonomous_peg_sorting.mp4`). See [`videos/README.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/videos/README.md).

### 2.6 Internal Buffer & Historical Archive (`dump/`)
Located at [`T1 2/dump/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/dump), this folder modularizes non-active assets:
* **`deprecated_runners/`**: Deprecated scripts (`hw_run.py`, `hw_test_step1.py`, `run_hardware_pipeline*.py`) using outdated kinematics.
* **`historical_demos/`**: Legacy demo folders (`demo_runs`, `sim`, `sim_demo`, `scripts`, `tests`). The original standalone demo is located in [`T1 2/demo1_20261107/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/demo1_20261107).
* **`test_images_and_output/`**: Early test outputs and image folders (`hw_test_output/`, `calibration_img/`, `test_images/`).
* **`scratch_notes_and_logs/`**: Early simulation logs and scratch notes (`MUJOCO_LOG.TXT`, `task_rough.txt`, `test.mp`, `imp.md`, `context_folder/`).

---

## 3. Task 3 Active Directory Breakdown (`T3 2/`)

Located at: [`/Users/aai/Desktop/ITR_Proj/T3 2/`](file:///Users/aai/Desktop/ITR_Proj/T3%202)

### 3.1 Primary Working Path & Quick Links
* **Canonical Working Code Path**:
  `T3 2/NEW/Task2_ITR-main (1)/Task2_ITR-main/Task2/taken/G9/Task2_ITR-main/buzzwire_calib/`
* **Convenient Root Pointers**:
  - [`T3 2/buzzwire_calib`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib) -> Symlink pointing directly to the canonical working code folder.
  - [`T3 2/analysis_and_dashboards`](file:///Users/aai/Desktop/ITR_Proj/T3%202/analysis_and_dashboards) -> Symlink pointing to the 3D dashboard generation scripts and interactive HTML reports (`buzz_wire_3d_calibration_and_averaging_dashboard.html`).
* **Internal Buffer ([`T3 2/dump/`](file:///Users/aai/Desktop/ITR_Proj/T3%202/dump))**: Contains superseded releases (`Task2_ITR-main/` and `Task2_ITR-main.zip`), duplicate nested `T3` trees, older intermediate `buzzwire_calib` folders, and duplicate markdown guides. All active guides live strictly inside [`T3 2/buzzwire_calib`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib).

### 3.2 High-Level CLI Launchers
* **[`buzzwire_run.sh`](file:///Users/aai/Desktop/ITR_Proj/T3%202/NEW/Task2_ITR-main%20(1)/Task2_ITR-main/Task2/taken/G9/Task2_ITR-main/buzzwire_calib/buzzwire_run.sh)**:
  Single-command execution harness:
  - `./buzzwire_run.sh goto L` / `R` / `home`
  - `./buzzwire_run.sh lr show|balanced|fast` (Left-to-Right wire traversal)
  - `./buzzwire_run.sh rl` (Safe Right-to-Left return)
  - `./buzzwire_run.sh dry-run lr` (Velocity & duration audit without physical motion)
  - `./buzzwire_run.sh sim lr` (Simulation on virtual arm)
* **[`buzzwire_teach.sh`](file:///Users/aai/Desktop/ITR_Proj/T3%202/NEW/Task2_ITR-main%20(1)/Task2_ITR-main/Task2/taken/G9/Task2_ITR-main/buzzwire_calib/buzzwire_teach.sh)**:
  Master teaching and build tool:
  - `./buzzwire_teach.sh new [NAME]` (Create fresh session)
  - `./buzzwire_teach.sh teleop` (Method 2: 6-DOF keyboard driving)
  - `./buzzwire_teach.sh record` (Method 1: Gravity-compensated free-drive teaching)
  - `./buzzwire_teach.sh inspect` (Validate segment continuity and gaps)
  - `./buzzwire_teach.sh merge` (Stitch segments, smooth, resample at 2 mm)
  - `./buzzwire_teach.sh replay` (Execute trajectory replay)
* **[`run_collect.sh`](file:///Users/aai/Desktop/ITR_Proj/T3%202/NEW/Task2_ITR-main%20(1)/Task2_ITR-main/Task2/taken/G9/Task2_ITR-main/buzzwire_calib/run_collect.sh)**: Sourced ROS 2 launch helper.

### 3.3 Active Scripts (`scripts/`)
* **`goto_target.py`**: Moves the HEAL arm safely between keyframes (Post L, Post R, Home) using quintic joint profiles and closed-loop position error tracking.
* **`replay_trajectory.py`**: High-speed Cartesian trajectory replayer using resolved-rate DLS-IK, Nakamura adaptive damping, and live carrot tracking.
* **`record_segment.py`**: Records raw free-drive waypoints at 50 Hz with atomic CSV flushing.
* **`teleop.py`**: 6-DOF resolved-rate keyboard teleoperation with real-time orientation locking, auto-tangent pitch (`g`), and hold-to-undo (`z`).
* **`merge_segments.py`**: Merges multiple teaching segments, checks endpoint gaps, applies Savitzky-Golay filtering, and resamples into uniform 2 mm arc increments.
* **`calibrate_and_optimize.py`**: Analyzes recorded points, aligns wire plane yaw, optimizes feedrates, and outputs forward (`calibrated_wire_LR.csv`) and return (`calibrated_wire_RL.csv`) files.
* **`goto_pose.py`**: Recovers arm to previous capture poses or home.
* **`selftest.py` & `selftest_teleop.py`**: Hardware-free pipeline and simulator tests.

### 3.4 Python Core Modules (`buzzwire_calib/`)
* **`backends.py`**: Hardware interfaces (ROS 2 Humble real cobot vs. PyKDL offline simulated arm).
* **`replay_core.py`**: Resolved-rate DLS execution engine with carrot leash stall monitoring.
* **`teleop_core.py`**: Keyboard velocity integrator with software joint limits and Cartesian bounding box clamps.
* **`trajectory.py`**: Cubic Hermite and quintic interpolation, arc-length resampling, Savitzky-Golay smoothing.
* **`simarm.py`**: 6-DOF kinematic forward and differential simulator for offline development without hardware.
* **`ros_io.py`**: ROS 2 publisher/subscriber wrappers (`/joint_states`, `/velocity_controller/commands`).

### 3.5 Session Data (`data/`)
Contains recorded sessions with atomic flush architecture:
* **`ACTIVE_SESSION`**: Text file pointing to the currently active session folder.
* **`session_20261116_104756/`** (Active Master Session):
  - `calibrated_wire_LR.csv`: Calibrated forward trajectory (Left to Right, $982\text{ mm}$ arc length).
  - `calibrated_wire_LR_smooth.csv`: Spline-smoothed version with continuous curvature.
  - `calibrated_wire_RL.csv`: Return trajectory (Right to Left).
  - `calibration_metrics.json`: Fitted geometric tolerances, gap statistics, and speed profiles.
  - `segments/`: Raw timestamped recording segments (`seg_000.csv`, `seg_001.csv`, etc.).

### 3.6 Dedicated Documentation Guides (`docs/` & root)
1. **`01_QUICKSTART_OPERATOR_GUIDE.md`**: Laboratory cheatsheet, terminal workflow, keyboard shortcuts.
2. **`02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md`**: Cobot IP (`192.168.1.25`), daemon bring-up, ROS 2 topics.
3. **`03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md`**: Hard movement stall recovery, leash timeout, backtrack undo (`z`).
4. **`04_CONTROL_MATHEMATICS_AND_TRAJECTORY_PROCESSING.md`**: SVD DLS kinematics, Nakamura damping, quintic velocity scaling.
5. **`05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md`**: Tool center point alignment, speed ladders (20% to 100%), competition strategy.
6. **`06_RECALIBRATE_FROM_SCRATCH_AND_VIDEO.md`**: Re-teaching from scratch, data collection, and video recording.
7. **`Operations.md`**: Operational protocols and check routines.

### 3.7 Submission Videos (`videos/`)
* **[`videos/`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos)**: Upload destination for Buzz Wire demonstration videos (primary: `task3_buzzwire_full_run.mp4`, smooth takes: `task3_lr_smooth.mp4`, `task3_rl_smooth.mp4`). Also accessible via symlink `buzzwire_calib/videos`. See [`videos/README.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos/README.md).

---

## 4. Hardware Specifications & Bringup Suites (`T1 2/device/` & `T1 2/openmanipulator_test/`)

These Task 1 resources are organized directly inside `T1 2/`:

### 4.1 `T1 2/device/` (Hardware Specs & Watchdog)
Hardware definitions, camera geometry, and safety guard:
* **[`device/ROBOT_INFO.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/ROBOT_INFO.md)**: OpenMANIPULATOR-X physical link lengths, limits, and servo pinouts.
* **[`device/CAMERA_INFO.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/CAMERA_INFO.md)**: RealSense D435i resolutions, intrinsics, and baseline geometry.
* **[`device/SAFETY_AND_PROTECTION.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/SAFETY_AND_PROTECTION.md)**: Safety limits, voltage thresholds, and thermal shutdown protocols.
* **[`device/hardware_protector.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/hardware_protector.py)**: Continuous background daemon monitoring servo bus voltage ($10.5\text{V} - 14.6\text{V}$), motor coil temperatures ($< 65^\circ\text{C}$), and over-current trip conditions.
* **[`device/images/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/images)**: Reference photographs of real arm setups, peg/socket boards, and demonstration videos.

### 4.2 `T1 2/openmanipulator_test/` (Low-Level Bringup)
Low-level direct Dynamixel SDK routines:
* `01_scan_and_ping.py`: Verifies RS-485 bus communication with all 5 servos.
* `02_gentle_gripper_test.py`: Gentle 13° gripper open/close safety test.
* `03_gentle_joint_nudge.py`: Nudges each joint sequentially to confirm motor direction.
* `04_keyboard_teleop.py`: Manual joint jogger for initial mechanical alignment.
* `05_go_to_home_pose.py`: Safely parks arm in zero-torque rest configuration.
* `06_test_pick_and_place.py`: Hardcoded pick-and-place verification.

---

## 5. Buffer & Archive Directory (`dump/`)

All files in [`dump/`](file:///Users/aai/Desktop/ITR_Proj/dump) are safely preserved for historical reference and buffer recovery:
* **`dump/images_buffer/`**: 25 legacy debug images from early OpenCV tuning (`diff_pan.jpg`, `clahe_enhanced.jpg`, etc.).
* **`dump/scratch_scripts/`**: 6 early test scripts and logs (`calibrate_and_test_arm.py`, `check_hardware.py`, `sweep_grid.py`, `hardware_status.txt`, `MUJOCO_LOG.TXT`, `repare.md`).
* **`dump/archives_backup/`**: Root ZIP backups (`G9123.zip`, `T1 2.zip`, `T3.zip`).
* **`dump/t3_buffer/`**: Superseded Nov 15 unzipped folder `Task2_ITR-main/` and its ZIP archive.
