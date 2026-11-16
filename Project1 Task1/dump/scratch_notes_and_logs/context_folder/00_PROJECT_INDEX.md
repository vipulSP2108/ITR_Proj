# Tri-Wizard Robotics — Task 1: Peg Sorting — Context Folder Index

> **Definitive reference for every function, module, and core challenge in the project.**
> Auto-generated from full source analysis. Each linked file is a standalone reference.

---

## Project Overview

| Item | Detail |
|------|--------|
| **Competition** | Tri-Wizard Robotics |
| **Task** | Task 1 — The Peg Sorting |
| **Robot** | ROBOTIS OpenMANIPULATOR-X (RM-X52-TNM), 4 DOF arm + 1 gripper |
| **Actuators** | Dynamixel XM430-W350-T × 5, Protocol 2.0, 1 Mbps |
| **Camera** | Intel RealSense D435i (640×480, overhead at ~470 mm) |
| **Simulator** | MuJoCo physics engine with mounted camera |
| **Peg–Socket Mapping** | Circle→Green, Square→Blue, Triangle→Yellow, Hexagon→Red/Orange |
| **Project Duration** | 6 days (2026-11-03 to 2026-11-08) |
| **Max Score** | 120 points |

---

## Module Documentation

### Vision Pipeline (`src/vision/`)
- [color_segmentation.md](vision/color_segmentation.md) — HSV color masking with dual-range hue wrap for red
- [shape_detection.md](vision/shape_detection.md) — Shape classification via enclosing-circle ratio + vertex count, 3D pose
- [single_camera_pipeline.md](vision/single_camera_pipeline.md) — Full perception pipeline (segment → detect → deproject → transform)
- [dynamic_workspace_perceiver.md](vision/dynamic_workspace_perceiver.md) — Self-assessing vision for movable board position
- [visual_drop_detector.md](vision/visual_drop_detector.md) — Vision-based drop/slip detection (EC-05)
- [camera_drift_monitor.md](vision/camera_drift_monitor.md) — Online camera recalibration via Kabsch SVD (EC-07)
- [synthetic_scene.md](vision/synthetic_scene.md) — Synthetic RGB-D scene generator for offline testing
- [dual_camera_manager.md](vision/dual_camera_manager.md) — Multi-threaded dual camera capture architecture

### Control (`src/control/`)
- [dynamixel_wrapper.md](control/dynamixel_wrapper.md) — Safe Dynamixel motor interface (port detection, signed 32-bit, multi-turn)
- [gripper_guard.md](control/gripper_guard.md) — Proprioceptive grasp monitoring + spiral search for jam recovery
- [teleop_keyboard.md](control/teleop_keyboard.md) — Interactive keyboard teleoperation with ANSI dashboard

### Planning (`src/planning/`)
- [kinematics.md](planning/kinematics.md) — Analytical closed-form FK/IK (L1=77, L2=130, L3=124, L4=126 mm)
- [calibration.md](planning/calibration.md) — Eye-to-hand calibration via Kabsch/Umeyama SVD
- [recovery_manager.md](planning/recovery_manager.md) — Master fault recovery engine (EC-05 through EC-09)
- [workspace_analyzer.md](planning/workspace_analyzer.md) — 4-tier reachability analysis + angled grasp + inward sweep
- [autonomous_homography_calibrator.md](planning/autonomous_homography_calibrator.md) — Self-calibration via novel gripper-flick TCP detection

### Simulation (`sim_demo/`)
- [sim_arm_model.md](simulation/sim_arm_model.md) — Analytic kinematics (metres/radians, L-pose zero convention)
- [sim_env.md](simulation/sim_env.md) — MuJoCo environment with minimum-jerk interpolation
- [sim_scene_builder.md](simulation/sim_scene_builder.md) — MJCF XML generation from YAML config
- [sim_planner.md](simulation/sim_planner.md) — Constant-tilt waypoint planning with feasibility sweep (0–60°)
- [sim_trajectory.md](simulation/sim_trajectory.md) — Quintic polynomial trajectory + DLS-IK fallback solver
- [sim_perception.md](simulation/sim_perception.md) — Color-mask perception with RANSAC plane fitting + shape classification
- [sim_workspace.md](simulation/sim_workspace.md) — Reachability grid analysis (min tilt per cell)
- [sim_pipeline.md](simulation/sim_pipeline.md) — Master 3-stage pipeline (HOME → CALIBRATE/WORKSPACE → PICK & PLACE)
- [sim_tracker.md](simulation/sim_tracker.md) — 25 Hz vision-based path verification with dynamic tolerance
- [sim_multi_shape.md](simulation/sim_multi_shape.md) — Multi-shape sorting: 4-DOF yaw alignment via base pan
- [sim_robot_interface.md](simulation/sim_robot_interface.md) — HAL abstraction (SimRobot / RealRobot)
- [sim_drift_monitor.md](simulation/sim_drift_monitor.md) — Online camera extrinsic drift detection
- [sim_grid_sweep.md](simulation/sim_grid_sweep.md) — 12×12 batch grid testing with heatmap + HTML gallery

### Configuration & Calibration
- [config_colors.md](config/config_colors.md) — HSV color thresholds and morphology settings
- [config_workspace.md](config/config_workspace.md) — Physical dimensions, camera params, manipulation heights
- [config_calibration.md](config/config_calibration.md) — Camera-to-robot 3D rigid transform (R, t, RMSE 0.85 mm)
- [config_homography.md](config/config_homography.md) — Planar homography (RMSE 0.12 mm)

### Hardware Reference
- [robot_info.md](hardware/robot_info.md) — OpenMANIPULATOR-X specs, kinematic chain, joint config, home pose
- [camera_info.md](hardware/camera_info.md) — RealSense D435i specs, mounting, calibration data
- [objects_and_board.md](hardware/objects_and_board.md) — Peg dimensions, socket board, grasping constraints
- [safety_protection.md](hardware/safety_protection.md) — Electrical safety, HardwareProtector watchdog, safety layers

### Demos & Results
- [live_hardware_sorter.md](demos/live_hardware_sorter.md) — Autonomous live hardware circle-peg sorting demo
- [hardware_pipeline.md](demos/hardware_pipeline.md) — Full 4-stage physical hardware pipeline
- [demo_runs.md](demos/demo_runs.md) — Timestamped demo results: circle (1.3mm error), square (6.4mm error)

### Tests
- [test_overview.md](tests/test_overview.md) — Test suite: vision pipeline, edge case recovery, FK/IK roundtrip

### Challenges & Architecture
- [architecture_overview.md](challenges/architecture_overview.md) — System architecture and data flow diagram
- [challenge_edge_cases.md](challenges/challenge_edge_cases.md) — 15 edge cases across 4 priority tiers (P0–P3)
- [challenge_reachability.md](challenges/challenge_reachability.md) — Workspace reachability and tilt strategy
- [challenge_calibration.md](challenges/challenge_calibration.md) — Camera-robot calibration approaches (Kabsch, homography, gripper-flick)
- [challenge_multi_shape.md](challenges/challenge_multi_shape.md) — Multi-shape sorting with 4-DOF orientation constraint
- [sim_vs_hardware.md](challenges/sim_vs_hardware.md) — **Critical**: Sim-to-real gap, unit mismatch, joint-zero offset, bridge strategy
- [project_timeline.md](challenges/project_timeline.md) — 6-day project evolution: hardware → vision → sim → demo → grid sweep
- [constraint_constants.md](challenges/constraint_constants.md) — All physical constants, thresholds, and magic numbers in one place

---

## Key Numbers at a Glance

| Parameter | Value | Source |
|-----------|-------|--------|
| Link lengths (L1–L4) | 77, 130, 124, 126 mm | `kinematics.py` |
| Camera height | 470 mm | `workspace_dimensions.yaml` |
| Focal length | 344 px | `workspace_dimensions.yaml` |
| Scale | 1.366 mm/px | 470 / 344 |
| Encoder resolution | 4096 counts = 360° | Dynamixel XM430 |
| Grasp height | 35 mm (hw) / 45 mm below top (sim) | Config / planner |
| Insert height | 28 mm | `workspace_dimensions.yaml` |
| Transit height | 105 mm (hw) / 30 mm clearance (sim) | Config / planner |
| Vertical reach max | ~218 mm | `workspace_analyzer.py` |
| Angled reach (60° tilt) | ~350 mm | `repare.md` |
| Camera calib RMSE | 0.85 mm | `camera_calib.yaml` |
| Homography RMSE | 0.12 mm | `homography_calib.yaml` |
| Home pose (raw) | [0/4096, 2048, 2048, 2048, 3130] | `calibrated_home.yaml` |
| Circle demo error | 1.3 mm | Demo run Nov 7 |
| Square demo error | 6.4 mm | Demo run Nov 8 |
| Tracking (circle) | 100% on-track, mean 4.1mm | Demo run Nov 7 |

---

## File Count Summary

| Directory | Files | Coverage |
|-----------|-------|----------|
| `vision/` | 8 | All 8 vision modules |
| `control/` | 3 | Dynamixel wrapper, gripper guard, teleop |
| `planning/` | 5 | Kinematics, calibration, recovery, workspace, homography |
| `simulation/` | 13 | Full sim_demo pipeline + grid sweep + HAL |
| `config/` | 4 | Colors, workspace, 3D calibration, homography |
| `hardware/` | 4 | Robot, camera, objects/board, safety |
| `demos/` | 3 | Live hardware, pipeline, run results |
| `tests/` | 1 | Test suite overview |
| `challenges/` | 8 | Edge cases, reachability, calibration, multi-shape, sim-vs-hw, timeline, constants, architecture |
| **Total** | **50** | **Complete project coverage** |
