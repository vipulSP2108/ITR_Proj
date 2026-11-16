# Task 2: The Wire-Trace Trial – Robot Calibration & Navigation Stack

> **Comprehensive Master Repository Documentation**: 6-DOF resolved-rate Cartesian teleoperation, gravity-compensated free-drive data collection, fault-tolerant trajectory merging, and high-speed autonomous 3D Buzz Wire replay for the **Addverb HEAL 6-DOF Collaborative Robot**.
>
> **Notice**: Cameras are **NOT** permitted for Task 2. All trajectory teaching, kinematics resolution, and replay are executed strictly in the robot base frame using joint encoders and KDL forward kinematics. (The earlier camera calibration and photogrammetric setup from Trial 3 is preserved for historical reference under `T3/` and `buzzwire_calib/docs/08_LEGACY_CAMERA_CALIBRATION.md`).

---

## 1. Quick Navigation to Dedicated Documentation

All technical depth, mathematical formulations, operational cheat sheets, and recovery playbooks are documented in five dedicated master guides:

| Guide | File Link | What It Covers |
| :--- | :--- | :--- |
| **01. Quickstart Operator Guide** | [01_QUICKSTART_OPERATOR_GUIDE.md](file:///Users/aai/Desktop/Task2_ITR/01_QUICKSTART_OPERATOR_GUIDE.md) | Terminal cheat sheet, dual-terminal workflow, keyboard shortcuts (`w/s/a/d/r/f`, `SPACE`, `v`, `z`, `g`), operator golden rules. |
| **02. Robot Controls & System Setup** | [02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md](file:///Users/aai/Desktop/Task2_ITR/02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md) | HEAL cobot IP (`192.168.1.25`), `./cobot run`, `./free_drive`, `./heal_server`, ROS 2 launch, KDL kinematics chain, offline simulation. |
| **03. Stuck Arm Recovery Playbook** | [03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md](file:///Users/aai/Desktop/Task2_ITR/03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md) | Resolving hard movement trips, stale robot state recovery, carrot leash stall protection, live backtrack undo (`z`), quintic recovery. |
| **04. Control Math & Trajectory Processing** | [04_CONTROL_MATHEMATICS_AND_TRAJECTORY_PROCESSING.md](file:///Users/aai/Desktop/Task2_ITR/04_CONTROL_MATHEMATICS_AND_TRAJECTORY_PROCESSING.md) | Resolved-rate DLS, Nakamura adaptive damping, Savitzky-Golay filtering, 2 mm arc resampling, time-optimal velocity profiling. |
| **05. Buzz Wire Trajectory Execution** | [05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md](file:///Users/aai/Desktop/Task2_ITR/05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md) | Wire stand geometry, frame alignment (`jog_frame_yaw_deg`, `tool_axis`), workspace bounding box, speed ladder (20% to 100%), competition strategy. |

---

## 2. Directory Structure

```
/Users/aai/Desktop/Task2_ITR/
├── README.md                                     # Master repository documentation (this file)
├── 01_QUICKSTART_OPERATOR_GUIDE.md               # Operator laboratory manual & shortcuts
├── 02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md         # Network credentials, daemons, ROS 2 topics
├── 03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md       # Fault matrix, leash protection, backtrack
├── 04_CONTROL_MATHEMATICS_AND_TRAJECTORY_PROCESSING.md # DLS math, SVD, filtering, velocity profiling
├── 05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md # Frames, bounding box, speed ladder strategy
│
├── buzzwire_calib/                               # Complete Task 2 Buzz Wire toolkit
│   ├── README.md                                 # Package documentation
│   ├── buzzwire_teach.sh                         # Master unified CLI launcher
│   ├── requirements.txt                          # Python dependencies
│   ├── run_collect.sh                            # Sourced ROS 2 launch script
│   ├── config/
│   │   └── calib_config.yaml                     # Hardware, teleop speeds, box limits, replay config
│   ├── scripts/
│   │   ├── teleop.py                             # 6-DOF keyboard driving & path recording
│   │   ├── record_segment.py                     # Free-drive hand teaching recorder
│   │   ├── merge_segments.py                     # Stitch, trim contacts, smooth, resample
│   │   ├── inspect_segments.py                   # Segment metrics, gap detection, ASCII coverage
│   │   ├── replay_trajectory.py                  # Autonomous closed-loop trajectory replay
│   │   ├── goto_pose.py                          # Quintic polynomial recovery tool (--home, --sim)
│   │   ├── selftest_teleop.py                    # 10 controller unit tests (leash, SVD, box, undo)
│   │   ├── selftest_pipeline.py                  # Simulated merge & replay validation
│   │   ├── selftest_solver.py                    # Kinematics solver test suite
│   │   └── selftest.py                           # Full simulated session and crash-resume test
│   ├── buzzwire_calib/                           # Core Python robotics library
│   │   ├── teleop_core.py                        # 6-DOF resolved-rate DLS & carrot controller
│   │   ├── replay_core.py                        # Multi-constraint time-optimal velocity follower
│   │   ├── trajectory.py                         # SegmentWriter, Waypoint, Savitzky-Golay merger
│   │   ├── backends.py                           # Unified RosBackend & SimBackend abstraction
│   │   ├── simarm.py                             # Analytical UR5/HEAL simulation model & FK
│   │   ├── keyboard.py                           # Terminal & pynput event listeners
│   │   ├── gripper.py                            # Asynchronous non-blocking Dynamixel client
│   │   ├── ros_io.py                             # ROS 2 node, KDL forward kinematics & Jacobian
│   │   └── session.py                            # Append-only crash-resilient session store
│   └── docs/                                     # Dedicated knowledge base & troubleshooting guides
│
└── T3/                                           # Trial 3 archive (camera calibration & photogrammetry)
```

---

## 3. End-to-End Workflow in 4 Steps

```
+-----------------------------------------------------------------------------------------+
| Step 1: TEACH A PATH (Method 1 Free-Drive OR Method 2 Keyboard Teleop)                  |
|   Terminal 1 (SSH cobot@192.168.1.25): ./cobot run -> ./free_drive OR ./heal_server     |
|   Terminal 2 (Laptop): ./buzzwire_teach.sh [record | teleop | sim]                      |
+-----------------------------------------------------------------------------------------+
                                             │
                                             v
+-----------------------------------------------------------------------------------------+
| Step 2: AUDIT & INSPECT RECORDED SEGMENTS                                               |
|   Terminal 2: ./buzzwire_teach.sh inspect [--plot]                                      |
|   Checks segment continuity, detects gaps, displays ASCII coverage vs ~1.0 m wire.      |
+-----------------------------------------------------------------------------------------+
                                             │
                                             v
+-----------------------------------------------------------------------------------------+
| Step 3: MERGE, SMOOTH & RESAMPLE TRAJECTORY                                             |
|   Terminal 2: ./buzzwire_teach.sh merge [--clearance 0.003]                             |
|   Trims 8 mm contact zones, bridges gaps, applies Savitzky-Golay jitter filter, resamples|
|   at 2 mm arc steps to create data/session_*/merged_trajectory.csv.                     |
+-----------------------------------------------------------------------------------------+
                                             │
                                             v
+-----------------------------------------------------------------------------------------+
| Step 4: AUTONOMOUS HIGH-SPEED REPLAY                                                    |
|   Terminal 1: ./cobot run -> ./heal_server + ros2 velocity_control launch               |
|   Terminal 2: ./buzzwire_teach.sh replay --dry-run                                      |
|               ./buzzwire_teach.sh replay --speed 0.2  (First pass, then 0.4 -> 1.0)     |
+-----------------------------------------------------------------------------------------+
```

---

## 4. Verification Test Status

All four comprehensive automated test suites have been executed and verified:

1. **Teleop Core Unit Tests (`python3 scripts/selftest_teleop.py`)**:
   - 10 automated test cases: Translation, orientation holding, pitch rotation, combined 6-DOF twists, workspace box clipping, backtrack breadcrumbs, auto-tangent alignment, carrot leash lagging, singularity damping, acceleration slew limiting.
   - **Result**: `TELEOP CORE SELFTEST PASSED` (worst drift $< 0.01\text{ mm}$, zero jerk).

2. **Pipeline End-to-End Test (`python3 scripts/selftest_pipeline.py`)**:
   - Generates synthetic 3D wire segments with contacts and overlaps $\rightarrow$ merges $\rightarrow$ executes simulation replay at speeds 0.4 and 1.0.
   - **Result**: `PIPELINE SELFTEST PASSED` (Speed 1.0 executed in $22.3\text{ s}$ with worst tracking error of **$1.69\text{ mm}$**, well within the safe ring aperture).

3. **Solver & Kinematics Test (`python3 scripts/selftest_solver.py`)**:
   - Validates numerical Jacobian and kinematics across all mounting configurations.
   - **Result**: `SOLVER SELFTEST PASSED` (worst error $< 0.045^\circ$, $< 0.69\text{ mm}$).

4. **Crash-Resilience & Undo Test (`python3 scripts/selftest.py`)**:
   - Tests 29 captures across 5 matrices, simulated terminal crash and restart, and undo mechanics.
   - **Result**: `SELFTEST PASSED` (Hand cam error: $0.21^\circ, 0.51\text{ mm}$; Fixed cam error: $0.12^\circ, 3.26\text{ mm}$).

5. **Smooth Quintic Recovery (`python3 scripts/goto_pose.py --home --sim --yes`)**:
   - Closed-loop quintic trajectory from home state to all-zeros.
   - **Result**: `Arrived.` in $12.8\text{ s}$ with zero jerk and zero overshoot.
