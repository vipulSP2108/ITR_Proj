# Task 3: The Wire-Trace Trial (Active Tested Workspace)

This directory ([`T3 2/`](file:///Users/aai/Desktop/ITR_Proj/T3%202)) is the **primary tested workspace** for Task 3 of the Tri-Wizard Robotics Competition: Autonomous 3D Buzz Wire Traversal using the Addverb HEAL 6-DOF Collaborative Robot.

> [!IMPORTANT]
> **Working Codebase Location**:
> All active scripts, configuration files, and recorded trajectory sessions live in:
> [`T3 2/buzzwire_calib`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib)
> *(which links directly to `T3 2/NEW/Task2_ITR-main (1)/Task2_ITR-main/Task2/taken/G9/Task2_ITR-main/buzzwire_calib`)*.
> **DO NOT modify the internal files under this path**—they contain the validated, lab-tested trajectory data!

---

## 1. Directory Structure

```
/Users/aai/Desktop/ITR_Proj/T3 2/
├── README.md                      # This master guide
├── buzzwire_calib -> ...          # Symlink to active working codebase
├── analysis_and_dashboards -> ... # Symlink to 3D trajectory dashboard & HTML reports
├── dump/                          # Task 3 internal buffer (superseded Nov 15 release)
│
├── Project3 Instructions/         # Official course specification, URDF nodes & IK references
│   ├── README                     # Cobot controller startup commands
│   ├── heal_end_effector_pub_node.py # KDL forward kinematics ROS 2 node
│   └── utils/                     # Quintic planner & DLS commander libraries
│
└── NEW/                           # Tested working codebase tree
    └── Task2_ITR-main (1)/
        └── Task2_ITR-main/
            └── Task2/
                └── taken/
                    └── G9/
                        └── Task2_ITR-main/
                            └── buzzwire_calib/ # << PRIMARY TESTED WORKING CODE
```

---

## 2. Dedicated Documentation Library

The active working folder ([`buzzwire_calib/`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib)) contains six comprehensive engineering guides:

| Document | Link | Description |
| :--- | :--- | :--- |
| **01. Quickstart Operator Guide** | [`01_QUICKSTART_OPERATOR_GUIDE.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/01_QUICKSTART_OPERATOR_GUIDE.md) | Terminal cheat sheet, dual-terminal workflow, keyboard shortcuts (`w/s/a/d/r/f`, `SPACE`, `v`, `z`, `g`), golden rules. |
| **02. Robot Controls & Setup** | [`02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md) | HEAL cobot IP (`192.168.1.25`), `./cobot run`, `./free_drive`, `./heal_server`, ROS 2 launch, KDL kinematics chain. |
| **03. Stuck Arm Recovery** | [`03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md) | Resolving hard movement trips, stale robot state recovery, carrot leash stall protection, live backtrack undo (`z`). |
| **04. Control Math & Trajectory** | [`04_CONTROL_MATHEMATICS_AND_TRAJECTORY_PROCESSING.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/04_CONTROL_MATHEMATICS_AND_TRAJECTORY_PROCESSING.md) | Resolved-rate DLS, Nakamura adaptive damping, Savitzky-Golay filtering, 2 mm arc resampling, time-optimal velocity profiling. |
| **05. Trajectory Execution** | [`05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/05_BUZZ_WIRE_TRAJECTORY_EXECUTION_AND_REPLAY.md) | Wire stand geometry, frame alignment, workspace bounding box, speed ladder (20% to 100%), competition strategy. |
| **06. Recalibrate From Scratch** | [`06_RECALIBRATE_FROM_SCRATCH_AND_VIDEO.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/06_RECALIBRATE_FROM_SCRATCH_AND_VIDEO.md) | Full guide to re-teaching the wire, rebuilding CSVs, and filming official demonstration video. |
| **Operations Protocol** | [`Operations.md`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib/Operations.md) | Lab operation checklists, safety protocols, and cross-shift handover notes. |

---

## 3. High-Level Command Cheat Sheet

Navigate into the active working folder:
```bash
cd "/Users/aai/Desktop/ITR_Proj/T3 2/buzzwire_calib"
```

### A. Autonomous Traversal Execution (`buzzwire_run.sh`)
```bash
./buzzwire_run.sh goto L                  # Drive arm safely to Left Start Post
./buzzwire_run.sh lr show                 # Demo speed: 24 mm/s (~69s duration)
./buzzwire_run.sh lr balanced             # Balanced speed: 39 mm/s (~55s duration)
./buzzwire_run.sh lr fast                 # Competition speed: 60 mm/s (~42s duration)
./buzzwire_run.sh rl                      # Safe return traversal (Right to Left)
./buzzwire_run.sh goto home               # Safely park arm in all-zeros home
./buzzwire_run.sh dry-run lr              # Velocity audit without moving robot
```

### B. Teaching a New Wire Path (`buzzwire_teach.sh`)
```bash
./buzzwire_teach.sh new my_session        # Create a fresh session folder
./buzzwire_teach.sh teleop                # Method 2: 6-DOF keyboard teleoperation
./buzzwire_teach.sh record                # Method 1: Gravity-compensated free-drive
./buzzwire_teach.sh inspect               # Check continuity and point-to-point gaps
./buzzwire_teach.sh merge                 # Stitch segments, smooth & resample at 2 mm
./buzzwire_run.sh calibrate               # Build forward and reverse calibrated CSVs
```

### C. Offline Simulation (No Robot Needed)
```bash
./buzzwire_teach.sh sim                   # Interactive 6-DOF simulated arm teleoperation
./buzzwire_teach.sh test                  # Run automated kinematics & solver unit tests
```

---

## 4. Emergency & Stuck Arm Recovery

* **Immediate Halt**: Press `ESC` during teleoperation or `Ctrl+C` in the terminal to immediately command zero velocity.
* **Backtrack (Live Undo)**: While driving via teleoperation, **press and hold `z`** to physically retrace steps and truncate the recording.
* **Recover from Stale / Tripped Controller**:
  1. Terminate low-level daemon on cobot controller (`Ctrl+C`).
  2. Rerun `sudo env HOME=/home/cobot ./cobot run` (type `1` + Enter).
  3. Start `./heal_server`.
  4. Run `python3 scripts/goto_target.py --target home` to return to home stance.
