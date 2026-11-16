# Task 3: The Wire-Trace Trial – Robot Calibration & Navigation Stack

> [!IMPORTANT]
> **ACTIVE WORKSPACE NOTICE**:
> This directory (`T3/`) contains early photogrammetric calibration references, ruler measurements, and baseline guides from November 12.
> **The active, tested, and maintained workspace for Task 3 is [`T3 2/`](file:///Users/aai/Desktop/ITR_Proj/T3%202)** (direct working codebase: [`T3 2/buzzwire_calib`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib)).
> All autonomous replay runs, teleoperation, and live trajectory teaching should be performed in [`T3 2/buzzwire_calib`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib).

> **Comprehensive Master Documentation**: Multi-camera hand-eye calibration, fault-tolerant free-drive data collection, and 3D Buzz Wire trajectory navigation for the **Addverb HEAL 6-DOF Collaborative Robot**.

---

## 1. Quick Navigation to Dedicated Documentation

Because team members will shift logins between different laptops and the robot controller, all operational context is fully documented in self-contained guides:

| Guide | File Link | What It Covers |
| :--- | :--- | :--- |
| **01. Quickstart Operator Guide** | [01_QUICKSTART_OPERATOR_GUIDE.md](file:///Users/aai/Desktop/ITR_Proj/T3/01_QUICKSTART_OPERATOR_GUIDE.md) | Terminal cheat sheet, keybindings (`SPACE`, `n`, `u`, `f`, `q`, `s`), operator golden rules. |
| **02. Robot Controls & ROS Setup** | [02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md](file:///Users/aai/Desktop/ITR_Proj/T3/02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md) | HEAL cobot IP (`192.168.1.25`), `./cobot run`, `./free_drive`, ROS 2 launch, KDL chain `base_link` -> `end-effector`. |
| **03. Stuck Arm & Edge-Case Playbook** | [03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md](file:///Users/aai/Desktop/ITR_Proj/T3/03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md) | Resolving hard movement trips, session crash auto-resume, smooth arm return with `goto_pose.py`. |
| **04. Calibration Math & Averaging** | [04_CALIBRATION_MATRICES_AND_AVERAGING.md](file:///Users/aai/Desktop/ITR_Proj/T3/04_CALIBRATION_MATRICES_AND_AVERAGING.md) | Burst frame averaging, chordal SE(3) pose mean, dual-camera joint solver, `new_ref` matrix analysis. |
| **05. Buzz Wire Navigation Integration** | [05_BUZZ_WIRE_NAVIGATION_INTEGRATION.md](file:///Users/aai/Desktop/ITR_Proj/T3/05_BUZZ_WIRE_NAVIGATION_INTEGRATION.md) | Ground frame $\{G\}$, 31 waypoints, tool pitch $\alpha$, $9.0\text{ mm}$ safe tube, DLS velocity control. |

---

## 2. Directory Structure

```
/Users/aai/Desktop/ITR_Proj/T3/
├── README.md                                    # Master repository documentation (this file)
├── 01_QUICKSTART_OPERATOR_GUIDE.md              # Operator field manual & keyboard shortcuts
├── 02_ROBOT_CONTROLS_AND_SYSTEM_SETUP.md        # HEAL robot daemon, ROS2 topics, KDL chain
├── 03_STUCK_ARM_RECOVERY_AND_EDGE_CASES.md      # Stuck arm recovery & edge case protection
├── 04_CALIBRATION_MATRICES_AND_AVERAGING.md     # Mathematical formulation of calibration
├── 05_BUZZ_WIRE_NAVIGATION_INTEGRATION.md      # Ground frame registration & 31 waypoints
│
├── buzzwire_calib/                              # Complete calibration toolkit
│   ├── run_collect.sh                           # Auto-detecting ROS2 launcher for data collection
│   ├── requirements.txt                         # Python dependencies
│   ├── config/
│   │   ├── calib_config.yaml                    # Hardware configuration (cameras, robot, board)
│   │   └── buzz_wire_calibration.json           # 31 waypoints & apparatus geometry
│   ├── scripts/
│   │   ├── collect_data.py                      # Interactive capture node (SPACE, n, u, f, q)
│   │   ├── goto_pose.py                         # Quintic velocity recovery tool (--last, --home)
│   │   ├── calibrate_extrinsics.py              # Joint dual-camera hand-eye solver
│   │   ├── register_buzzwire.py                 # Maps 31 waypoints to robot base frame
│   │   ├── capture_intrinsics.py                # Lens calibration capture tool
│   │   ├── calibrate_intrinsics.py              # Intrinsic distortion parameter optimizer
│   │   ├── make_board.py                        # ChArUco board rendering script
│   │   ├── selftest_solver.py                   # Math solver unit test suite
│   │   └── selftest.py                          # Full end-to-end simulated collector & solver test
│   └── buzzwire_calib/                          # Core Python library
│       ├── collector.py                         # Burst sampling & jitter detection engine
│       ├── session.py                           # Append-only crash-resilient event store
│       ├── handeye.py                           # Joint non-linear optimizer & outlier rejector
│       ├── transforms.py                        # SE(3) chordal pose averaging
│       ├── board.py                             # ChArUco & OpenCV detector wrappers
│       ├── cameras.py                           # Auto-reconnecting threaded video sources
│       └── ros_io.py                            # ROS2 KDL forward kinematics & topics
│
├── Project3 Instructions/                       # Reference robot code from lab
│   ├── README                                   # Original robot command instructions
│   ├── heal_end_effector_pub_node.py            # Latched URDF / KDL FK node
│   ├── utils/
│   │   ├── dls_velocity_commander.py            # Damped Least Squares Cartesian controller
│   │   └── quintic_planner.py                   # 5th-order polynomial trajectory generator
│   └── Dual_Arm/
│       ├── homing_codes/heal_going_home.py      # HEAL cobot zero-homing node
│       └── sample_ik/heal_dls_ik.py             # DLS inverse kinematics solver
│
├── Buzz Wire Robot Navigation Calibration/      # Photogrammetric calibration data
│   ├── buzz_wire_robot_navigation_calibration_guide.md  # Detailed measurement report
│   ├── buzz_wire_robot_navigation_calibration_data.md   # Apparatus dimensions & waypoints
│   └── *.jpeg                                   # 14 high-res reference photos with steel ruler
│
├── new_ref/                                     # Lab matrix photos showing board placements
└── buzzwire_calib_package.zip                   # Complete self-contained deployment bundle
```

---

## 3. End-to-End Workflow in 4 Steps

### Step 1: Physical Setup & Intrinsics
1. Print the ChArUco calibration board using `python3 buzzwire_calib/scripts/make_board.py` at 100% scale. Glue it to a rigid flat acrylic or MDF backing plate.
2. Measure the physical printed square with digital calipers (e.g., $39.5\text{ mm}$) and enter it in `buzzwire_calib/config/calib_config.yaml`.
3. If not already done, calibrate lens intrinsics for both cameras:
   ```bash
   python3 buzzwire_calib/scripts/capture_intrinsics.py
   python3 buzzwire_calib/scripts/calibrate_intrinsics.py
   ```

### Step 2: Free-Drive Data Collection
1. **Terminal 1 (Robot Arm)**:
   ```bash
   ssh cobot@192.168.1.25
   cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
   sudo env HOME=/home/cobot ./cobot run     # Type 1, press ENTER
   ./free_drive
   ```
2. **Terminal 2 (ROS Laptop)**:
   ```bash
   cd buzzwire_calib
   ./run_collect.sh
   ```
   - Guide the arm so both cameras see the board clearly.
   - Press **`SPACE`** (or **`c`**): captures a 12-frame burst, verifies stillness (< 4 mrad), and saves the averaged data.
   - Rotate the arm tool $\ge 45^\circ$ across two axes and press **`SPACE`** again (take 3 to 6 poses).
   - Press **`n`** (Next Matrix): move the physical board to a new table location.
   - Repeat across $\ge 4$ matrices, then press **`f`** (FINISH).

### Step 3: Solve Extrinsics
```bash
python3 scripts/calibrate_extrinsics.py
```
Outputs in `results/<session>/`:
- `extrinsics.json`: All calibrated transforms ($T^{\text{ee}}_{\text{cam0}}$, $T^{\text{base}}_{\text{cam1}}$, $T^{\text{cam1}}_{\text{cam0}}$).
- `publish_tf.sh`: Ready-to-run ROS 2 static TF transform publishers.

### Step 4: Buzz Wire Trajectory Navigation
1. Register the Buzz Wire apparatus to the robot base frame using 3-point touch or camera detection:
   ```bash
   python3 scripts/register_buzzwire.py --touch-fl X1 Y1 Z1 --touch-fr X2 Y2 Z2 --touch-rl X3 Y3 Z3
   ```
2. Outputs `results/buzz_wire_robot_trajectory.json` containing all 31 waypoints with dynamic loop pitch orientations $\alpha(s)$.
3. Replay with the DLS velocity commander (`Project3 Instructions/utils/dls_velocity_commander.py`) to trace the wire without contact.

---

## 4. Verification Test Status

Both comprehensive automated test suites have been executed and passed:
1. `python3 scripts/selftest_solver.py`:
   - Validates all 4 camera/board mount combinations (eye-in-hand, eye-to-hand, dual fixed, dual hand).
   - Verifies rejection of unobservable setups.
   - **Result**: `SOLVER SELFTEST PASSED` (worst-case error $< 0.05^\circ, < 1.2\text{ mm}$).
2. `python3 scripts/selftest.py`:
   - Simulates robot arm + 2 cameras + moving board across 5 matrices (29 captures).
   - Verifies crash/resume, undo, stale-robot detection, arm jitter rejection, and full joint calibration.
   - **Result**: `SELFTEST PASSED` (Hand cam error: $0.16^\circ, 0.79\text{ mm}$; Fixed cam error: $0.16^\circ, 4.0\text{ mm}$; Stereo consistency: $0.13^\circ, 0.37\text{ mm}$).

