# System Architecture & Data Flow

**Purpose**: How all modules connect and data flows from camera to motor commands.

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     PERCEPTION LAYER                         │
│                                                              │
│  Camera (RGB-D) → Color Segmentation → Depth Slicing        │
│                 → Shape Classification → 3D Deprojection     │
│                 → Calibration Transform → Robot Coordinates   │
│                                                              │
│  Modules: color_segmentation, shape_detection,               │
│           single_camera_pipeline, dynamic_workspace_perceiver│
└────────────────────────┬────────────────────────────────────┘
                         │ TargetPose (shape, color, robot_3d, yaw)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                      PLANNING LAYER                          │
│                                                              │
│  Reachability Analysis → Tilt Selection → Waypoint Generation│
│  Orientation Alignment → Trajectory Smoothing                │
│                                                              │
│  Modules: workspace_analyzer, kinematics, planner,           │
│           trajectory (quintic + DLS IK)                      │
└────────────────────────┬────────────────────────────────────┘
                         │ Joint angles (q1, q2, q3, q4) + gripper
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                      CONTROL LAYER                           │
│                                                              │
│  Dynamixel Wrapper → Profile Velocity → Motor Commands       │
│  Gripper Guard → Grasp Monitoring → Drop Detection           │
│                                                              │
│  Modules: dynamixel_wrapper, gripper_guard                   │
└────────────────────────┬────────────────────────────────────┘
                         │ Motor load, position feedback
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                     RECOVERY LAYER                           │
│                                                              │
│  Recovery Manager ← GripperGuard exceptions                  │
│                   ← VisualDropDetector alerts                │
│                   ← CameraDriftMonitor drift                 │
│                   → Re-scan → Re-plan → Re-execute           │
│                                                              │
│  Modules: recovery_manager, visual_drop_detector,            │
│           camera_drift_monitor                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Data Flow: Complete Pick-and-Place Cycle

```
1. HOME POSE
   └→ dynamixel_wrapper.write_position() for all joints

2. PERCEPTION
   └→ Camera captures BGR + Depth
   └→ ColorSegmenter.segment_all() → color masks
   └→ ShapeDetector.classify_contour() → shapes + orientations
   └→ depth_slice_top_face() → refined masks
   └→ Deprojection (intrinsics) → camera 3D
   └→ EyeToHandCalibrator → robot 3D
   └→ OR: DynamicWorkspacePerceiver.assess() → robot 3D (hardware path)

3. PLANNING
   └→ WorkspaceAnalyzer.classify_target() → reachability zone
   └→ Select tilt angle (0°–60°)
   └→ InverseKinematics(target, pitch) → joint angles
   └→ Generate waypoint sequence (hover → descend → grasp → lift → transit → insert → release)
   └→ For non-circles: add alignment waypoint with symmetry-aware rotation

4. EXECUTION
   └→ For each waypoint pair:
       └→ QuinticTrajectory interpolation
       └→ At each control step (20ms):
           └→ Cartesian position → IK → joint angles
           └→ dynamixel_wrapper.write_position()
       └→ GripperGuard.monitor_transit_trajectory()
       └→ If DROPPED: RecoveryManager.recover_from_drop()
       └→ If EMPTY_AIR: retry with adjusted height

5. VERIFICATION
   └→ VisualDropDetector.verify_socket_placement() → SOCKET_FILLED/EMPTY
   └→ If EMPTY: re-attempt insertion
```

---

## Two Parallel Pipelines

### Hardware Pipeline (Physical Robot)

```
run_live_hardware_sorter.py
    → DynamicWorkspacePerceiver (single RGB frame, no depth)
    → Homography pixel_to_robot() (RMSE 0.12mm)
    → kinematics.py IK (millimetres)
    → dynamixel_wrapper.py motor commands
    → Real Dynamixel XM430 actuators
```

### Simulation Pipeline (MuJoCo)

```
run_pipeline.py
    → MuJoCo RGB-D renders
    → Kabsch SVD calibration (RMSE 0.08mm)
    → arm_model.py IK (metres)
    → MuJoCo joint actuator commands
    → Physics simulation with contacts
```

### Shared Concepts, Separate Implementations

| Concept | Hardware | Simulation |
|---------|----------|-----------|
| Kinematics | `kinematics.py` (mm) | `arm_model.py` (m) |
| Calibration | Homography + Kabsch | Kabsch only |
| Perception | `DynamicWorkspacePerceiver` | `perception.py` |
| Trajectory | `smooth_move()` | `QuinticTrajectory` + DLS IK |
| Motor control | Dynamixel SDK | MuJoCo actuators |

---

## Configuration Files (Single Source of Truth)

```
config/
├── colors.yaml              # HSV thresholds for color segmentation
├── calibrated_home.yaml     # Home pose encoder values
├── camera_calib.yaml        # 3D Kabsch calibration (R, t)
├── homography_calib.yaml    # 2D homography matrix
└── workspace_dimensions.yaml # Physical measurements, camera params
```

---

## Key Architectural Decisions

1. **Analytical IK over numerical**: 4-DOF closed-form is faster and more predictable than iterative Jacobian. DLS IK is used only in simulation for Cartesian path tracking.

2. **Two calibration paths**: Homography for hardware (best 2D accuracy), Kabsch for simulation (full 3D with perfect marker tracking).

3. **Deterministic recovery**: No machine learning or probabilistic recovery. Every failure mode has a fixed protocol. Predictable behaviour for competition.

4. **Separation of concerns**: Vision knows nothing about motor control. Planning knows nothing about pixel coordinates. Clean interfaces via TargetPose dataclass.

5. **Configuration-driven**: All tunable parameters in YAML files. No magic numbers buried in code. Change setup? Edit config, not code.
