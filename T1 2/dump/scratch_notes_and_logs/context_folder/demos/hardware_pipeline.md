# `run_hardware_pipeline.py` — Physical Hardware Pipeline

**Module**: `T1 2/run_hardware_pipeline.py`
**Purpose**: Full 4-stage physical hardware pipeline — from power-on to autonomous pick-and-place.

---

## Pipeline Stages

### Stage 1: Home Pose
- Move all joints to calibrated home position
- Verify encoder readings match expected values
- Camera warm-up (8 frames to settle auto-exposure)

### Stage 2: Autonomous Calibration (Homography)
- Uses `AutonomousHomographyCalibrator`
- Arms moves to 4 known stances
- Gripper-flick technique detects TCP in camera frame
- Computes 3×3 Homography matrix H (pixel → robot XY)
- Saves to `config/homography_calib.yaml`

### Stage 3: Side-Park Position
- Pan base to -90° (robot arm out of camera FOV)
- Zero table occlusion for clean workspace capture
- `DynamicWorkspacePerceiver` scans for all pegs and sockets

### Stage 4: Autonomous Pick-and-Place
- For each detected peg:
  1. Reachability check (vertical / angled / sweep)
  2. IK computation with pitch fallback cascade
  3. Smooth trajectory execution
  4. Gripper monitoring throughout
  5. Visual verification after placement

---

## Key Functions

### `smooth_move(arm, targets, profile_vel=40, steps=15)`

Shortest angular path interpolation with multi-turn wrap-around handling:

```python
for each step in [1..steps]:
    for each joint:
        delta = target - current
        if joint == 11:  # multi-turn base
            delta = shortest_angular_path(delta)  # handles ±4096 wrap
        goal = current + (delta * step / steps)
        arm.write_position(joint_id, goal)
    sleep(0.04)
```

### `pixel_to_robot(u, v)`

Two-level coordinate transform:
1. **Primary**: Homography projection via `config/homography_calib.yaml`
2. **Fallback**: 3D rigid transform via `config/camera_calib.yaml` (if homography unavailable)

---

## Integration Points

| Module | Role in Pipeline |
|--------|-----------------|
| `OpenManipulatorController` | Motor I/O |
| `AutonomousHomographyCalibrator` | Stage 2 calibration |
| `DynamicWorkspacePerceiver` | Stage 3 perception |
| `OpenManipulatorKinematics` | IK for Stage 4 trajectories |
| `GripperGuard` | Transit monitoring |
| `smooth_move` | All arm motion |

---

## CLI Usage

```bash
python run_hardware_pipeline.py                    # Full pipeline
python run_hardware_pipeline.py --until workspace  # Stop after perception
python run_hardware_pipeline.py --single circle    # One shape only
python run_hardware_pipeline.py --dry-run          # Elevated Z, no contact
```
