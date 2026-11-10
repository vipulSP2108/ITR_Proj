# `robot_interface.py` — Hardware Abstraction Layer (HAL)

**Module**: `sim_demo/robot_interface.py` (273 lines)
**Purpose**: Abstract base class with sim and real implementations. Allows the perception/planning/execution pipeline to run identically on MuJoCo simulation or real hardware.

---

## Architecture

```
RobotInterface (ABC)
    ├── SimRobot (wraps SimEnv)
    └── RealRobot (wraps OpenManipulatorController + GripperGuard)
```

---

## Abstract Base Class: `RobotInterface`

| Method | Signature | Description |
|--------|-----------|-------------|
| `read_joints()` | `→ np.ndarray` | Current joint angles (4 values) |
| `move_joints(q, duration)` | `→ None` | Move to joint target |
| `move_cartesian(p0, p1, tilt, dur)` | `→ None` | Straight-line TCP motion |
| `set_gripper(close, duration)` | `→ None` | Open/close gripper |
| `grasp_state()` | `→ str` | "held" / "empty" / "unknown" |
| `estop()` | `→ None` | Emergency stop |
| `capture_rgbd()` | `→ (rgb, depth)` | Camera capture |
| `settle(seconds)` | `→ None` | Wait for dynamics to settle |

---

## SimRobot

Wraps `SimEnv` with direct pass-through to MuJoCo simulation.

### Grasp Detection
```python
held = 0.018 < gripper_stroke < 0.042  # something between fingers
empty = gripper_stroke >= 0.042         # fingers at full open
```

---

## RealRobot

Wraps `OpenManipulatorController` + `GripperGuard` for physical hardware.

### Key Conversion Constants

| Parameter | Value |
|-----------|-------|
| `ZERO_COUNTS` | 2048 |
| `COUNTS_PER_RAD` | 4096 / (2π) ≈ 651.9 |
| Gripper open | 1700 counts |
| Gripper close | 2150 counts |

### Cartesian Motion

Interpolates N = max(duration × 20, 10) IK steps along the straight line from p0 to p1. Sends each as a position command to the Dynamixel servos.

### Camera Capture

Uses OpenCV webcam capture + synthetic planar depth at 0.61m (no RealSense SDK depth — simplified to a flat plane assumption).

### Dry-Run Mode

`dry_run=True`: Prints all commands without executing. Elevates Cartesian targets by +50mm to prevent any contact even if commands leak through.

---

## The Sim-to-Real Bridge

This HAL is the **recommended architecture** from `repare.md` for deploying the sim-validated pipeline on real hardware. The key challenge is the **unit and convention mismatch**:

| Property | SimRobot | RealRobot |
|----------|----------|-----------|
| Units | metres / radians | mm / counts |
| Joint zero | L-pose (forearm horizontal) | Straight vertical |
| q3 offset | ~1.386 rad difference | — |
| Camera | Simulated overhead | Real RealSense |
| Depth | MuJoCo rendered | Flat plane assumption |

The conversion happens inside `RealRobot` methods, isolating the rest of the pipeline from hardware specifics.
