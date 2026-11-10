# `sim_env.py` — MuJoCo Simulation Environment

**Module**: `sim_demo/sim_env.py`
**Purpose**: Physics-based simulation with minimum-jerk interpolation, Cartesian straight-line motion, and RealSense-like depth noise.

---

## Core Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CTRL_DT` | 20 ms | Control loop timestep |
| Damping | 0.03 | DLS IK solver damping factor |
| Max IK iterations | 25 | Per control step |
| IK tolerance | 0.1 mm | Convergence criterion |

---

## Motion Strategy

### Minimum-Jerk Joint Interpolation
Uses quintic polynomial for smooth joint trajectories between waypoints:
```
s(τ) = 10τ³ - 15τ⁴ + 6τ⁵    where τ = t/T ∈ [0, 1]
```
Properties: zero velocity and acceleration at start/end. Minimizes jerk (derivative of acceleration).

### Cartesian Straight-Line Motion
At every control step (20ms), IK is solved to keep the TCP on a straight line in Cartesian space. Prevents the arc-shaped paths that pure joint interpolation produces.

---

## Depth Noise Model (Matching RealSense D435i)

The simulated camera adds realistic noise to depth images:
- Gaussian depth noise
- Edge dropout (depth values lost at occlusion boundaries)
- Quantization to match sensor resolution

This ensures the perception pipeline tested in simulation faces similar challenges to hardware.

---

## Design Decisions

- **20ms control dt**: Matches real Dynamixel position command rate. Faster is unnecessary; slower causes visible jitter.
- **Cartesian straight-line over joint-space interpolation**: Joint-space creates arc paths that risk collisions with board edges. Cartesian lines keep predictable, collision-avoidable paths.
- **Minimum-jerk profile**: Smoothest possible motion. No sudden accelerations that could jar the peg loose from the gripper.
