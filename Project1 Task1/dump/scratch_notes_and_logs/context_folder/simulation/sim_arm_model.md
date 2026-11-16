# `arm_model.py` — Analytic Kinematics (Simulation Side)

**Module**: `sim_demo/arm_model.py`
**Purpose**: Self-contained FK/IK for MuJoCo simulation. Built from published ROBOTIS dimensions only — never reads MuJoCo ground truth.

---

## Geometry Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `SHOULDER_Z` | 0.077 m | Joint-2 axis height above base |
| `ELBOW_OFF` | (0.024, 0.128) m | Joint-3 relative to Joint-2 (x, z) at q2=0 |
| `FOREARM` | 0.124 m | Joint-3 → Joint-4 |
| `TOOL` | 0.126 m | Joint-4 → TCP |
| `MARKER_LOCAL` | (0.0, 0.030) m | Tool marker ball offset from wrist |

Derived:
- `L2_EFF = hypot(0.024, 0.128) ≈ 0.1302 m` (effective shoulder→elbow length)
- `GAMMA = atan2(0.024, 0.128) ≈ 10.6°` (forward lean of upper arm at q2=0)

---

## Joint Limits (from URDF)

| Joint | Min (rad) | Max (rad) | Degrees |
|-------|-----------|-----------|---------|
| q1 (pan) | -π | π | ±180° |
| q2 (shoulder) | -0.57π | 0.50π | -103° to +90° |
| q3 (elbow) | -0.30π | 0.44π | -54° to +79° |
| q4 (wrist) | -0.57π | 0.65π | -103° to +117° |

---

## Key Functions

### `fk_planar(q2, q3, q4) → dict`
Returns planar (r, z) points for shoulder, elbow, wrist, TCP, marker, and sigma angle.

### `fk(q) → dict`
Full 3D FK. Converts planar results to 3D via base rotation q1.
- Returns: {shoulder, elbow, wrist, tcp, marker} in metres
- `tilt = π/2 - sigma` (0 = vertical downward, + = forward)

### `ik(x, y, z, tilt=0.0, elbow_up=True) → np.ndarray or None`
- `tilt` parameter (not `pitch_rad`): 0 = straight down, positive = forward tilt
- Relationship: `sigma = π/2 - tilt`
- Uses `GAMMA` correction for the elbow offset geometry
- Returns (q1, q2, q3, q4) in radians or None

### `max_vertical_reach(z=0.045) → float`
Brute-force sweep: tests r from 0.05 to 0.40m in 2mm steps at tilt=0. Returns largest reachable r.

### Preset Poses
- `HOME_Q = [0, 0, 0, 0]` — "L" pose from CAD
- `SIDE_PARK_Q = [-π/2, 0, 0, 0]` — Arm to -Y side, out of camera view

---

## Differences from Hardware `kinematics.py`

| Aspect | `arm_model.py` (sim) | `kinematics.py` (hardware) |
|--------|----------------------|---------------------------|
| Units | **Metres** | **Millimetres** |
| Tilt convention | `tilt` (0 = down) | `pitch_rad` (-π/2 = down) |
| Elbow offset | Explicit `ELBOW_OFF=(0.024, 0.128)` | Simplified `L2=130mm` |
| Encoder handling | No encoders | Full raw↔radian conversion |
| L2 effective | `hypot(24, 128) = 130.2mm` | `130mm` (rounded) |
| GAMMA correction | Explicit | Implicit in angle convention |
