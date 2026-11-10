# `kinematics.py` — Analytical FK/IK for OpenMANIPULATOR-X

**Module**: `src/planning/kinematics.py`
**Purpose**: Closed-form forward and inverse kinematics. Zero-latency computation (no iterative solver needed for the hardware pipeline).

---

## Robot Geometry

```
            L2 = 130mm
    ┌──────────────────┐
    │ Shoulder (J2)     │ Elbow (J3)
    │                   │
    │ L1 = 77mm        │ L3 = 124mm
    │ (base height)     │
    │                   │ Wrist (J4)
   ─┴─                 │
  Base (J1)            │ L4 = 126mm
                       │
                      TCP (gripper center)
```

| Link | Length | From → To |
|------|--------|-----------|
| L1 | 77 mm | Base origin → Shoulder pitch axis (vertical) |
| L2 | 130 mm | Shoulder → Elbow |
| L3 | 124 mm | Elbow → Wrist |
| L4 | 126 mm | Wrist → TCP (gripper fingertip center) |

**Coordinate Frame**: Origin = center of base on table surface, +X forward, +Y left, +Z up.

---

## Encoder Conversions

```python
COUNTS_PER_RAD = 4096 / (2π) ≈ 651.9
RAD_PER_COUNT  = (2π) / 4096 ≈ 0.001534
CENTER_COUNT   = 2048
```

---

## `raw_to_radians(raw_counts, base_offset_count=0) → [q1, q2, q3, q4]`

Converts Dynamixel raw encoder values to joint radians.

**Critical detail — Motor 13 and 14 are inverted**:
```python
q3 = -(raw_counts[2] - 2048) × RAD_PER_COUNT
q4 = -(raw_counts[3] - 2048) × RAD_PER_COUNT
```
Physical servo horns face +Y direction. Decreasing raw counts = forward/downward rotation. The negation maps this to the standard kinematic convention (positive = forward/downward).

**Motor 11 (base pan)**: Uses modular arithmetic with optional offset for multi-turn mode. When `base_offset_count=0`, wraps around 4096 with center at 2048.

---

## `radians_to_raw(radians, base_wrap_count=4096) → [raw1, raw2, raw3, raw4]`

Inverse of `raw_to_radians`. Same inversion for motors 13/14:
```python
q3_raw = 2048 - round(radians[2] × COUNTS_PER_RAD)
q4_raw = 2048 - round(radians[3] × COUNTS_PER_RAD)
```

**Edge case**: When q1 ≈ 0 and base_wrap_count=4096, result maps to 4096 (not 0) to match the calibrated home position where raw=0 means 180° (facing backward).

---

## `forward_kinematics(q1, q2, q3, q4) → (x, y, z, pitch_rad)`

Planar 2R+tool chain rotated by q1:

```python
# Internal planar angles (th = geometric link angles, NOT motor angles)
th2 = π/2 - q2      # shoulder geometric angle
th3 = -q3            # elbow geometric angle
th4 = -q4            # wrist geometric angle
pitch = th2 + th3 + th4  # total tool pitch angle

# Planar reach and height
r = L2·cos(th2) + L3·cos(th2+th3) + L4·cos(pitch)
z = L1 + L2·sin(th2) + L3·sin(th2+th3) + L4·sin(pitch)

# 3D via base rotation
x = r·cos(q1)
y = r·sin(q1)
```

Returns: (x_mm, y_mm, z_mm, pitch_rad)

---

## `inverse_kinematics(x, y, z, pitch_rad=-π/2) → (q1, q2, q3, q4) or None`

**Default pitch = -π/2** = tool pointing straight down = vertical top-grasp.

### Algorithm (Analytical Closed-Form)

1. **Base yaw**: `q1 = atan2(y, x)`
2. **Horizontal reach**: `r_target = √(x² + y²)`
3. **Wrist center** (subtract L4 along pitch direction):
   ```
   r_wrist = r_target - L4·cos(pitch)
   z_wrist = z - L1 - L4·sin(pitch)
   ```
4. **Reachability check**: `d = √(r_wrist² + z_wrist²)` must be in `[|L2-L3|, L2+L3]`
5. **Law of Cosines** for elbow angle:
   ```
   cos_th3 = (d² - L2² - L3²) / (2·L2·L3)
   th3 = atan2(-√(1 - cos²), cos_th3)    ← elbow-up solution
   ```
6. **Shoulder angle**:
   ```
   th2 = atan2(z_wrist, r_wrist) - atan2(L3·sin(th3), L2 + L3·cos(th3))
   ```
7. **Wrist angle**: `th4 = pitch - th2 - th3`
8. **Convert to motor angles**: `q2 = π/2 - th2`, `q3 = -th3`, `q4 = -th4`

### Joint Limits Enforced

| Joint | Limit |
|-------|-------|
| q2 (Shoulder) | ±90° (+5° tolerance) |
| q3 (Elbow) | ±118° |
| q4 (Wrist) | ±110° |

Returns `None` if any limit exceeded or kinematically unreachable.

---

## Design Decisions

- **Analytical over numerical**: 4-DOF arm has closed-form solution. No need for iterative Jacobian methods. Instantaneous computation (<1μs).
- **Elbow-up only**: Single solution branch. Avoids ambiguity and keeps arm above the table.
- **Inverted q3/q4**: Physical servo orientation requires negation. This is the #1 source of bugs when porting — always check sign convention.
- **pitch_rad = -π/2 default**: Vertical downward grasp is the most common and stable configuration.

---

## Coordinate System Summary

```
           +Z (up)
            |
            |
            |______ +X (forward, toward board)
           /
          /
         +Y (left)
        
  q1 = 0: arm points along +X
  q1 = +π/2: arm points along +Y (left)
  Home pose raw [0, 2048, 2048, 2048]: arm along -X (backward, raw=0 = 180°)
```
