# `multi_shape.py` — Multi-Shape Sorting with Orientation Alignment

**Module**: `sim_demo/multi_shape.py` (142 lines)
**Purpose**: Handles the 4-DOF orientation constraint for non-circle shapes. The OpenMANIPULATOR-X has no wrist roll — gripper horizontal orientation is locked to `q1 = atan2(y, x)`.

---

## The 4-DOF Orientation Problem

When the arm picks a peg at position A and places it at position B:
- The base pan rotates by `Δq1 = q1_place − q1_pick`
- This rotation **changes the gripper's yaw** by exactly `Δq1`
- The peg rotates with the gripper

For the peg to fit the socket, the final peg yaw must match the socket yaw (modulo the shape's rotational symmetry).

---

## Symmetry Constants

| Shape | Order | Interval | Meaning |
|-------|-------|----------|---------|
| Circle | 0 (∞) | 0° | Any rotation fits |
| Square | 4 | 90° | 4-fold symmetry |
| Hexagon | 6 | 60° | 6-fold symmetry |
| Triangle | 3 | 120° | 3-fold symmetry |

---

## Key Function: `minimal_symmetry_angle_diff(angle1, angle2, shape)`

Computes the minimal angular difference between two orientations, accounting for rotational symmetry:

```python
diff = (angle1 - angle2) % (2π)
if order == 0:  # circle
    return 0.0
interval = 2π / order
diff_mod = diff % interval
return min(diff_mod, interval - diff_mod)
```

---

## Class: `MultiShapePlanner`

### `evaluate_shape_alignment(shape, peg_xy, socket_xy, peg_yaw, socket_yaw)`

1. Compute `q1_pick = atan2(peg_y, peg_x)`
2. Compute `q1_place = atan2(socket_y, socket_x)`
3. `delta_q1 = q1_place − q1_pick`
4. `predicted_yaw = peg_yaw + delta_q1`
5. `yaw_error = minimal_symmetry_angle_diff(predicted_yaw, socket_yaw, shape)`
6. Compatible if `yaw_error < 12°` (compliance window)

Returns: `MultiPegPlan` with `yaw_alignment_error_deg` and `is_yaw_compatible`.

### `plan_all_pegs(pegs, sockets)`

Sorting order: **Circle → Hexagon → Square → Triangle**

Circle first because:
- C∞ symmetry = no alignment needed
- Validates pure Cartesian accuracy before attempting orientation-sensitive shapes
- Simplest possible test of the pick-and-place pipeline

---

## Alignment Execution (`run_pipeline.py`)

For non-circle shapes, the planner inserts an `align_with_hole` waypoint:

1. Arm arrives above the socket with peg
2. Base pan micro-adjusts by computed alignment angle
3. ±1.5° compliance dither applied
4. Descend and insert

This was the last feature added to the project (commit `e8937cd`, Nov 8).

---

## Design Decisions

- **±12° compliance window**: Accounts for detection noise, slight peg flex, and socket clearance (~1–2mm per side)
- **No wrist roll workaround**: The only way to rotate a held peg is via base pan. This couples position and orientation — cannot independently adjust yaw without moving the TCP laterally.
- **Circle-first ordering**: De-risks the demo. If circle works, the Cartesian pipeline is validated. If a non-circle fails, it's an orientation issue, not a fundamental pipeline failure.
