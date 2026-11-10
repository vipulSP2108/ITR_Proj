# `planner.py` — Waypoint-Based Pick-and-Place Planning

**Module**: `sim_demo/planner.py`
**Purpose**: Generates waypoint sequences for peg pick-and-place with multi-shape support and tilt-grasp strategies.

---

## Height Constants

| Height | Value | Purpose |
|--------|-------|---------|
| `GRASP_DEPTH` | 45 mm | Peg grasping height above table |
| `HOVER_ABOVE` | 15 mm | Hover clearance above grasp/place point |
| `CARRY_CLEARANCE` | 30 mm | Transit height above maximum obstacle |

---

## Waypoint Sequence

### Standard Pick-and-Place (Circle Peg)
1. **Hover above peg** — (peg_x, peg_y, GRASP_DEPTH + HOVER_ABOVE)
2. **Descend to grasp** — (peg_x, peg_y, GRASP_DEPTH)
3. **Close gripper**
4. **Lift to carry height** — (peg_x, peg_y, carry_z)
5. **Transit to socket** — (socket_x, socket_y, carry_z)
6. **Descend to hover** — (socket_x, socket_y, GRASP_DEPTH + HOVER_ABOVE)
7. **Insert into socket** — (socket_x, socket_y, GRASP_DEPTH)
8. **Open gripper**
9. **Retreat vertically** — (socket_x, socket_y, carry_z)

### Non-Circle Shapes (Square, Triangle, Hexagon)
Additional **`align_with_hole`** waypoint inserted between steps 5 and 6:
- Hovers above socket
- Computes symmetry-aware minimal rotation
- Applies base pan correction for yaw alignment
- Then descends for insertion

---

## Tilt Strategy

Tests tilt angles from 0° to 60° in 5° increments:
1. Try vertical (0° tilt) first
2. If IK fails (target too far), increase tilt
3. Use first successful tilt angle for ALL waypoints in the trajectory

From actual demo run:
> "chosen constant tool tilt: 50 deg" (for target at 29.7 cm from base)

**Constant tilt**: Once chosen, same tilt for all waypoints. Changing tilt mid-trajectory would require re-grasping.

---

## Symmetry-Aware Alignment

| Shape | Symmetry Order | Rotation Period | Example |
|-------|---------------|-----------------|---------|
| Circle | ∞ | N/A (no rotation needed) | — |
| Square | 4 | 90° | 47° actual → -43° correction |
| Triangle | 3 | 120° | 50° actual → -70° correction |
| Hexagon | 6 | 60° | 35° actual → -25° correction |

Formula: `minimal_rotation = ((current_yaw - target_yaw + period/2) % period) - period/2`
