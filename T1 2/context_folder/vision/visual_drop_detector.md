# `visual_drop_detector.py` — Vision-Based Drop/Slip Detection (EC-05)

**Module**: `src/vision/visual_drop_detector.py`
**Purpose**: Detects peg drops, slips, and placement failures using camera vision during transit and after placement.

---

## Constants

**Shape-Color Map** (peg identity):
| Shape | Color |
|-------|-------|
| Circle | Green |
| Square | Blue |
| Triangle | Yellow |
| Hexagon | Orange/Red |

---

## Enum: `VisualDropStatus`

| Status | Meaning |
|--------|---------|
| `SECURE_IN_HAND` | Peg confirmed held by gripper |
| `DROPPED_ON_TABLE` | Peg fell back onto table surface |
| `MISSED_AT_PICKUP` | Peg never left original position |
| `PARTIAL_SLIP_TILTED` | Peg tilted in gripper (aspect ratio > 1.55) |
| `SOCKET_EMPTY` | Socket unoccupied after placement attempt |
| `SOCKET_FILLED` | Socket confirmed filled |

---

## Key Methods

### `verify_post_lift(frame, pickup_pos, ee_pos, color) → VisualDropStatus`

Three sequential checks after lifting a peg:
1. **Still at pickup?** — Color blob within 45mm of original position → `MISSED_AT_PICKUP`
2. **Near end-effector?** — Color blob within 60mm of EE position → `SECURE_IN_HAND`
3. **Elsewhere on table?** — Color blob found elsewhere → `DROPPED_ON_TABLE`

**Tilt detection**: If largest contour's bounding rect aspect ratio > 1.55 → `PARTIAL_SLIP_TILTED`

### `detect_tabletop_dropped_peg(frame, color, depth_map) → (x, y, z) or None`

Finds the largest contour of the peg's color, deprojection to robot frame. Used by `RecoveryManager` to re-locate a dropped peg for retry.

### `verify_socket_placement(frame, socket_pos, color, tolerance_mm=40.0) → VisualDropStatus`

Post-placement check: looks for peg color within `tolerance_mm` of the socket center. Returns `SOCKET_FILLED` or `SOCKET_EMPTY`.

---

## Design Decisions

- **Distance-based state machine**: Three concentric zones (pickup, EE, elsewhere) give unambiguous classification without complex tracking.
- **Aspect ratio for tilt**: Tilted pegs appear elongated. 1.55 threshold distinguishes tilted grip from normal top-down view.
- **40mm placement tolerance**: Accounts for camera-to-robot calibration error (~0.85mm RMSE) plus peg settling.
- **Color-specific detection**: Uses the known shape→color mapping to search for the exact peg, not any peg.

---

## Recovery Integration
```
verify_post_lift() == DROPPED_ON_TABLE
    → detect_tabletop_dropped_peg() → new position
    → RecoveryManager.recover_from_drop() → retry pick-and-place

verify_socket_placement() == SOCKET_EMPTY
    → RecoveryManager detects placement failure → retry insertion
```
