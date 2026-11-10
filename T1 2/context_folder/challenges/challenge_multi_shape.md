# Challenge: Multi-Shape Sorting with Orientation Alignment

**Problem**: Non-circular pegs (square, triangle, hexagon) must be rotated to match their socket orientation before insertion.

---

## The Orientation Problem

A circle peg can be inserted at any rotation — orientation doesn't matter. But:
- A **square** peg must align to within ~5° of the square socket
- A **triangle** peg must align to within ~5° of the triangular socket
- A **hexagon** peg must align to within ~5° of the hexagonal socket

The robot must:
1. Detect the peg's current orientation (yaw angle)
2. Detect the socket's orientation
3. Compute the minimal rotation needed
4. Apply that rotation via base pan (Joint 1)

---

## Symmetry-Aware Minimal Rotation

Each shape has rotational symmetry — multiple valid insertion angles exist:

| Shape | Symmetry Order | Period | Valid Angles |
|-------|---------------|--------|--------------|
| Circle | ∞ | — | Any |
| Square | 4 | 90° | 0°, 90°, 180°, 270° |
| Triangle | 3 | 120° | 0°, 120°, 240° |
| Hexagon | 6 | 60° | 0°, 60°, 120°, 180°, 240°, 300° |

### Formula
```python
period = 360 / symmetry_order
minimal_rotation = ((current_yaw - target_yaw + period/2) % period) - period/2
```

### Example: Square peg at 47°, socket at 0°
```
period = 90°
raw_diff = 47° - 0° = 47°
(47 + 45) % 90 - 45 = 92 % 90 - 45 = 2 - 45 = -43°
Minimal rotation = -43° (not -47°, because 90° symmetry)
```

---

## Implementation

### Orientation Detection (`shape_detection.py`)
- **Triangle**: Vector from centroid to apex vertex
- **Square/Hexagon**: `cv2.minAreaRect()` angle
- **Fallback**: PCA on contour points

### Alignment Phase (`planner.py`)
For non-circle shapes, an additional `align_with_hole` waypoint is inserted:
1. Hover above socket
2. Compute symmetry-aware minimal rotation
3. Apply as base pan (Joint 1) correction
4. Then descend for insertion

### Base Pan Correction
The base pan joint (Joint 1) rotates the entire arm around the vertical axis. By rotating q1 by the computed minimal rotation, the peg (held in the gripper) rotates to match the socket.

---

## Git History Evidence

Commit `e8937cd`:
> "Add square peg sorting with dedicated hole alignment phase and 90-degree symmetry handling"

This was a major feature addition, moving from circle-only sorting to multi-shape support.

---

## Challenges

1. **Orientation estimation noise**: `minAreaRect` angle has ±2° noise. Acceptable with ±5° socket tolerance.
2. **Gripper rotation ≠ peg rotation**: Peg may slip slightly in gripper during rotation. Visual verification catches this.
3. **Hexagon ambiguity**: 60° symmetry means orientation detection must be accurate to ±30°. The hue-based detection is reliable enough.
4. **Triangle apex detection**: Must find the vertex farthest from centroid. With noisy contours, this requires robust vertex detection.
