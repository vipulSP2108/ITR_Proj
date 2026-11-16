# Physical Objects — Pegs and Socket Board

**Reference**: `device/OBJECTS_AND_BOARD.md`, `config/workspace_dimensions.yaml`

---

## Pegs

All pegs are approximately **4.5 cm cross-section × 10 cm tall**.

| Shape | Color | HSV Hue Range | BGR Display | Symmetry Order |
|-------|-------|---------------|-------------|---------------|
| Triangle | Bright Yellow | H: 16–32 | (0, 200, 255) | 3 (120°) |
| Square | Deep Blue | H: 95–130 | (255, 100, 0) | 4 (90°) |
| Hexagon | Orange/Salmon Red | H: 0–12 ∪ 170–179 | (60, 80, 220) | 6 (60°) |
| Circle | Green | H: 38–80 | (80, 180, 80) | ∞ (invariant) |

**Saturation**: 65–255 (all colors)
**Value**: 65–255 (all colors, slightly different per shape)

**Red hue wrapping**: Hexagon/Red uses dual HSV range because red hue spans the 0/180 boundary in OpenCV's HSV space.

---

## Socket Board

| Parameter | Value |
|-----------|-------|
| Material | Laser-cut MDF |
| Total dimensions | 300 × 100 mm |
| Layout | Single vertical strip, 4 cutouts top→bottom |
| Cutout order | Triangle → Square → Hexagon → Circle |
| Insertion clearance | ~1–2 mm per side |

### Individual Socket Dimensions (from `workspace_dimensions.yaml`)

| Socket | Width (mm) | Height (mm) |
|--------|-----------|-------------|
| Triangle | 55 | 48 |
| Square | 50 | 50 |
| Hexagon | 52 | 45 |
| Circle | 50 (diameter) | — |

---

## Grasping Constraints

| Parameter | Value |
|-----------|-------|
| Gripper open gap | ~60 mm (real) / 70 mm (sim) |
| Peg cross-section | ~45 mm |
| Palm depth | 56 mm behind fingertips |
| Max grasp depth | 45 mm from peg top |
| Pre-grasp hover | +60–80 mm above peg |

**Why grasp the top 45mm?** Grasping lower drives the peg into the 56mm-deep palm cavity. This was discovered through failed tilted-grasp attempts — the peg bottoms out against the palm and cannot be lifted.

---

## Workspace Layout

| Element | Position | Notes |
|---------|----------|-------|
| Green foam mat | 300×300 mm | Peg placement area |
| Socket board | Adjacent to mat | 300×100 mm strip |
| Robot base | Pixel (485, 215) in camera frame | Coordinate origin |

### Manipulation Heights (from `workspace_dimensions.yaml`)

| Phase | Z (mm) | Purpose |
|-------|--------|---------|
| z_hover | 105 | Safe transit clearance |
| z_transit | 105 | Carrying height between pick and place |
| z_sock_hover | 85 | Hover above socket before insertion |
| z_grasp | 35 | Gripper engagement with peg |
| z_insert | 28 | Peg-in-socket seating depth |

---

## Shape Sorting Strategy

**Circle first**: C∞ rotational symmetry means no orientation alignment needed — isolates pure Cartesian translation for validation.

**Non-circles require yaw alignment**: The 4-DOF arm has no wrist roll. Gripper orientation is locked to `q1 = atan2(y, x)`. When picking a peg and placing it in a socket at different angles, the base pan rotation `Δq1` changes gripper yaw. The `multi_shape.py` module checks whether `peg_yaw + Δq1 ≡ socket_yaw (mod symmetry)` within a ±12° compliance window.
