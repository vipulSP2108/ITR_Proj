# `workspace_dimensions.yaml` — Physical Dimensions & Camera Setup

**File**: `config/workspace_dimensions.yaml`
**Purpose**: Ground-truth physical measurements verified with ruler. Single source of truth for all geometric computations.

---

## Camera Setup

| Parameter | Value | Measurement Method |
|-----------|-------|--------------------|
| Height above board | 470.0 mm | Physical ruler |
| Sensor width | 640 px | RealSense spec |
| Sensor height | 480 px | RealSense spec |
| Focal length | 344.0 px | RealSense D435 IR stream, ~86° FOV |
| Principal point | (320, 240) | Image center |
| Table scale | **1.366 mm/px** | 470mm ÷ 344px |

---

## Workspace Physical Layout

### Green Foam Mat
- 300 × 300 mm (30 × 30 cm)
- Expected image size: 220 × 220 px
- Pegs are placed on this mat for pickup

### Wooden Board
- **300 mm long, 100 mm wide**
- **MOVABLE** — not fixed to table, must be detected dynamically
- Cutout gap: 25 mm (73.5 mm center-to-center)
- Cutout order (top to bottom in image): **Triangle → Square → Hexagon → Circle**

### Cutout Dimensions

| Shape | Primary Dimension | Expected px |
|-------|-------------------|-------------|
| Circle | ⌀50.0 mm | r ≈ 18.3 px |
| Hexagon | 45.0 mm across flats | span ≈ 33 px |
| Square | 50.0 × 50.0 mm | side ≈ 36.6 px |
| Triangle | 45.0 mm height | h ≈ 33 px |

---

## Manipulation Heights

| Phase | Height (mm) | Purpose |
|-------|-------------|---------|
| Hover | 105.0 | Safe transit clearance |
| Grasp | 35.0 | Peg pickup height |
| Transit | 105.0 | Carry height between pick and place |
| Socket hover | 85.0 | Pre-insertion hover |
| Insert | 28.0 | Peg insertion depth into board cutout |

---

## Robot Mounting

| Parameter | Value |
|-----------|-------|
| Base center pixel | u=485, v=215 |
| Home pose direction | -X (raw count 0/4096 = 180°) |
| Board direction | +X (raw count 2048 = 0°) |
| Foam mat direction | +Y (raw count 3072 = +90°) |
| Vertical | +Z (upward) |
