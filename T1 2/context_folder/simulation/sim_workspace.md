# `workspace.py` — Reachability Grid Analysis

**Module**: `sim_demo/workspace.py`
**Purpose**: Maps the full workspace reachability at specific Z heights, classifying each grid cell by required tilt angle.

---

## Grid Sweep

Sweeps X and Y coordinates across the workspace at a fixed Z height (typically grasp height):
- X range: configurable, typically 0–400mm
- Y range: configurable, typically -200 to +200mm
- Step size: configurable (10mm for batch, 2mm for fine)

At each (x, y, z) point:
1. Try IK with tilt = 0° (vertical)
2. If fails, try tilt = 5°, 10°, ..., 60°
3. Classify the cell

---

## Tilt Classification

| Category | Tilt Range | Meaning |
|----------|-----------|---------|
| `vertical` | 0–5° | Standard top-grasp, most stable |
| `tilt30` | 5–30° | Moderate tilt, still reliable |
| `tilt60` | 30–60° | Significant tilt, reduced grasp quality |
| `unreachable` | > 60° or no solution | Cannot be grasped |

---

## Output

- ASCII heatmap (terminal visualization)
- CSV file with (x, y, tilt, reachable) data
- Used by batch grid sweep (`batch_grid_sweep.py`) to generate comprehensive workspace maps

---

## Key Finding from Grid Sweeps

The workspace is NOT a simple circle. Due to joint limits:
- **Vertical reach**: ~218mm radius at z=45mm
- **With 50° tilt**: ~295mm radius
- **Asymmetric**: Y-direction reach differs from X due to shoulder/elbow joint limit interaction
- **Dead zones**: Near the base (< 95mm) and at extreme angles
