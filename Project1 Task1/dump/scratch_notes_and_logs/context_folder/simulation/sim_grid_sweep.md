# `batch_grid_sweep.py` — Systematic Workspace Testing

**Module**: `sim_demo/batch_grid_sweep.py` (429 lines)
**Purpose**: Iterates over a 12×12 grid (144 cells), running the full pipeline at each peg position. Generates ASCII heatmaps, CSV logs, and an HTML gallery of results.

---

## Grid Coordinate System

```python
def grid_to_xy(gx, gy):
    # gx ∈ [0, 11], gy ∈ [0, 11]
    # Robot at grid (6, 1) → metric (0, 0)
    x = (gy - 1) * 0.025  # metres
    y = (gx - 6) * 0.03   # metres
```

Grid step: 25mm in X, 30mm in Y. Total coverage: 275mm × 330mm.

---

## Outcome Categories

| Outcome | Meaning |
|---------|---------|
| `SUCCESS` | Peg placed in socket, < threshold distance |
| `OUT_OF_REACH` | IK cannot find a feasible tilt |
| `PEG_NOT_FOUND` | Perception failed to detect peg |
| `GRASP_FAILED` | Gripper closed on air / peg slipped |
| `MOTION_ABORTED` | Tracking abort (4+ frames over 40mm) |
| `FAILED` | Other failure |

---

## Output Structure

```
sweep_<timestamp>/
├── records/
│   ├── gx00_gy00/
│   │   ├── storyboard.png
│   │   └── final_check.png
│   ├── gx00_gy01/
│   │   └── ...
│   └── ...
├── grid_sweep_results.csv
├── grid_sweep_results.json
└── index.html              ← Responsive dark-themed gallery
```

### CSV Columns
gx, gy, x_m, y_m, shape, outcome, error_mm, tilt_deg, duration_s

---

## ASCII Heatmap

ANSI-colored terminal output:
- **Green**: SUCCESS
- **Red**: FAILED/ABORT
- **Yellow**: OUT_OF_REACH
- **Cyan**: GRASP_FAILED
- **Gray**: Not yet tested

Example:
```
   0  1  2  3  4  5  6  7  8  9 10 11
 0  .  .  .  .  .  .  .  .  .  .  .  .
 1  .  R  .  .  .  .  H  .  .  .  .  .
 2  .  .  ✓  ✓  ✓  ✓  ✓  ✓  .  .  .  .
 3  .  .  ✓  ✓  ✓  ✓  ✓  ✓  ✓  .  .  .
 ...
```

---

## HTML Gallery

Responsive dark-themed page with:
- Grid overview showing outcomes by color
- Click-through to individual cell storyboards and final-check images
- Summary statistics (success rate, mean error, mean tilt)

---

## Key Functions

| Function | Line | Purpose |
|----------|------|---------|
| `grid_to_xy(gx, gy)` | 57 | Grid → metric conversion |
| `parse_output(stdout)` | 65 | Extract outcome/error/tilt from subprocess |
| `print_heatmap(results, ...)` | 107 | ANSI terminal heatmap |
| `generate_html_gallery(...)` | 141 | Responsive HTML gallery |
| `main()` | 215 | Outer loop, CSV/JSON checkpointing every 3 runs |

---

## Integration

Each grid cell runs as a **subprocess** calling `run_grid_simulation.py`, which delegates to `run_pipeline.main()`. This isolates MuJoCo state between runs and allows crash recovery.

Progress is saved every 3 runs to `grid_sweep_results.csv/json`, so interrupted sweeps can be resumed.
