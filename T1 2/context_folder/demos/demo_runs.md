# Demo Run Results

**Locations**: `demo_runs/demo_<timestamp>/`

Each run produces: `summary.md`, `run_log.json`, checkpoint images (4 stage directories), config snapshot.

---

## Run 1: Circle Peg — Full Pipeline (2026-11-07 21:11:40)

| Metric | Value |
|--------|-------|
| **Outcome** | SUCCESS |
| **Attempts** | 1 |
| **Wall-clock** | 135 s |
| **Vision error** | 1.3 mm (peg → socket) |
| **Tool tilt** | 60° constant |
| **Tracking: On-track** | 100% |
| **Tracking: Mean error** | 4.1 mm |
| **Tracking: P95** | 19.4 mm |
| **Tracking: Max** | 22.9 mm |
| **Cross-track: Mean** | 0.2 mm |
| **Cross-track: Max** | 0.9 mm |
| **Direction cosine: Mean** | 1.00 |

**Key decision**: 60° constant tool tilt chosen because vertical top-grasp max reach (21.6 cm) cannot reach the circle socket at 31.7 cm from base.

---

## Run 2: Circle Peg — Workspace Analysis Only (2026-11-08 03:29:15)

| Metric | Value |
|--------|-------|
| **Pipeline stage** | Stopped at `--until workspace` |
| **Calibration** | 16/16 marker poses, 0.10 mm fit residual, 0.35 px reprojection |
| **Peg distance** | 16.6 cm from base |
| **Socket distance** | 31.7 cm from base |
| **Vertical reach limit** | 21.6 cm |

**Finding**: Confirmed that vertical grasp cannot reach the socket. Angled grasp at 60° extends effective reach to cover the full workspace.

---

## Run 3: Square Peg — With Alignment (2026-11-08 04:37:48)

| Metric | Value |
|--------|-------|
| **Outcome** | SUCCESS |
| **Attempts** | 1 |
| **Wall-clock** | 24 s |
| **Vision error** | 6.4 mm |
| **Tool tilt** | 50° (different from circle due to different reach) |
| **Calibration poses** | 8 (reduced via `--calib-poses 8`) |
| **Tracking: On-track** | 100% |
| **Tracking: Mean error** | 4.5 mm |
| **Tracking: P95** | 21.0 mm |
| **Tracking: Max** | 28.8 mm |

**Key feature**: First successful non-circle sort with dedicated `align_with_hole` phase for 90° symmetry handling.

---

## Videos Produced

| Video | Location | Content |
|-------|----------|---------|
| `demo1_circle_peg_sorting.mp4` | `demo1_20261107/` | Simulation circle sort |
| `random_circle_sort.mp4` | `demo1_20261107/` | Random-placement circle sort (sim) |
| `live_hardware_circle_sort.mp4` | `demo1_20261107/` | Real hardware circle sort |

---

## Storyboard Images

Each demo generates a multi-panel storyboard showing trajectory checkpoints:

| Checkpoint | Description |
|------------|-------------|
| 01 | Home/Ready pose |
| 02 | Perception scan |
| 03 | Pre-grasp hover above peg |
| 04 | Descend to peg |
| 05 | Firm grasp |
| 06 | Vertical lift |
| 07 | Mid-transit |
| 08 | Above socket |
| 09 | Socket insert |
| 10 | Release peg |
| 11 | Vertical retreat |
| 12 | Return home |
