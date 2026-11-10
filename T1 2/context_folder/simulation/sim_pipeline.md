# `run_pipeline.py` — Master 3-Stage Simulation Pipeline

**Module**: `sim_demo/run_pipeline.py`
**Purpose**: Complete autonomous peg sorting in MuJoCo: calibrate → perceive → pick-and-place with camera-verified tracking.

---

## Three Stages

### Stage 1: HOME
- Command all joints to zero → "L" pose
- Verify TCP position via FK matches simulator
- Record home pose encoders

### Stage 2: WORKSPACE (Calibration + Detection)

**2a. Camera-Robot Calibration**:
- Move to 8 calibration poses (configurable with `--calib-poses`)
- At each: FK gives robot-frame marker position; camera detects marker via sphere fitting
- Kabsch SVD computes (R, t)
- Typical result: RMSE 0.08mm, reprojection 0.37px

**2b. Workspace Analysis**:
- Detect all shapes on board (ordering: circle, hexagon, square, triangle top→bottom)
- Locate target peg position
- Compute radial distance from base
- Select grasp tilt angle (vertical if reachable, otherwise sweep to find minimum tilt)

### Stage 3: PICK & PLACE
- Generate waypoint sequence via `planner.py`
- Execute with Cartesian straight-line motion via `trajectory.py`
- **Camera-verified tracking**: At every control step, camera checks TCP is on planned path
- Grasp failure recovery if gripper position indicates empty grasp
- **Final socket occupancy verification**: Camera confirms peg is in socket

---

## Command-Line Interface

```bash
python run_pipeline.py \
    --peg-xy 0.15 -0.06 \     # Peg position (metres)
    --shape square \            # Peg shape
    --until pick \              # Stop after pick (debug)
    --calib-poses 8 \           # Number of calibration poses
    --linger 3.0 \              # Pause at end (seconds)
    --record-dir ./records \    # Output directory
    --no-video                  # Skip video recording
```

---

## Tracking Metrics (from demo run)

| Metric | Value | Threshold |
|--------|-------|-----------|
| ON_TRACK percentage | 100% | > 95% |
| Mean tracking error | 4.5 mm | < 10 mm |
| P95 tracking error | 21.0 mm | < 30 mm |
| Max tracking error | 28.8 mm | < 50 mm |
| Marker visible | 100% | > 90% |
| Cross-track mean | 15.3 mm | — |
| Direction cosine mean | 1.00 | > 0.95 |

---

## Output Structure (per run)

```
demo_YYYYMMDD_HHMMSS/
├── 01_home_pose/           # Home verification images
├── 02_calibration/         # Calibration data, poses, error plots
├── 03_workspace_analysis/  # Shape detection, reachability maps
├── 04_pick_and_place/      # Trajectory images, tracking plots, video
├── config_snapshot/        # Exact parameters used
├── run_log.json            # Machine-readable full log
└── summary.md              # Human-readable summary
```
