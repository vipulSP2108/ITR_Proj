# `run_live_hardware_sorter.py` — Autonomous Live Hardware Demo

**Module**: `demo1_20261107/run_live_hardware_sorter.py`
**Purpose**: Full autonomous circle peg sorting on real OpenMANIPULATOR-X hardware with video recording, HUD telemetry, and storyboard generation.

---

## Demo Flow

### Class: `LiveHardwareSorter`

**Stage 1: Initialize Hardware**
- Auto-detect U2D2 port on macOS
- Clear hardware error latches (reboot motors if needed)
- Set profile_vel=25, profile_acc=10 (gentle motion)
- Enable torque on all 5 joints

**Stage 2: Move to Calibrated Home**
- Home targets: [0, 2048, 2048, 2048, 3130]
  - Joint 1 raw=0 → 180° (arm pointing backward)
  - Joints 2–4 raw=2048 → 0° ("L" pose)
  - Gripper raw=3130 → ~90° open
- Uses `smooth_move()` with duration=2.5s, 10 interpolation steps

**Stage 3: Vision Perception**
- Opens `DynamicWorkspacePerceiver` on camera frame
- Detects circle socket position and peg position
- Saves baseline peg ROI for post-lift verification
- Computes 3D pick/place coordinates

**Stage 4: Execute Trajectory**
11 waypoint stages:
1. Hover above circle peg (z=105mm)
2. Vertical descend onto peg (z=35mm)
3. Firm gripper grasp (close to 2300)
4. Vertical lift with peg (z=105mm)
5. Transit across corridor
6. Hover above circle socket (z=85mm)
7. Lower peg into socket (z=28mm)
8. Release peg (open to 3130)
9. Vertical retreat (z=85mm)
10. Return to home

**IK Strategy**: Tries pitch from -90° to -40° in 5° steps. First valid solution wins.

### Feedback Loops

| Stage | Check | Method |
|-------|-------|--------|
| 5 (Grasp) | Gripper effort | `read_load(15)`, threshold >10% |
| 6 (Lift) | Peg lifted | ROI difference >15 pixel intensity |

---

## Video & Documentation

- **Live video recording**: 640×480 @ 15fps MP4 with real-time HUD overlay
- **HUD telemetry**: Stage name, target coordinates, motor commands, actual positions, grip state
- **Milestone snapshots**: High-res JPEG at every stage checkpoint
- **9-panel storyboard**: Composite image of all stages

---

## Output Files

```
demo1_20261107/
├── live_hardware_circle_sort.mp4    # Full run video with HUD
├── imp_images/
│   ├── hw_checkpoint_01_home_ready.jpg
│   ├── hw_02_perception_scan.jpg
│   ├── hw_checkpoint_03_pre_grasp_hover.jpg
│   ├── ... (one per stage)
│   └── hw_00_full_path_storyboard.jpg   # 3×3 composite
```

---

## Key Parameters (Configurable via CLI)

```bash
python run_live_hardware_sorter.py \
    --camera-idx 0 \     # Overhead camera device index
    --z-grasp 35.0 \     # Override grasp height (mm)
    --z-insert 28.0      # Override insert height (mm)
```
