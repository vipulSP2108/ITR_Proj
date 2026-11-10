# `autonomous_homography_calibrator.py` — Self-Calibration via Gripper Flick

**Module**: `src/planning/autonomous_homography_calibrator.py`
**Purpose**: Fully autonomous camera-to-robot calibration using differential gripper flick detection. No manual point collection needed.

---

## Core Idea

Instead of manually clicking calibration points, the robot creates its own calibration data:
1. Move to a known pose (FK gives robot-frame TCP)
2. Open gripper → capture frame
3. Close gripper → capture frame
4. Subtract the two frames → gripper finger movement reveals TCP location in image
5. Repeat at multiple poses → build pixel↔robot point pairs
6. Compute homography matrix H

---

## Key Methods

### `detect_tcp_by_gripper_flick(open_frame, closed_frame) → (u, v)`

1. Compute `absdiff = |open_frame - closed_frame|`
2. Threshold the difference → binary mask of moved pixels
3. Find two largest contour clusters (left and right gripper fingers)
4. TCP = midpoint of the two cluster centroids

**Why this works**: Only the gripper fingers move between frames. Everything else (arm links, table, board) cancels out in the absolute difference.

### `run_autonomous_calibration() → (H_3x3, R_3x3, t_3x1)`

**4 Calibration Stances** at Z=100mm:

| Stance | Name | Location |
|--------|------|----------|
| 1 | Near Mat | Close to green mat |
| 2 | Far Mat | Far side of green mat |
| 3 | Near Board | Close to board area |
| 4 | Center Table | Workspace center |

At each stance:
1. FK computes robot-frame TCP position
2. `detect_tcp_by_gripper_flick()` gives pixel position
3. Pair: (u, v) ↔ (X_rob, Y_rob)

Then:
- Compute **planar homography H** from the 4+ pairs using `cv2.findHomography()`
- Also compute full 3D (R, t) using camera height and focal length

### `pixel_to_robot(u, v) → (X_rob, Y_rob)`

Applies homography transform:
```
[X', Y', W'] = H @ [u, v, 1]
X_rob = X' / W'
Y_rob = Y' / W'
```

---

## Camera Parameters Used

| Parameter | Value | Source |
|-----------|-------|--------|
| Focal length | 344 px | RealSense D435 IR stream |
| Principal point | (320, 240) | Image center |
| Camera height | 470 mm | Physical measurement |

---

## Calibration Result (`homography_calib.yaml`)

```yaml
num_stances: 4
rmse_mm: 0.12
homography_matrix_3x3:
  - [-1.217, 0.118, 527.41]
  - [-0.108, 0.989, -175.13]
  - [-0.001, 0.001, 1.0]
```

**RMSE = 0.12mm** — significantly better than the 3D Kabsch calibration (0.85mm) because homography operates on the dominant XY plane where most manipulation happens.

---

## Design Decisions

- **Autonomous over manual**: No human-in-the-loop for calibration. Reproducible, fast, and eliminable as error source.
- **Gripper flick over marker tracking**: No need for ArUco/AprilTag markers. Uses the robot's own anatomy.
- **4 stances**: Minimum for robust homography. Distributed across workspace for low extrapolation error.
- **Z=100mm for stances**: High enough to be visible in all overhead camera views, low enough that perspective distortion is minimal.
- **Homography (2D) over full 3D**: The manipulation workspace is essentially planar (table surface). Homography captures this plane perfectly with fewer parameters.

---

## Comparison: Homography vs. Kabsch 3D Calibration

| Aspect | Homography (this module) | Kabsch 3D (`calibration.py`) |
|--------|--------------------------|-------------------------------|
| RMSE | **0.12 mm** | 0.85 mm |
| Dimensions | 2D (XY plane) | Full 3D |
| Stances needed | 4 | 8 |
| Autonomous | Yes (gripper flick) | Semi (needs visible marker) |
| Use case | Hardware pipeline | Simulation pipeline |
