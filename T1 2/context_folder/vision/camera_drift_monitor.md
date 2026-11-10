# `camera_drift_monitor.py` — Online Camera Recalibration (EC-07)

**Module**: `src/vision/camera_drift_monitor.py`
**Purpose**: Detects camera displacement (bumps) in real-time and performs sub-2ms online recalibration.

---

## Edge Case: EC-07 — Camera Bump During Operation

If someone bumps the camera mount during a sort run, all pixel-to-robot transforms become invalid. This module detects drift and recalibrates on-the-fly.

---

## Constants

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `NOMINAL_BOARD_LANDMARKS` | 8 fixed 3D points | Socket centers + board edge reference points in robot frame |
| `MAX_TRANSLATION_DRIFT_MM` | 4.0 mm | Threshold to trigger recalibration |
| `MAX_ROTATION_DRIFT_DEG` | 1.5° | Threshold to trigger recalibration |

---

## Key Methods

### `compute_drift(current_landmarks) → (translation_mm, rotation_deg)`

1. Projects current camera-observed points to robot space using existing calibration
2. Compares against `NOMINAL_BOARD_LANDMARKS`
3. Translation drift = Euclidean norm of mean position shift
4. Rotation drift = angle from trace formula: `θ = arccos((trace(R) - 1) / 2)`

### `recalibrate_online(camera_points, robot_points) → (R, t)`

Full Kabsch SVD recalibration in under 2ms:
1. Centroid subtraction on both point sets
2. Covariance matrix H = camera_centered.T @ robot_centered
3. SVD: U, S, Vt = svd(H)
4. R = Vt.T @ U.T (with reflection handling: det(R) < 0 → flip)
5. t = centroid_robot - R @ centroid_camera
6. Saves updated calibration to `camera_calib.yaml`

---

## Detection Workflow
```
Periodic check (every N frames):
    observe board landmarks in camera
    compute_drift()
    if translation > 4.0mm OR rotation > 1.5°:
        recalibrate_online()
        log warning: "Camera bump detected, recalibrated"
    else:
        continue with existing calibration
```

---

## Design Decisions

- **8 landmark points**: More than minimum 3 for overdetermined system. Redundancy improves RMSE.
- **Dual thresholds**: Separate translation and rotation checks catch pure shifts AND pure rotations.
- **Sub-2ms recalibration**: SVD on 8 points is <2ms. No pipeline stall needed.
- **Board landmarks as reference**: Board socket positions are the most precisely known 3D points (machined holes).
