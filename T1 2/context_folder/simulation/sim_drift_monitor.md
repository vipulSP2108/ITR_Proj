# `drift_monitor.py` — Camera Extrinsic Drift Detection

**Module**: `sim_demo/drift_monitor.py` (92 lines)
**Purpose**: Online camera extrinsic drift detection and correction. Monitors socket board landmarks to detect bumps, vibration, or thermal drift during operation.

---

## Class: `SimCameraDriftMonitor`

### Tolerances

| Parameter | Value | Unit |
|-----------|-------|------|
| `tol_trans_mm` | 4.0 | mm |
| `tol_rot_deg` | 1.5 | degrees |

### Methods

#### `set_baseline_from_holes(holes)`
Stores baseline 3D positions of all detected sockets. These are the reference against which future captures are compared.

#### `check_and_update(current_holes, T_robot_cam, cam_model)`

1. Match current socket detections to baseline by nearest-neighbor
2. Compute Kabsch alignment between baseline and current socket positions
3. If translation drift > 4.0 mm OR rotation drift > 1.5°:
   - Flag `drift_detected = True`
   - Return updated T_robot_cam (corrected transform)
4. Otherwise return existing transform unchanged

**Computation time**: <1 ms (Kabsch on 4 points)

---

## Design Rationale

The socket board is fixed to the table. Its holes have known relative positions. If the camera shifts (vibration, bump, thermal expansion of the mount), the detected hole positions shift by the same rigid transform. By comparing detected positions against the baseline, we can:

1. Detect that drift occurred
2. Compute the exact correction
3. Apply it to T_robot_cam without re-running full calibration

This addresses **Edge Case EC-07** (Camera Extrinsic Shift).

---

## Sim vs Hardware Versions

| Property | `sim_demo/drift_monitor.py` | `src/vision/camera_drift_monitor.py` |
|----------|----------------------------|--------------------------------------|
| Input | Socket 3D positions | Camera calibration YAML |
| Baseline | Socket positions from detection | Loaded from config file |
| Kabsch | In-module | Via `EyeToHandCalibrator` |
| Purpose | Same: detect and correct drift | Same |
