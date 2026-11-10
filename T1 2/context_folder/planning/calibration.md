# `calibration.py` — Eye-to-Hand Calibration (Kabsch/Umeyama SVD)

**Module**: `src/planning/calibration.py`
**Purpose**: Computes the rigid transformation (R, t) from camera frame to robot base frame.

---

## Problem Statement

Camera sees objects in camera coordinates (X_cam, Y_cam, Z_cam). Robot needs positions in robot base coordinates (X_rob, Y_rob, Z_rob).

**Transform**: `P_robot = R × P_camera + t`

Where R is 3×3 rotation matrix and t is 3×1 translation vector.

---

## Class: `EyeToHandCalibrator`

### `compute_rigid_transform(camera_points, robot_points) → (R, t, rmse)`

**Kabsch/Umeyama Algorithm**:

1. **Centroid subtraction**: Remove mean from both point sets
   ```
   cam_centered = cam_points - mean(cam_points)
   rob_centered = rob_points - mean(rob_points)
   ```

2. **Cross-covariance matrix**: `H = cam_centered.T @ rob_centered`

3. **Singular Value Decomposition**: `U, S, Vt = svd(H)`

4. **Rotation matrix**: `R = Vt.T @ U.T`
   - **Reflection check**: If `det(R) < 0`, flip sign of last column of Vt before recomputing R

5. **Translation**: `t = centroid_robot - R @ centroid_camera`

6. **RMSE**: Root mean square of `||R @ cam_point + t - rob_point||` over all pairs

### `save(filepath)` / `load(filepath)`

Saves/loads to YAML format:
```yaml
rotation_matrix: [[...], [...], [...]]
translation_vector_mm: [280, -180, 420]
rmse: 0.85
```

---

## Calibration Data (`camera_calib.yaml`)

```
R = [[ 0.735,  0.625, -0.263],
     [ 0.678, -0.677,  0.285],
     [-0.000, -0.388, -0.922]]

t = [280, -180, 420] mm

RMSE = 0.85 mm
```

---

## Calibration Procedure (Hardware)

1. Move robot TCP to N known positions (N ≥ 4, typically 8)
2. At each position, record:
   - Robot joint angles → FK → robot-frame TCP position
   - Camera observation → camera-frame TCP position
3. Feed paired points to `compute_rigid_transform()`
4. Save result to `camera_calib.yaml`

---

## Simulation Calibration (from `run_pipeline.py`)

In simulation, calibration uses the tool marker (colored ball) visible to the overhead camera:
1. Move arm to 8 diverse poses at Z=100mm
2. Camera detects marker position via sphere fitting
3. FK gives robot-frame position
4. Kabsch SVD computes (R, t)
5. Typical fit: **RMSE = 0.08mm**, reprojection = 0.37px

---

## Design Decisions

- **Kabsch SVD over least-squares**: SVD is numerically stable and guarantees orthogonal R (no shearing artifacts).
- **Reflection handling**: Without det(R) check, mirrored solutions are possible. The flip correction ensures proper rotation.
- **8 calibration poses**: Minimum is 3 (for 6-DOF rigid transform with 3 rotation + 3 translation unknowns). 8 provides overdetermined system for better RMSE.
- **RMSE < 1mm achieved**: 0.85mm on hardware, 0.08mm in simulation. Simulation is better because no physical measurement noise.
