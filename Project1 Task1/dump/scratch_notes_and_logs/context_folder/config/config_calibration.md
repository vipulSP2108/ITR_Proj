# `camera_calib.yaml` — Camera-to-Robot 3D Calibration

**File**: `config/camera_calib.yaml`
**Purpose**: Rigid transform (R, t) from camera frame to robot base frame, computed via Kabsch SVD.

---

## Calibration Data

### Rotation Matrix
```
R = [[ 0.735,  0.625, -0.263],
     [ 0.678, -0.677,  0.285],
     [-0.000, -0.388, -0.922]]
```

### Translation Vector
```
t = [280, -180, 420] mm
```

### Fit Quality
- **RMSE = 0.85 mm**

---

## Usage
```python
P_robot = R @ P_camera + t
```

Transforms any point from camera 3D coordinates to robot base frame.

---

## Notes
- RMSE 0.85mm is good for this class of eye-to-hand calibration with a consumer depth camera
- The translation [280, -180, 420] reflects: camera is ~280mm forward of base in X, 180mm to the right in Y, 420mm above in Z
- Used by `SingleCameraPipeline` for the primary 3D perception path
- The `autonomous_homography_calibrator` achieves better 2D accuracy (0.12mm) for the planar workspace
