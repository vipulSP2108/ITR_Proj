# `single_camera_pipeline.py` — Complete Perception Pipeline

**Module**: `src/vision/single_camera_pipeline.py`
**Purpose**: End-to-end scene understanding: color segmentation → shape detection → depth slicing → 3D deprojection → robot frame transformation.

---

## Dataclass: `TargetPose`

| Field | Type | Description |
|-------|------|-------------|
| `shape` | str | Shape classification result |
| `color` | str | Color name |
| `centroid_2d` | (int, int) | Pixel location |
| `camera_3d` | (float, float, float) | Camera-frame XYZ (mm) |
| `robot_3d` | (float, float, float) | Robot base-frame XYZ (mm) |
| `yaw_deg` | float | Orientation angle |
| `is_socket` | bool | Peg or socket flag |

---

## Class: `SingleCameraPipeline`

### Constructor
```python
SingleCameraPipeline(calib_path="config/camera_calib.yaml")
```

**Camera intrinsics** (hardcoded, matching RealSense D435i):
- fx = 615, fy = 615, cx = 320, cy = 240

Loads: `EyeToHandCalibrator`, `ColorSegmenter`, `ShapeDetector`

### `depth_slice_top_face(mask, depth_map, slice_thickness_mm=20.0) → np.ndarray`

**Critical function**: Isolates the top polygon face of a peg from depth data.

1. Apply mask to depth map (zero out non-object pixels)
2. Find **5th percentile** of non-zero depths → `Z_min` (top surface)
3. Keep only pixels within `[Z_min, Z_min + slice_thickness_mm]`
4. Return: refined binary mask of just the top face

**Why 5th percentile?** Rejects depth noise outliers while reliably finding the topmost surface. The 20mm slice thickness matches peg heights (~15–20mm).

### `process_scene(bgr_image, depth_map) → (list[TargetPose], list[TargetPose])`

**Full pipeline**:
1. `ColorSegmenter.segment_all()` → color masks
2. For each color mask:
   - `depth_slice_top_face()` → refined top-face mask
   - `ShapeDetector` classifies contours
   - Deprojection: pixel (u, v) + depth → camera 3D point
   - `EyeToHandCalibrator` transforms camera → robot frame
3. `ShapeDetector.detect_sockets()` on grayscale → socket poses
4. Returns: `(detected_pegs, detected_sockets)`

### Deprojection Formula
```
X_cam = (u - cx) * depth / fx
Y_cam = (v - cy) * depth / fy
Z_cam = depth
```
Then: `P_robot = R @ P_camera + t` (from calibration)

---

## Design Decisions

- **Depth slicing before shape classification**: Critical for stacked/touching objects. The top face slice gives clean contours even when pegs are adjacent.
- **5th percentile, not minimum**: Raw minimum depth catches noise spikes. 5th percentile is robust.
- **Separate peg and socket detection paths**: Pegs use color+depth; sockets use grayscale threshold on the board area. Different visual signatures need different approaches.

---

## Dependency Chain
```
Camera frames (RGB-D)
    → ColorSegmenter.segment_all()
    → depth_slice_top_face()
    → ShapeDetector.classify_contour()
    → Deprojection (intrinsics)
    → EyeToHandCalibrator.transform()
    → TargetPose (robot_3d)
    → Planner / RecoveryManager
```
