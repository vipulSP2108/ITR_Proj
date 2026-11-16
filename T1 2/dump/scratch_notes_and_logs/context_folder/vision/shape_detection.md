# `shape_detection.py` — Shape Classification & 3D Pose Estimation

**Module**: `src/vision/shape_detection.py`
**Purpose**: Classify peg/socket shapes from contours, estimate orientation (yaw), and compute 3D centroids via depth deprojection.

---

## Dataclass: `DetectedObject`

| Field | Type | Description |
|-------|------|-------------|
| `shape` | str | `"circle"`, `"square"`, `"triangle"`, `"hexagon"` |
| `color` | str | Color name from segmentation |
| `centroid_2d` | (int, int) | Pixel coordinates (u, v) |
| `centroid_3d` | (float, float, float) | Camera-frame XYZ in mm |
| `yaw_deg` | float | Orientation angle for insertion alignment |
| `contour` | np.ndarray | OpenCV contour |
| `confidence` | float | Classification confidence |
| `symmetry_order` | int | Rotational symmetry (circle=∞, hex=6, square=4, tri=3) |

---

## Class: `ShapeDetector`

### Constants
- `min_area = 400` px² — reject noise
- `max_area = 60000` px² — reject background blobs

### `classify_contour(cnt) → (str, float, int)`

Uses **enclosing circle area ratio** = contour_area / enclosing_circle_area:

| Ratio Range | Shape | Symmetry Order |
|-------------|-------|----------------|
| > 0.91 | Circle | ∞ (treated as 1) |
| < 0.58 | Triangle | 3 |
| 0.58 – 0.76 | Square | 4 |
| 0.77 – 0.91 | Hexagon | 6 |

Secondary check: `cv2.approxPolyDP` with epsilon = 2% of arc length → vertex count validation.

### `_get_robust_depth_at_centroid(depth_map, cx, cy) → float`
Median of non-zero depth values in a **3×3 pixel patch** around centroid. Handles depth dropout from IR stereo matching failures.

### `_estimate_orientation(contour, shape) → float`

| Shape | Method |
|-------|--------|
| Triangle | Vector from centroid to apex (vertex farthest from centroid) |
| Square | `cv2.minAreaRect()` angle |
| Hexagon | `cv2.minAreaRect()` angle |
| Circle | 0° (no orientation needed) |
| Fallback | PCA on contour points |

### `calculate_minimal_rotation(current_yaw, target_yaw, symmetry_order) → float`
Computes minimum rotation considering symmetry: `period = 360° / symmetry_order`. Maps rotation into `[-period/2, +period/2]`.

**Example**: Square at 47° targeting 0° → minimal rotation = -43° (not -47°), because 90° symmetry means 47° ≡ -43° mod 90°.

### `detect_sockets(gray_image) → list[DetectedObject]`
1. Otsu threshold on grayscale
2. Morphological opening (7×7 kernel)
3. Find contours in `board_roi` (inset 6px from edges to avoid board outline)
4. Classify each contour

---

## Design Decisions

- **Area ratio over vertex counting alone**: Vertex count from `approxPolyDP` is fragile under camera noise. Area ratio is scale-invariant and robust.
- **3×3 median depth**: Single-pixel depth is unreliable with IR stereo. Median over a small patch rejects outliers while staying spatially accurate.
- **Symmetry-aware rotation**: Avoids unnecessary 180° or 90° rotations that waste time and risk cable tangles.

---

## Dependency Chain
```
ColorSegmenter masks → ShapeDetector.classify_contour()
                     → ShapeDetector._estimate_orientation()
                     → ShapeDetector._get_robust_depth_at_centroid()
                     → DetectedObject
                     → SingleCameraPipeline / RecoveryManager
```
