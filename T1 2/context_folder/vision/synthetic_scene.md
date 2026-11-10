# `synthetic_scene.py` — Synthetic RGB-D Scene Generator

**Module**: `src/vision/synthetic_scene.py`
**Purpose**: Generates realistic synthetic camera frames for unit testing the vision pipeline without hardware.

---

## Class: `CameraIntrinsics`

Matches RealSense D435i:
- fx = 615, fy = 615, cx = 320, cy = 240
- Resolution: 640 × 480

### Utility Methods
- `deproject_pixel_to_point(u, v, depth_mm) → (X, Y, Z)` — Inverse pinhole projection
- `project_point_to_pixel(x, y, z) → (u, v)` — Forward pinhole projection

---

## Class: `SyntheticSceneGenerator`

### Depth Layers
| Layer | Depth (mm) | Description |
|-------|------------|-------------|
| Table surface | 500 | Background plane |
| Board surface | 475 | Wooden board (25mm thick) |
| Peg top face | 400 | Peg sitting on board (~75mm above board accounting for peg height) |

### Noise Model (Matching RealSense D435i Characteristics)

| Noise Type | Parameter | Purpose |
|------------|-----------|---------|
| Vignette gradient | Radial falloff from center | Simulates lens light falloff |
| Gaussian color noise | σ = 3.5 | Sensor read noise |
| Edge depth dropout | Random zeroing at depth edges | IR stereo matching failure at occlusion boundaries |
| Depth fluctuation | σ = 1.2 mm | Depth measurement noise |

---

## Design Decisions

- **Separate from MuJoCo**: This generator creates OpenCV-compatible images for unit testing `ColorSegmenter`, `ShapeDetector`, and `SingleCameraPipeline` without MuJoCo.
- **Realistic noise**: The noise model reproduces the actual failure modes of the RealSense D435i (edge dropout, IR dot interference, depth noise).
- **Fixed layer depths**: Matches physical setup dimensions for correct depth slicing behavior.
