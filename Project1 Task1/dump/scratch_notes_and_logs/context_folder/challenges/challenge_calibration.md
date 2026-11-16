# Challenge: Camera-to-Robot Calibration

**Problem**: Accurately mapping pixel coordinates to physical robot coordinates is the foundation of all autonomous operation. Two approaches were developed.

---

## Approach 1: Kabsch SVD 3D Calibration (`calibration.py`)

### Method
- Manual/semi-automatic: move arm to N poses, collect paired (camera, robot) 3D points
- SVD gives optimal rigid transform (R, t) minimizing least-squares error
- Full 3D: handles arbitrary camera mounting angles

### Results
- **RMSE: 0.85mm** (hardware), **0.08mm** (simulation)
- Used in simulation pipeline (marker-based detection)

### Limitations
- Requires depth measurement (noisy on RealSense D435i)
- Depth noise propagates into calibration error
- Needs visible marker at calibration poses

---

## Approach 2: Autonomous Homography (`autonomous_homography_calibrator.py`)

### Method
- Fully autonomous: robot creates own calibration data via gripper flick
- Computes 2D planar homography (3×3 matrix)
- Only needs RGB, no depth

### Results
- **RMSE: 0.12mm** — 7× better than 3D Kabsch
- Used in hardware pipeline

### Why Better
1. No depth sensor noise (homography uses only pixel positions)
2. Models the actual manipulation plane directly
3. Autonomous = no human measurement error
4. Gripper flick gives sub-pixel TCP localization

---

## When to Use Which

| Scenario | Best Approach | Why |
|----------|--------------|-----|
| Hardware pipeline | Homography | Best 2D accuracy, autonomous |
| Simulation pipeline | Kabsch 3D | Marker tracking available, no depth noise |
| Non-planar manipulation | Kabsch 3D | Homography only works for one plane |
| Camera bump recovery | Kabsch SVD (online) | Fast recalibration from board landmarks |

---

## Key Calibration Numbers

| Metric | Kabsch 3D | Homography |
|--------|-----------|-----------|
| RMSE (hardware) | 0.85 mm | **0.12 mm** |
| RMSE (simulation) | 0.08 mm | N/A |
| Stances | 8 | 4 |
| Depth required | Yes | No |
| Autonomous | No (semi) | **Yes** |
| 3D capable | Yes | No (2D only) |

---

## Residual Error Budget

At RMSE = 0.12mm (homography), the dominant remaining error sources are:
1. Camera pixel quantization (~0.5 px = ~0.7mm)
2. Peg centroid estimation from noisy contours (~0.5mm)
3. FK errors from encoder resolution (4096 counts/rev = 0.088°/count)
4. Mechanical backlash in Dynamixel gearboxes

Total system accuracy for peg placement: **~5–7mm** (confirmed by demo run: vision error 6.4mm)
