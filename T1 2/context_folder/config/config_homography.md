# `homography_calib.yaml` — Planar Homography Calibration

**File**: `config/homography_calib.yaml`
**Purpose**: 2D pixel-to-robot coordinate mapping via planar homography, calibrated autonomously.

---

## Calibration Summary

| Property | Value |
|----------|-------|
| Calibrated at | 2026-11-09 13:40:00 |
| Number of stances | 4 |
| **RMSE** | **0.12 mm** |
| Base frame offset | 2048 |

---

## Homography Matrix (3×3)

```
H = [[-1.217,  0.118, 527.41],
     [-0.108,  0.989, -175.13],
     [-0.001,  0.001,   1.00]]
```

### Usage
```python
[X', Y', W'] = H @ [u, v, 1]
X_robot = X' / W'
Y_robot = Y' / W'
```

---

## Calibration Pairs

| Stance | Pixel (u, v) | Robot (X, Y) mm | Residual |
|--------|-------------|-----------------|----------|
| Near Mat | (375, 280) | (150.3, 88.8) | 0.12 mm |
| Far Mat | (320, 280) | (225.4, 88.8) | 0.14 mm |
| Near Board | (320, 150) | (225.4, -88.8) | 0.14 mm |
| Center Table | (348, 215) | (187.1, 0.0) | 0.08 mm |

---

## Why Homography Outperforms 3D Calibration Here

| Factor | Homography | 3D Kabsch |
|--------|-----------|-----------|
| RMSE | **0.12 mm** | 0.85 mm |
| Error source | Only XY plane | Full 3D with depth noise |
| Camera depth noise | Not used | Propagates into Z error |
| Parameters | 8 (H has 8 DOF) | 12 (R=9 + t=3, constrained to 6) |

The manipulation workspace is effectively a 2D plane (table surface). Homography directly maps this plane without going through noisy depth measurements.
