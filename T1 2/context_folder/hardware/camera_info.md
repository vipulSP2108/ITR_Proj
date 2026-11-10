# Camera Hardware — Intel RealSense D435i

**Reference**: `device/CAMERA_INFO.md`

---

## Specifications

| Property | Value |
|----------|-------|
| **Model** | Intel RealSense D435i |
| **Connection** | USB 3.1 Gen 1 SuperSpeed |
| **Serial** | 261143061202 |
| **Firmware** | 50.f1 |
| **USB** | VID 0x8086, PID 0x0b3a |

---

## Depth Sensor

| Parameter | Value |
|-----------|-------|
| Technology | Active stereo IR |
| Max resolution | 1280×720 @ 90 fps |
| Range | 0.3–3.0 m |
| FOV | 87° × 58° × 95° (H × V × D) |

## RGB Sensor

| Parameter | Value |
|-----------|-------|
| Max resolution | 1920×1080 @ 30 fps |
| Working resolution | 640×480 (pipeline default) |
| FOV | 69° × 42° × 77° (H × V × D) |
| Recording resolution | 1280×720 @ 15 fps (live demo) |

## IMU

| Parameter | Value |
|-----------|-------|
| Sensor | Bosch BMI055 (6-DOF) |
| Data | Accelerometer + Gyroscope |

---

## Mounting Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| Height above table | 470 mm | `workspace_dimensions.yaml` |
| Focal length | 344 px | `workspace_dimensions.yaml` |
| Principal point | (320, 240) px | `autonomous_homography_calibrator.py` |
| Table scale | 1.366 mm/px | 470 / 344 |
| Orientation | Overhead (straight down) | Sim assumption |

**Critical uncertainty**: The sim assumes a perfectly vertical overhead camera at 470mm. Legacy notes indicate the real camera may be angled 45–55°. This changes all projective geometry — parallax, occlusion patterns, and pixel-to-robot mapping.

---

## Calibration Data

### Eye-to-Hand (3D Rigid Transform)
- **File**: `config/camera_calib.yaml`
- **Method**: Kabsch/Umeyama SVD from arm-motion correspondence
- **Translation**: [280, -180, 420] mm
- **RMSE**: 0.85 mm

### Homography (2D Pixel→Robot)
- **File**: `config/homography_calib.yaml`
- **Method**: Autonomous gripper-flick TCP detection at 4 stances
- **RMSE**: 0.12 mm (reprojection)

---

## macOS Notes

- Requires camera permissions for Terminal/IDE (System Settings → Privacy → Camera)
- First few frames may be dark (auto-exposure settling) — code includes 8-frame warm-up
- `test_camera_stream.py` probes indices 0–2 to find valid camera
