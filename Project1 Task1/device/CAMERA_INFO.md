# Intel RealSense Depth Camera D435i Device Profile

Hardware profile detected directly via macOS System Profiler on USB 3.1 SuperSpeed bus.

---

## 1. Hardware Specifications

| Parameter | Detected Value / Specification |
| :--- | :--- |
| **Model** | **Intel(R) RealSense(TM) Depth Camera 435i** |
| **Manufacturer** | Intel Corporation |
| **Vendor ID (VID)** | `0x8086` |
| **Product ID (PID)** | `0x0b3a` |
| **Serial Number** | `261143061202` |
| **Firmware Version** | `50.f1` |
| **Bus Connection** | USB 3.1 Gen 1 SuperSpeed (Up to 5 Gb/s) |
| **Power Consumption**| 720 mA required (900 mA provided by Mac USB port) |
| **Location ID** | `0x01200000` |

---

## 2. Sensor Capabilities

* **Depth Technology**: Active Stereo Infrared with built-in IR Pattern Projector.
  * Depth FOV: $87^\circ \pm 3^\circ \times 58^\circ \pm 1^\circ \times 95^\circ \pm 3^\circ$
  * Depth Stream Resolution: Up to $1280 \times 720$ at 90 fps.
  * Ideal Operating Range: $0.3\text{ m}$ to $3.0\text{ m}$.
* **RGB Sensor**:
  * RGB Stream Resolution: Up to $1920 \times 1080$ at 30 fps.
  * RGB Sensor FOV: $69^\circ \times 42^\circ \times 77^\circ$.
* **Inertial Measurement Unit (IMU)**:
  * Bosch BMI055 (6-DOF: 3-axis accelerometer + 3-axis gyroscope).

---

## 3. macOS Setup & Camera Permissions

On macOS, camera video streams are protected by Apple's Privacy & Security Sandbox:

1. Open **System Settings** on your Mac.
2. Navigate to **Privacy & Security** $\to$ **Camera**.
3. Toggle the switch **ON** for:
   * **Terminal** (or **iTerm2** / your IDE / **Visual Studio Code** / **Python**).
4. If prompted with *"Terminal would like to access the camera"*, click **Allow**.

---

## 4. Testing the Camera on macOS

Run the camera test script:
```bash
cd /Users/aai/Desktop/ITR_Proj/T1
python3 scripts/test_camera_stream.py
```

