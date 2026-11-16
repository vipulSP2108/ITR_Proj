# Hardware Devices & Safety Index (`device/`)

This directory catalogs all hardware devices, physical specifications, object dimensions, images, and electrical safety protection layers.

---

## Catalog Index

| Document / Tool | Description |
| :--- | :--- |
| **[`CAMERA_INFO.md`](CAMERA_INFO.md)** | Detected specs of the **Intel RealSense D435i** (serial number, USB 3.1 speed, resolution, macOS permissions) |
| **[`ROBOT_INFO.md`](ROBOT_INFO.md)** | Full kinematic parameters, link lengths, servo IDs, and ready pose specs of the **OpenMANIPULATOR-X** |
| **[`OBJECTS_AND_BOARD.md`](OBJECTS_AND_BOARD.md)** | Physical dimensions, color profiles (HSV), and socket order of the 4 pegs and the laser-cut board |
| **[`SAFETY_AND_PROTECTION.md`](SAFETY_AND_PROTECTION.md)** | Electrical safety guide, voltage boundaries, thermal limits, and USB galvanic isolation advice |
| **[`hardware_protector.py`](hardware_protector.py)** | Active **Software Auto-Disconnector Layer** that monitors voltage, temp, and load in real time |
| **[`images/`](images/)** | Reference images of the real physical pegs/board and CAD ready pose |

---

## Quick Health Diagnostic
To inspect the real-time electrical health of the arm servos:
```bash
python3 device/hardware_protector.py
```

