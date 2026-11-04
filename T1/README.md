# Tri-Wizard Robotics: Task 1 (The Peg-Sorting)

Autonomous perception, kinematics planning, and pick-and-place manipulation for sorting geometric pegs using ROBOTIS OpenMANIPULATOR-X and an Intel RealSense D435i RGB-D Camera.

---

## 1. Setup & Environment

* **Operating System**: macOS (Apple Silicon / Intel) or Linux (Ubuntu 22.04)
* **Python Version**: Python 3.10+ (tested on Python 3.13)
* **Hardware**:
  * **Robot**: ROBOTIS OpenMANIPULATOR-X (5x DYNAMIXEL XM430-W350-T actuators)
  * **Controller**: ROBOTIS U2D2 USB-to-RS485 adapter (`/dev/cu.usbserial-*`)
  * **Sensor**: Intel RealSense D435i Depth Camera (mounted on aluminum vertical profile)
  * **Power**: 12V DC, 5A power supply

### Installation
```bash
pip install dynamixel-sdk pyserial opencv-python-headless numpy scipy pyyaml matplotlib
```

---

## 2. Quick Start / Usage

### Mode A: Interactive Keyboard Teleoperation (Baseline)
Manual control with real-time joint angle feedback and emergency stop:
```bash
cd openmanipulator_test
python3 04_keyboard_teleop.py
```
* **Controls**:
  * Joint 1 (Base): `A` / `D`
  * Joint 2 (Shoulder): `W` / `S`
  * Joint 3 (Elbow): `I` / `K`
  * Joint 4 (Wrist): `U` / `J`
  * Gripper: `O` / `P` or `SPACE`
  * Default Ready Stance: `H`
  * Emergency Stop: `ESC` or `X`

### Mode B: Autonomous Pick-and-Place (Full Autonomy)

#### 1. Test Single Figure in Dry Run (Mid-Air, Zero Risk):
```bash
cd T1
python3 scripts/run_autonomous_sorter.py --single square --dry-run
```

#### 2. Test Single Figure Live on Tabletop:
```bash
python3 scripts/run_autonomous_sorter.py --single square --live
```

#### 3. Sort All 4 Figures Autonomously:
```bash
python3 scripts/run_autonomous_sorter.py --all --live
```

---

## 3. Configuration & Parameters

* **Color Thresholds**: [`config/colors.yaml`](config/colors.yaml)
  * Calibrated HSV lower/upper bounds for Yellow (Triangle), Blue (Square), Orange/Red (Hexagon), and Green (Circle).
* **Camera Calibration**: [`config/camera_calib.yaml`](config/camera_calib.yaml)
  * Rigid 3D transformation $(\mathbf{R}, \mathbf{t})$ from Camera Frame to Robot Base Frame.
* **Hardware Profiles**: See [`device/`](../device/) for complete device spec sheets.

---

## 4. Electrical Safety & Protection

* **Software Watchdog**: [`device/hardware_protector.py`](../device/hardware_protector.py)
  * Continuously supervises voltage ($10.5\text{V} \sim 14.6\text{V}$), temperature ($< 65^\circ\text{C}$), and motor load.
  * Trips an automated disconnector, cutting motor torque instantly upon any electrical surge or mechanical stall.
* **Physical Isolation**: Recommended USB Galvanic Isolator (ADuM3160) to isolate Mac USB lines from 12V motor transients.

