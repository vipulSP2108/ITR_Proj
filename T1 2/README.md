# Task 1: Autonomous Peg Sorting Robot (Active Tested Workspace)

This directory ([`T1 2/`](file:///Users/aai/Desktop/ITR_Proj/T1%202)) is the **primary tested workspace** for Task 1 of the Tri-Wizard Robotics Competition: Autonomous Perception, Kinematics Planning, and Pick-and-Place Manipulation using the **ROBOTIS OpenMANIPULATOR-X** and an **Intel RealSense D435i** Depth Camera.

> [!NOTE]
> All scripts and commands in this folder run standalone directly on the workstation connected via USB to the U2D2 adapter and RealSense camera. macOS users must run commands from a real terminal (`Terminal.app` or IDE terminal) to grant camera access.

---

## 1. Hardware Architecture & Interfaces

* **Robot Arm**: ROBOTIS OpenMANIPULATOR-X (4-DOF articulated arm + parallel gripper).
* **Actuators**: 5x DYNAMIXEL XM430-W350-T smart servos running at 1 Mbps on DYNAMIXEL Protocol 2.0:
  - ID 11: Base Pan
  - ID 12: Shoulder Pitch
  - ID 13: Elbow Pitch
  - ID 14: Wrist Pitch
  - ID 15: Gripper
* **USB Interface**: ROBOTIS U2D2 (`/dev/cu.usbserial-*` on macOS, `/dev/ttyUSB0` on Linux).
* **Power Supply**: 12V DC, 5A connected via U2D2 power hub.
* **Camera**: Intel RealSense D435i mounted overhead at $Z \approx 510\text{ mm}$ above the green sorting mat.

---

## 2. Pre-Flight Hardware Health Checks (30 Seconds)

Open a terminal in [`T1 2/`](file:///Users/aai/Desktop/ITR_Proj/T1%202):
```bash
cd /Users/aai/Desktop/ITR_Proj/"T1 2"
```

### A. Check Arm Actuators & 12V Supply
```bash
python3 calibration/00_check_arm.py
```
*Expected Output*: Five status lines with `volt=12.0V hw_err=0x00`, followed by `ARM OK`.

### B. Verify RealSense Camera Stream
* macOS:
  ```bash
  ffmpeg -f avfoundation -list_devices true -i "" 2>&1 | grep "435i RGB"
  ```
* Linux:
  ```bash
  rs-enumerate-devices | grep "Name"
  ```

### C. Visual Inspection & Layout Suggestion
```bash
# Check what the robot sees (outputs runs/<time>/scan1_annotated.jpg):
python3 pick_place.py --detect-only

# View suggested peg layout on live feed (outputs suggested_layout.jpg):
python3 pick_place.py --suggest-layout
```

---

## 3. Autonomous Execution Commands

Execute the autonomous sorting pipeline via [`pick_place.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/pick_place.py):

| Goal | Command | Description |
| :--- | :--- | :--- |
| **Full Autonomous Run** | `python3 pick_place.py` | Detects all pegs and holes, sorts all shapes in optimal path order. |
| **Custom Shape Order** | `python3 pick_place.py --order circle,square,hexagon,triangle` | Enforces exact sequence; treats remaining pegs as collision obstacles. |
| **Single Shape Isolation** | `python3 pick_place.py --shape circle` | Sorts one specific shape (options: `circle`, `square`, `hexagon`, `triangle`). |
| **Planning Dry-Run** | `python3 pick_place.py --plan-only` | Generates motion paths, swept finger envelopes, and obstacles in `runs/<time>/plan1.jpg` without moving motors. |
| **Force Insertion** | `python3 pick_place.py --force` | Bypasses small orientation tolerances if peg is slightly misaligned with hole. |

Every run logs complete telemetry, scan images, collision envelopes, and video to a timestamped folder: `runs/<date_time>/`.

---

## 4. Camera-to-Robot Extrinsic Calibration

If the camera, arm base, or sorting mat shifts position, run the automated 40-point calibration routine (~10 minutes):

```bash
./run_calibration.sh
```

### Verification & Jog Testing
```bash
# Fingertip moves 25 mm above robot coordinate (150, 60); saves photo with projected crosshair:
python3 calibration/02_verify_calibration.py --xy 150 60

# Interactive jog check across all reachable ArUco markers:
python3 calibration/02_verify_calibration.py --jog
```

### Healthy Calibration Benchmarks
* Poses used: $\ge 25$ of 40
* Fingertip reprojection RMS: $< 5.0\text{ px}$ ($\approx 2.5\text{ mm}$)
* ArUco corner reprojection RMS: $< 1.0\text{ px}$
* Calibrated camera height: $\approx 510\text{ mm}$

Output written to [`config/camera_robot_calib.yaml`](file:///Users/aai/Desktop/ITR_Proj/T1%202/config/camera_robot_calib.yaml).

---

## 5. Important Hardware Nuances (Discovered in Lab)

> [!WARNING]
> * **Zero Arm Pose**: Joints 12–14 at raw encoder count `2048` define an **L-shaped arm** (forearm horizontal), NOT straight vertical.
> * **Gripper Position Counts**:
>   - Fully Open = `3250 counts`
>   - Fingers Touching = `~1385 counts`
>   - Operates in Dynamixel Current-Limited Mode 5 to prevent servo stalling.
> * **Workspace Rules**:
>   - Holes must remain within $250\text{ mm}$ radial distance from the robot base.
>   - Pegs placed side-by-side require $9\text{–}10\text{ cm}$ center-to-center spacing to permit finger clearance.

---

## 6. Hardware Specifications & Bringup Test Suites

Inside this directory are the hardware documentation and low-level diagnostic suites:

* **[`device/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device)**:
  - [`device/ROBOT_INFO.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/ROBOT_INFO.md): Complete mechanical and joint specifications for the OpenMANIPULATOR-X.
  - [`device/CAMERA_INFO.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/CAMERA_INFO.md): RealSense D435i resolutions, intrinsics, and mounting geometries.
  - [`device/SAFETY_AND_PROTECTION.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/SAFETY_AND_PROTECTION.md): Electrical limits, brownout thresholds, and hardware protector documentation.
  - [`device/hardware_protector.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/hardware_protector.py): Standalone background safety daemon monitoring voltage, thermal levels, and motor load.
  - [`device/images/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/device/images): High-resolution physical lab reference photos of the real robot arm, pegs, sockets, and camera mounts.
* **[`openmanipulator_test/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/openmanipulator_test)**:
  - Low-level physical hardware bringup test suite (`01_scan_and_ping.py` through `06_test_pick_and_place.py`), teleoperation, and home calibration.

---

## 7. Internal Workspace Modularization & Dump Buffer

To maintain high readability and avoid confusion during laboratory operations, non-active assets have been modularized into [`dump/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/dump):
* **`dump/deprecated_runners/`**: Early test scripts (`hw_run.py`, `hw_test_step1.py`, `run_hardware_pipeline*.py`) using outdated kinematics.
* **`dump/historical_demos/`**: Legacy prototypes from early development phases (`demo1_20261107`, `demo_runs`, `sim`, `sim_demo`, `scripts`, `tests`).
* **`dump/test_images_and_output/`**: Early test outputs and image folders (`hw_test_output/`, `calibration_img/`, `test_images/`).
* **`dump/scratch_notes_and_logs/`**: Early logs, scratch notes, and context files (`MUJOCO_LOG.TXT`, `imp.md`, `context_folder/`).
* See [`dump/README.md`](file:///Users/aai/Desktop/ITR_Proj/T1%202/dump/README.md) for full details.
