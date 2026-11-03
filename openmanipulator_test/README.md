# OpenMANIPULATOR-X Quick Hardware Test on macOS

This folder provides a plug-and-play, safe test suite to verify that your Mac can communicate with and control the physical **ROBOTIS OpenMANIPULATOR-X** arm via the **U2D2** adapter.

---

## 1. What SDK and Tools Are We Using?

* **SDK**: Official ROBOTIS `dynamixel-sdk` (Python 3, Protocol 2.0).
* **Communication**: `pyserial` talking directly to the U2D2 FTDI chip on macOS (`/dev/cu.usbserial-*`).
* **Motor Models**: ROBOTIS DYNAMIXEL XM430-W350-T actuators.
* **Safety Feature**: Every motion commands `PROFILE_VELOCITY = 20 ~ 25` (very slow and gentle, ~1.5 RPM) with automatic torque release in `finally:` blocks so the arm never jerks or stalls.

---

## 2. Hardware Wiring Checklist

1. **U2D2 Adapter**: Connect U2D2 USB cable to your Mac (USB-A port or via a USB-C hub/dongle).
2. **Robot Power**: Plug the 12V DC power adapter into the OpenMANIPULATOR-X power distribution hub.
3. **Power Switch**: Turn on the power switch on the power hub (the red LED on the DYNAMIXEL servos should blink once).
4. **4-Pin/3-Pin Cables**: Ensure the cable from U2D2 goes into the first motor (Joint 1 / ID 11) and daisy-chains up to the gripper (ID 15).

---

## 3. Test Scripts (Run in Order)

### Step 1: Scan and Ping All 5 Motors
Scans all Mac USB serial ports, detects U2D2, and pings all 5 motors without enabling torque:
```bash
cd openmanipulator_test
python3 01_scan_and_ping.py
```
**Expected Output**:
```
✓ ID 11 | Joint 1 (Base Pan)        | Model #1020 | Deg: +0.0°
✓ ID 12 | Joint 2 (Shoulder Pitch)   | Model #1020 | Deg: -35.2°
✓ ID 13 | Joint 3 (Elbow Pitch)      | Model #1020 | Deg: +42.1°
✓ ID 14 | Joint 4 (Wrist Pitch)      | Model #1020 | Deg: -10.5°
✓ ID 15 | Joint 5 (Gripper)          | Model #1020 | Deg: +0.0°
[SUCCESS] All 5 actuators successfully detected and responding on macOS!
```

---

### Step 2: Gentle Gripper Movement (Safe Physical Verification)
Gently opens the gripper by ~13 degrees, pauses 1.5 seconds, returns back, and releases torque. Moving the gripper carries **zero risk** of arm collisions:
```bash
python3 02_gentle_gripper_test.py
```

---

### Step 3: Gentle Base Joint Nudge (Joint 1, ID 11)
Rotates Joint 1 horizontally by $+5^\circ$ at ultra-low velocity, pauses 2 seconds, returns smoothly to starting angle, and releases torque:
```bash
python3 03_gentle_joint_nudge.py
```

---

### Step 4: Full Interactive Keyboard Teleoperation
Live keyboard dashboard to move individual joints and toggle the gripper:
```bash
python3 04_keyboard_teleop.py
```
* **Base Pan (Joint 1)**: `A` (Left) / `D` (Right)
* **Shoulder Pitch (Joint 2)**: `W` (Up) / `S` (Down)
* **Elbow Pitch (Joint 3)**: `I` (Up) / `K` (Down)
* **Wrist Pitch (Joint 4)**: `U` (Up) / `J` (Down)
* **Gripper (Joint 5)**: `O` (Open) / `P` (Close) or `SPACE` (Toggle)
* **Step Size**: `1` (1.0°), `2` (2.5°), `3` (5.0°)
* **Torque Control**: `T` (Toggle torque ON/OFF)
* **Emergency Stop**: `ESC` or `X` (Cuts torque immediately)
* **Quit**: `Q` (Safely disables torque and exits)

---

### Step 5: Move to Default Ready Pose (L-Shape)
Command the arm to smoothly take the default upright ready pose:
```bash
python3 05_go_to_home_pose.py
```

---

### Step 6: 1-Peg Autonomous Pick-and-Place State Machine
Executes the full 8-stage pick-and-place sequence (approach, vertical descent, grasp, lift, transit, insert, release, return):

**Dry Run (Recommended First - Safe in Mid-Air)**:
```bash
python3 06_test_pick_and_place.py
```
* Runs the entire sequence elevated in mid-air so you can watch the robot smoothly execute the pick-and-place trajectory with zero collision risk.

**Live Tabletop Run**:
```bash
python3 06_test_pick_and_place.py --live
```
* Executes actual tabletop grasping and socket insertion.

---

## 4. Troubleshooting on macOS

* **"Port is busy or cannot be opened"**:
  Close **DYNAMIXEL Wizard 2.0** if it is open in the background, as only one program can access the U2D2 serial port at a time.
* **"No U2D2 USB adapter detected"**:
  Unplug and replug the USB cable, or check `ls /dev/cu.usb*` in Terminal.
* **"Actuator not responding"**:
  Check that the 12V power supply is switched ON and connected to the robot.

