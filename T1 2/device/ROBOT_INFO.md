# ROBOTIS OpenMANIPULATOR-X Device Profile

Hardware specification and calibration parameters for the tabletop robot arm.

---

## 1. Hardware Overview

| Item | Details |
| :--- | :--- |
| **Model** | **ROBOTIS OpenMANIPULATOR-X (RM-X52-TNM)** |
| **Degrees of Freedom** | 4 Revolute Arm Joints + 1 Linear Parallel Gripper |
| **Actuator Model** | ROBOTIS DYNAMIXEL XM430-W350-T (Model #1020) |
| **Operating Voltage** | 12.0V DC (Allowed range: 10.0V ~ 14.8V) |
| **Communication Protocol**| DYNAMIXEL Protocol 2.0 |
| **Baud Rate** | `1,000,000` bps (1 Mbps) |
| **Hardware Controller**| ROBOTIS U2D2 (USB-to-RS485/TTL Converter) |
| **macOS Port Path** | `/dev/cu.usbserial-FTA2W88F` (FTDI VCP Driver) |

---

## 2. Actuator Mapping & Joint Table

| Joint | Servo ID | Joint Name | Operating Mode | Safe Limits | Function |
| :---: | :---: | :--- | :---: | :---: | :--- |
| **1** | **11** | Base Pan | Mode 4 (Multi-turn) | $\pm 160^\circ$ | Rotates arm horizontally |
| **2** | **12** | Shoulder Pitch | Mode 3 (Position) | $\pm 90^\circ$ | Lifts upper arm |
| **3** | **13** | Elbow Pitch | Mode 3 (Position) | $\pm 100^\circ$ | Bends forearm |
| **4** | **14** | Wrist Pitch | Mode 3 (Position) | $\pm 105^\circ$ | Tilts end-effector / tool |
| **5** | **15** | Gripper | Mode 3 (Position) | $1400 \sim 2600$ | Parallel finger open/close |

---

## 3. Kinematic Link Lengths

```
Base (0,0,0) ──[L1 = 77mm]──► Shoulder ──[L2 = 130mm]──► Elbow ──[L3 = 124mm]──► Wrist ──[L4 = 126mm]──► Gripper TCP
```

* $L_1 = 77.0\text{ mm}$ (Base plate to Shoulder pitch axis)
* $L_2 = 130.0\text{ mm}$ (Shoulder pitch axis to Elbow pitch axis)
* $L_3 = 124.0\text{ mm}$ (Elbow pitch axis to Wrist pitch axis)
* $L_4 = 126.0\text{ mm}$ (Wrist pitch axis to Gripper fingertip center)
* **Maximum Horizontal Reach**: $\approx 380\text{ mm}$
* **Maximum Vertical Reach**: $\approx 450\text{ mm}$

---

## 4. Default Ready / Home Pose

Visual CAD reference saved at: [`device/images/openmanipulator_ready_pose.jpg`](file:///Users/aai/Desktop/ITR_Proj/device/images/openmanipulator_ready_pose.jpg)

* **Joint 1**: Centered forward ($0^\circ$)
* **Joint 2**: Straight vertical upright ($0^\circ$ / 2048 counts)
* **Joint 3**: $90^\circ$ forward horizontal bend ($\approx 1100$ counts)
* **Joint 4**: Collinear horizontal with forearm ($\approx 2048$ counts)
* **Gripper**: Neutral open ($\approx 1800$ counts)

