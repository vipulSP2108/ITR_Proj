# OpenMANIPULATOR-X Keyboard Teleoperation Controls

Quick reference guide for manual robot control via keyboard.

---

## Visual Key Layout

```
         [ W ]                     [ I ]                     [ U ]                     [ O ] [ P ]
     Shoulder UP                 Elbow UP                  Wrist UP                   Open / Close
 [ A ]   [ S ]   [ D ]       [ J ]   [ K ]   [ L ]                                     [ SPACE ]
Base L  Shoulder  Base R            Elbow DN                                        Toggle Gripper
          DOWN
```

---

## Complete Key Mappings Table

| Key | Action | Motor Controlled | Motion Direction |
| :---: | :--- | :--- | :--- |
| **`A`** | Base Pan Left | **Joint 1 (ID 11)** | Rotate counter-clockwise ($-Yaw$) |
| **`D`** | Base Pan Right | **Joint 1 (ID 11)** | Rotate clockwise ($+Yaw$) |
| **`W`** | Shoulder Up | **Joint 2 (ID 12)** | Lift upper arm upward ($-Pitch$) |
| **`S`** | Shoulder Down | **Joint 2 (ID 12)** | Lower upper arm downward ($+Pitch$) |
| **`I`** | Elbow Up | **Joint 3 (ID 13)** | Bend forearm upward ($-Pitch$) |
| **`K`** | Elbow Down | **Joint 3 (ID 13)** | Extend forearm downward ($+Pitch$) |
| **`U`** | Wrist Up | **Joint 4 (ID 14)** | Tilt end-effector upward ($-Pitch$) |
| **`J`** | Wrist Down | **Joint 4 (ID 14)** | Tilt end-effector downward ($+Pitch$) |
| **`O`** | Open Gripper | **Joint 5 (ID 15)** | Move gripper fingers apart |
| **`P`** | Close Gripper | **Joint 5 (ID 15)** | Close gripper fingers together |
| **`SPACE`** | Toggle Gripper | **Joint 5 (ID 15)** | Alternates between Open and Closed |

---

## Precision & Safety Controls

| Key | Function | Description |
| :---: | :--- | :--- |
| **`1`** | **Fine Step (`1.0°`)** | Ultra-precise micro adjustments |
| **`2`** | **Medium Step (`2.5°`)** | Balanced default speed |
| **`3`** | **Coarse Step (`5.0°`)** | Fast travel over larger distances |
| **`H`** | **Default Ready Pose** | Automatically moves smoothly to the default L-shape posture |
| **`T`** | **Torque Toggle** | Toggles motor power ON/OFF (allows hand positioning when OFF) |
| **`ESC` / `X`** | **EMERGENCY STOP** | Instantly cuts torque to all motors to stop motion |
| **`Q`** | **Quit** | Safely disables torque and closes serial port |

---

## How to Launch

From the test folder:
```bash
cd /Users/aai/Desktop/ITR_Proj/openmanipulator_test
python3 04_keyboard_teleop.py
```

Or from the main T1 folder:
```bash
cd /Users/aai/Desktop/ITR_Proj/T1
python3 src/control/teleop_keyboard.py
```

