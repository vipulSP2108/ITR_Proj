# 01. Quickstart Operator Guide: Free-Drive Data Collection & Calibration

> **For the Operator in the Lab**: This document is your step-by-step terminal cheat sheet. Keep it open while hand-guiding the Addverb HEAL robot arm across the calibration matrices.

---

## High-Level Workflow Overview

```
+-----------------------------------------------------------------------------------+
| Terminal 1 (Robot Arm): SSH cobot@192.168.1.25 -> ./cobot run -> ./free_drive      |
+-----------------------------------------------------------------------------------+
                                          |
                                          v
+-----------------------------------------------------------------------------------+
| Terminal 2 (ROS Laptop): ./run_collect.sh                                         |
|   1. Guide arm to Matrix #0 -> Press [SPACE] (Capture burst + average)            |
|   2. Rotate arm tool >= 45° -> Press [SPACE] (Repeat 3 to 6 times per matrix)      |
|   3. Move board or select Matrix #1 -> Press [n] (Next Matrix)                    |
|   4. Repeat across >= 4 matrices -> Press [f] (Finish)                            |
+-----------------------------------------------------------------------------------+
                                          |
                                          v
+-----------------------------------------------------------------------------------+
| Terminal 2: python3 scripts/calibrate_extrinsics.py                               |
|   Computes T_ee_cam0, T_base_cam1, stereo agreement, and exports TF publishers.   |
+-----------------------------------------------------------------------------------+
```

---

## Step 1: Start Free-Drive Mode on the Robot (Terminal 1)

1. Open your first terminal and SSH into the HEAL cobot controller:
   ```bash
   ssh cobot@192.168.1.25
   ```
   *Password*: `cobot`

2. Navigate to the controller image directory and launch the robot hardware runtime:
   ```bash
   cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
   sudo env HOME=/home/cobot ./cobot run
   ```
   *Password*: `cobot`

3. When prompted, type `1` and press `ENTER`.

4. Enable manual gravity-compensation free-drive:
   ```bash
   ./free_drive
   ```
   > [!IMPORTANT]
   > The arm is now compliant (gravity-compensated). You can gently move the arm by hand. Always keep one team member near the physical E-Stop.

---

## Step 2: Start the Data Collector (Terminal 2)

On the laptop connected to the robot network (192.168.1.x):

1. Open a second terminal and navigate to the project directory:
   ```bash
   cd ~/Desktop/ITR_Proj/T3/buzzwire_calib   # or wherever extracted
   ```

2. Run the collector launcher:
   ```bash
   ./run_collect.sh
   ```
   *(Or manually: `source /opt/ros/humble/setup.bash && source ~/Debojit_WS/addverb_ws/install/setup.bash && python3 scripts/collect_data.py`)*

3. If an earlier unfinished session exists, the script asks:
   ```text
   Found UNFINISHED session session_20261112_...: matrix #1, 8 captures.
   Resume it? [Y/n]:
   ```
   - Press **`ENTER`** or **`y`** to resume exactly where you left off.
   - Press **`n`** only if you want to start a brand new calibration session.

---

## Step 3: Interactive Data Collection Key Commands

| Key | Alternate | Function | What Happens Under the Hood |
| :---: | :---: | :--- | :--- |
| **`SPACE`** | **`c`**, `ENTER` | **Capture & Average** | Waits 0.4s for arm to settle, captures 12 frames per camera, checks that arm jitter < 4 mrad, median-averages joint angles, computes FK pose, and saves sharpest frames + corners. |
| **`n`** | **`m`** | **Next Matrix** | Increments the matrix counter (`matrix #0` -> `matrix #1`). Move the physical ChArUco board to a new table location. |
| **`u`** | - | **Undo Last Capture** | Moves the last capture to trash and updates counters if a capture had bad lighting or movement. |
| **`s`** | - | **Status** | Prints live capture counts, current matrix, and hardware health. |
| **`f`** | - | **FINISH** | Seals the session, writes summary manifest, and outputs next calibration command. |
| **`q`** | - | **Quit (Resume Later)** | Safely closes without finishing. All saved captures are preserved. Resume anytime! |

---

## Step 4: Operator Collection Golden Rules (For Highest Accuracy)

To get sub-millimeter calibration accuracy suitable for the Buzz Wire trial:

1. **Both Cameras Must See the Board**:
   The live status or preview window will tell you if the board is detected. Each camera must detect at least 8 corners in at least 4 burst frames.
2. **Rotate the Tool $\ge 45^\circ$ Between Captures**:
   Pure translation does **not** constrain hand-eye orientation. You must pitch, roll, and yaw the tool center point $\ge 45^\circ$ around multiple axes between captures on the same matrix!
3. **Take 3 to 6 Captures Per Matrix**:
   Keep the board fixed on the table, take 4 diverse arm orientations (SPACE), then move the board.
4. **Use $\ge 4$ Distinct Board Placements (Matrices)**:
   Place the board:
   - Near the left front of the workspace
   - Near the right front
   - Elevated on a flat block / stand
   - Tilted $\sim 15^\circ - 30^\circ$ relative to the table surface
5. **Arm Stillness**:
   After guiding the arm to a pose, let go or hold firmly still before pressing SPACE. If the arm moves $> 4\text{ mrad}$ during the burst, the capture is automatically rejected to prevent motion blur and encoder mismatch.

---

## Step 5: Solve Extrinsics & Generate Robot Transforms

Once you press **`f`**, run the calibration solver:

```bash
python3 scripts/calibrate_extrinsics.py
```
*(Optionally specify session: `python3 scripts/calibrate_extrinsics.py --session data/session_20261112_XXXXXX`)*

### Interpreting the Results
The script prints:
- `cam0 (hand): N obs, residual < 1.0 mm / 0.15 deg RMS` (Outstanding)
- `cam1 (fixed): N obs, residual < 3.0 mm / 0.20 deg RMS` (Healthy)
- `model vs direct stereo agreement: < 1.0 deg / < 2.0 mm`

### Generated Output Files
In `results/<session_name>/`:
- `extrinsics.json`: Full 4x4 transformation matrices, translations ($x,y,z$), and quaternions ($x,y,z,w$) for:
  - $T^{\text{ee}}_{\text{cam0}}$ (`hand` camera relative to `end-effector`)
  - $T^{\text{base}}_{\text{cam1}}$ (`fixed` camera relative to `base_link`)
  - $T^{\text{cam1}}_{\text{cam0}}$ (camera-to-camera stereo baseline)
  - $T^{\text{base}}_{\text{board}, k}$ (board pose for each matrix $k$)
- `publish_tf.sh`: Ready-to-run ROS2 `static_transform_publisher` bash commands.
- `per_observation_residuals.csv`: Individual capture residuals and outlier rejection flags.

