# 01. Quickstart Operator Guide: Buzz Wire Teaching, Merging & Replay

> **For the Lab Operator & Team**: This document is your step-by-step terminal cheat sheet and operational manual. Keep it open while teaching and executing the 3D Buzz Wire trajectory with the **Addverb HEAL 6-DOF Collaborative Robot**.
>
> **Notice**: No cameras are used or required for this task. All kinematics are resolved directly in the robot base frame using joint encoders and KDL forward kinematics.

---

## 1. High-Level Workflow Architecture

```
+-----------------------------------------------------------------------------------------+
| TERMINAL 1 (Robot Controller SSH): ssh cobot@192.168.1.25 -> sudo env HOME=... ./cobot  |
|                                                                                         |
|   Choose Mode:                                                                          |
|     ├── For Method 1 (Free-Drive):  ./free_drive                                        |
|     └── For Method 2 (Teleop/Replay): ./heal_server + ros2 velocity_control launch      |
+-----------------------------------------------------------------------------------------+
                                             |
                                             v
+-----------------------------------------------------------------------------------------+
| TERMINAL 2 (Control Laptop): ./buzzwire_teach.sh [command]                              |
|                                                                                         |
|   Step 1. TEACHING (Choose either or combine both):                                     |
|           Method 1: ./buzzwire_teach.sh record    (Push arm by hand, SPACE for gripper)  |
|           Method 2: ./buzzwire_teach.sh teleop    (6-DOF keys, auto-held orientation)   |
|           Offline : ./buzzwire_teach.sh sim       (Practise keys on simulated arm)      |
|                                                                                         |
|   Step 2. INSPECTION:                                                                   |
|           ./buzzwire_teach.sh inspect [--plot]   (Check segments, gaps, coverage)       |
|                                                                                         |
|   Step 3. TRAJECTORY MERGING:                                                           |
|           ./buzzwire_teach.sh merge               (Stitch, trim contacts, smooth, 2mm)  |
|                                                                                         |
|   Step 4. AUTONOMOUS REPLAY:                                                            |
|           ./buzzwire_teach.sh replay --dry-run    (View time-optimal speed profile)     |
|           ./buzzwire_teach.sh replay --speed 0.2  (First slow pass, then 0.4 -> 1.0)   |
+-----------------------------------------------------------------------------------------+
```

---

## 2. Terminal Cheat Sheet

### Terminal 1: Robot Hardware Controller (SSH)

1. Connect to the robot controller via Ethernet:
   ```bash
   ssh cobot@192.168.1.25
   ```
   *Password*: `cobot`

2. Navigate to the cobot runtime directory and initialize motor drives:
   ```bash
   cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
   sudo env HOME=/home/cobot ./cobot run
   ```
   *Sudo Password*: `cobot`  
   When prompted, type `1` and press `ENTER`.

3. **Start the appropriate robot daemon for your task**:
   - **For Method 1 (Hand Guiding)**:
     ```bash
     ./free_drive
     ```
     *(The arm is now compliant and gravity-compensated. Push it gently by hand).*
   - **For Method 2 (Keyboard Driving) OR Replay**:
     ```bash
     ./heal_server
     ```
     *(In a separate terminal or background, start the ROS 2 velocity controller)*:
     ```bash
     ros2 launch addverb_cobot_control velocity_control.launch.py
     ```

> [!IMPORTANT]
> The robot can only run **ONE** mode at a time (`./free_drive` XOR `./heal_server`). To switch, press `Ctrl+C` in Terminal 1 and launch the other binary. All recorded segment files (`seg_NNNN.csv`) remain completely valid across modes.

---

### Terminal 2: Control Laptop

Always run commands from the project directory:
```bash
cd /Users/aai/Desktop/Task2_ITR/buzzwire_calib
```

#### Step 1: Practice Without Hardware (Safe Practice)
Before touching the real robot, practice the 6-DOF keyboard controls on the built-in simulated UR5/HEAL arm:
```bash
./buzzwire_teach.sh sim
```
Or execute all automated self-tests:
```bash
./buzzwire_teach.sh test
```

---

#### Step 2: Teach the Wire Path

##### Option A: Method 1 – Hand Guiding (Gravity Compensation)
Robot in `./free_drive`. Run on laptop:
```bash
./buzzwire_teach.sh record
```
- Press `ENTER` to begin recording.
- Hand-guide the wand ring through the 3D copper wire loop.
- **Keybindings**:
  - `SPACE`: Toggle gripper open/close (locks wand in place).
  - `p`: Pause/resume recording.
  - `x`: Mark contact (if you heard the buzzer touch; the merger trims this region).
  - `q` (or `Ctrl+C`): Save segment (`seg_NNNN.csv`) and exit.

> [!TIP]
> **Divide and Conquer**: Record the wire in 3 to 5 overlapping segments (e.g., Valley 1 -> Peak 1, Peak 1 -> Valley 2) rather than trying to complete one continuous perfect take. The merger automatically stitches them together!

##### Option B: Method 2 – 6-DOF Keyboard Teleoperation
Robot in `./heal_server` + velocity controller. Run on laptop:
```bash
./buzzwire_teach.sh teleop
```
- Keep the terminal window focused.
- Drive the arm smoothly with full 6-DOF Cartesian control.
- Orientation is locked automatically while translating.
- **Keybindings**:
  - `v`: Start / Stop recording segment.
  - `w` / `s`: Forward / Backward along wire depth ($\pm Y$).
  - `a` / `d`: Left / Right along wire length ($\mp X$).
  - `r` / `f`: Up / Down vertical ($\pm Z$).
  - `i` / `k`: Pitch up / down (rotation about $Y$).
  - `j` / `l`: Yaw left / right (rotation about $Z$).
  - `u` / `o`: Roll counter-clockwise / clockwise (rotation about $X$).
  - `1` `2` `3` `4` `5`: Select speed level (2, 5, 10, 20, 40 mm/s).
  - `z` (HOLD): **Backtrack & Live Undo** — retraces breadcrumb history and truncates CSV rows.
  - `g`: Toggle auto-tangent mode (tool automatically tilts with wire slope).
  - `SPACE`: Toggle gripper open/close.
  - `x`: Mark buzzer contact.
  - `c`: Re-sync target carrot to current arm pose.
  - `TAB`: Toggle jog frame (`world` $\leftrightarrow$ `tool`).
  - `q`: Quit teleop (saves active segment).

---

#### Step 3: Inspect Recorded Data
Before merging, review segment coverage and continuity:
```bash
./buzzwire_teach.sh inspect
```
To visualize the 3D trajectory segments with Matplotlib:
```bash
./buzzwire_teach.sh inspect --plot
```
- Checks spatial gap distances between consecutive segments.
- Displays total arc length and coverage progress bar against the ~1.0 m wire length.

---

#### Step 4: Merge Trajectory
Merge all segments in the active session into a unified, smoothed trajectory:
```bash
./buzzwire_teach.sh merge
```
- Automatically trims 8 mm radius around contact marks.
- Stitches overlapping segments ($< 15\text{ mm}$).
- Inserts smooth cubic SLERP bridges across small gaps ($15\text{--}120\text{ mm}$).
- Applies Savitzky-Golay filtering to eliminate operator hand tremor.
- Resamples uniformly at 2 mm arc-length intervals.
- Generates `data/session_*/merged_trajectory.csv` and `merge_report.json`.

*(Optional flags)*:
- `--clearance 0.003`: Apply +3.0 mm outward radial clearance offset.
- `--exclude 2`: Skip a bad segment #2.
- `--reverse-seg 1`: Reverse a segment recorded in the backward direction.

---

#### Step 5: High-Speed Autonomous Replay
Robot in `./heal_server` + velocity controller.

1. **Dry-Run Analysis (No robot motion)**:
   ```bash
   ./buzzwire_teach.sh replay --dry-run
   ```
   Inspects the curvature-constrained speed profile, total duration, and top 5 slowest spots.

2. **First Hardware Pass (Conservative 20% speed)**:
   ```bash
   ./buzzwire_teach.sh replay --speed 0.2
   ```
   - Pre-start prompt: Press `SPACE` to grip the wand, verify workspace clearance, press `ENTER` to execute.
   - Smooth quintic straight-line approach brings the arm to the trajectory start.
   - Closed-loop carrot-on-a-leash tracks the path without human intervention.
   - Real-time keys during replay:
     - `p`: Pause / resume.
     - `[` / `]`: Decrease / increase speed by 20%.
     - `q`: Emergency stop (sends 0 velocity immediately).

3. **Speed Optimization (Towards 100% speed)**:
   - Check the printed `Worst tracking error: X.XX mm`.
   - Gradually increase speed: `--speed 0.4` $\rightarrow$ `0.6` $\rightarrow$ `0.8` $\rightarrow$ `1.0`.
   - If tracking error exceeds 4.0 mm, you are approaching the robot's physical torque/velocity limits.

---

## 3. Operator Golden Rules

1. **Buddy System**: Never run hardware alone. One operator stands with their hand poised over the physical **Emergency Stop (E-Stop)** button.
2. **Terminal Focus**: Keyboard teleop captures keystrokes only when its terminal window is focused. Never switch windows while driving the robot.
3. **Low Speed First**: Always execute new trajectories at `--speed 0.2` before accelerating to full competition speed.
4. **Arm Recovery**: If the arm trips or lags, press `c` in teleop or run `python3 scripts/goto_pose.py --home` to smoothly return to home.
5. **Clean Merges**: Check `merge_report.json` for any `LARGE GAP` warnings before attempting replay.
