# 03. Stuck Arm Recovery Playbook & Edge-Case Protection

> **Critical Fault & Recovery Guide**: How to recover immediately when the Addverb HEAL robot trips on hard movements, enters fault states, loses connection, or needs to return to saved poses.

---

## Edge Case Matrix & Action Summary

| Scenario | Symptom | Auto Protection | Operator Recovery Action |
| :--- | :--- | :--- | :--- |
| **Hard motion trips drive in free-drive** | Arm becomes stiff or limp; status shows `!! ROBOT STATE STALE` | Capture blocked safely; session preserved | In robot terminal: Ctrl+C `./free_drive` -> re-run `./free_drive`. Collector resumes instantly. |
| **SSH disconnect / terminal crash** | Connection dropped mid-session | Events append-only log with `fsync()`; temp captures | Re-run `./run_collect.sh` -> Answer `Y` to "Resume it?". Continue on same matrix. |
| **Arm displaced / want previous pose** | Arm moved away from matrix | All joint states and FK poses logged | Run `python3 scripts/goto_pose.py --last` (or `--matrix N`, `--home`). |
| **Arm stalls during `goto_pose.py`** | Motion lagging reference (> 0.12 rad for > 2s) | Aborts immediately with zero velocity | Clear obstruction/fault. Re-run command (auto-replanes from current arm pose). |
| **Arm moved during capture** | Operator hand jitter while pressing SPACE | Capture rejected if jitter > 4 mrad | Hold arm steady or let go; press `SPACE` again. |
| **Camera USB glitch / cable bump** | Video stream freezes | Auto-reconnect thread in `OpenCVCamera` | Script re-opens video device in background; capture blocked until frames flow. |

---

## Detailed Recovery Procedures

### 1. Hard Movement Trip in `./free_drive`

#### Why It Happens
The Addverb HEAL cobot has internal torque and velocity thresholds. If you jerk or push the arm violently into a singular configuration or against a joint limit, the motor drives enter a protective safety state and stop broadcasting joint states.

#### Live Status Indicator
The collector terminal will show:
```text
matrix #2 (captures here: 3) | total 11 | !! ROBOT STATE STALE (2.4s) - is free_drive / heal_server running?
```
Any attempt to capture is refused (`FAIL not capturing: ROBOT STATE STALE`), protecting your dataset from corrupted or stale readings.

#### Recovery Steps
1. **Do NOT kill the collector script**. All your captures are safely stored on disk.
2. In the robot SSH terminal (Terminal 1):
   - Press **`Ctrl + C`** to stop `./free_drive`.
   - If the main cobot daemon also faulted, restart it:
     ```bash
     sudo env HOME=/home/cobot ./cobot run
     ```
     Type **`1`** and press **`ENTER`**.
   - Restart free drive:
     ```bash
     ./free_drive
     ```
3. Return to Terminal 2 (Collector):
   - Within 0.5 seconds, the status line updates from `!! ROBOT STATE STALE` to:
     ```text
     matrix #2 (captures here: 3) | total 11 | ready
     ```
   - Continue capturing! No restart needed.

---

### 2. Session Crash / SSH Disconnect / Power Loss

#### The Fault-Tolerant Session Architecture
Every capture event, matrix increment, and undo operation is logged in an append-only JSONL file (`data/<session>/events.jsonl`) with an immediate `os.fsync(fd)`.
Captures are staged in hidden folders (`.tmp_cap_XXXX`) and atomically renamed (`cap_XXXX`) only when all image frames, corner detections (`detections.npz`), joint angles, and FK poses are fully flushed.

#### Resuming Your Session
If your terminal closes, SSH connection drops, or laptop battery dies:
1. Re-open terminal and run:
   ```bash
   ./run_collect.sh
   ```
2. The launcher detects the unfinished session and prompts:
   ```text
   Found UNFINISHED session session_20261112_143022: matrix #3, 16 captures.
   Resume it? [Y/n] (n = start a new session):
   ```
3. Press **`ENTER`** or **`y`**.
   - You resume directly on `matrix #3`.
   - All 16 previous captures are preserved.
   - The counter advances seamlessly (`capture #17`).

---

### 3. Driving Arm Back to Saved Poses (`goto_pose.py`)

If you had to move the arm away or restart the robot, you can command the arm to return smoothly to any previous calibration pose using the built-in quintic velocity commander.

#### Prerequisites
1. Stop `./free_drive` in the robot terminal.
2. Start `./heal_server` in the robot terminal.
3. On the ROS 2 workstation, launch the velocity controller:
   ```bash
   ros2 launch addverb_cobot_control velocity_control.launch.py
   ```

#### Command Options
```bash
# Return to the very last captured pose of the session:
python3 scripts/goto_pose.py --last

# Return to the last capture of Matrix #2:
python3 scripts/goto_pose.py --matrix 2

# Return to specific capture ID #7:
python3 scripts/goto_pose.py --capture 7

# Return arm to the standard all-zero HOME configuration:
python3 scripts/goto_pose.py --home
```

#### Kinematic Profile & Velocity Clamping
- Uses a quintic polynomial profile:
  $$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \qquad \dot{s}(\tau) = 30\tau^2 - 60\tau^3 + 30\tau^4$$
- Zero initial and final velocity and acceleration: **No jerk, no overshoot**.
- Joint velocities are strictly clamped to $\pm 0.25\text{ rad/s}$ ($\approx 14^\circ/\text{s}$).

#### Stuck Detection During Automated Drive
`goto_pose.py` continuously compares the reference trajectory $q_{\text{ref}}(t)$ against the live joint feedback $q(t)$:
```python
err = q_ref - q
if np.abs(err).max() > 0.12:  # > ~6.9 degrees error
    if now - err_bad_since > 2.0:
        send(np.zeros(6))     # Stop immediately!
        sys.exit("ABORT: arm is not following...")
```
If an obstacle obstructs the arm or a joint stalls, **it commands zero velocity and aborts safely**.
**To resume**: Clear the obstruction, then re-issue the exact same `goto_pose.py` command. It will read the arm's current position and plan a smooth trajectory from where it sits now to the goal.

After reaching the goal pose:
1. Stop the velocity controller (`Ctrl + C`).
2. Switch back to `./free_drive` on the robot.
3. Relaunch `./run_collect.sh` to resume data collection.

---

### 4. Arm Motion During Capture Burst

#### The Guard
When you press `SPACE`, the collector samples 12 consecutive frames and robot joint angles over $\approx 0.4$ seconds:
$$Q = [q_0, q_1, \dots, q_{11}]^T \in \mathbb{R}^{12 \times 6}$$
$$\Delta q_{\max} = \max_{j} \left( \max(Q_{:,j}) - \min(Q_{:,j}) \right)$$
If $\Delta q_{\max} > 0.004\text{ rad}$ ($\approx 0.23^\circ$), the capture is rejected:
```text
FAIL robot moved during capture (0.0071 rad > 0.004). Hold still / let go of the arm and press capture again.
```
This guarantees that the camera images and robot FK poses correspond to an identical, motionless configuration.

---

### 5. Camera Disconnect & Video Stream Recovery

In `buzzwire_calib/cameras.py`:
- Each camera runs in an independent worker thread.
- If frames drop or the USB bus resets, it retries opening the device indefinitely.
- If frames fail $> 30$ consecutive reads, the device is released and cleanly reopened.
- The collector status line warns `!! CAMERA cam0 NO FRAMES (1.2s)` and blocks capture until streaming resumes.
