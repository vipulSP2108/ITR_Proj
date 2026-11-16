# 03. Stuck Arm Recovery Playbook & Edge-Case Protection

> **Critical Safety & Fault Recovery Guide**: Procedures and automated safeguards to recover immediately when the Addverb HEAL cobot trips on fast movements, enters fault states, stalls on wire contact, or requires deterministic return to safe configurations.

---

## 1. Edge Case Matrix & Action Summary

| Scenario | Symptom / Error | Code Safeguard | Operator Immediate Action |
| :--- | :--- | :--- | :--- |
| **Hard Motion Trip in Free-Drive** | Arm becomes stiff or limp; `!! joint states stale (2.5s)` | Capture loop pauses safely; zero data lost | In Robot Terminal (SSH): `Ctrl+C` `./free_drive` $\rightarrow$ re-run `./free_drive`. Collector auto-resumes. |
| **Buzzer Contact During Teaching** | Wand rings on wire | User marks `x` (or auto-pin); merger trims 8 mm sphere | In Teleop: Hold `z` (backtracks arm & deletes CSV rows). In Free-Drive: Press `q`, reposition arm, record next segment. |
| **Stall / Collision / Lag in Teleop** | HUD displays `lag > 12mm` then `!! arm NOT FOLLOWING` | Carrot stays on leash ($12\text{ mm}$); auto-stops if `lag_s > 2.5s` | Check obstacle. Hold `z` to retrace backwards. Press `c` to re-sync carrot to arm pose. |
| **Stall During Replay Execution** | Tracking error exceeds $30\text{ mm}$ for $> 1.5\text{ s}$ | Aborts immediately; publishes 0 joint velocity; saves state | Clear obstruction. Inspect robot. Run `./buzzwire_teach.sh replay --resume` to continue. |
| **Terminal / SSH Disconnect** | SSH drop or laptop crash mid-recording | Append-only CSV flushed per sample; torn rows discarded | Reconnect. Run `./buzzwire_teach.sh [teleop\|record]`. Answer `Y` to "Resume latest session?". |
| **Singularity Proximity** | Kinematics approaching gimbal lock / wrist alignment | SVD check: $\sigma_{\text{min}} < 0.03 \rightarrow$ increases damping, scales speed | Arm slows smoothly instead of whipping joints. Maintain path or nudge away. |
| **Arm Out of Position / Return Home** | Need to reset arm to initial home state | Closed-loop quintic velocity planner | Run `python3 scripts/goto_pose.py --home` (or `--sim`). |

---

## 2. Detailed Recovery Procedures

### 1. Hard Movement Trip in `./free_drive`

#### Root Cause
The Addverb HEAL controller monitors motor current and joint angular accelerations. If an operator pushes the arm violently into an obstacle or approaches an internal mechanical stop, safety relays trigger a protective drive shutdown, halting `/joint_states` publication.

#### System Response
The recording script immediately detects the stall:
```text
[REC seg0002] pts=142 contacts=0 grip=open !! joint states STALE (free_drive stopped?) - waiting, nothing is lost
```
The recording thread suspends writing to prevent corrupt zero-readings.

#### Recovery Steps
1. In the robot SSH terminal (Terminal 1), press `Ctrl+C` to terminate `./free_drive`.
2. Re-execute the free-drive binary:
   ```bash
   ./free_drive
   ```
3. The recording terminal on your laptop will automatically detect incoming `/joint_states` and resume streaming waypoints without lost data.

---

### 2. Live Backtrack & Undo During Teleop (`z` key)

When piloting the robot with the keyboard, human error in the tight 3D wire loops can cause accidental wire contact. Rather than discarding the entire session, the system incorporates an integrated **Breadcrumb History & Live Truncation Engine**:

```
[Forward Motion] ────> Breadcrumbs recorded every 1 mm into deque
                            │
                            v
[Contact Occurs] ────> Operator HOLDS 'z'
                            │
                            ├── 1. Controller retraces exact backwards path along breadcrumbs
                            └── 2. SegmentWriter truncates CSV file on disk to match rewound step
```

#### How to Operate:
1. The moment you hear the buzzer or see the ring near the wire, **hold `z`**.
2. The virtual carrot reverses step-by-step along the historic path at the current speed level.
3. The on-disk segment CSV is truncated instantaneously via `os.truncate()` to the rewound point.
4. Release `z`, adjust your steering keys (`w/s/a/d/r/f`), and continue recording cleanly.

---

### 3. Carrot-on-a-Leash & Stall Abort Protection

During teleoperation and replay, a common failure mode in robotic trajectory tracking is "integrator windup" where a reference trajectory races ahead while the physical robot is jammed.

#### Mathematical Protection (`teleop_core.py`):
The desired pose $T_{\text{des}}$ is tethered to actual pose $T_{\text{cur}}$ by a strict spatial leash:
$$\|p_{\text{des}} - p_{\text{cur}}\| \le \delta_{\text{lead}} \quad (12\text{ mm})$$
$$\|\text{Log}(R_{\text{des}} R_{\text{cur}}^T)\| \le \theta_{\text{lead}} \quad (0.12\text{ rad})$$

If the robot is physically blocked:
1. $T_{\text{des}}$ stops advancing and waits at the boundary of the leash.
2. A lag timer `lag_s` increments:
   $$\text{lag\_s} \leftarrow \text{lag\_s} + \Delta t$$
3. If `lag_s > 2.5 s`:
   - Velocity commands are immediately zeroed: $\dot{q} \leftarrow \mathbf{0}$.
   - The HUD alerts: `!! arm NOT FOLLOWING - stopped. Use z to back off.`
   - Pressing `c` resets and snaps $T_{\text{des}}$ directly to the arm's current physical pose.

---

### 4. Trajectory Replay Abort & `--resume` Execution

If trajectory replay encounters an unexpected obstruction or tracking error $> 30\text{ mm}$ for $> 1.5\text{ s}$:
1. The replay controller safely cuts velocity to zero.
2. The exact arc position $s_{\text{stop}}$ is saved to `data/session_*/replay_state.json`.
3. To resume motion without re-running the entire wire:
   ```bash
   ./buzzwire_teach.sh replay --resume --speed 0.2
   ```
   The controller will:
   - Perform a slow, straight approach to $s_{\text{stop}} - 10\text{ mm}$ (backing up slightly to guarantee smooth re-entry).
   - Re-engage the velocity profile and complete the remaining path.

---

### 5. Deterministic Arm Reset (`scripts/goto_pose.py`)

If the arm has been manually moved into an awkward configuration, use the quintic polynomial positioning tool to return it smoothly:

```bash
# Return arm to zero home position
python3 scripts/goto_pose.py --home

# Or run in simulation mode to preview trajectory
python3 scripts/goto_pose.py --home --sim --yes
```

#### Quintic Interpolation Features:
- Zero jerk and zero acceleration at boundary points:
  $$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau \in [0, 1]$$
- Clamped joint speeds: $\max_i |\dot{q}_i| \le 0.25\text{ rad/s}$.
- Closed-loop proportional feedback correcting motor tracking errors.
- Stalls and holds safely if `/joint_states` drops.
