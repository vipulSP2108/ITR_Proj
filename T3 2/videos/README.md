# Task 3 Video Submissions & Performance Benchmarks: Addverb HEAL Cobot Buzz Wire Navigation

This directory hosts recorded video demonstrations, benchmark execution logs, and online streaming references for the Addverb HEAL 6-DOF Cobot Buzz Wire Navigation system.

---

## 1. Online Video Demonstrations & Playlist

* **YouTube Demonstration Playlist**:
  [Task 3 Autonomous Buzz Wire Navigation Playlist](https://www.youtube.com/playlist?list=PLaEtGsMR43mE)
  *(Includes full continuous runs on the physical robot; the 3rd video in this playlist demonstrates our high-speed optimal traversal run).*

---

## 2. Benchmark Proof & Minimum Execution Time

The terminal screenshot below records the robot controller's live real-time benchmark execution, demonstrating the **minimum traversal time** achieved on the physical wire while preserving zero electrical continuity contacts:

* **Benchmark Screenshot**: [`Screenshot 2026-11-16 at 10.48.21 PM.png`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos/Screenshot%202026-11-16%20at%2010.48.21%20PM.png)

```text
IITGN-robotics$ ./buzzwire_run.sh lr --speed 0.65
[buzzwire_run] EXECUTING FORWARD: Left Start (L) -> Right Goal (R)
Trajectory : calibrated_wire_LR.csv
Speed      : 0.65
Smoothing  : Standard (SVD Planar Projection, Preserves Demonstrated Contours, 1246 mm arc)
Trajectory : .../buzzwire_calib/data/session_20261116_104756/calibrated_wire_LR.csv: 630 pts, 1246 mm, est. 67.3 s at speed 0.65 (v 4-39 mm/s)
[INFO] [buzzwire_teleop]: KDL FK ready: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
Start pose is 1 mm / 0 deg from the arm (a straight-line approach will be driven first).
PRE-START: SPACE = gripper open/close, ENTER = start, q = abort
Gripper=open ...
[follow] 1245.1/1246 mm v= 10mm/s x1.00 err= 3.0mm wrist= 7.0deg
DONE in 101.8 s. Worst tracking error 5.47 mm
```

### Benchmark Metrics Summary
| Parameter | Value | Assessment / Margin |
| :--- | :--- | :--- |
| **Total 3D Arc Traversal** | **$1246\text{ mm}$** | Full wire span covered from Left anchor to Right goal |
| **Waypoint Density** | **$630\text{ points}$** | High spatial resolution ($\approx 1.98\text{ mm}$ per waypoint) |
| **Minimum Traversal Time** | **$101.8\text{ s}$** ($1\text{ min } 41.8\text{ s}$) | Fastest verified physical traversal |
| **Worst Tracking Error** | **$5.47\text{ mm}$** | Well within the inner ring clearance radius |
| **Buzzer / Continuity Alarm** | **0 activations (Clear)** | Zero contact maintained through entire traversal |

---

## 3. Local High-Definition Video Demonstrations

The following high-definition (1080p @ 60fps) video captures of the physical arm correspond to the logged benchmark runs:

| Video File Name | Traversal Direction & Mode | Commanded Speed | Video Specs & Duration | Notes & Verification |
| :--- | :--- | :--- | :--- | :--- |
| [`FINAL LR_SPEED_0.65.mp4`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos/FINAL%20LR_SPEED_0.65.mp4) | **Forward: Left $\to$ Right (L $\to$ R)** | `--speed 0.65` | $1080\text{p60}$, $102.97\text{ s}$ ($71.5\text{ MB}$) | Physical proof video matching the **$101.8\text{ s}$** minimum time terminal log. |
| [`RL SPEED_01.mp4`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos/RL%20SPEED_01.mp4) | **Return: Right $\to$ Left (R $\to$ L)** | Controlled Return | $1080\text{p60}$, $106.71\text{ s}$ ($71.1\text{ MB}$) | Reverse wire navigation confirming symmetric bidirectional clearance. |

---

## 4. Designated Submission Naming Conventions & Commands

When uploading further takes or evaluations, reference this convention:

| Priority | Exact File Name | Associated Execution Command | Description |
| :--- | :--- | :--- | :--- |
| **Primary (Full Demonstration)** | `task3_buzzwire_full_run.mp4` | Combined L $\to$ R $\to$ L | **Master video**: Arm navigates Left-to-Right and returns Right-to-Left along the 3D wire track with 0 contact buzzes. |
| **Fast Benchmark Run** | [`FINAL LR_SPEED_0.65.mp4`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos/FINAL%20LR_SPEED_0.65.mp4) | `./buzzwire_run.sh lr --speed 0.65` | Fast competition traversal demonstrating optimal speed performance ($101.8\text{ s}$). |
| **Return Traversal Run** | [`RL SPEED_01.mp4`](file:///Users/aai/Desktop/ITR_Proj/T3%202/videos/RL%20SPEED_01.mp4) | `./buzzwire_run.sh rl --speed 0.10` | Safe return traversal Right-to-Left along the wire track. |
| Smooth L $\to$ R Take | `task3_lr_smooth.mp4` | `./buzzwire_run.sh lr -smooth --speed 0.35` | Forward Left-to-Right traversal along the Savitzky-Golay spline-smoothed trajectory. |
| Smooth R $\to$ L Take | `task3_rl_smooth.mp4` | `./buzzwire_run.sh rl -smooth --speed 0.35` | Safe return traversal Right-to-Left along the smooth path. |

---

## 5. Recording & Camera Framing Guidelines

* **Framing & View**:
  - Position the camera to show the **entire Addverb HEAL 6-DOF arm**, the **ring/loop end-effector**, the entire **3D curved wire game apparatus**, and the **continuity buzzer / LED alarm indicator**.
  - The clearance between the ring and the wire should be clearly visible throughout the motion.
* **Audio**:
  - Keep audio enabled so examiners can hear that the **continuity buzzer does not sound** during traversal.
* **Resolution & Format**:
  - Container: `.mp4` (preferred) or `.mov`
  - Codec: H.264
  - Resolution: 1080p (1920×1080) or 720p (1280×720) at 30 fps or 60 fps.
