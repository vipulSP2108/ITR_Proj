# Task 3 Video Submissions: Addverb HEAL Cobot Buzz Wire Navigation

This folder is the designated upload location for Task 3 video recordings.

---

## 1. Designated File Names for Upload

When uploading videos to this folder, please name the files according to this convention:

| Priority | Exact File Name | Associated Execution Command | Description |
| :--- | :--- | :--- | :--- |
| **Primary (Full Demonstration)** | `task3_buzzwire_full_run.mp4` | Combined L $\to$ R $\to$ L | **Master video**: Arm navigates Left-to-Right and returns Right-to-Left along the 3D wire track with 0 contact buzzes. |
| **Take 1 (Smooth L $\to$ R)** | `task3_lr_smooth.mp4` | `./buzzwire_run.sh lr -smooth --speed 0.35` | Forward Left-to-Right traversal along the Savitzky-Golay spline-smoothed trajectory. |
| **Take 2 (Smooth R $\to$ L)** | `task3_rl_smooth.mp4` | `./buzzwire_run.sh rl -smooth --speed 0.35` | Safe return traversal Right-to-Left along the smooth path. |
| Take 3 (Raw L $\to$ R) | `task3_lr_raw.mp4` | `./buzzwire_run.sh lr --speed 0.35` | Forward Left-to-Right traversal using raw recorded waypoints. |
| Take 4 (Raw R $\to$ L) | `task3_rl_raw.mp4` | `./buzzwire_run.sh rl --speed 0.35` | Return Right-to-Left traversal using raw recorded waypoints. |
| Fast Run Take | `task3_lr_fast.mp4` | `./buzzwire_run.sh lr -smooth --speed 0.65` | Fast competition traversal demonstrating optimal speed performance. |

---

## 2. Recording & Camera Framing Guidelines

* **Framing & View**:
  - Position the camera to show the **entire Addverb HEAL 6-DOF arm**, the **ring/loop end-effector**, the entire **3D curved wire game apparatus**, and the **continuity buzzer / LED alarm indicator**.
  - The clearance between the ring and the wire should be clearly visible throughout the motion.
* **Audio**:
  - Keep audio enabled so examiners can hear that the **continuity buzzer does not sound** during traversal.
* **Resolution & Format**:
  - Container: `.mp4` (preferred) or `.mov`
  - Codec: H.264
  - Resolution: 1080p (1920×1080) or 720p (1280×720) at 30 fps or 60 fps.
