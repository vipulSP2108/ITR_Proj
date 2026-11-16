# Run 2: Continuous Single-Pass Peg Sort (2026-11-12 13:05:25)

This folder contains the complete multi-modal telemetry, dual-angle video recordings, sequential execution photos, and verification evidence for **Run 2** of the OpenMANIPULATOR-X Autonomous Peg Sorting System.

---

## 1. Directory Structure

```text
OneGO_20261112_130525/
├── README.md                                             # This document
├── video/                                                # Synchronized continuous video captures
│   ├── run.mp4                                           # Overhead system stream (6m 07s, 18 MB)
│   ├── run_preview.mp4                                   # Faststart web preview (4.1 MB)
│   ├── ExternalMobile_ClockTimerProof_20261112_130526.mp4 # Mobile camera capture with live running stopwatch (48 MB, 1080p60)
│   └── FinalAssignment_PegPlacement_20261112_131235.mp4  # Close-up inspection of all placed pegs (11s, 40 MB)
└── images/                                               # 32 sequential stage photos, plans, and workspace scans
```

---

## 2. Multi-Angle Synchronization & Authenticity Proof

To eliminate any doubt regarding autonomous operation and ensure full academic integrity, this run was simultaneously recorded across multiple perspectives with a visible physical time reference:

### A. System Overhead View: [`video/run.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/run.mp4)
* **Duration**: 6 minutes 07 seconds ($13:05:25 - 13:11:32$)
* **Perspective**: Calibrated Intel RealSense D435i mounted directly overhead at $Z \approx 510\text{ mm}$.
* **Content**: Live perception bounding boxes, segmented peg contours, hole coordinates, and planned pick-and-place trajectories.

### B. External Mobile View with Running Clock Timer: [`video/ExternalMobile_ClockTimerProof_20261112_130526.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/ExternalMobile_ClockTimerProof_20261112_130526.mp4)
* **Start Time**: `13:05:26` (initiated within 1 second of the robot controller start).
* **The Authenticity Guarantee**:
  - The mobile camera was positioned to capture both the entire physical robotic cell and a **running digital clock timer / stopwatch**.
  - The uninterrupted second-by-second ticking of the timer confirms that the video is an authentic, continuous, unedited recording of real-time autonomous execution.
  - Demonstrates that zero manual intervention, teleoperation, or pause/resume splicing occurred.
* **Storage Constraint Note**: Mobile recording ran at high-bitrate $1080\text{p60}$ for $\approx 3\text{m } 06\text{s}$ until internal phone memory became full and halted recording.

### C. Final Placement Inspection: [`video/FinalAssignment_PegPlacement_20261112_131235.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/FinalAssignment_PegPlacement_20261112_131235.mp4)
* **Timestamp**: `13:12:35` (recorded immediately following the completion of the sorting routine).
* **Content**: High-definition inspection pass verifying that all geometric pegs (circle, square, hexagon, triangle) are accurately placed within their designated sockets with zero collisions or misalignments.

---

## 3. High-Resolution Full Archive

The uncompressed $628\text{ MB}$ master camera recording is archived in the project Google Drive:
* [Task 1 Google Drive Cloud Archive](https://drive.google.com/drive/folders/1UufguEVD1WDDrdQYMmFn4csoTbAUtxeu?usp=sharing)
