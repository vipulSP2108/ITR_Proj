# Task 1 Video Submissions: OpenMANIPULATOR-X Peg Sorting

This folder is the designated upload location and repository guide for Task 1 video recordings.

---

## 1. Cloud Video Archive & Demonstration Links

* **Google Drive Full-Resolution Archive**:
  [Task 1 Google Drive Video Folder](https://drive.google.com/drive/folders/1UufguEVD1WDDrdQYMmFn4csoTbAUtxeu?usp=sharing)
  *(Contains full uncompressed high-framerate recordings, raw mobile camera takes, and extended multi-peg runs).*

---

## 2. Recorded Demonstration Runs & Authenticity Verification

Each recorded run folder is modularized into `video/` (continuous MP4 video recordings) and `images/` (sequential stage photos, motion plans, and workspace scans):

| Run Folder | Primary Run Video | Authenticity & Verification Assets | Images Path & Contents |
| :--- | :--- | :--- | :--- |
| **Run 1: Full Sequence with Parking** (`OneGO_withparking_20261112_122225/`) | [`video/run.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_withparking_20261112_122225/video/run.mp4) ($17\text{ MB}$, $6\text{m } 58\text{s}$) <br> Preview: [`run_preview.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_withparking_20261112_122225/video/run_preview.mp4) ($4.0\text{ MB}$) | Combined single-take run showing all shapes placed followed by safe return to home parking pose. | [`images/`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_withparking_20261112_122225/images) (40 stage photos, 5 motion plans, 7 scans) |
| **Run 2: Continuous Single-Pass Sort & Live Timer Proof** (`OneGO_20261112_130525/`) | [`video/run.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/run.mp4) ($18\text{ MB}$, $6\text{m } 07\text{s}$) <br> Preview: [`run_preview.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/run_preview.mp4) ($4.1\text{ MB}$) | 1. **Live Running Clock Timer Proof**: [`video/ExternalMobile_ClockTimerProof_20261112_130526.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/ExternalMobile_ClockTimerProof_20261112_130526.mp4) <br> 2. **Final Socket Inspection**: [`video/FinalAssignment_PegPlacement_20261112_131235.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/FinalAssignment_PegPlacement_20261112_131235.mp4) | [`images/`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/images) (32 stage photos, 4 motion plans, 5 scans) |

---

## 3. Autonomy & Authenticity Verification: Dual-Camera Proof (Run 2)

To demonstrate the **complete autonomy**, continuous timeline, and authenticity of the autonomous perception-planning-manipulation pipeline, Run 2 was recorded simultaneously with two independent cameras:

1. **System Overhead Camera Capture (`video/run.mp4`)**:
   - Recorded from the overhead Intel RealSense perspective from timestamp `13:05:25` to `13:11:32` ($6\text{ min } 07\text{ s}$).
   - Shows the live perception bounding boxes, ArUco fiducials, and end-effector trajectory tracking.

2. **External Mobile Camera with Running Clock Timer (`video/ExternalMobile_ClockTimerProof_20261112_130526.mp4`)**:
   - **Start Timestamp**: `20261112_130526` (started synchronously with the robot pipeline at `13:05:26`).
   - **Continuous Clock Timer Proof**: The camera was framed to capture both the entire OpenMANIPULATOR-X workspace and a **live running digital stopwatch / clock timer**.
   - **Authenticity Assurance**: The uninterrupted progression of the clock timer proves that:
     - No manual teleoperation or human intervention occurred during the run.
     - The video was not spliced, edited, paused, or speed-ramped.
     - The pipeline executed fully autonomously in real physical time.
   - *Note on mobile duration*: Mobile recording ran for $\approx 3\text{m } 06\text{s}$ until internal phone storage capacity was exhausted and automatically paused.

3. **Final Socket Assignment Close-Up Inspection (`video/FinalAssignment_PegPlacement_20261112_131235.mp4`)**:
   - **Timestamp**: `20261112_131235` (recorded immediately after the autonomous run finished at `13:11:32`).
   - Provides clear high-definition close-up verification that each geometric peg (circle, square, hexagon, triangle) is completely inserted into its corresponding socket on the green mat.

---

## 4. Video Compression & Cloud Archive Alignment

* **In-Repository Version**:
  - The mobile timer recording [`video/ExternalMobile_ClockTimerProof_20261112_130526.mp4`](file:///Users/aai/Desktop/ITR_Proj/Project1%20Task1/videos/OneGO_20261112_130525/video/ExternalMobile_ClockTimerProof_20261112_130526.mp4) has been encoded with high-efficiency hardware H.264 at $1080\text{p60}$ ($48\text{ MB}$, under GitHub's $50\text{ MB}$ streaming threshold). This ensures instant in-browser playback on GitHub without requiring external players or Git LFS setup.
* **Raw Master Cloud Archive**:
  - The original uncompressed $628\text{ MB}$ raw camera recording ($28.3\text{ Mbps}$ mobile master) is preserved in the [Task 1 Google Drive Folder](https://drive.google.com/drive/folders/1UufguEVD1WDDrdQYMmFn4csoTbAUtxeu?usp=sharing).

---

## 5. Recording & Camera Framing Guidelines

* **Framing & View**:
  - Show a wide, stable angle capturing the **entire OpenMANIPULATOR-X arm**, the **overhead Intel RealSense camera**, and the **green sorting mat** with all pegs and sockets visible.
  - Ensure the robot base and gripper finger contact are clearly visible.
* **Resolution & Format**:
  - Container: `.mp4` (preferred) or `.mov`
  - Codec: H.264
  - Resolution: 1080p (1920×1080) or 720p (1280×720) at 30 fps or 60 fps.
* **Lighting**:
  - Ensure even lab lighting without harsh glare on the green sorting mat or colored acrylic pegs.
