# Important Project Knowledge Base & Requirements (`imp.md`)

This document tracks all critical requirements, user instructions, architecture decisions, edge cases, and testing milestones for **Task 1: The Peg-Sorting**.

---

## 1. Task Overview & Core Objectives

* **Project**: Task 1: The Peg-Sorting (Tri-Wizard Robotics Endgame).
* **Robot**: ROBOTIS OpenMANIPULATOR-X (Official Overview: https://docs.robotis.com/docs/systems/openmanipulator_x/overview/).
  * Actuators: DYNAMIXEL XM430-W350-T (4 Arm Joints + 1 Gripper).
  * Joint IDs:
    * ID 11: Joint 1 (Base Pan / Yaw)
    * ID 12: Joint 2 (Shoulder Pitch)
    * ID 13: Joint 3 (Elbow Pitch)
    * ID 14: Joint 4 (Wrist Pitch)
    * ID 15: Gripper (Parallel Open/Close)
  * Protocol: DYNAMIXEL Protocol 2.0. Default Baud Rate: 1,000,000 bps (1 Mbps).
* **Hardware Controller**: ROBOTIS U2D2 (USB-to-TTL / RS485 converter).
  * Port on macOS: `/dev/cu.usbserial-FTA2W88F` (CONFIRMED & PHYSICALLY VERIFIED ON HARDWARE).
  * Actuators Verified: 5/5 responding and moving physically.
    * ID 11: Joint 1 (Base Pan)
    * ID 12: Joint 2 (Shoulder Pitch)
    * ID 13: Joint 3 (Elbow Pitch)
    * ID 14: Joint 4 (Wrist Pitch)
    * ID 15: Gripper
  * Default Ready Pose (L-shape from CAD) tested & verified on physical arm.
  * Interactive keyboard teleoperation with E-Stop tested & verified on physical arm.
* **macOS Management & Software Toolchain**:
  * **DYNAMIXEL Wizard 2.0**: Available directly on the **Mac App Store** (Native Universal build for Apple Silicon & Intel).
  * **Python Control Library**: `dynamixel-sdk` (installed & verified on this Mac with `pyserial`).
  * **Pure Python Architecture**: The robot can be driven 100% natively on macOS via Python without requiring Ubuntu/ROS.
  * **Hardware Verification Suite**: Located in `openmanipulator_test/`:
    * `01_scan_and_ping.py`: Auto-detects U2D2 port on macOS and pings IDs 11-15 (VERIFIED WORKING).
    * `02_gentle_gripper_test.py`: Safely opens/closes gripper (ID 15) by ~13° with low velocity.
    * `03_gentle_joint_nudge.py`: Rotates base joint (ID 11) horizontally by +5° and returns smoothly.
    * `04_keyboard_teleop.py` (and `T1/src/control/teleop_keyboard.py`): Real-time interactive teleoperation:
      * Base Pan: `A` (Left) / `D` (Right)
      * Shoulder Pitch: `W` (Up) / `S` (Down)
      * Elbow Pitch: `I` (Up) / `K` (Down)
      * Wrist Pitch: `U` (Up) / `J` (Down)
      * Gripper: `O` (Open) / `P` (Close) or `SPACE` (Toggle)
      * Step size: `1` (1°), `2` (2.5°), `3` (5°)
      * Safety: `T` (Torque Toggle), `ESC` / `X` (Emergency Stop)
* **Sensors**: Single RGB-D Camera Setup (UPDATED & CONFIRMED FROM PHYSICAL SETUP):
  * **Mounting**: Intel RealSense D435i mounted on an aluminum extrusion stand at the front-right of the table, angled downward at ~45°-55° overlooking the entire workspace.
  * **Coverage**: Full view of both the green working mat (peg pickup zone) and the wooden socket board.
  * **Single-Camera Strategy**:
    1. **Perspective Side-Wall Elimination via Depth Slicing**:
       Because the camera is angled, the 10 cm tall peg side walls are visible. By slicing depth to the top plane ($Z \in [Z_{\text{top}} - 15\text{mm}, Z_{\text{top}} + 15\text{mm}]$), we isolate the pure 2D top polygon (Square, Triangle, Hexagon, Circle) with zero perspective distortion!
    2. **Zero-Occlusion Snapshot & Freeze**:
       Arm parks in `HOME_READY` out of view $\to$ Camera captures global snapshot $\to$ Freezes all 4 peg and socket poses in memory $\to$ Arm executes pick-and-place without camera occlusion interference $\to$ Returns to Home for post-insertion verification snapshot!
    3. **Eye-to-Hand Calibration ($\mathbf{T}_{\text{cam}}^{\text{robot}}$)**:
       Rigid 3D transformation $(\mathbf{R}, \mathbf{t})$ transforms Camera coordinates $(X_c, Y_c, Z_c)$ directly into Robot Base $(X_r, Y_r, Z_r)$.
* **Device Knowledge Base**: Full hardware documentation cataloged in [`device/`](file:///Users/aai/Desktop/ITR_Proj/device/):
  * `CAMERA_INFO.md`: Complete RealSense D435i profile & stream configurations.
  * `ROBOT_INFO.md`: OpenMANIPULATOR-X kinematics, servo tables, link lengths.
  * `OBJECTS_AND_BOARD.md`: Real peg colors, dimensions, socket ordering.
  * `SAFETY_AND_PROTECTION.md`: Electrical safety, surge protection & galvanic isolation.
  * `hardware_protector.py`: Active software auto-disconnector watchdog layer.
  * `images/`: High-resolution reference photos of real pegs, board, and CAD stance.
* **Objects**: 4 pegs (CONFIRMED FROM PHYSICAL SETUP):
  * **Triangle Peg**: **Yellow** (~4–6 cm width, ~10 cm height)
  * **Square Peg**: **Blue** (~4–6 cm width, ~10 cm height)
  * **Hexagon Peg**: **Orange / Red** (~4–6 cm width, ~10 cm height)
  * **Circle / Cylinder Peg**: **Green** (~4–6 cm diameter, ~10 cm height)
* **Target**: Socket board with matching shaped cutouts (CONFIRMED FROM PHYSICAL SETUP):
  * Layout: Single vertical strip with laser-cut red internal socket lining.
  * Socket Sequence (Top to Bottom): Triangle $\to$ Square $\to$ Hexagon $\to$ Circle.
* **Modes**:
  1. **Full Autonomy**: Automatic perception $\to$ planning $\to$ pick $\to$ place $\to$ verification.
  2. **Teleoperation**: Keyboard-controlled manual override with single-key Emergency Stop (`E-Stop`).

---

## 2. Judging Rubric & Deliverables

| Category | Points | Notes |
| :--- | :--- | :--- |
| **Autonomy Success** | **50** | Fully autonomous pick-and-place of all 4 pegs |
| **Teleoperation Baseline** | **30** | Working keyboard teleop with E-Stop |
| **Vision: Color Detection** | **20** | Robust color segmentation in varying conditions |
| **Vision: Shape Detection** | **20** | Accurate classification of Square, Triangle, Circle, Hexagon |
| **GitHub README** | **10** | Setup, calibration guide, launch scripts, reproducibility |
| **Bonus: Fastest Execution** | **+15** | Lowest cycle time among teams |
| **Bonus: Zero Failures** | **+5** | No dropped pegs or missed insertions |
| **Total Possible** | **120** | (100 base + 20 bonus) |

---

## 3. User Directives & Constraints

1. **Pre-Hardware Testing (Zero-Risk Staging)**:
   * **Mandatory directive**: Every component (vision, pose estimation, kinematics, planning, teleoperation) MUST be fully simulated and verified via unit tests and mock/physics simulation before deploying on actual physical hardware.
2. **Dynamic Knowledge Persistence**:
   * All critical insights, configuration details, formulas, and updates must be recorded in this `imp.md` file.

---

## 4. Multi-Tier Testing Roadmap

* **Tier 1: Synthetic Vision Unit Testing** (COMPLETED & VERIFIED)
  * Implemented synthetic RGB-D generator (`src/vision/synthetic_scene.py`) with 16-bit depth, Gaussian noise, lighting gradient, and depth boundary dropouts.
  * Implemented HSV color segmenter (`src/vision/color_segmentation.py`) with multi-range wrap-around support for red.
  * Implemented shape detector (`src/vision/shape_detection.py`) with circle ratio, polygon approximation, and 3D deprojection.
  * Verified 100% accuracy across 40 randomized pegs in `tests/test_vision_pipeline.py`.
  * Visual diagnostic output verified at `tests/output_debug_vision.png`.
* **Tier 2: Kinematics & Workspace Feasibility** (COMPLETED & VERIFIED)
  * Implemented analytical Forward & Inverse Kinematics (`src/planning/kinematics.py`) with zero-latency closed-form solution.
  * Verified 0.000 mm roundtrip accuracy in `tests/test_kinematics.py`.
  * OpenMANIPULATOR-X Link Lengths: $L_1 = 77\text{ mm}$, $L_2 = 130\text{ mm}$, $L_3 = 124\text{ mm}$, $L_4 = 126\text{ mm}$.
  * Enforced physical joint limits: $q_2 \in [\pm 90^\circ]$, $q_3 \in [\pm 105^\circ]$, $q_4 \in [\pm 110^\circ]$.
  * Built single-peg pick-and-place state machine script: `openmanipulator_test/06_test_pick_and_place.py` (and `T1/scripts/test_single_peg_pick_and_place.py`).
* **Tier 3: Autonomous Single-Camera Sorter Pipeline** (COMPLETED & READY FOR TESTING)
  * Implemented depth-slicing single-camera vision pipeline (`src/vision/single_camera_pipeline.py`).
  * Implemented Eye-to-Hand rigid 3D calibration (`src/planning/calibration.py` & `config/camera_calib.yaml`).
  * Built master autonomous sorter (`scripts/run_autonomous_sorter.py`):
    * `--single <shape>`: Tests 1 figure (e.g. Square, Triangle, Hexagon, Circle).
    * `--all`: Sorts all 4 figures sequentially.
    * `--dry-run`: Runs safely elevated in mid-air (zero collision risk).
    * `--live`: Executes live tabletop pick and place.
* **Tier 4: Software 3D Simulation & Visual Verification** (COMPLETED & READY)
  * **MuJoCo 3D Interactive Simulator** (`sim/simulate_mujoco.py` & `sim/scene.xml`): Full physics simulation replicating the workbench, green mat, socket board, RealSense stand, 4 colored pegs, and OpenMANIPULATOR-X arm with real collision and gravity.
  * **3D Trajectory Visualizer** (`sim/visualize_trajectory_3d.py`): Zero-dependency 3D plot generating Cartesian trajectory curves and robot joint poses ([`trajectory_3d_preview.png`](file:///Users/aai/Desktop/ITR_Proj/device/images/trajectory_3d_preview.png)).
* **Tier 3: Hardware Abstraction Layer (HAL)**
  * Unified `RobotInterface` base class.
  * `MockRobot` (memory/headless execution) vs. `RealManipulatorX` (Dynamixel SDK).
  * Smooth switching via runtime flag (`--mode sim` vs `--mode real`).
* **Tier 4: MuJoCo Physics & End-to-End Simulation**
  * Full physics simulation in MuJoCo with Manipulator X URDF/MJCF.
  * Virtual RGB-D cameras rendering to the real perception pipeline.
  * Closed-loop pick, transit, align, and insert testing.

---

## 5. Critical Edge Cases & Mitigation Matrix

Detailed Architectural Breakdown: [`EDGE_CASES.md`](file:///Users/aai/Desktop/ITR_Proj/T1/EDGE_CASES.md)

| Priority | Category | Edge Case | Root Cause | Engineering Solution |
| :--- | :--- | :--- | :--- | :--- |
| **P0** | **Safety** | **Thermal & Voltage Breach** | High ambient temp or power supply fluctuation. | HardwareProtector cuts torque (<10ms) if $T > 65^\circ\text{C}$ or $V \notin [10.5, 14.6]\text{V}$. |
| **P0** | **Safety** | **Comm Timeout (`COMM_RX_TIMEOUT`)** | Loose USB/TTL cable on U2D2 bus. | Auto-reconnection retry loop (3x) + safe motion abort. |
| **P1** | **Fault Recovery** | **Object Slip / Drop during Transfer** | Insufficient grip or sudden jerk drops peg. | **Dual Modality (Motor + Optical):**<br>1. *Motor*: Gripper count snaps shut ($c > 2310$) + load drops ($< 15\text{ mA}$).<br>2. *Camera*: Checkpoint 1 verifies lift clearance on mat & in-hand ROI; Checkpoint 2 detects tabletop fallen pegs; Checkpoint 3 confirms socket occupancy.<br>**Self-Healing:** Halt $\to$ Elevate $+Z$ $\to$ `HOME_READY` $\to$ Optical Re-scan $\to$ Re-localize $\to$ Re-pick! |
| **P1** | **Kinematics** | **Object Out of Reachable Workspace** | Peg falls or placed outside vertical reach ($r > 215\text{ mm}$). | **Multi-Tier Fallback:**<br>1. *Angled Grasp* ($\phi \in [-65^\circ, -45^\circ]$) extends reach to $290\text{ mm}$.<br>2. *Inward Sweep / Rake* drags peg inward from $290\text{--}340\text{ mm}$.<br>3. Operator alert with exact deficit $\Delta r$. |
| **P1** | **Perception** | **Camera Extrinsic Shift (Accidental Bump)** | Stand bumped; camera pose $(\mathbf{R}, \mathbf{t})$ drifts. | **Online Recalibration:** Detects 4 socket board landmarks in 3D $\to$ Computes drift $\Delta \mathbf{t}, \Delta \mathbf{R} \to$ Solves Kabsch SVD ($< 2\text{ ms}$) $\to$ Auto-updates $\mathbf{T}_c^r$. |
| **P1** | **Manipulation** | **Empty / False Grasp (Air Grasp)** | Gripper closed on empty space. | If gripper reaches $> 2320$ counts with $< 15\%$ effort $\to$ abort transit $\to$ open $\to$ re-scan. |
| **P1** | **Insertion** | **Socket Jamming / Edge Resistance** | 1–2 mm chamfer misalignment prevents seating. | Detect load spike on J2/J3/J4 $\to$ execute Archimedean micro-spiral search ($\pm 1.5\text{ mm}$ XY wobble) until seated. |
| **P1** | **Manipulation** | **Peg Toppled on Side** | 10 cm peg knocked over; aspect ratio $> 1.8$. | Detect long axis from image moments $\to$ switch to horizontal side grasp $\to$ tilt wrist $90^\circ$ in air. |
| **P2** | **Perception** | **Peg-to-Peg Clustering / Occlusion** | Multiple pegs touching on mat. | Color-channel separation + Watershed contour splitting. |
| **P2** | **Perception** | **Lighting / Specular Glare** | White glare washes out HSV. | CIELAB luminance decoupling + morphological closing. |
| **P2** | **Perception** | **Depth Dropout on Edges** | RealSense IR stereo shadow on boundary. | Sample median depth from 5x5 ROI around centroid, discarding 0 mm. |
| **P2** | **Perception** | **Circle vs Hexagon Disambiguation** | Rounded corners misclassified by approxPolyDP. | Dual-metric: Enclosing Circle Area Ratio ($A / \pi R^2 > 0.91$ circle, $0.77 \le \text{ratio} \le 0.91$ hex). |
| **P3** | **Insertion** | **Peg Stickiness on Retreat** | Tight rubber finger friction pulls peg out. | Release dwell (150ms) + horizontal wiggle ($\pm 2\text{ mm}$) before upward lift. |
| **P3** | **Kinematics** | **Multi-Turn Encoder 360° Wrap** | Joint 1 unwinds full circle. | Modulo shortest path target: `round(pos / 4096) * 4096 + target_delta`. |

---

## 6. Mathematical Formulas & Conventions

### 2D to 3D Deprojection (Pinhole Camera Model)
Given pixel coordinates $(u, v)$, depth $Z$ in millimeters, and camera intrinsics $(f_x, f_y, c_x, c_y)$:
$$X_c = \frac{(u - c_x) \times Z}{f_x}$$
$$Y_c = \frac{(v - c_y) \times Z}{f_y}$$
$$Z_c = Z$$

### Extrinsic Transformation (Camera to Robot Base)
$$\mathbf{P}_{\text{robot}} = \mathbf{R}_{c}^{r} \mathbf{P}_c + \mathbf{T}_{c}^{r}$$

### Rotational Symmetry Angle Normalization
For a target socket angle $\theta_s$ and peg orientation $\theta_p$ with symmetry order $N$:
$$\Delta \theta = (\theta_s - \theta_p) \pmod{\frac{360^\circ}{N}}$$
$$\text{If } \Delta \theta > \frac{180^\circ}{N}, \quad \Delta \theta \leftarrow \Delta \theta - \frac{360^\circ}{N}$$
This guarantees the wrist rotates the minimum possible angle ($|\Delta \theta| \le \frac{180^\circ}{N}$).

### Shape Disambiguation Metric (Enclosing Circle Ratio)
$$\text{Ratio} = \frac{\text{Contour Area}}{\pi R_{\text{enclosing}}^2}$$
* **Circle**: $\text{Ratio} > 0.91$ (or $\ge 7$ vertices at $\epsilon = 0.020 \times P$)
* **Hexagon**: $\text{Ratio} \in [0.77, 0.91]$ (6 vertices at $\epsilon = 0.020 \times P$)
* **Square**: $\text{Ratio} \in [0.58, 0.76]$ (4 vertices)
* **Triangle**: $\text{Ratio} < 0.58$ (3 vertices)

---

## 7. Keyboard Teleoperation Key Mappings (Quick Reference)

Dedicated reference file: [`KEY_MAPPINGS.md`](file:///Users/aai/Desktop/ITR_Proj/T1/KEY_MAPPINGS.md)

### Motor Joint Motion Controls
* **Joint 1 (Base Pan, ID 11)**:
  * `A` : Rotate Left ($-Yaw$)
  * `D` : Rotate Right ($+Yaw$)
* **Joint 2 (Shoulder Pitch, ID 12)**:
  * `W` : Lift Arm Up ($-Pitch$)
  * `S` : Lower Arm Down ($+Pitch$)
* **Joint 3 (Elbow Pitch, ID 13)**:
  * `I` : Forearm Up ($-Pitch$)
  * `K` : Forearm Down ($+Pitch$)
* **Joint 4 (Wrist Pitch, ID 14)**:
  * `U` : Tilt Wrist Up ($-Pitch$)
  * `J` : Tilt Wrist Down ($+Pitch$)
* **Joint 5 (Gripper, ID 15)**:
  * `O` : Open Fingers
  * `P` : Close Fingers
  * `SPACE` : Toggle Open / Close

### Precision & Safety Controls
* `1` : Fine Step ($1.0^\circ$)
* `2` : Medium Step ($2.5^\circ$ - default)
* `3` : Coarse Step ($5.0^\circ$)
* `H` : **Move to Default Ready Pose** (L-shape posture from reference CAD image)
* `T` : Toggle Torque (ON / OFF)
* `ESC` / `X` : **EMERGENCY STOP** (instantly cuts power to all motors)
* `Q` : Quit & Safely Release Torque

---

## 8. Default Ready / Home Pose Specifications (Reference Image)
* **Posture Description**: L-shaped ready stance:
  * **Joint 1 (Base Pan, ID 11)**: Centered forward (closest multiple of $360^\circ$ wrap $\approx 4096$)
  * **Joint 2 (Shoulder Pitch, ID 12)**: Vertical ($90^\circ$ to table, motor zero $= 2048$)
  * **Joint 3 (Elbow Pitch, ID 13)**: $90^\circ$ forward horizontal bend ($\approx 1100$ counts)
  * **Joint 4 (Wrist Pitch, ID 14)**: Horizontal in-line with forearm ($\approx 2048$ counts)
  * **Joint 5 (Gripper, ID 15)**: Open ($\approx 1800$ counts)
* **Scripts**:
  * Standalone: `openmanipulator_test/05_go_to_home_pose.py`
  * Teleop Hotkey: Press `H` in `04_keyboard_teleop.py`

---

## 9. Simulation & Verification Suite (Tier 4 Verified)

* **Off-Screen 3D MP4 Video Renderer** (`sim/render_pick_and_place.py`):
  * Headless MuJoCo renderer producing full 25 FPS video: [`sim/openmanipulator_pick_and_place.mp4`](file:///Users/aai/Desktop/ITR_Proj/T1/sim/openmanipulator_pick_and_place.mp4).
  * Completely bypasses macOS AppKit GUI thread restrictions.
  * Verified 360 frames (14.4 seconds), illustrating the complete 10-step pick, lift, transit, insert, and retreat cycle for the Blue Square peg.
* **Interactive 3D Physics Simulator** (`sim/simulate_mujoco.py`):
  * Runs on macOS via `mjpython sim/simulate_mujoco.py` with `mujoco.viewer.launch(model, data)` on the Cocoa main thread and `set_mjcb_control()`.
  * Allows live 3D mouse rotation, zooming, and physics inspection.
* **Reachable Kinematic Envelopes**:
  * Vertical Grasp ($\text{pitch} = -90^\circ$):
    * Pickup zone $(X=160, Y=60)$: $Z \in [40, 100]\text{ mm}$ reachable.
    * Socket zone $(X=160, Y=-60 \text{ to } -140)$: $Z \in [40, 80]\text{ mm}$ reachable.
  * Home Ready Stance: Directly specified via joint angles `[0.0, 0.0, 1.45, 0.0]` (FK: $X \approx 248\text{ mm}, Y=0\text{ mm}, Z \approx 237\text{ mm}, \text{pitch} \approx 0^\circ$).

---

## 10. Step-by-Step Validation Guide (1st Figure Test: Blue Square)

### Step A: 3D Software Simulation (Zero Risk)
1. **View Rendered 3D Pick-and-Place Video**:
   * Open [`sim/openmanipulator_pick_and_place.mp4`](file:///Users/aai/Desktop/ITR_Proj/T1/sim/openmanipulator_pick_and_place.mp4) in macOS QuickTime / Finder to see the robot execute the complete sorting cycle.
2. **Launch Interactive 3D Physics GUI**:
   ```bash
   cd /Users/aai/Desktop/ITR_Proj/T1
   mjpython sim/simulate_mujoco.py
   ```
   *(Click & drag to orbit in 3D around the table and inspect contact forces).*

### Step B: Hardware Connection & Safety Check
1. Ensure the **12V DC power brick** is connected to the ROBOTIS power hub and switched ON (red LED indicator lit).
2. Plug the **ROBOTIS U2D2 USB cable** into the Mac.
3. Plug the **Intel RealSense D435i camera** into a USB 3.0 port.
4. Verify communication:
   ```bash
   cd /Users/aai/Desktop/ITR_Proj/openmanipulator_test
   python3 01_scan_and_ping.py
   ```
   *(Must show all 5 actuators IDs 11–15 responding).*

### Step C: Camera Feed Verification
```bash
cd /Users/aai/Desktop/ITR_Proj/T1
python3 scripts/test_camera_stream.py
```
*(Verifies camera stream and saves a test snapshot to `device/images/test_camera_capture.jpg`).*

### Step D: Physical 1-Peg Test (Blue Square)
1. **Mid-Air Dry Run (Safe Elevation)**:
   ```bash
   cd /Users/aai/Desktop/ITR_Proj/T1
   python3 scripts/run_autonomous_sorter.py --single square --dry-run
   ```
   *(Robot executes the pick-and-place sequence 20 mm higher in the air with no physical contact).*
2. **Live Tabletop Run**:
   ```bash
   cd /Users/aai/Desktop/ITR_Proj/T1
   python3 scripts/run_autonomous_sorter.py --single square --live
   ```
   *(Picks the blue square peg from the green mat and places it into the square socket cutout).*



