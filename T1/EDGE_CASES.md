# Comprehensive Edge Cases & Fault Recovery Architecture (`EDGE_CASES.md`)

This document identifies, prioritizes, and outlines the deterministic recovery algorithms for all physical, perceptual, kinematic, and environmental edge cases in **Task 1: The Peg-Sorting**.

---

## 1. Edge Case Priority Hierarchy Overview

| Priority Level | Category | Failure Impact | Recovery Objective |
| :--- | :--- | :--- | :--- |
| **Tier P0: Critical Safety & Protection** | Electrical, Thermal, Hardware Stalls | Actuator burnout, board short-circuit, Mac damage | Instant torque cut (<10ms), state isolation, safe cooling/halt. |
| **Tier P1: Task-Critical Execution Failures** | Object Slip/Drop, Out-of-Reach, Camera Bump, Insertion Jam, Peg Toppling | Autonomous cycle fails, dropped peg, zero points | In-flight detection, automated re-scan, trajectory re-planning, self-healing recovery. |
| **Tier P2: Perception & Sensing Variances** | Glare, Clustering, Mutual Occlusion, Depth Dropout, Color Ambiguity | Contours merged, noisy depth, misclassification | Multi-spectral HSV + Lab filtering, ROI median depth, watershed contour splitting. |
| **Tier P3: Tolerances & Boundary Cases** | Friction Stickiness, Partial Seating, Multi-turn Wrap | Incomplete insertion, joint angle winding | Compliant search, micro-spiral seating, symmetry wrap modulo. |

---

## 2. Priority P0: Critical Safety & Hardware Integrity

### EC-01: Thermal & Electrical Overload (Overheating / Voltage Spikes)
* **Symptom**: Servo temperature exceeds safe threshold ($T > 65^\circ\text{C}$), or voltage deviates from safe operating band ($V < 10.5\text{V}$ or $V > 14.6\text{V}$).
* **Detection**: Background thread in `device/hardware_protector.py` continuously samples `ADDR_PRESENT_TEMPERATURE` (146) and `ADDR_PRESENT_INPUT_VOLTAGE` (144) at 10 Hz across all 5 servos.
* **Deterministic Action**:
  1. Instantly write `ADDR_TORQUE_ENABLE = 0` on all IDs.
  2. Emit high-priority terminal alarm with motor ID, voltage, and temperature telemetry.
  3. Lock robot state until temperature drops below $50^\circ\text{C}$.

### EC-02: Actuator Communication Timeout (`COMM_RX_TIMEOUT`)
* **Symptom**: USB cable pulled or loose connection causes packet loss (`res = -3001`).
* **Detection**: Wrapper catches non-zero return codes on read/write transactions.
* **Deterministic Action**:
  1. Re-open port handle up to 3 retry attempts with 200ms backoff.
  2. If port fails, halt execution, preserve current waypoint progress, and signal operator.

---

## 3. Priority P1: Task-Critical Execution Failures (Core User Directives)

```
                              ┌────────────────────────────────────────┐
                              │     MASTER RECOVERY STATE MACHINE      │
                              └──────────────────┬─────────────────────┘
                                                 │
                   ┌─────────────────────────────┼─────────────────────────────┐
                   ▼                             ▼                             ▼
       ┌───────────────────────┐   ┌───────────────────────────┐   ┌───────────────────────┐
       │   OBJECT DROP / SLIP  │   │   OUT-OF-REACH DETECTED   │   │   CAMERA BUMP / SHIFT │
       │        (EC-05)        │   │          (EC-06)          │   │        (EC-07)        │
       └───────────┬───────────┘   └─────────────┬─────────────┘   └───────────┬───────────┘
                   │                             │                             │
       1. Halt transit motion        1. Flag IK / workspace limit  1. Detect board displacement
       2. Retract arm to HOME_READY  2. Test angled grasp fallback 2. Calculate delta R & t
       3. Overhead camera re-scan    3. If distance > 220 mm:      3. Run closed-form Kabsch
       4. Re-localize dropped peg       Perform INWARD SWEEP          recalibration (<5ms)
       5. Plan new pickup trajectory 4. If out of bounds:          4. Update T_cam_to_robot
       6. Resume sorting cycle          Alert operator with margin 5. Resume perception
```

---

### EC-05: Object Slip / Drop during Transfer (User Case 1)
* **Symptom**: During vertical lift or transit toward the socket board, the peg slips from the gripper fingers and falls onto the tabletop or mat.
* **Dual-Modality Multi-Layer Detection (Proprioceptive + Optical Camera Ground Truth)**:
  1. **Proprioceptive (Gripper Motor Feedback)**:
     * When grasping a 40–50 mm peg, the gripper stops at encoder count $c \in [1950, 2240]$ and draws holding current ($I \approx 40 \sim 120\text{ mA}$).
     * If the peg slips out, the gripper snaps shut to its closed limit ($c > 2310$ counts) and current drops to idle ($I < 15\text{ mA}$).
     * Continuously sampled at 20 Hz during the transit trajectory via `GripperGuard`.
  2. **Optical Camera Ground Truth Verification (`VisualDropDetector`)**:
     * **Checkpoint 1 (Post-Lift Verification)**: Evaluated immediately upon lifting along $+Z$ to $Z = 85\text{ mm}$:
       - *Pickup Mat Clearance*: Confirms the original $(X_p, Y_p)$ pickup coordinates on the green mat are now empty. If the colored peg is still detected on the mat, grasp failed at pickup (`MISSED_AT_PICKUP`).
       - *In-Hand ROI Inspection*: Confirms the target color blob is elevated in the air at the gripper 3D coordinates.
       - *Tilt / Partial Slip Check*: Checks the bounding aspect ratio of the peg in-hand. If the aspect ratio $> 1.55$, the peg is tilted/slipping within the fingers before complete detachment (`PARTIAL_SLIP_TILTED`).
     * **Checkpoint 2 (Tabletop Drop Scanner)**: If a slip is suspected during transit, the camera scans the entire table and mat surface ($Z \in [20, 75]\text{ mm}$). If a colored blob matching the peg is detected outside the gripper, a drop is confirmed and its exact $(X_{\text{drop}}, Y_{\text{drop}}, Z_{\text{drop}})$ 3D position is extracted.
     * **Checkpoint 3 (Post-Placement Socket Occupancy)**: After releasing the peg and returning to `HOME_READY`, the camera takes an optical verification snapshot of the target socket cutout:
       - Confirms the socket is filled with the matching colored peg (`SOCKET_FILLED`).
       - If the cutout remains hollow/empty (`SOCKET_EMPTY`), marks the placement incomplete and triggers autonomous re-pick.
* **Deterministic Recovery Protocol**:
  1. **Immediate Motion Halt**: Stop the transit motion immediately to prevent colliding with or kicking the dropped peg.
  2. **Safe Vertical Retract**: Elevate arm strictly along $+Z$ to transit clearance ($Z = 120\text{ mm}$).
  3. **Return to Home Pose**: Move arm to `HOME_READY` stance to clear the camera's field of view.
  4. **Dynamic Overhead Re-Scan**: Capture a fresh RGB-D frame of the workspace.
  5. **Re-Localization**: `VisualDropDetector.detect_tabletop_dropped_peg` localizes the dropped peg at its new $(X_{\text{new}}, Y_{\text{new}}, Z_{\text{new}})$ coordinates and orientation $\theta_{\text{new}}$.
  6. **Re-Plan & Execute**: Check reachable workspace via `WorkspaceAnalyzer`, compute new approach waypoints, grasp the peg, and resume the pick-and-place sequence. Max retry count: 3 attempts.

---

### EC-06: Object Out of Reachable Workspace (User Case 2)
* **Symptom**: Peg is located or rolls beyond the arm's kinematic reach ($r > r_{\max}$ or $r < r_{\min}$), or analytical IK returns `None`.
* **Detection**:
  * Before generating any physical motion command, the `WorkspaceAnalyzer` checks:
    $$r_{\text{target}} = \sqrt{X^2 + Y^2}$$
    * For vertical top-down grasp ($\phi = -90^\circ$): Valid workspace radius is $r \in [110\text{ mm}, 215\text{ mm}]$.
    * Absolute physical arm extension ($L_2 + L_3 + L_4$): Maximum radius is $r_{\text{abs}} = 380\text{ mm}$.
  * If $r > 215\text{ mm}$, vertical IK fails.
* **Deterministic Recovery Protocol**:
  1. **Tier A: Angled Grasp Fallback (Reach Extension)**:
     * When $r \in [215\text{ mm}, 290\text{ mm}]$, relaxing the vertical pitch constraint from $\phi = -90^\circ$ to $\phi \in [-65^\circ, -45^\circ]$ extends the effective forward reach by up to 75 mm while maintaining solid finger contact on the 10 cm peg side walls.
  2. **Tier B: Rake / Sweep Inward Maneuver**:
     * When $r \in [290\text{ mm}, 340\text{ mm}]$ (too far for secure grasp, but within arm reach):
       * The arm positions the closed gripper *behind* the peg ($r_{\text{peg}} + 35\text{ mm}$), lowers to table clearance ($Z = 20\text{ mm}$), and gently drags/sweeps radially inward by 80–120 mm into the prime workspace ($r \approx 160\text{ mm}$).
       * Arm retracts to `HOME_READY`, camera re-scans the new location, and standard pick proceeds.
  3. **Tier C: Clear Operator Notification**:
     * If $r > 340\text{ mm}$ (completely beyond physical reach), the system gracefully halts, flashes an alert displaying the exact coordinates $(X, Y, Z)$ and reach deficit $\Delta r = r - 215\text{ mm}$, and prompts the operator to nudge the peg onto the green mat.

---

### EC-07: Camera Extrinsic Shift / Accidental Bump (User Case 3)
* **Symptom**: Stand or cable is bumped during setup, shifting the camera position $(X_c, Y_c, Z_c)$ or tilt angle $(\alpha, \beta, \gamma)$. All projected 3D coordinates $(\mathbf{P}_{\text{robot}} = \mathbf{R} \mathbf{P}_c + \mathbf{t})$ become invalid by several centimeters.
* **Detection (Visual Anchor Tracking)**:
  * The wooden socket board has fixed, rigid geometry that does not move relative to the robot base.
  * In the baseline setup, the 4 socket centers form an invariant straight line with known spacing:
    * Triangle $\to$ Square: $\Delta Y = 50.0\text{ mm}$
    * Square $\to$ Hexagon: $\Delta Y = 50.0\text{ mm}$
    * Hexagon $\to$ Circle: $\Delta Y = 50.0\text{ mm}$
  * During the pre-flight `HOME_READY` scan, `CameraDriftMonitor` detects the 4 socket centroids in 3D camera space.
  * Extrinsic Drift Metric:
    $$\text{Drift}_{\text{trans}} = \|\mathbf{t}_{\text{measured}} - \mathbf{t}_{\text{nominal}}\|$$
    $$\text{Drift}_{\text{rot}} = \arccos\left(\frac{\text{Tr}(\mathbf{R}_{\text{measured}} \mathbf{R}_{\text{nominal}}^T) - 1}{2}\right)$$
  * If $\text{Drift}_{\text{trans}} > 4.0\text{ mm}$ or $\text{Drift}_{\text{rot}} > 1.5^\circ$, camera shift is flagged!
* **Deterministic Recovery Protocol (Automated Online Recalibration)**:
  1. The system extracts the 3D measured positions of the 4 sockets from the current image.
  2. Pairs them with known true CAD socket coordinates in the robot base frame.
  3. Solves the Kabsch SVD algorithm ($O(1)$ computation, $< 2\text{ ms}$) to compute the updated extrinsic matrix $[\mathbf{R}_{\text{new}} \mid \mathbf{t}_{\text{new}}]$.
  4. Automatically updates `T1/config/camera_calib.yaml` and pipeline memory.
  5. The sorting sequence continues without requiring manual recalibration!

---

### EC-08: Empty / False Grasp (Air Grasp)
* **Symptom**: Arm descends to pick a peg, closes gripper, but peg was missed or shifted.
* **Detection**:
  * Gripper motor (ID 15) reaches fully closed encoder count ($> 2320$ counts) with low torque effort ($< 15\%$).
* **Deterministic Recovery**:
  1. Do not proceed to socket.
  2. Open gripper immediately.
  3. Elevate $+30\text{ mm}$ along $+Z$, move to `HOME_READY`.
  4. Re-scan mat and re-estimate peg centroid.

---

### EC-09: Socket Jamming / Edge Resistance during Insertion
* **Symptom**: During socket descent, the peg edge catches on the 1–2 mm chamfer lip of the laser-cut board, preventing full downward travel.
* **Detection**:
  * Joint 2/3/4 Present Load spikes ($> 55\%$) while vertical position $Z$ has not reached nominal seat depth ($Z_{\text{seated}} \approx 38\text{ mm}$).
* **Deterministic Recovery (Micro-Spiral Compliant Search)**:
  1. Stop downward feed immediately.
  2. Ascend $+5\text{ mm}$ along $+Z$.
  3. Execute an Archimedean micro-spiral search pattern in the XY plane:
     $$\Delta X = r(t) \cos(\omega t), \quad \Delta Y = r(t) \sin(\omega t), \quad r(t) \le 2.0\text{ mm}$$
     while maintaining a gentle downward force ($15\%$ torque limit).
  4. The peg self-aligns with the chamfer and drops flush into the cutout.
  5. Open gripper and retreat.

---

### EC-10: Peg Toppled on Side (Knocked Over)
* **Symptom**: 10 cm tall peg falls sideways. Instead of a 2D regular polygon, camera sees an elongated rectangle of dimension $45\text{ mm} \times 100\text{ mm}$.
* **Detection**:
  * Bounding box aspect ratio $> 1.8$ (nominal top face aspect ratio is $\approx 1.0 \pm 0.15$).
  * Depth difference across the contour indicates tilted cylinder/prism orientation.
* **Deterministic Recovery**:
  1. Detect the principal axis of the fallen peg using image moments / minimum area rectangle angle.
  2. Switch grasp mode from "Top Grasp" to "Horizontal Side Grasp":
     * Approach perpendicular to long axis.
     * Grasp at center-of-mass ($50\text{ mm}$ along length).
  3. Lift arm and tilt wrist pitch $q_4$ by $90^\circ$ in mid-air to re-orient the peg vertically before inserting.

---

## 4. Priority P2: Perception & Sensing Variances

### EC-11: Peg-to-Peg Clustering / Mutual Occlusion
* **Symptom**: Two pegs are touching or clustered together on the mat; single contour detected.
* **Mitigation**:
  * Perform color-channel separation first (HSV masks for yellow, blue, orange, green are orthogonal).
  * If two pegs of the same color ever appear, apply Distance Transform + Watershed segmentation to split connected shapes at the neck.

### EC-12: Specular Glare & Ambient Lighting Shifts
* **Symptom**: Overhead laboratory lamps create bright white specular spots on peg top surfaces, washing out HSV color values ($S \to 0, V \to 255$).
* **Mitigation**:
  * Combine HSV segmentation with CIELAB color space ($L^*a^*b^*$), which separates luminance ($L^*$) from chromaticity ($a^*, b^*$).
  * Apply morphological closing (`cv2.MORPH_CLOSE`) to fill internal glare holes before contour extraction.

### EC-13: Depth Dropout on Vertical Boundaries
* **Symptom**: Intel RealSense IR stereo shadow creates noisy/zero depth pixels along peg edges.
* **Mitigation**:
  * Never use boundary pixels for depth calculation.
  * Extract a 5x5 pixel central kernel around the contour centroid $(u_c, v_c)$ and take the statistical median depth, filtering out 0 mm dropouts.

---

## 5. Priority P3: Execution & Tolerance Edge Cases

### EC-14: Friction Stickiness / Peg Lift-Out during Retreat
* **Symptom**: Close fit in socket causes the peg to stick between gripper rubber pads and lift out slightly during vertical retreat.
* **Mitigation**:
  * After opening gripper to full release width ($c = 1700$), add a 150 ms dwell time followed by a slight horizontal wiggle ($\pm 2\text{ mm}$) before upward motion.

### EC-15: Multi-Turn Encoder Wrap (> 4096 counts)
* **Symptom**: Joint 1 (Base Pan) takes a full $360^\circ$ circle to return forward.
* **Mitigation**:
  * Normalize raw target counts: `raw_target = (target_rad * COUNTS_PER_RAD) + (round(current_pos / 4096) * 4096)`.
  * Guarantees shortest path execution ($|\Delta q_1| \le 180^\circ$).

