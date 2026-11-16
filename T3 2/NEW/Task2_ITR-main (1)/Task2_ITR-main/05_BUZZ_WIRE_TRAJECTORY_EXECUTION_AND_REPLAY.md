# 05. Buzz Wire Trajectory Execution & Competition Replay

> **Task 2 Endgame Manual**: Execution guide for autonomous collision-free traversal of the 3D Buzz Wire game, frame calibrations, speed tuning strategies, and winning the Wire-Trace Trial.

---

## 1. System Coordinate Frames & Physical Setup

```
+-----------------------------------------------------------------------------------------+
|                                    Robot Base Frame {B}                                 |
+-----------------------------------------------------------------------------------------+
                                             │
                       Rot(z, jog_frame_yaw_deg)
                                             v
+-----------------------------------------------------------------------------------------+
|                                   Jog World Frame {W}                                   |
|   +X: Along wire length (left post -> right post)                                       |
|   +Y: Depth / normal to vertical wire plane                                             |
|   +Z: Upward vertical                                                                   |
+-----------------------------------------------------------------------------------------+
                                             │
                               T_base_ee (KDL Forward Kinematics)
                                             v
+-----------------------------------------------------------------------------------------+
|                                     Tool Frame {T}                                      |
|   Origin: Robot End-Effector mounting flange                                            |
|   Tool Axis (tool_axis = [0,0,1]): Points along wand ring axis                           |
+-----------------------------------------------------------------------------------------+
```

### Physical Apparatus Dimensions
- **Wire Material**: $4.0\text{ mm}$ outer diameter solid copper wire.
- **Topography**: 3D spatial zig-zag with 4 valleys and 3 peaks between two rigid end posts.
- **Span**: Total X span $\approx 280\text{--}288\text{ mm}$.
- **Height**: Valley minimum $\approx 25\text{ mm}$, peak maximum $\approx 157\text{ mm}$ above baseboard.
- **Steepness**: Ascending/descending wire legs have pitch angles $\approx \pm 77^\circ$.
- **Wand Probe**: Brass ring mounted on non-conductive stick held by the HEAL Dynamixel gripper.
- **Clearance Tube**: Ring inner diameter minus wire diameter provides only $\approx 9.5\text{--}16.0\text{ mm}$ of radial clearance!

---

## 2. Frame Alignment & Parameter Tuning

To ensure intuitive driving and correct auto-tangent pitch tracking, calibrate the jog frame in `config/calib_config.yaml`:

### A. Calibrating `jog_frame_yaw_deg`
1. Launch teleoperation in free air:
   ```bash
   ./buzzwire_teach.sh teleop
   ```
2. Position the wand tip at the wire's **Left Post**, press `p` (print pose), and record $(X_A, Y_A)$.
3. Move the wand tip to the wire's **Right Post**, press `p`, and record $(X_B, Y_B)$.
4. Compute the yaw angle:
   $$\text{yaw\_deg} = \text{atan2}(Y_B - Y_A, X_B - X_A) \times \frac{180^\circ}{\pi}$$
5. Enter this value into `config/calib_config.yaml`:
   ```yaml
   teleop:
     jog_frame_yaw_deg: <computed_angle>
   ```
6. Restart teleop: Pressing `d` now moves directly along the wire length from left to right!

### B. Calibrating Auto-Tangent Pitch (`tool_axis`)
During wire traversal, the ring must tilt as the wire climbs and dips so the wire remains centered within the ring aperture.
- Default tool axis: `tool_axis: [0, 0, 1]` (tool $Z$).
- Wire plane normal: `plane_normal_base: [0, 1, 0]` (normal to wire board).
- In teleop, toggle auto-tangent with key **`g`**.
- As you translate diagonally (`w`+`r`), the tool will automatically rotate about the plane normal to match the tangent of travel!

### C. Setting Safety Workspace Bounding Box
To physically prevent the arm from colliding with table fixtures or walls:
```yaml
teleop:
  workspace_box_m: [-0.65, -0.25, -0.40, 0.05, 0.10, 0.45]
```
If the desired carrot attempts to cross these bounds, the controller clips position instantly and zeroes command velocity on that axis.

---

## 3. End-to-End Execution Protocol

### Step 1: Pre-Flight Verification & Dry Run
Before moving the physical robot, verify the merged trajectory profile:
```bash
./buzzwire_teach.sh replay --dry-run
```
Output:
```text
Trajectory data/session_20261115_054500/merged_trajectory.csv: 342 pts, 894 mm, est. 24.8 s at speed 0.50 (v 4-30 mm/s)
  slowest spot: s= 142.0 mm  v=  4.0 mm/s  at [-410.2 -269.1  235.4] mm (Peak 1)
  slowest spot: s= 285.5 mm  v=  4.0 mm/s  at [-350.1 -269.2  115.2] mm (Valley 2)
  slowest spot: s= 428.0 mm  v=  4.2 mm/s  at [-290.4 -269.0  240.1] mm (Peak 2)
```
- Confirm that the slowest spots correspond to the actual tight U-bends.
- Ensure estimated duration is reasonable.

---

### Step 2: Phase 1 – Straight-Line Approach
1. Position the robot in free air roughly $50\text{--}150\text{ mm}$ away from the trajectory start.
2. Launch replay:
   ```bash
   ./buzzwire_teach.sh replay --speed 0.2
   ```
3. **Pre-Start Screen**:
   ```text
   PRE-START: SPACE = gripper open/close, ENTER = start, q = abort
   gripper=closed  (SPACE toggle / ENTER start / q abort)
   ```
   - Insert the wand into the gripper and press `SPACE` to grip securely.
   - Visually check that the path between the current arm position and the first waypoint is free of cables or the wire post.
   - Press `ENTER`.
4. The arm executes a smooth, slow ($30\text{ mm/s}$) straight-line Cartesian translation directly to Waypoint 0.

---

### Step 3: Phase 2 – Autonomous Wire Traversal & HUD Telemetry
Once within $3\text{ mm}$ of the start waypoint, the controller transitions to `[follow]` mode.

#### Live Operator HUD:
```text
[follow]  452.1/894 mm  v= 24mm/s x1.00  err= 1.2mm sigma=0.18
```
- `452.1/894 mm`: Current arc-length progress along the wire.
- `v= 24mm/s`: Current instantaneous Cartesian path speed.
- `x1.00`: Real-time user speed multiplier (adjust on the fly with `[` and `]`).
- `err= 1.2mm`: Cartesian tracking error between virtual carrot and physical tool.
- `sigma=0.18`: Smallest Jacobian singular value ($\sigma > 0.03$ indicates ample dexterity).

#### Real-Time Keystrokes:
- **`p`**: Pause execution immediately (holds position safely). Press `p` again to resume.
- **`[` / `]`**: Decrement / increment speed multiplier by 20%.
- **`q`**: Immediate emergency stop (publishes zero velocity).

---

## 4. Speed Escalation Strategy (Winning the Trial)

In competition evaluation:
- **100 Points**: Full wire traversal with zero buzzer contacts.
- **10 Bonus Points**: Fastest completion time among all teams.

### Progressive Tuning Ladder:
1. **Pass 1 (`--speed 0.2`)**:
   - Verify zero contact along the entire length.
   - Check printed `Worst tracking error: X.XX mm` at run completion.
   - Target error: $< 1.5\text{ mm}$.
2. **Pass 2 (`--speed 0.4`)**:
   - Traversal speed doubles; duration cuts in half ($\approx 35\text{ s}$).
   - Observe arm motion at Peak 2 and Peak 3.
3. **Pass 3 (`--speed 0.6` to `0.8`)**:
   - Fast dynamic traversal ($\approx 18\text{--}24\text{ s}$).
   - Joint saturation feedback automatically throttles the carrot in high-curvature bends.
4. **Pass 4 (`--speed 1.0`)**:
   - Full competition run.
   - Traversal finishes in $< 18\text{ s}$ with worst-case tracking error $< 2.5\text{ mm}$!
