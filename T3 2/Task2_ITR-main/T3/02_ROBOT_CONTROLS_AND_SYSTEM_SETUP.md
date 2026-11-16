# 02. Robot Controls, ROS 2 Architecture & Hardware Setup

> **Reference Specification**: Hardware network addresses, daemon binaries, ROS 2 topics, kinematics chains, and camera sources for the Addverb HEAL 6-DOF Collaborative Robot.

---

## 1. Network & Robot Controller Credentials

The Addverb HEAL robot controller runs on a dedicated internal Linux host accessible over Ethernet:

- **IP Address**: `192.168.1.25`
- **SSH Username**: `cobot`
- **SSH Password**: `cobot`
- **Sudo Password**: `cobot`
- **Host Workspace Path**: `/home/cobot/addverb_images/gripper_fix_2026-09-08/`

To test connectivity from your control laptop:
```bash
ping 192.168.1.25
ssh cobot@192.168.1.25
```

---

## 2. Low-Level Robot Controller Binaries

All low-level controller executables reside in `/home/cobot/addverb_images/gripper_fix_2026-09-08/`:

### A. The Hardware Runtime Daemon (`./cobot run`)
Must be started before any other robot operation:
```bash
cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
sudo env HOME=/home/cobot ./cobot run
```
When prompted for execution mode, type **`1`** and press **`ENTER`**. This initializes the motor drives, CAN bus communication, safety stops, and joint encoders.

### B. Controller Modes (Choose One at a Time)

```
                       +-----------------------+
                       | sudo ./cobot run (1)  |
                       +-----------------------+
                                   |
         +-------------------------+-------------------------+
         |                                                   |
         v                                                   v
+------------------+                               +------------------+
|  ./free_drive    |                               |  ./heal_server   |
| (Hand-guiding &  |                               |  (ROS 2 Velocity |
|  Data Capture)   |                               |   Trajectory)    |
+------------------+                               +------------------+
         |                                                   |
 (Calibration Mode)                                  (Navigation Mode)
```

1. **`./free_drive` (Manual Gravity Compensation)**:
   - Enables compliant teaching. Motors actively cancel the arm's gravitational load so the operator can smoothly position and orient the end-effector by hand.
   - Used for **Data Collection (`run_collect.sh`)** and **manual teaching**.
2. **`./heal_server` (Low-Level Server for ROS 2 Control)**:
   - Sets the robot drives to receive external real-time joint velocity commands.
   - Required whenever running ROS 2 nodes, trajectory replay, `heal_going_home.py`, or `goto_pose.py`.
3. **`./base_rigid` (Rigid Stiff Mode)**:
   - Locks joints rigidly in their current state; can be used for zeroing/homing verification.

---

## 3. ROS 2 Software Stack & Velocity Control

On the ROS 2 workstation (or on the cobot PC if running ROS locally):

### Sourcing Environment
```bash
source /opt/ros/humble/setup.bash
cd ~/Debojit_WS/addverb_ws/
source install/setup.bash
```

### Launching the Velocity Controller
Only run this command when `./heal_server` is already active in the robot terminal:
```bash
ros2 launch addverb_cobot_control velocity_control.launch.py
```

### Core ROS 2 Topics & QoS Contract

| Topic Name | Message Type | QoS Policy | Direction | Description |
| :--- | :--- | :--- | :---: | :--- |
| `/joint_states` | `sensor_msgs/msg/JointState` | `SensorData` (Best Effort, volatile) | Robot -> Laptop | Published at 100–200 Hz with 6 joint positions and velocities. |
| `/robot_description` | `std_msgs/msg/String` | `Reliable`, **`TRANSIENT_LOCAL`** (latched) | Robot -> Laptop | Full URDF XML model. Required by KDL parser to construct FK chain. |
| `/velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | `Reliable` (depth 1) | Laptop -> Robot | 6 joint velocity commands in radians/second (`[qdot1, ..., qdot6]`). |
| `/end_effector_pose` | `geometry_msgs/msg/PoseStamped` | `Reliable` (depth 10) | Node -> Laptop | Computed forward kinematics pose of `end-effector` relative to `base_link`. |

> [!WARNING]
> **/robot_description Latched Topic**:
> The HEAL `controller_manager` publishes URDF as `TRANSIENT_LOCAL`. Standard ROS subscribers with default `VOLATILE` QoS will never receive it! Both `ros_io.py` and `heal_end_effector_pub_node.py` explicitly subscribe with `QoSDurabilityPolicy.TRANSIENT_LOCAL` to guarantee immediate latch reception.

---

## 4. Kinematic Chain & Link Names

The HEAL URDF chain is configured as follows:

- **Base Link**: `base_link`
- **Tip Link**: `end-effector` *(Note: spelled with a hyphen `-`, not an underscore `_`)*
- **Joint Order**: `['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']`
- **Kinematic Solver**: PyKDL recursive forward position solver (`kdl.ChainFkSolverPos_recursive`).

---

## 5. Camera Configuration & Mounting Options

Configured in `config/calib_config.yaml`:

```yaml
cameras:
  cam0:
    source: "0"          # Eye-in-hand (mount: hand)
    mount: hand
    width: 1280
    height: 720
    fps: 30
    fourcc: MJPG
  cam1:
    source: "2"          # Eye-to-hand fixed camera (mount: fixed)
    mount: fixed
    width: 1280
    height: 720
    fps: 30
    fourcc: MJPG
```

### Supported Camera Input Formats
1. **USB / V4L2 Device Index**: `"0"`, `"2"`, or `"/dev/video0"`.
2. **ROS 2 Raw Image Topic**: `"ros:/camera/color/image_raw"`.
3. **ROS 2 Compressed Image Topic**: `"rosc:/camera/color/image_raw/compressed"`.

### Mount Semantics
- **`mount: hand` (Eye-in-Hand)**: Camera is rigidly attached to the robot's end-effector. The solver estimates $T^{\text{ee}}_{\text{cam0}}$.
- **`mount: fixed` (Eye-to-Hand)**: Camera is statically mounted on an aluminum extrusion, tripod, or table facing the workspace. The solver estimates $T^{\text{base}}_{\text{cam1}}$.

