# 02. Robot Controls, ROS 2 Architecture & System Setup

> **Reference Specification**: Hardware network addresses, low-level controller binaries, ROS 2 topics and services, KDL kinematics chains, and simulation interfaces for the **Addverb HEAL 6-DOF Collaborative Robot**.

---

## 1. Network & Robot Controller Credentials

The Addverb HEAL cobot controller operates on an embedded Linux industrial PC communicating over Ethernet:

- **Static IP Address**: `192.168.1.25`
- **Subnet Mask**: `255.255.255.0`
- **SSH Username**: `cobot`
- **SSH Password**: `cobot`
- **Sudo Password**: `cobot`
- **Controller Binary Path**: `/home/cobot/addverb_images/gripper_fix_2026-09-08/`

### Verifying Network Connectivity
Configure your control laptop to a static IP on the same subnet (e.g., `192.168.1.50`, mask `255.255.255.0`), then verify connection:
```bash
ping 192.168.1.25
ssh cobot@192.168.1.25
```

---

## 2. Low-Level Controller Executables & Modes

All low-level hardware executables reside in `/home/cobot/addverb_images/gripper_fix_2026-09-08/`.

```
                        sudo env HOME=/home/cobot ./cobot run
                                       |
                               (Type 1 + ENTER)
                                       |
              +------------------------+------------------------+
              |                                                 |
              v                                                 v
       ./free_drive                                      ./heal_server
   (Gravity Compensation)                         (Joint Velocity Commander)
              |                                                 |
              v                                                 v
   Method 1: Hand Teaching                        Method 2: Teleop & Replay
```

### A. Hardware Runtime Daemon (`./cobot run`)
Must be launched first in every session to initialize the CAN bus, motor drives, and safety relays:
```bash
cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
sudo env HOME=/home/cobot ./cobot run
```
*Prompt*: Type `1` and press `ENTER`.

### B. Free-Drive Mode (`./free_drive`)
Enables real-time gravity compensation on all 6 joints. The arm remains stationary when released and moves with negligible manual force (< 5 N).
```bash
./free_drive
```
- **Used by**: Method 1 Data Collection (`./buzzwire_teach.sh record`).
- **Control Loop**: High-rate internal torque controller balancing arm gravitational moments.
- **Safety**: Do not push violently into joint limits or singular configurations to avoid tripping safety relays.

### C. Joint Velocity Server (`./heal_server`)
Opens socket communications accepting joint velocity reference vectors $\dot{q} = [\dot{q}_1, \dots, \dot{q}_6]^T$ in $\text{rad/s}$.
```bash
./heal_server
```
- **Used by**: Method 2 Keyboard Teleop (`./buzzwire_teach.sh teleop`), Trajectory Replay (`./buzzwire_teach.sh replay`), and Recovery (`goto_pose.py`).
- **Required Companion Launch**: On the ROS 2 workstation, launch the velocity interface:
  ```bash
  ros2 launch addverb_cobot_control velocity_control.launch.py
  ```

---

## 3. ROS 2 Communication Interface & Node Topology

The system communicates over standard ROS 2 Humble interfaces:

```
+-----------------------------------------------------------------------------------------+
|                                    ROS 2 WORKSPACE                                      |
+-----------------------------------------------------------------------------------------+
       |                                      ^                            |
       | /joint_states                        | /velocity_controller/      | /gripper_controller/
       | (sensor_msgs/JointState)             | commands                   | command
       |                                      | (Float64MultiArray)        | (addverb_cobot_msgs)
       v                                      |                            v
+-----------------------+            +---------------------+     +-----------------------+
|  RosRobotState        |            |  PoseController     |     |  GripperHelper        |
|  - Joint cache        |            |  - Resolved-rate DLS|     |  - Async thread       |
|  - KDL FK & Jacobian  |            |  - Carrot tracking  |     |  - Open / Close toggle|
+-----------------------+            +---------------------+     +-----------------------+
```

### Topics & Services Specification

| Name | Type | Direction | Rate | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Sub | 100–200 Hz | Current position $q$, velocity $\dot{q}$, effort for all 6 joints. |
| `/velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | Pub | Up to 100 Hz | Desired joint velocities $\dot{q}_1 \dots \dot{q}_6$ in $\text{rad/s}$. |
| `/robot_description` | `std_msgs/msg/String` | Sub | Latched (QoS Transient Local) | Complete URDF XML used by KDL to build kinematics tree. |
| `/gripper_controller/command` | `addverb_cobot_msgs/srv/Gripper` | Srv Client | On Demand | Dynamixel gripper position (1.0 = open, 0.0 = closed) and force. |

---

## 4. KDL Kinematics Chain & Forward Kinematics

Cartesian control and recording require exact forward kinematics $T_{\text{base}}^{\text{ee}}(q) \in \text{SE}(3)$ and geometric Jacobian $J(q) \in \mathbb{R}^{6 \times 6}$.

### Chain Definition
The kinematics tree is dynamically constructed by `RosRobotState` in `buzzwire_calib/ros_io.py`:
1. Subscribes to `/robot_description` with `QoSDurabilityPolicy.TRANSIENT_LOCAL`.
2. Parses XML using `urdf_parser_py.urdf.URDF` and builds `kdl_parser_py.urdf.treeFromUrdfModel`.
3. Extracts kinematics chain from `robot.base_link` (`base_link`) to `robot.tip_link` (`end-effector`).
4. Instantiates:
   - `kdl.ChainFkSolverPos_recursive`: Computes recursive forward position kinematics.
   - `kdl.ChainJntToJacSolver`: Computes 6-DOF geometric Jacobian in the base frame.

```python
# Geometric Jacobian evaluation in base frame:
ja = kdl.JntArray(6)
for i in range(6): ja[i] = q[i]
jac = kdl.Jacobian(6)
self._jac_solver.JntToJac(ja, jac)
J = np.array([[jac[r, c] for c in range(6)] for r in range(6)])
```

---

## 5. End-Effector Gripper Subsystem

The Addverb HEAL robot utilizes an integrated Dynamixel parallel-jaw gripper.

### Dual-Path Gripper Architecture (`buzzwire_calib/gripper.py`)
1. **Primary Path (Direct ROS 2 Service)**:
   - Uses `rclpy` asynchronous client to call `/gripper_controller/command`.
   - Non-blocking: Executes in a dedicated background daemon thread.
   - Response latency: $\approx 15\text{--}25\text{ ms}$.
2. **Fallback Path (CLI Subprocess)**:
   - If message definitions are missing from the host Python path, invokes `ros2 service call /gripper_controller/command addverb_cobot_msgs/srv/Gripper "{position: 1.0, grasp_force: 100.0}"`.
   - Never stalls the high-rate 100 Hz Cartesian control loop.

---

## 6. Zero-Hardware Offline Simulation (`SimBackend`)

To enable testing and training without risking the physical arm, a complete simulated 6-DOF arm is provided in `buzzwire_calib/simarm.py`.

```bash
# Launch teleoperation in simulation mode
./buzzwire_teach.sh sim

# Run end-to-end synthetic pipeline test
./buzzwire_teach.sh test
```

### Characteristics of `SimBackend`:
- Standard UR5/HEAL Denavit-Hartenberg kinematics parameterization.
- Analytical forward kinematics with numerical central-difference Jacobian ($h = 10^{-6}\text{ rad}$).
- Home position: $q_0 = [0.3, -1.3, 1.7, -0.4, 1.57, 0.0]^T\text{ rad}$.
- Virtual gripper state tracking (`open` $\leftrightarrow$ `closed`).
- Enables full testing of backtrack undo, workspace limits, auto-tangent, and replay speed profiling.
