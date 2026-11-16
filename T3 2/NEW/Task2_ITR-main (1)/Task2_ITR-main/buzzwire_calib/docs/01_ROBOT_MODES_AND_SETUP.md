# 01 · Robot modes, setup, configuration

## The two robot modes (only ONE at a time)
```
 ssh cobot@192.168.1.25  (password cobot)
 cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
 sudo env HOME=/home/cobot ./cobot run     # type 1 + ENTER  -> drives, CAN, safety up
        ├── ./free_drive    gravity-compensated hand guiding         -> Method 1 (record)
        └── ./heal_server   accepts joint-velocity commands (ROS 2)  -> Method 2 (teleop) and replay
```
Velocity mode also needs the controller launch from the lab README (`ros2 launch addverb_cobot_control velocity_control.launch.py` – check the exact name in your README).
**Switching:** stop one (Ctrl-C), start the other. Recorded files stay valid – segments from both methods merge together.

| tool | robot mode | ROS needed |
|---|---|---|
| `record` | free_drive | yes |
| `teleop` | heal_server + velocity controller | yes |
| `sim`, `inspect`, `merge`, `replay --dry-run`, `test` | none | no |
| `replay` | heal_server + velocity controller | yes |

Topics: `/joint_states` (JointState, 6 joints, ~100-200 Hz), `/velocity_controller/commands` (Float64MultiArray, rad/s),
`/robot_description` (URDF, used for KDL FK), service `/gripper_controller/command`.

## Laptop setup
```bash
pip install -r requirements.txt
source /opt/ros/humble/setup.bash && source ~/<your_ws>/install/setup.bash     # or: WS=~/<your_ws> ./buzzwire_teach.sh teleop
# FK needs these python packages (come with the HEAL workspace on the lab PC): PyKDL, urdf_parser_py, kdl_parser_py
```
`buzzwire_teach.sh` sources ROS for you (set `WS` if your workspace is not `~/Debojit_WS/addverb_ws`).

## Config (`config/calib_config.yaml`) – what to look at first
| key | meaning | default |
|---|---|---|
| `robot.joint_names/base_link/tip_link` | must match the URDF | joint1..6, base_link, end-effector |
| `gripper.open_position/close_position/force` | 1.0 / 0.0 / 100 (from `heal_dynamixel_gripper.py`) | |
| `gripper.workspace_path` | only for the CLI fallback (sources `install/setup.bash`) | "" (uses `$WS`) |
| `teleop.lin_speeds_mmps` / `ang_speeds_degps` | speed levels 1–5 | 2,5,10,20,40 mm/s |
| `teleop.default_level` | start level | 2 |
| `teleop.workspace_box_m` | `[xmin,xmax,ymin,ymax,zmin,zmax]` the target may not leave. **Set it around the wire!** | null |
| `teleop.joint_limits_rad` | `[[lo,hi]×6]` if known | null |
| `teleop.max_joint_vel` | rad/s cap (uniform scaling) | 0.35 |
| `teleop.jog_frame_yaw_deg` | rotate WASD so `a/d` follow the wire (`docs/07`) | 0 |
| `teleop.tool_axis`, `plane_normal_base`, `auto_tangent` | auto-pitch settings | [0,0,1], [0,1,0], false |
| `teleop.key_backend` | `terminal` or `pynput` | terminal |
| `replay.*` | v_max 0.06 m/s, a_max, a_lat, w_max, max_joint_vel 0.5 | |
| `wire.expected_length_m` | only for the coverage bar in `inspect` | 1.0 |
(`capture`, `cameras`, `calibration` sections are legacy – ignore.)

## First-hour commissioning (do in this order)
1. `./buzzwire_teach.sh test` and `./buzzwire_teach.sh sim` (learn the keys).
2. Robot in heal_server: `ros2 topic echo /joint_states --once` (6 joints?) and `ros2 topic hz /joint_states`.
3. Gripper: `ros2 service call /gripper_controller/command addverb_cobot_msgs/srv/Gripper "{position: 1.0, grasp_force: 100.0}"`. Which value opens?
4. Set `teleop.workspace_box_m` (jog the tool by hand to the box corners with `p` and read the xyz).
5. `teleop --level 1` in free space: check each key moves the right way (a/d, w/s, r/f, i/k...). Fix `jog_frame_yaw_deg`.
6. Only then go near the wire.
