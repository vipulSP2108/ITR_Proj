# 06 · Troubleshooting & edge cases

| symptom | likely cause | fix |
|---|---|---|
| "rclpy not importable" | ROS not sourced | `source /opt/ros/humble/setup.bash` + workspace, or `WS=... ./buzzwire_teach.sh ...` |
| "No /joint_states or no KDL chain after 20 s" | wrong robot mode / `/robot_description` missing / PyKDL packages missing / wrong `tip_link` | check mode, `ros2 topic list`, config `robot.*` |
| `!! joint states stale` | free_drive/heal_server stopped or crashed | restart it; tools wait and resume (recorded data is safe) |
| teleop: arm doesn't move | not in velocity mode; velocity controller not launched | `ros2 topic echo /velocity_controller/commands`; start the launch file |
| teleop: `a` moves the wrong way | base frame not aligned with wire | set `teleop.jog_frame_yaw_deg` (docs/07) |
| tap moves too little/too much | terminal key-hold model | `key_hold_s`, `xset r rate 200 40`, use a lower speed level |
| "arm NOT FOLLOWING" | collision, joint limit, brake/fault | back off with `z`, clear fault, `c` to re-sync |
| stick falls | gripper not closed / force too low | SPACE to close; raise `gripper.force`; check which value closes |
| gripper message FAILED | service name/type/package not found; workspace not sourced | `ros2 service list` (look for a gripper service); set `gripper.*` / `WS` |
| arm freezes mid-replay | e-stop, fault, comms | tool aborts (zero velocity); fix, then `replay --resume` |
| merge: LARGE GAP warning | a part of the wire not recorded | record that part, merge again |
| merge: path doubles back | segment recorded backwards or two passes over the same place | `--reverse-seg N` / `--exclude N` |
| replay: "Too far" | arm far from the path start | teleop the arm near the start, or `--force` (check the straight path!) |
| replay slow in some spots | speed profile (bends/rotation) – by design | `replay --dry-run` shows the slowest spots |
| replay error > 4 mm | arm at its limit | lower `--speed`, check `max_joint_vel` |

**Crash/kill safety:** segment rows are flushed per sample; a killed run loses ≤ 1 sample. A torn last CSV line is ignored when loading. The merge never touches raw segments (`./buzzwire_teach.sh clean` removes only merged output).
**Resuming a session:** `record`/`teleop` ask "Resume latest session?" – answer Y to keep adding segments (`--new` for a fresh one, `--session DIR` for a specific one).
**Which Python file does what?** see `docs/00`.
