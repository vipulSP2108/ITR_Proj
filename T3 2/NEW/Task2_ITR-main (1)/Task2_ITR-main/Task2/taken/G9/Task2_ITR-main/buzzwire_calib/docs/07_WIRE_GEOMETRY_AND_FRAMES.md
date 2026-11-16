# 07 · Wire geometry and frames

## Wire facts (UNVERIFIED – from photos and an LLM-generated keypoint script; measure!)
* Copper wire ≈ 4 mm diameter, bent as a vertical zig-zag with 4 valleys / 3 peaks between two end posts; U-bend inner gap ≈ 23–36 mm; X span ≈ 280–288 mm; heights roughly 25–157 mm above the base board; legs are steep (the script mentions ±77° tool pitch).
* True centreline length (3D) is ≈ 1 m (the legs are vertical) → `wire.expected_length_m: 1.0` (approximate, only used for the coverage bar).
* The ring/loop must not touch the wire; clearance is only a few mm → teach slowly and replay conservatively first.
Measure the real wire and update this file.

## Frames
* **Base frame**: robot `base_link`. All recorded/replayed poses are here.
* **Jog (world) frame** for teleop: base rotated about Z by `teleop.jog_frame_yaw_deg`. Goal: +X = along the wire (left→right), +Y = depth, +Z = up.
* **Tool frame** (TAB): the end-effector axes.

## Setting `jog_frame_yaw_deg`
1. `./buzzwire_teach.sh teleop` (yaw 0), level 1, in free air, **press `p`** → note `pos` A.
2. Move the tool (any method) from the wire's left post toward the right post: the direction of the wire's long axis. Press `p` → pos B.
3. `yaw_deg = atan2(B_y − A_y, B_x − A_x)` in degrees (python: `math.degrees(math.atan2(dy, dx))`). Put it in the config.
4. Restart teleop; `d` should now run along the wire. (If it runs the wrong way along the wire, add 180.)

## Tool axis for auto-tangent
`teleop.tool_axis` = the tool-frame axis that should point along the direction of travel in the wire plane (the ring's axis, if the ring slides along the wire). Check with `g` on, free air, slow: the tool should tilt to follow a diagonal `a`+`r` motion. If it tilts the wrong way/90° off, change `tool_axis` (e.g. `[1,0,0]`, `[0,0,1]`) or `plane_normal_base` (the wire-plane normal, usually the jog Y axis `[0,1,0]`).
