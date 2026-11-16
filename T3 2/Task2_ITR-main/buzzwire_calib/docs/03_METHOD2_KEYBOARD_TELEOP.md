# 03 · Method 2 – keyboard teleop (all 6 DOF)

Robot: `./heal_server` + velocity controller. Laptop: `./buzzwire_teach.sh teleop` (or `sim` to practise).
Keep the **terminal window focused**; keys act only there (default backend).

## Keys
| group | keys |
|---|---|
| Move (hold) | `a/d` −X/+X (along the wire) · `w/s` +Y/−Y (depth) · `r/f` +Z/−Z (up/down) |
| Rotate (hold) | `i/k` pitch +/− (about Y) · `j/l` yaw +/− (about Z) · `u/o` roll +/− (about X) |
| Speed | `1 2 3 4 5` → 2/5/10/20/40 mm/s and 3/6/12/20/30 °/s (edit in config) |
| Gripper | `SPACE` open ↔ close |
| Record | `v` start/stop a segment · `x` mark contact · `z` hold = **backtrack** |
| Modes | `TAB` jog frame world ↔ tool · `g` auto-tangent on/off · `c` re-sync · `ESC` stop+re-sync |
| Info | `p` print pose · `h` help · `q` quit |
Keys combine (w+d = diagonal at the same speed; translation + rotation together is fine).

## What the code does for you (so you only press keys)
* **Orientation is held** while you translate – no drift of the ring. Rotation keys rotate about the TCP, so the tip stays put.
* **Smooth starts/stops**: acceleration limited (0.10 m/s², 0.6 rad/s²) – a key tap never jerks the arm.
* **Leash**: the target may lead the real arm by ≤ 12 mm / 0.12 rad. If the arm stops following (collision, limit, fault) for 2.5 s → motion stops and you are told. HUD `lag=` shows the current lead.
* **Singularity guard**: near a singular pose the speed drops and damping rises instead of the arm whipping around.
* **Joint-speed cap** (0.35 rad/s): the whole motion slows uniformly (direction preserved).
* **Workspace box** (config): the target can't leave it.
* **Stale joint states**: command → 0, then re-sync automatically when data returns.
* **Backtrack `z`**: retraces the path you came along (1 mm breadcrumbs) at the current speed level. **While recording it also deletes the rewound samples** – so "I hit the wire → hold z → carry on" leaves a clean path.
* **Auto-tangent `g`** (optional, test it in free air first): swings the tool axis (`teleop.tool_axis`) in the wire plane (normal = `teleop.plane_normal_base`) to follow your direction of travel – that's the pitch the steep legs need, so you can steer with `a/d/r/f` only.
* **Tool frame (`TAB`)**: moves/rotates relative to the tool (like a first-person camera). World frame is easier near the wire.

## Frames
World frame = robot base rotated by `jog_frame_yaw_deg` about Z (see `docs/07` to set it). In that frame +X should run along the wire (left → right), +Y is depth, +Z up.

## Recommended technique near the wire
1. Level 1–2 near the wire, 3 on straight bits.
2. Use `r/f` and `a/d` together to follow the legs; with auto-tangent on, pitch follows. Without it, add `i/k` as needed.
3. Contact or doubt → release keys (robot decelerates smoothly) → hold `z`.
4. Gripper: `SPACE` once to close on the stick; keep your hand off keys while doing it.
5. `v` to start recording *before* entering the wire, `v` again after leaving it (each start/stop = a segment).

## Keyboard backends
* `terminal` (default): safe, works over SSH. A terminal cannot see key release → a key is "held" for 0.12 s after the last auto-repeat character, so a single tap moves `speed × 0.12 s` (2 mm/s → 0.24 mm; 40 mm/s → 4.8 mm) and holding moves continuously after the OS repeat delay (~0.25–0.5 s). Shorten it with `xset r rate 200 40`.
* `pynput` (`--keys pynput`): true key-up/down like a game, but **global**: it reacts even if another window has focus – use only on a dedicated machine.

## Tuning
| want | change |
|---|---|
| gentler starts | lower `teleop.lin_accel`, `ang_accel` |
| slower fine mode | edit `lin_speeds_mmps[0..1]` |
| arm lags / `lag=` large | lower speed level, check `max_joint_vel`, lower `kp_*` if oscillating |
| tap moves too far | lower `key_hold_s` (min ~0.08; auto-repeat is ~30 Hz) |
