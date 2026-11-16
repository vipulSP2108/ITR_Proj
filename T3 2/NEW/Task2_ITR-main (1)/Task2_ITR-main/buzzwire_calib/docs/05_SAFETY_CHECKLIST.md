# 05 · Safety checklist (every robot session)

**Before powering/moving**
- [ ] Someone's hand is next to the E-STOP. Workspace clear of cables, posts, people.
- [ ] Wire fixture clamped (it must not slide when touched).
- [ ] Stick/ring seated; gripper closed (SPACE) – check the stick does not slip when the arm moves.
- [ ] `./buzzwire_teach.sh test` passes on this machine.
- [ ] `teleop.workspace_box_m` set around the wire; `teleop.joint_limits_rad` set if known.
- [ ] Correct robot mode for the tool (see `docs/01`).

**First run of anything new:** speed level 1 / `--speed 0.1`, in free air, away from the wire.

**During**
- Keep the terminal focused (default keys only act there). Do not type elsewhere.
- Release all keys = the arm decelerates to zero. `ESC` = stop + re-sync. `q` = quit (zero velocity sent). E-STOP for anything else.
- HUD `lag=` growing / "NOT FOLLOWING" → stop, find the cause (collision, limit) before continuing.
- Replay: never leave the robot running unattended; raise speed in steps.

**Known gaps (not protected by code):** the wire/fixture is not modelled (no collision avoidance); real joint limits unknown unless you enter them; the replay approach is a straight line.
