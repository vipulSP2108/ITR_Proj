# Re-teach from scratch → LR / RL (raw + smooth) → video

Every new teach goes into its **own new folder** under `data/`. The old sessions are never touched.
The folder you create becomes the **active session**, stored in `data/ACTIVE_SESSION`.
`buzzwire_teach.sh` and `buzzwire_run.sh` both use it automatically.

Run everything from this folder:

```bash
cd "/home/iitgn-robotics/ITR_2026/G9/NEW/Task2_ITR-main (1)/Task2_ITR-main/Task2/taken/G9/Task2_ITR-main/buzzwire_calib"
```

---

## 0. Robot check (do this first, it is why the last run froze)

Robot side: `./heal_server` running, e-stop released, no fault on the pendant.

```bash
ros2 control list_controllers
```

`velocity_controller` must be **active**. If it isn't, activate it and deactivate whatever is active instead:

```bash
ros2 control switch_controllers --activate velocity_controller --deactivate joint_trajectory_controller
```

A quick motion test: if this doesn't move the arm, nothing else will.

```bash
./buzzwire_run.sh goto home
```

---

## 1. Make a fresh session folder

```bash
./buzzwire_teach.sh new
```

This creates `data/session_YYYYMMDD_HHMMSS/segments/` and makes it active.
To pick a name yourself, run `./buzzwire_teach.sh new session_final_video`.

## 2. Teach the wire (record the data)

**Keyboard teleop** (robot in `heal_server` + velocity control):

```bash
./buzzwire_teach.sh teleop
```

**Or hand-guided free drive** (robot in `./free_drive`):

```bash
./buzzwire_teach.sh record
```

- Start the segment with the loop at the **start post L**. End it at the **goal post R**.
- One clean continuous segment from L to R is best. If you have to stop, run the command again: it adds `seg_0001`, `seg_0002` and so on to the same folder.
- If you botch a start, simply start again. You'll leave the bad segment out in step 4.

## 3. Check what you recorded

```bash
./buzzwire_teach.sh status
```

This lists every segment with its start → end point (mm) and length. Note the IDs of the **good** segments.
False starts are short, or begin and end in the wrong place.

## 4. Build LR, RL, LR_smooth, RL_smooth

If all segments are good:

```bash
./buzzwire_teach.sh build
```

If some are false starts, keep only the good ones in L→R order. For example, the old session used segments 2 and 3:

```bash
./buzzwire_teach.sh build --only 2 3
```

`build` = merge + calibrate. It writes these four files into the session folder:

| File | Direction | Type |
|---|---|---|
| `calibrated_wire_LR.csv` | L → R | raw (demonstrated contours) |
| `calibrated_wire_RL.csv` | R → L | raw |
| `calibrated_wire_LR_smooth.csv` | L → R | smooth |
| `calibrated_wire_RL_smooth.csv` | R → L | smooth |

At the end it prints `LR path: L = [...]  R = [...]`. **L must be the start post and R the goal post.**
If they aren't, rebuild with the correct `--only` list.
(If all segments are merged blindly, the path gets scrambled. On the old data that gave 1918 mm instead of 1342 mm.)

## 5. Check before moving the robot (no motion)

```bash
./buzzwire_run.sh dry-run lr --speed 0.35
```

```bash
./buzzwire_run.sh dry-run lr -smooth --speed 0.35
```

## 6. Record the four videos

`goto L` / `goto R` now take the post poses from the **new** session (first and last point of the LR path), not the old hard-coded values.

**Video 1: L → R, raw**

```bash
./buzzwire_run.sh goto L
```

```bash
./buzzwire_run.sh lr --speed 0.35
```

**Video 2: R → L, raw** (the arm is already at R after video 1)

```bash
./buzzwire_run.sh rl --speed 0.35
```

**Video 3: L → R, smooth** (the arm is back at L after video 2)

```bash
./buzzwire_run.sh lr -smooth --speed 0.35
```

**Video 4: R → L, smooth**

```bash
./buzzwire_run.sh rl -smooth --speed 0.35
```

At the `PRE-START` prompt: SPACE = gripper, **ENTER = start** (start the camera, then press ENTER), q = abort.
During a run: `p` pause, `[` / `]` slower / faster, `q` stop. To continue after a stop, run `python3 scripts/replay_trajectory.py --resume --speed 0.35`.

Speed: 0.35 is safe for a first take. Once a take is clean, re-shoot at `--speed 0.5` or `0.65` for a shorter video.

---

## Switching sessions

```bash
./buzzwire_teach.sh use data/session_20261115_122231
```

That makes the old session active again. For a single run you can also add `--session data/<folder>` to any `buzzwire_run.sh` command.
