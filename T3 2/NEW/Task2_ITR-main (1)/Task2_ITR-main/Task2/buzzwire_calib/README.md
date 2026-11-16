# Buzz-Wire robot – calibration toolkit (HEAL arm + 2 cameras)

Collects calibration data while you hand-guide the arm in `./free_drive`, then computes
camera intrinsics, hand-eye (camera ↔ robot) and camera ↔ camera transforms.

## 0. One-time setup
```bash
pip install -r requirements.txt          # needs opencv-contrib (aruco), scipy, pyyaml
python3 scripts/make_board.py            # prints the ChArUco board at true size -> glue on a RIGID flat plate
```
**Measure the printed square with calipers** and put the real values in `config/calib_config.yaml`
(`board.square_length`, `board.marker_length`). Then set, for each camera, its `source` and `mount`:
* `hand`  = camera rides on the arm, `fixed` = camera stands still in the room
* `calibration.board_mount: world` = the board ("matrix") sits still on the table, you place it at several spots
  `ee` = the board is bolted to the gripper (then both cameras must be `fixed`)

## 1. Intrinsics (do once per camera / lens setting, robot not needed)
```bash
python3 scripts/capture_intrinsics.py        # SPACE = save, q = quit. ~25 views per camera, tilted, all image corners
python3 scripts/calibrate_intrinsics.py      # -> results/intrinsics/cam0.json, cam1.json  (aim for RMS < 0.5 px)
```
A camera that only ever sees the board in a handful of fixed placements **cannot** self-calibrate reliably – the
toolkit refuses and tells you to do this step.

## 2. Collect data (your workflow)
Robot terminal (from your README): `ssh cobot@192.168.1.25` → `./cobot run` → `1` → **`./free_drive`**
Second terminal (ROS workspace sourced – or just use the launcher):
```bash
./run_collect.sh            # = source ROS + workspace, then scripts/collect_data.py
```
| key | action |
|---|---|
| **SPACE** | capture: averaged joint states + FK pose + a burst of frames from **both** cameras |
| **n** | next matrix / board placement |
| **u** | undo last capture |
| **f** | FINISH (all data collected) |
| **q** | quit without finishing (resume later) |
| s | status |

Per matrix: place the board → hand-guide the arm so **both cameras see it** → SPACE. Take 3–6 captures per matrix
from clearly different arm poses (**rotate the tool ≥ 45° about at least two axes** – this matters most for accuracy),
press `n`, move the board, repeat. Use ≥ 4 matrices. A live preview window shows detected corners per camera.

What each capture does for accuracy: waits for the arm to settle, grabs 12 frames per camera, **rejects the capture if
the arm moved** (> 4 mrad), requires the board in ≥ 4 frames of every camera, median-averages the joint angles,
averages the FK pose, and stores the sharpest frames + raw corner detections.

## 3. If something gets stuck (free-drive freeze, crash, SSH drop, camera unplug …)
* Everything is saved after every key press (append-only log + atomic capture folders). **Nothing is lost.**
* Restart `./run_collect.sh` → it finds the unfinished session and asks *Resume? [Y/n]* – you continue on the same
  matrix with the same counters. (`--session data/<name>` to pick one, `--new` for a fresh one.)
* Robot state or a camera going stale is shown live (`!! ROBOT STATE STALE`) and captures are refused until it
  returns – the program keeps running; no restart needed. A camera that disconnects is re-opened automatically.
* Want the arm back where it was? Stop free_drive, start `./heal_server` + the velocity controller launch, then
  ```bash
  python3 scripts/goto_pose.py --last            # last captured pose of the session
  python3 scripts/goto_pose.py --matrix 3        # or --capture 7, or --home
  ```
  It uses a smooth quintic profile with closed-loop correction, clamps velocity (0.25 rad/s), pauses on stale joint
  states, and **aborts with zero velocity if the arm does not follow (stuck)** – clear it and re-run the same command.
  Afterwards restart `./free_drive` and `collect_data.py` (resumes).

## 4. Calibrate
```bash
python3 scripts/calibrate_extrinsics.py --session data/session_XXXX      # default: latest session
```
Both cameras are fitted **jointly** (shared board/gripper unknowns), with a linear AX=XB initialisation, robust
non-linear refinement, automatic outlier rejection, and a cross-check of the camera↔camera transform against the
direct simultaneous board measurement. Output in `results/<session>/`:
`extrinsics.json` (all transforms, xyz + quaternion xyzw, 4×4), `per_observation_residuals.csv`, `publish_tf.sh`
(ready `static_transform_publisher` commands). Healthy result: residual < 3 mm / 0.3°, stereo agreement < 1° / 10 mm,
no warnings. Use `--exclude 7 12` to drop bad captures and re-run.

Conventions: `T_a_b` = pose of frame b expressed in frame a. `T_base_cam` (fixed camera) or `T_ee_cam` (hand camera);
camera frame is OpenCV optical (x right, y down, z forward).

## Tests (no hardware)
`python3 scripts/selftest_solver.py` (all mount combinations) and `python3 scripts/selftest.py` (simulated robot +
2 cameras + board through the real collector incl. crash/resume/undo/stale guards and the real calibration).
