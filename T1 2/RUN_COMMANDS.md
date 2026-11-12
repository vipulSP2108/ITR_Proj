# Peg sorting robot - commands to run it yourself

Open a **real terminal** (Terminal.app or the IDE terminal - macOS only gives the camera to those) and go to the project:

```bash
cd ~/Desktop/ITR_Proj/"T1 2"
```

## 0. Before every session (30 seconds)

| Check | Command | Good result |
|---|---|---|
| Arm powered and talking | `python calibration/00_check_arm.py` | five lines with `volt=12.0V hw_err=0x00`, then `ARM OK` |
| Colour camera present | `ffmpeg -f avfoundation -list_devices true -i ""` | a line ending in `435i RGB` |
| What does the robot see | `python pick_place.py --detect-only` | `runs/<time>/scan1_annotated.jpg` shows every peg and hole labelled |
| Where should I stand the pegs (empty mat) | `python pick_place.py --suggest-layout` | `suggested_layout.jpg`: an outline for each peg drawn on the live picture |

If the camera list only shows `435i Depth`: unplug the RealSense and plug it into a USB 3 port with its USB 3 cable.

## 1. Pick and place

| What you want | Command |
|---|---|
| **Real run** - all pegs, the robot decides the order | `python pick_place.py` |
| **Your own priority order** (other pegs are obstacles meanwhile) | `python pick_place.py --order circle,square,hexagon,triangle` |
| One peg only | `python pick_place.py --shape circle` (or `square`, `hexagon`, `triangle`) |
| Look and plan, move nothing | `python pick_place.py --plan-only` then open `runs/<time>/plan1.jpg` |
| Try a peg even if its rotation does not match the hole | add `--force` |

Every run writes a folder `runs/<date_time>/` with the scans, the plan pictures (green band = area swept by
the carried peg and fingers, red rings = other pegs and their buffers), a photo at each step and `run.mp4`.

At the end it prints, for example:

    RESULT: verified in their holes: ['circle', 'square'] | not in hole after the move: none | failed grasps: none

A peg is only listed as verified after the next camera scan sees it standing upright in its hole.
A peg reported as `(leaning)` reached its hole but is not seated - push it in by hand.

### Rules for placing things on the table
* Everything stands on the green mat, fully inside the camera picture (check with `--detect-only`).
* Pegs standing side by side need about 9-10 cm between their centres, or the fingers cannot go between them
  (the script tells you how far to move them).
* Peg colours: green = circle, blue = square, orange = hexagon, yellow = triangle.
* Holes must be within about 25 cm of the robot base.
* Keep hands and people out of the camera view while it runs.

## 2. Is the calibration still right? (1 minute, pegs can stay)

```bash
python pick_place.py --check-calibration
```

The arm shows its fingertip to the camera at a few free spots and prints, for example:

    calibration error now: mean 1.6 mm, worst 2.6 mm over 4 spots (fine)

* mean up to about 4 mm: fine, carry on.
* mean above about 5 mm: the camera, board or robot was moved - recalibrate (section 3).
  (2026-11-11: 7 mm gave a missed grasp and a leaning peg; a bumped keyboard moved the camera by 13 mm.)
* Do this check at the start of every session and after anything near the camera was touched.

## 3. Recalibrate (10 minutes) - when the camera, robot or table was moved

1. Take **all pegs off the mat** (the arm sweeps 45 mm above it). The flat board may stay.
2. Put at least one printed ArUco marker (38 mm, any of ids 0-4) flat on the mat in view of the camera.
3. Run:

```bash
./run_calibration.sh
```

The script prints `calibration quality: GOOD` or `POOR (reason)`, then runs the check of section 2
(good = mean error below about 3 mm). It also warns if the mat or board is cut off by the picture edge
and if the camera moved while calibrating. Then run `python pick_place.py --suggest-layout`.
Details and troubleshooting: `calibration/README.md`.

## 4. Fine-tuning without recalibrating

All numbers are in `config/pick_place.yaml` (nothing is hard-coded in the scripts):

| If you see this | Change this |
|---|---|
| A peg lands a little short of / beyond its hole | `placement_fine_tune_mm: outward: <shape>` (+ = further from the robot, mm) |
| Peg not let go at the hole | `gripper: open_extra_mm` (how wide the fingers open) |
| You want more / less distance to other pegs | `safety_buffers_mm: hard_clearance`, `preferred_clearance`, `finger_clearance` |
| Arm swings too low / too high over the pegs | `travel_pose: grip_centre_above_mat` |

Fixed object sizes (peg height 100 mm etc.) are in `config/task_objects.yaml`.

## 5. If something goes wrong

* **Stop the arm:** press `Ctrl-C` in the terminal. The motors keep holding their position.
* `STOP: a peg seems to be lying down` - stand it up or remove it and run again.
* `cannot pick: ... fingers would pass N mm from ...` - move those two pegs further apart.
* `it would arrive N deg rotated in its hole` - the arm cannot turn its wrist; turn the board so that hole's
  edges point at the robot base, or add `--force`.
* `Camera ... produced no frames` / `colour camera is not available` - replug the RealSense (USB 3).
* Do **not** use the old scripts (`hw_run.py`, `run_hardware_pipeline*.py`, `../calibrate_and_test_arm.py`):
  they use wrong arm geometry.

## 6. Keep your work safe

Commit after every good session (`git add -A && git commit -m "..."`). On 2026-11-11 an accidental
`git reset --hard` / "Discard all changes" wiped a whole evening of uncommitted changes.
