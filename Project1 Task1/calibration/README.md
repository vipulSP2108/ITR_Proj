# Camera <-> robot calibration (OpenMANIPULATOR-X + overhead RealSense)

Everything here runs offline. Start scripts from a **real terminal** (macOS only gives the
camera to terminal apps), inside the `Project1 Task1` folder.

## When to redo it
Any time the camera, the robot base, or the table/mat has moved. It takes ~10 minutes:

    ./run_calibration.sh

## Before you start
1. 12 V supply on, U2D2 and RealSense USB plugged in.
2. Mat clear of loose things (flat board and paper ArUco markers can stay). Camera USB cable
   must not hang into the picture.
3. At least one printed ArUco marker (DICT_4X4_50, 38 mm) flat on the mat, visible to the camera.
   More markers spread over the mat = a better check.
4. Hands away while the arm moves (it stays below 250 mm height).

## The steps
| Script | What it does | Moves arm? |
|---|---|---|
| `00_check_arm.py` | pings the 5 motors, prints voltage/errors | no |
| `01_calibrate_camera_robot.py` | puts the closed fingertip at 40 known robot positions (45 mm and 125 mm above the mat - it never touches anything), opens/closes the gripper so the camera can find the fingertip, then fits the camera pose (rotation matrix + translation) to all pairs and to the 38 mm ArUco markers | yes |
| `01_calibrate_camera_robot.py --solve` | recomputes from the saved images only | no |
| `01_calibrate_camera_robot.py --probe` | additionally re-measures the mat height by touching it once (only needed if the mat/table changed thickness) | yes |
| `02_verify_calibration.py --xy 150 60` | fingertip goes 25 mm above that robot point; photo saved with a crosshair where the camera model expects the tip | yes |
| `02_verify_calibration.py --jog` | same for every reachable marker, and you nudge the tip (w/s/a/d + Enter); the average nudge is saved as `xy_correction_mm` | yes |
| `snap_pose.py --no-arm --out x.jpg` | just save a camera picture to `calibration/output/` | no |

## What "good" looks like
`01_calibrate...` prints, for example (values from 2026-11-10):

    poses used                    : 27 of 40
    fingertip reprojection RMS    : 4.6 px = 2.5 mm
    ArUco corner reprojection RMS : 0.92 px
    focal length                  : 888 px
    camera position (robot frame) : [236. 150. 511.] mm
    calibration quality: GOOD

* Quality POOR, or fewer than 20 poses used -> somebody/something moved in the picture during
  the run. Open `calibration/output/calib_tips.jpg`: in every tile the yellow circle (detected
  fingertip) must sit between the two finger tips, next to the green cross (model). Run again.
* If you have two markers on one printed strip, the distance between their centres printed by
  the script should match a ruler (2026-11-10: 92.7 mm computed, 93 mm measured).
* `verify_point_*.jpg`: crosshair between the fingertips = the calibration is usable.

## Results
`config/camera_robot_calib.yaml`, read by `src/vision/camera_model.py`:

    from src.vision.camera_model import CameraModel
    cm = CameraModel()
    x, y = cm.pixel_to_robot(u, v, z)     # z = height of the thing you see (cm.table_z = mat)

Robot frame: origin on the base rotation axis, +X = where the arm points at base count 0,
+Y to the left of that, +Z up, millimetres. Kinematics: `src/planning/omx_kinematics.py`.

## Hardware facts found on 2026-11-10 (do not "fix" these back)
* Joints 12-14 at 2048 counts = L-shaped arm (forearm horizontal), NOT straight up.
  The old `src/planning/kinematics.py` assumed straight up - that was the hardware bug.
* Gripper: open = 3250 counts, fingers touch = ~1385. Runs in current-limited mode 5.
* Wrist servo zero is ~9 deg off and the shoulder ~3 deg (found by the calibration, stored as
  `joint_offsets_rad`, applied by `hw_common.ik_cmd`).
* At long reach the arm sags; never command the fingertip lower than ~15 mm above the mat.
* The camera stream lags ~0.3 s and changes during its first seconds; `hw_common.LiveCamera`
  waits for it to settle. Do not grab frames with one-shot ffmpeg calls.

## After calibrating: pick and place
    python pick_place.py --detect-only      # what does the camera see? -> runs/<time>/scan1_annotated.jpg
    python pick_place.py --plan-only        # also draws the planned carry path -> runs/<time>/plan1.jpg
    python pick_place.py --shape circle     # top-grasp the circular peg, put it in the circular hole
    python pick_place.py                    # every peg it can, best order first
    python pick_place.py --order circle,square   # you fix the order; the other pegs are obstacles meanwhile

Nothing about the table layout is hard-coded. On every scan the camera measures the mat outline
(= the workspace), the holes and the pegs, and converts them with the current calibration.
What you may tune lives in three files:
* `config/task_objects.yaml`   - fixed object sizes (all pegs 100 mm tall; circle d=40, square 40,
  triangle side 33, hexagon side 20) and which colour is which shape
* `config/pick_place.yaml`     - heights above the mat, gripper numbers and every safety buffer
* `config/calibration_setup.yaml` - what the calibration run visits (grid, heights, marker size, table height)

Collision handling (see the header of `pick_place.py` and `src/planning/path2d.py`):
* the moving body is the carried peg PLUS the fingers; it is routed around every other standing
  peg with A*, never closer than `hard_clearance` (30 mm edge to edge) and preferring
  `preferred_clearance` (60 mm). plan1.jpg shows it: green band = swept area, thick red = other
  peg, thin red / orange rings = hard / preferred buffer;
* a pick or release is refused if the fingers would come within `finger_clearance` of a neighbour;
  near neighbours the fingers open less wide (down to `open_extra_min_mm`);
* with several pegs every order is tried and the best fully feasible one is used; if nothing is
  feasible, one peg is first moved to a free parking spot on the mat;
* the grip height on the peg is chosen so the peg hangs as upright as possible at release, and
  the script prints how much closer the board would need to be for a fully upright release;
* after releasing, the gripper rises straight up above the peg before turning away;
* the base only ever turns with the arm tucked up in the `travel_pose` (gripper 270 mm above the
  mat, ~140 mm over the peg tops). Scans are taken from that pose too. (On 2026-11-10 a swing in
  the old, lower L pose clipped a placed square peg and knocked it over.)
* the scene is re-scanned after every peg; a peg lying down stops the run with a message.

Limits to know:
* Two pegs standing side by side at the same distance from the robot need roughly 95 mm between
  their centres, otherwise a finger cannot go between them with the 15 mm buffer; the script says
  how far to move them.
* Top grasp reaches about 250 mm from the base axis; an upright release of a 100 mm peg needs the
  hole within about 200 mm.
* The arm has no wrist-roll joint: the jaws are always parallel to the line from the base. Closing
  them turns a square so two faces lie along that line, and carrying turns it with the base. It
  therefore arrives at its hole turned by the hole's bearing from the base. With the 49.5 mm
  square hole and 3 mm of position play that leaves about +-5 deg: the square hole's edges must
  point at the robot base. Otherwise the script refuses (or use --force and accept a bad fit).
* Tested on hardware (2026-11-10): circle alone; circle with a square as obstacle; square then
  circle chosen automatically - circle seated, square arrived 11 deg rotated, stood on the hole
  rim and later fell over (that is what the tolerance check above now prevents).

Old scripts (`hw_run.py`, `hw_test_step1.py`, `run_hardware_pipeline*.py`, `../calibrate_and_test_arm.py`)
use the wrong kinematics/gripper values - do not use them on the hardware.
