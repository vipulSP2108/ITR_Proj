#!/bin/bash
# Full camera <-> robot calibration, start to finish.  Run from a real terminal:
#     cd "T1 2" && ./run_calibration.sh
# Re-run whenever the camera, the robot base or the table has been moved.
set -e
cd "$(dirname "$0")"
echo "=== 0/3  Arm health check ==="
python calibration/00_check_arm.py
echo
echo "=== 1/3  Calibration (arm moves for ~7 minutes - keep hands AND people out of the camera view) ==="
python calibration/01_calibrate_camera_robot.py
echo
echo "=== 2/3  Verification: fingertip goes to 3 new points (and any reachable ArUco marker) ==="
python calibration/02_verify_calibration.py --xy 150 60 --xy 200 -70 --xy 240 20
echo
echo "Done. Open calibration/output/verify_point_*.jpg : the yellow crosshair must sit between the two fingertips."
echo "Also see calibration/output/calib_overlay.jpg (robot axes + 50 mm grid drawn on the table)."
echo "Optional fine-tune by hand:  python calibration/02_verify_calibration.py --jog"
