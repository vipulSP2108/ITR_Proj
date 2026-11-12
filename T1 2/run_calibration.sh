#!/bin/bash
# Full camera <-> robot calibration, start to finish.  Run from a real terminal:
#     cd "T1 2" && ./run_calibration.sh
# Re-run whenever the camera, the robot base or the table has been moved.
# BEFORE: take all pegs off the mat (the arm sweeps 45 mm above it), leave the ArUco marker on it.
cd "$(dirname "$0")"
echo "=== 0/2  Arm health check ==="
python calibration/00_check_arm.py || { echo "Arm not OK - check the 12 V supply and cables."; exit 1; }
echo
echo "=== 1/2  Calibration (arm moves for ~9 minutes - keep hands AND people out of the camera view) ==="
python calibration/01_calibrate_camera_robot.py
if [ $? -ne 0 ]; then
  echo
  echo "Calibration quality is POOR (reason printed above). Make sure nothing moves, then run ./run_calibration.sh again."
  exit 1
fi
echo
echo "=== 2/2  Independent check: the camera measures the fingertip at new spots ==="
python pick_place.py --check-calibration
echo
echo "Done. Good = 'calibration error now: mean' below about 3 mm."
echo "Pictures: calibration/output/calib_overlay.jpg (robot axes + 50 mm grid on the table),"
echo "          runs/<newest>/check_sheet.jpg (yellow cross = expected fingertip, red circle = measured)."
echo "Next: python pick_place.py --suggest-layout   (shows where to stand the pegs)"
