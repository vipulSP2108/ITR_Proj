# 08 · Legacy: camera / hand-eye calibration (not used for the Buzz Wire task)

The assignment forbids cameras, so these are **out of scope** and kept only for reference:
`scripts/collect_data.py`, `capture_intrinsics.py`, `calibrate_intrinsics.py`, `calibrate_extrinsics.py`, `goto_pose.py`, `make_board.py`,
`selftest.py`, `selftest_solver.py`, `run_collect.sh`, modules `calibration.py handeye.py intrinsics.py board.py cameras.py collector.py`.
They implement ChArUco-based intrinsics, joint two-camera hand-eye solving (board on gripper or static, cameras fixed or on the arm), crash-safe resumable capture sessions and a recovery mover (`goto_pose.py --last/--home`).
Requires `opencv-contrib-python`. `selftest_solver.py` is still run by `./buzzwire_teach.sh test`.
`goto_pose.py` (velocity mode) is still handy to drive the arm back to a saved joint configuration, but only works with *camera-session* captures.
