# Demo run summary

* run folder: `demo_20261108_030002`
* command line: `run_pipeline.py --until workspace`
* wall-clock time: 13 s

## 1. Home pose
* encoders (rad): [-0.0, 0.0, 0.0, -0.0]  -> L pose: **True**
* TCP from arm dimensions: [0.274, -0.0, 0.205] m (model vs simulator: 0.000 mm)

## 2a. Camera <-> robot calibration
* marker detected in 16/16 poses, used 16
* fit residual 0.10 mm, reprojection 0.35 px
* (simulation-only) error vs the true camera pose: 0.07 mm / 0.022 deg

## 2b. Workspace analysis
* shapes found on board (top->bottom): ['circle', 'hexagon', 'square', 'triangle']
* circle peg at (16.0, 7.1) cm, 17.5 cm from the base
* circle socket at (29.5, -11.5) cm, 31.7 cm from the base
* vertical top-grasp reach at grasp height: 21.8 cm
* **chosen constant tool tilt: 60 deg**
  * vertical top-grasp cannot reach every waypoint: using a tilted grasp (peg stays vertical in the gripper)

## Files
* `01_home_pose/`, `02_calibration/`, `03_workspace_analysis/`, `04_pick_and_place/` - images, JSON, CSV, plots, video
* `run_log.json` - everything in machine-readable form
* `config_snapshot/` - the exact layout/vision settings used
