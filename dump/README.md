# Dump & Archive Buffer Directory (`dump/`)

This directory serves as a non-destructive buffer for scratch scripts, legacy debug images, redundant unzipped folders, and backup archives that were moved out of the active workspaces (`T1 2/`, `T3 2/`, and root) to maintain clean directory structure and high readability.

> [!NOTE]
> **Zero Data Loss**: Nothing has been deleted. All original files, temporary inspection captures, and backup archives are safely retained here for future audit or recovery.

---

## Directory Contents

### 1. `images_buffer/`
Contains 25 temporary debugging, thresholding, and inspection images generated during early vision development and camera calibration (e.g., `annotated_inspection.jpg`, `clahe_enhanced.jpg`, `socket_crop_*.jpg`, `th_*.jpg`, `diff_*.jpg`).

### 2. `scratch_scripts/`
Contains early scratch scripts, test routines, and logs from initial hardware bring-up:
* `calibrate_and_test_arm.py`: Early arm motion and camera test routine.
* `check_hardware.py`: Ping test for Dynamixel servos and sensors.
* `sweep_grid.py`: Early tabletop grid sweeping script.
* `hardware_status.txt`: Snapshot of motor status and port communication.
* `MUJOCO_LOG.TXT`: Simulation log from early MuJoCo kinematics tests.
* `repare.md`: Early troubleshooting notes.

### 3. `archives_backup/`
Contains root backup ZIP archives:
* `G9123.zip`: Full lab archive of Group 9 deliverables.
* `T1 2.zip`: Baseline backup of Task 1 workspace.
* `T3.zip`: Baseline backup of Task 3 workspace.

### 4. Legacy Task Snapshots (`T1/` & `T3/`)
* **[`dump/T1/`](file:///Users/aai/Desktop/ITR_Proj/dump/T1)**: Historical baseline snapshot of Task 1 from November 8. Preserved for commit history and reference (active work is in [`T1 2/`](file:///Users/aai/Desktop/ITR_Proj/T1%202)).
* **[`dump/T3/`](file:///Users/aai/Desktop/ITR_Proj/dump/T3)**: Historical photogrammetric calibration reference data, ruler photos, and JSON models from November 12 (active work is in [`T3 2/`](file:///Users/aai/Desktop/ITR_Proj/T3%202)).

### 5. Task-Specific Internal Buffers
Each active task workspace now has its own internal `dump/` directory for task-specific legacy archives and scratch files:
* **Task 1 Internal Dump**: [`T1 2/dump/`](file:///Users/aai/Desktop/ITR_Proj/T1%202/dump) (deprecated runners, historical demos, test output images, scratch logs).
* **Task 3 Internal Dump**: [`T3 2/dump/`](file:///Users/aai/Desktop/ITR_Proj/T3%202/dump) (superseded November 15 unzipped folder `Task2_ITR-main/`, duplicate nested trees, and older archives).
