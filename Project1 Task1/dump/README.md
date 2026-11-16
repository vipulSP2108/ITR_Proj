# Task 1 Internal Dump & Historical Archive (`T1 2/dump/`)

This directory serves as a non-destructive buffer for superseded scripts, historical demos, early test outputs, and developer scratch files that were modularized out of the root [`T1 2/`](file:///Users/aai/Desktop/ITR_Proj/T1%202) workspace.

> [!NOTE]
> **Safety Notice**:
> Old scripts in this directory (`hw_run.py`, `hw_test_step1.py`, `run_hardware_pipeline*.py`) use deprecated kinematics offsets and outdated gripper limits. **Do not execute them on the physical robot**. All active hardware control is handled exclusively by [`pick_place.py`](file:///Users/aai/Desktop/ITR_Proj/T1%202/pick_place.py) and [`run_calibration.sh`](file:///Users/aai/Desktop/ITR_Proj/T1%202/run_calibration.sh).

---

## Directory Organization

### 1. `deprecated_runners/`
Contains early pipeline iterations developed prior to the unified `pick_place.py` architecture:
* `hw_run.py`: Early hardware test script.
* `hw_test_step1.py`: Initial step 1 verification script.
* `run_hardware_pipeline.py`: Pipeline version 1.
* `run_hardware_pipeline_v2.py`: Pipeline version 2.

### 2. `historical_demos/`
Contains early prototypes and simulation work from early project phases:
* `demo1_20261107/`: Early November 7 demonstration codebase (a backward-compatible symlink `demo1_20261107` is retained in `T1 2/`).
* `demo_runs/`: November 8 demo run telemetry and logs.
* `sim/` & `sim_demo/`: Early simulation models and kinematics tests.
* `scripts/`: Legacy utilities and coordinate converters.
* `tests/`: Early unit test routines.

### 3. `test_images_and_output/`
Contains early visual captures and thresholding outputs:
* `hw_test_output/`: 22 intermediate test images and masks from November 10 (`raw_frame.jpg`, `board_contours.jpg`, etc.).
* `calibration_img/`: Legacy calibration photos.
* `test_images/`: Raw test photos from initial setup.

### 4. `scratch_notes_and_logs/`
Contains early working logs, developer notes, and temporary files:
* `MUJOCO_LOG.TXT`: Simulation log from early MuJoCo trials.
* `task_rough.txt`: Initial problem scoping notes.
* `test.mp`: Scratch kinematics calculations.
* `imp.md`: Working notes from early bring-up.
* `context_folder/`: System configuration notes.
