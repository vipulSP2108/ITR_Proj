# Task 3 Internal Dump & Historical Archive (`T3 2/dump/`)

This directory serves as a non-destructive buffer for superseded code versions, redundant duplicate trees, and duplicate markdown documentation that were modularized out of [`T3 2/`](file:///Users/aai/Desktop/ITR_Proj/T3%202) to eliminate repetition and maintain clarity.

---

## Directory Organization

### 1. `duplicate_markdown_guides/`
Contains redundant copies of the `01_` through `05_` markdown guides that were previously replicated across multiple intermediate directories. The authoritative, active copies are maintained exclusively in:
[`T3 2/buzzwire_calib/`](file:///Users/aai/Desktop/ITR_Proj/T3%202/buzzwire_calib)

### 2. `duplicate_t3_snapshots/`
Contains unzipped duplicate trees of the early baseline `T3` folder that were nested inside `NEW/...`:
* `T3_outer/`: Duplicate `T3` tree previously at `NEW/Task2_ITR-main (1)/Task2_ITR-main/T3`.
* `T3_inner/`: Duplicate `T3` tree previously at `.../Task2/taken/G9/Task2_ITR-main/T3`.
* `Task2_duplicates/`: Duplicate folders (`Buzz Wire Robot Navigation Calibration`, `new_ref`, `Project3 Instructions`) from `Task2/`.

### 3. `intermediate_buzzwire_calib/`
Contains older, un-updated copies of `buzzwire_calib` that were superseded by the final lab-tested version in `taken/G9/Task2_ITR-main/buzzwire_calib/`:
* `buzzwire_calib_outer`: Superseded copy from `NEW/Task2_ITR-main (1)/Task2_ITR-main/buzzwire_calib`.
* `buzzwire_calib_task2`: Superseded copy from `NEW/.../Task2/buzzwire_calib`.

### 4. `Task2_ITR-main/` & `Task2_ITR-main.zip`
The original unzipped folder and 2.4 MB ZIP archive from the November 15 release.
