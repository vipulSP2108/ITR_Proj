#!/usr/bin/env python3
"""
Master Execution Pipeline for Demo 1 (Single Circle Peg Sorting & Workspace Analysis).

This script chains:
  1. Multi-View Workspace Analysis:
     - Commands arm to 4 survey vantage angles (Home 90°, Pan Left, Pan Right, Elevated)
     - Captures overhead camera shots to eliminate occlusions
     - Produces 2x2 survey grid and segmented reachability map
  2. Single Circle Peg Trajectory Execution & Verification:
     - Validates continuous rotational symmetry (C_inf) pick-and-place
     - Moves from 90° Home -> Hover -> Descend -> Firm Grasp -> Vertical Lift ->
       Mid Transit -> Socket Hover -> Insert -> Release -> Retreat -> 90° Home
     - Performs live Cartesian tracking, path verification (100% ON_TRACK), and slip detection
     - Exports video and 9-panel storyboard

Outputs:
  • All images saved to: T1/demo1_20261107/imp_images/
  • Full MP4 video:     T1/demo1_20261107/demo1_circle_peg_sorting.mp4
"""

import os
import sys
import subprocess

DEMO_DIR = os.path.dirname(os.path.abspath(__file__))

def run_step(script_name, description):
    print("\n" + "=" * 74)
    print(f"  EXECUTING: {description}")
    print(f"  Script: {script_name}")
    print("=" * 74 + "\n")
    script_path = os.path.join(DEMO_DIR, script_name)
    res = subprocess.run([sys.executable, script_path], cwd=DEMO_DIR)
    if res.returncode != 0:
        print(f"\n[ERROR] Step '{script_name}' failed with return code {res.returncode}")
        sys.exit(res.returncode)

def main():
    print("*" * 74)
    print("   OpenMANIPULATOR-X DEMO 1: MULTI-VIEW WORKSPACE & CIRCLE PEG PIPELINE")
    print("*" * 74)

    # Step 1: Multi-View Workspace Analysis
    run_step("workspace_multi_view_analyzer.py", "Multi-View Workspace Analysis & Reachability Scan")

    # Step 2: Trajectory Execution & Verification
    run_step("circle_peg_trajectory_tracker.py", "Circle Peg Trajectory Execution & Vision Path Verification")

    print("\n" + "*" * 74)
    print("   ✓ DEMO 1 MASTER PIPELINE EXECUTION COMPLETE!")
    print("*" * 74)
    print("\nDeliverables generated in T1/demo1_20261107/imp_images/:")
    print("  • 00_multi_vantage_survey_grid.png       (2x2 multi-angle workspace survey)")
    print("  • 01_vantage_1_Home_90deg.png            (Pose 1: Home 90° Upright Ready Pose)")
    print("  • 01_vantage_2_Pan_Left_34deg.png        (Pose 2: Pan Left 34°)")
    print("  • 01_vantage_3_Pan_Right_34deg.png       (Pose 3: Pan Right 34°)")
    print("  • 01_vantage_4_Elevated_Survey.png       (Pose 4: Elevated Survey Pose)")
    print("  • 02_workspace_segmentation_map.png      (Zones, Target Peg, Cutout, Vector)")
    print("  • 03_path_checkpoint_*.png               (10 individual path milestone snapshots)")
    print("  • 04_full_path_verification_storyboard.png (9-panel full path verification storyboard)")
    print("  • demo1_circle_peg_sorting.mp4           (Full 25 FPS 3D simulation video)")

if __name__ == "__main__":
    main()

