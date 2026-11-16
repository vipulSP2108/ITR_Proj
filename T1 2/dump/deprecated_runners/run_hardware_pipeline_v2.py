#!/usr/bin/env python3
"""
Vision-Guided Hardware Pipeline v2 for OpenMANIPULATOR-X Peg Sorting.

Key improvements over v1:
- ALL 4 peg shapes: circle, square, hexagon, triangle
- ZERO hardcoded coordinates — everything detected via camera
- Vision checks at every waypoint during trajectory
- Obstacle detection and avoidance (camera stand, etc.)
- Real-time visualization overlays (trajectory, checkpoints, detections)
- Gripper feedback monitoring during transit

Usage:
    python run_hardware_pipeline_v2.py [--no-display] [--record output.mp4]
"""

import os
import sys
import cv2
import time
import math
import argparse
import numpy as np
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from src.control.dynamixel_wrapper import OpenManipulatorController
from src.control.gripper_guard import GripperGuard, GraspState
from src.planning.kinematics import OpenManipulatorKinematics
from src.planning.workspace_analyzer import WorkspaceAnalyzer
from src.vision.workspace_vision import WorkspaceVision, WorkspaceScan
from src.planning.vision_trajectory import (
    VisionTrajectoryExecutor, TrajectoryPlan, MoveResult, Waypoint
)
from src.planning.trajectory_visualizer import TrajectoryVisualizer

# ─── Constants ────────────────────────────────────────────────────────
HOME_POSE = [0, 2048, 2048, 2048, 3130]     # [j11, j12, j13, j14, j15]
SIDE_PARK_POSE = [3072, 2048, 2048, 2048, 3130]  # arm to +Y side, clears camera FOV
GRIPPER_OPEN = 1700
GRIPPER_CLOSE = 2200

SHAPE_SORT_ORDER = ["circle", "square", "hexagon", "triangle"]


class HardwarePipelineV2:
    """
    4-stage vision-guided hardware pipeline:
      Stage 1: Initialize & Home
      Stage 2: Side-park & Full Workspace Scan
      Stage 3: Vision-Guided Pick & Place (all shapes)
      Stage 4: Verify & Report
    """

    def __init__(self, display: bool = True, record_path: str = None):
        self.display = display
        self.record_path = record_path
        self.video_writer = None

        self.arm: OpenManipulatorController = None
        self.cap = None
        self.vision: WorkspaceVision = None
        self.executor: VisionTrajectoryExecutor = None
        self.gripper: GripperGuard = None
        self.visualizer: TrajectoryVisualizer = None

        self.scan: WorkspaceScan = None
        self.results = {}
        self.frame_log = []

    # ─── Setup ────────────────────────────────────────────────────────

    def setup(self) -> bool:
        """Initialize all hardware and vision components."""
        print("\n" + "=" * 60)
        print("  HARDWARE PIPELINE v2 — Vision-Guided Peg Sorting")
        print("  All 4 shapes | Zero fixed coords | Real-time vision")
        print("=" * 60 + "\n")

        # 1. Connect to arm
        print("[1/4] Connecting to OpenMANIPULATOR-X...")
        self.arm = OpenManipulatorController()
        if not self.arm.connect():
            print("  FAILED: Cannot connect to U2D2 adapter.")
            return False
        if not self.arm.initialize_all_joints(profile_vel=50, profile_acc=15):
            print("  FAILED: Joint initialization failed.")
            return False
        self.gripper = GripperGuard(self.arm)
        print("  OK: Arm connected and initialized.\n")

        # 2. Open camera
        print("[2/4] Opening camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            print("  FAILED: No camera found.")
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(1.0)  # let camera auto-expose
        print("  OK: Camera opened.\n")

        # 3. Initialize vision
        print("[3/4] Initializing vision pipeline...")
        config_dir = os.path.join(SCRIPT_DIR, 'config')
        self.vision = WorkspaceVision(config_dir)
        print(f"  Scale: {self.vision.scale:.3f} mm/px")
        print(f"  Robot base pixel: ({self.vision.base_u}, {self.vision.base_v})")
        print(f"  Homography: {'loaded' if self.vision.homography is not None else 'using pinhole model'}")
        print()

        # 4. Initialize trajectory executor
        print("[4/4] Setting up trajectory executor...")
        self.executor = VisionTrajectoryExecutor(
            arm_controller=self.arm,
            camera_capture=self.cap.read,
            workspace_vision=self.vision,
            gripper_guard=self.gripper,
        )

        self.visualizer = TrajectoryVisualizer(
            robot_to_pixel_fn=self.vision.robot_to_pixel,
            base_u=self.vision.base_u,
            base_v=self.vision.base_v,
            scale=self.vision.scale,
        )

        self.executor.on_waypoint_cb = self._on_waypoint
        self.executor.on_vision_cb = self._on_vision_frame

        if self.record_path:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.record_path, fourcc, 15.0, (640, 480))
            print(f"  Recording to: {self.record_path}")

        print("  OK: All systems ready.\n")
        return True

    # ─── Stage 1: Home ────────────────────────────────────────────────

    def stage_home(self):
        """Move arm to home position."""
        print("\n--- STAGE 1: HOME ---")
        self.executor.go_home(duration=3.0)
        time.sleep(0.5)
        self._read_and_display("Home position")
        print("  Arm at home.\n")

    # ─── Stage 2: Workspace Scan ─────────────────────────────────────

    def stage_scan(self) -> bool:
        """Park arm to side and run full workspace perception."""
        print("\n--- STAGE 2: WORKSPACE SCAN ---")

        print("  Parking arm to side for clear camera view...")
        self.executor.go_side_park(duration=2.5)
        time.sleep(1.0)

        print("  Capturing workspace scan...")
        # Take multiple frames and use the last (auto-exposure settled)
        for _ in range(5):
            ret, frame = self.cap.read()
            time.sleep(0.1)

        if not ret or frame is None:
            print("  FAILED: No frame from camera.")
            return False

        self.scan = self.vision.full_scan(frame)

        print(f"\n  DETECTION RESULTS:")
        print(f"  Pegs found: {len(self.scan.pegs)}")
        for p in self.scan.pegs:
            x, y = p.robot_xy
            r = math.sqrt(x**2 + y**2)
            print(f"    {p.shape:10s} ({p.color:6s}) at ({x:7.1f}, {y:7.1f}) mm  r={r:.0f}mm  yaw={p.yaw_deg:.1f}°")

        print(f"  Board ROI: {self.scan.board_roi}")
        print(f"  Sockets found: {len(self.scan.sockets)}")
        for s in self.scan.sockets:
            x, y = s.robot_xy
            print(f"    [{s.shape:10s}] at ({x:7.1f}, {y:7.1f}) mm  yaw={s.yaw_deg:.1f}°")

        print(f"  Assignments: {list(self.scan.assignments.keys())}")

        if self.scan.annotated_frame is not None:
            self._show_frame(self.scan.annotated_frame, "Workspace Scan")
            path = os.path.join(SCRIPT_DIR, "workspace_scan.jpg")
            cv2.imwrite(path, self.scan.annotated_frame)
            print(f"  Saved scan to {path}")

        if not self.scan.assignments:
            print("\n  WARNING: No peg-socket assignments made!")
            print("  Check: Are pegs visible? Is board in camera FOV?")
            return False

        print()
        return True

    # ─── Stage 3: Pick & Place ───────────────────────────────────────

    def stage_pick_and_place(self):
        """Execute vision-guided pick and place for all matched pegs."""
        print("\n--- STAGE 3: VISION-GUIDED PICK & PLACE ---\n")

        sorted_shapes = [s for s in SHAPE_SORT_ORDER if s in self.scan.assignments]
        print(f"  Sort order: {sorted_shapes}\n")

        for shape_name in sorted_shapes:
            peg, socket = self.scan.assignments[shape_name]

            print(f"\n  === {shape_name.upper()} PEG ===")

            # Re-scan before each pick to account for any drift
            print("  Re-scanning workspace for fresh positions...")
            self.executor.go_side_park(duration=2.0)
            time.sleep(0.8)

            ret, frame = self.cap.read()
            if ret and frame is not None:
                fresh_scan = self.vision.full_scan(frame)
                if shape_name in fresh_scan.assignments:
                    fresh_peg, fresh_socket = fresh_scan.assignments[shape_name]
                    peg = fresh_peg
                    socket = fresh_socket
                    print(f"  Updated positions from fresh scan")

            # Plan trajectory
            plan = self.executor.plan_pick_and_place(
                peg_shape=peg.shape,
                peg_color=peg.color,
                pick_xy=peg.robot_xy,
                place_xy=socket.robot_xy,
                peg_yaw=peg.yaw_deg,
                socket_yaw=socket.yaw_deg,
                symmetry_order=peg.symmetry_order,
            )

            # Execute with vision feedback
            result, message = self.executor.execute_plan(plan)

            self.results[shape_name] = {
                "result": result.value,
                "message": message,
                "pick": peg.robot_xy,
                "place": socket.robot_xy,
            }

            if result == MoveResult.SUCCESS:
                print(f"  SUCCESS: {message}")
            else:
                print(f"  FAILED: {message}")
                # Return home safely before trying next peg
                print("  Returning to home for safety...")
                self.executor.go_home(duration=2.5)
                time.sleep(0.5)

    # ─── Stage 4: Verify & Report ────────────────────────────────────

    def stage_verify(self):
        """Final verification scan and summary report."""
        print("\n--- STAGE 4: VERIFICATION ---\n")

        self.executor.go_side_park(duration=2.0)
        time.sleep(1.0)

        ret, frame = self.cap.read()
        if ret and frame is not None:
            final_scan = self.vision.full_scan(frame)

            remaining_pegs = [p.shape for p in final_scan.pegs]
            print(f"  Remaining pegs on mat: {remaining_pegs if remaining_pegs else 'NONE (all sorted!)'}")

            if final_scan.annotated_frame is not None:
                path = os.path.join(SCRIPT_DIR, "final_verification.jpg")
                cv2.imwrite(path, final_scan.annotated_frame)
                self._show_frame(final_scan.annotated_frame, "Final Verification")

        print(f"\n{'='*60}")
        print("  PIPELINE RESULTS SUMMARY")
        print(f"{'='*60}")
        total = len(self.results)
        success = sum(1 for r in self.results.values() if r["result"] == "success")
        print(f"  Total attempts: {total}")
        print(f"  Successful: {success}/{total}")
        for shape, r in self.results.items():
            status = "✓" if r["result"] == "success" else "✗"
            print(f"    {status} {shape:10s}: {r['message']}")
        print(f"{'='*60}\n")

        # Return home
        self.executor.go_home(duration=2.5)

    # ─── Callbacks ───────────────────────────────────────────────────

    def _on_waypoint(self, step: int, waypoint, plan: TrajectoryPlan):
        """Called after each waypoint is reached."""
        frame = self.executor.get_frame()
        if frame is not None and self.visualizer:
            overlay = self.visualizer.draw_full_overlay(
                frame,
                plan.waypoints,
                step,
                peg_shape=plan.peg_shape,
                peg_color=plan.peg_color,
                gripper_open=waypoint.gripper_open,
            )
            self._show_frame(overlay, f"Step {step + 1}: {waypoint.name}")

    def _on_vision_frame(self, stage_name: str, frame: np.ndarray):
        """Called during vision checks with annotated frames."""
        if self.display:
            cv2.putText(frame, f"Vision: {stage_name}", (10, 470),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            self._show_frame(frame, f"Vision Check: {stage_name}")

    def _read_and_display(self, title: str = ""):
        """Read a frame and display it."""
        ret, frame = self.cap.read()
        if ret and frame is not None:
            self._show_frame(frame, title)

    def _show_frame(self, frame: np.ndarray, title: str = "Pipeline"):
        """Display a frame and optionally record it."""
        if self.video_writer:
            self.video_writer.write(frame)

        if self.display:
            cv2.imshow("Pipeline v2", frame)
            cv2.waitKey(1)

    # ─── Main Run ────────────────────────────────────────────────────

    def run(self):
        """Execute the full 4-stage pipeline."""
        try:
            if not self.setup():
                print("\nSetup failed. Exiting.")
                return

            self.stage_home()
            if not self.stage_scan():
                print("\nWorkspace scan failed. Check camera and peg/board placement.")
                self.executor.go_home()
                return

            self.stage_pick_and_place()
            self.stage_verify()

        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Returning home...")
            if self.executor:
                self.executor.go_home(duration=3.0)

        except Exception as e:
            print(f"\nERROR: {e}")
            import traceback
            traceback.print_exc()
            if self.executor:
                try:
                    self.executor.go_home(duration=3.0)
                except Exception:
                    pass

        finally:
            self.cleanup()

    def cleanup(self):
        """Release all resources."""
        if self.video_writer:
            self.video_writer.release()
        if self.cap:
            self.cap.release()
        if self.arm:
            self.arm.disconnect(disable_torque=False)
        if self.display:
            cv2.destroyAllWindows()
        print("\nCleanup complete.")


def main():
    parser = argparse.ArgumentParser(description="Vision-Guided Peg Sorting Pipeline v2")
    parser.add_argument("--no-display", action="store_true", help="Disable GUI display")
    parser.add_argument("--record", type=str, default=None, help="Record video to file")
    args = parser.parse_args()

    pipeline = HardwarePipelineV2(
        display=not args.no_display,
        record_path=args.record,
    )
    pipeline.run()


if __name__ == "__main__":
    main()
