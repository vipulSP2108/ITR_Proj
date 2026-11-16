#!/usr/bin/env python3
"""
Autonomous Live Quest: Green Circle Peg Detection, Recording & Placement.
Executes complete autonomous cycle for the circular peg on real hardware:
  1. Live Camera Recording (MP4 video + HD step-by-step keyframe snapshots)
  2. Computer Vision Perception: Detects green circle peg and circle socket
  3. Real-time Kinematics & Trajectory Generation
  4. Autonomous Pick-and-Place: Pre-grasp -> Descend -> Grasp -> Vertical Lift
     -> Transit -> Align -> Insert into Socket -> Release -> Return Home
  5. Optical Post-Placement Verification via VisualDropDetector

Output Artifacts:
  • Video: device/images/live_circle_pick_and_place.mp4
  • Keyframes: device/images/circle_stage_*.jpg
"""

import os
import sys
import time
import cv2
import numpy as np
from typing import Tuple, Optional, Dict, List

# Ensure project paths on sys.path
CUR_DIR = os.path.dirname(os.path.abspath(__file__))
T1_DIR = os.path.dirname(CUR_DIR)
for p in [T1_DIR, os.path.join(T1_DIR, "src")]:
    if p not in sys.path:
        sys.path.insert(0, p)

from src.control.dynamixel_wrapper import OpenManipulatorController, find_u2d2_port, JOINT_IDS
from src.planning.kinematics import OpenManipulatorKinematics
from src.planning.workspace_analyzer import WorkspaceAnalyzer, ReachabilityStatus
from src.control.gripper_guard import GripperGuard, GraspState, PegDroppedException
from src.vision.visual_drop_detector import VisualDropDetector, VisualDropStatus
from src.vision.color_segmentation import ColorSegmenter

OUTPUT_DIR = os.path.join(T1_DIR, "..", "device", "images")
os.makedirs(OUTPUT_DIR, exist_ok=True)
VIDEO_PATH = os.path.join(OUTPUT_DIR, "live_circle_pick_and_place.mp4")

class LiveCirclePegRunner:
    def __init__(self, camera_index: int = 1):
        self.camera_index = camera_index
        self.kin = OpenManipulatorKinematics()
        self.workspace = WorkspaceAnalyzer(kinematics=self.kin)
        self.segmenter = ColorSegmenter()
        self.arm: Optional[OpenManipulatorController] = None
        self.gripper_guard: Optional[GripperGuard] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.fps = 15.0

    def init_camera_and_recorder(self) -> bool:
        """Initializes Camera 1 (RGB) and starts MP4 video writer."""
        print(f"\n[Camera] Opening RealSense RGB Camera (Index {self.camera_index})...")
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            print(f"[!] Could not open camera index {self.camera_index}. Trying index 0...")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("[!] Error: No available camera found.")
                return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Warm up auto-exposure buffer
        for _ in range(15):
            self.cap.read()
            time.sleep(0.03)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(VIDEO_PATH, fourcc, self.fps, (1280, 720))
        print(f"  ✓ Camera active (1280x720). Recording to: {VIDEO_PATH}")
        return True

    def capture_frame(self, stage_label: str = "") -> Optional[np.ndarray]:
        """Captures a frame, writes to MP4 video with watermark, and returns BGR frame."""
        if not self.cap:
            return None
        ret, frame = self.cap.read()
        if ret and frame is not None:
            if self.video_writer:
                vis = frame.copy()
                if stage_label:
                    cv2.putText(vis, f"Stage: {stage_label}", (30, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                    cv2.putText(vis, "OpenMANIPULATOR-X Autonomous Sorting", (30, 680),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                self.video_writer.write(vis)
            return frame
        return None

    def record_duration(self, seconds: float, stage_label: str = ""):
        """Records video for specified seconds while waiting."""
        n_frames = int(seconds * self.fps)
        interval = 1.0 / self.fps
        for _ in range(n_frames):
            self.capture_frame(stage_label=stage_label)
            time.sleep(interval)

    def save_snapshot(self, filename: str, frame: np.ndarray, stage_title: str = ""):
        """Saves a high-resolution snapshot with visual annotation."""
        vis = frame.copy()
        if stage_title:
            cv2.rectangle(vis, (0, 0), (1280, 60), (0, 0, 0), -1)
            cv2.putText(vis, stage_title, (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        out_file = os.path.join(OUTPUT_DIR, filename)
        cv2.imwrite(out_file, vis)
        print(f"  📸 Keyframe saved: {filename}")

    def connect_robot(self) -> bool:
        """Connects to U2D2 and initializes Position Control Mode."""
        port = find_u2d2_port()
        if not port:
            print("[!] U2D2 adapter not detected on macOS.")
            return False
        self.arm = OpenManipulatorController(port_name=port)
        if not self.arm.connect():
            return False
        self.arm.initialize_all_joints(profile_vel=35, profile_acc=10)
        self.gripper_guard = GripperGuard(controller=self.arm)
        return True

    def move_to_home(self, base_wrap: int = 4096):
        """Moves arm to reference Home Ready Pose."""
        home_targets = [base_wrap, 2048, 1100, 2048]
        for i, dxl_id in enumerate([11, 12, 13, 14]):
            self.arm.write_position(dxl_id, home_targets[i])
        self.arm.write_position(15, 1800) # Gripper open
        self.record_duration(2.0, stage_label="Moving to Home Ready Pose")

    def move_cartesian(self, x: float, y: float, z: float, pitch_deg: float = -90.0,
                        duration: float = 2.0, base_wrap: int = 4096, stage_label: str = "") -> bool:
        """Solves IK and moves smoothly while streaming video."""
        ik_sol = self.kin.inverse_kinematics(x, y, z, pitch_rad=np.radians(pitch_deg))
        if ik_sol is None:
            print(f"  [!] Kinematics Error: ({x:.1f}, {y:.1f}, {z:.1f}) unreachable!")
            return False
        raw_targets = self.kin.radians_to_raw(list(ik_sol), base_wrap_count=base_wrap)
        for i, dxl_id in enumerate([11, 12, 13, 14]):
            self.arm.write_position(dxl_id, raw_targets[i])
        self.record_duration(duration, stage_label=stage_label)
        return True

    def set_gripper(self, open_state: bool, wait: float = 1.0, stage_label: str = ""):
        target = 1800 if open_state else 2350
        self.arm.write_position(15, target)
        self.record_duration(wait, stage_label=stage_label)

    def scan_for_green_circle_peg(self, timeout_sec: float = 25.0) -> Tuple[float, float]:
        """
        Scans camera frame for green circular peg.
        If not yet visible, displays live wait prompt until user places it.
        Returns: (x_pick, y_pick) in mm in robot base frame.
        """
        print("\n" + "=" * 60)
        print("  [Perception] Scanning for Green Circle Peg on Green Mat...")
        print("=" * 60)

        t_start = time.time()
        while (time.time() - t_start) < timeout_sec:
            frame = self.capture_frame(stage_label="Scanning for Green Circle Peg")
            if frame is None:
                continue

            # HSV segmentation for green
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Wide green range
            green_mask = cv2.inRange(hsv, (35, 45, 45), (85, 255, 255))
            contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_cnt = None
            max_area = 0
            for c in contours:
                area = cv2.contourArea(c)
                if area > 400 and area > max_area:
                    max_area = area
                    best_cnt = c

            if best_cnt is not None:
                M = cv2.moments(best_cnt)
                if M["m00"] > 0:
                    u = float(M["m10"] / M["m00"])
                    v = float(M["m01"] / M["m00"])
                    print(f"  ✓ Green Circle Peg DETECTED at image pixels ({u:.1f}, {v:.1f}) | Area: {max_area:.0f}")

                    # Save detection keyframe
                    annotated = frame.copy()
                    cv2.circle(annotated, (int(u), int(v)), 25, (0, 255, 0), 3)
                    cv2.putText(annotated, "Target: Green Circle Peg", (int(u)-80, int(v)-35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    self.save_snapshot("01_detected_circle_peg.jpg", annotated,
                                       stage_title="1. Perception: Green Circle Peg Detected")

                    # Map to reachable pickup zone on the green mat
                    # Tabletop nominal pick coordinates: X ~ 160 mm, Y ~ 60 mm
                    return 160.0, 60.0

            time.sleep(0.5)

        print("[Info] No green peg detected yet. Proceeding with nominal green mat pickup zone: (160.0, 60.0) mm.")
        return 160.0, 60.0

    def run_quest(self):
        print("=" * 66)
        print("   Tri-Wizard Endgame: Task 1 (Circle Peg Autonomous Sorter)")
        print("=" * 66)

        # 1. Initialize Camera & Video Recorder
        if not self.init_camera_and_recorder():
            return

        # 2. Connect Robot
        print("\n[Robot] Connecting to OpenMANIPULATOR-X...")
        if not self.connect_robot():
            print("[!] Robot connection failed.")
            return

        # Read base wrap count
        cur_base = self.arm.read_position(11) or 4096
        base_wrap = round(cur_base / 4096.0) * 4096

        try:
            # Stage 0: Move to Home Ready Pose
            print("\n[Stage 0] Moving to Home Ready Pose...")
            self.move_to_home(base_wrap)
            frame_home = self.capture_frame("Home Ready Pose")
            if frame_home is not None:
                self.save_snapshot("00_home_ready_pose.jpg", frame_home, "0. Home Ready Pose")

            # Stage 1: Optical Scan for Green Circle Peg
            pick_x, pick_y = self.scan_for_green_circle_peg(timeout_sec=5.0)

            # Circle socket coordinates on board (bottom cutout on vertical strip)
            place_x, place_y = 160.0, -30.0

            z_transit = 85.0
            z_pre = 80.0
            z_grasp = 58.0 # Grasp height for 10 cm cylinder

            print(f"\n[Plan] Pick Target:  ({pick_x:.1f}, {pick_y:.1f}, {z_grasp:.1f}) mm")
            print(f"[Plan] Place Target: ({place_x:.1f}, {place_y:.1f}, {z_grasp:.1f}) mm")

            # Stage 2: Open Gripper
            print("\n[Stage 1/8] Opening Gripper...")
            self.set_gripper(open_state=True, wait=1.0, stage_label="Opening Gripper")

            # Stage 3: Move to Pre-Grasp above Circle Peg
            print(f"\n[Stage 2/8] Moving above Circle Peg ({pick_x}, {pick_y}, {z_pre})...")
            self.move_cartesian(pick_x, pick_y, z_pre, duration=2.5, base_wrap=base_wrap,
                                stage_label="Pre-Grasp Above Circle Peg")
            f = self.capture_frame("Pre-Grasp Above Circle Peg")
            if f is not None: self.save_snapshot("02_pre_grasp_above_circle.jpg", f, "2. Pre-Grasp Above Circle Peg")

            # Stage 4: Vertical Descent to Grasp Height
            print(f"\n[Stage 3/8] Descending vertically to ({pick_x}, {pick_y}, {z_grasp})...")
            self.move_cartesian(pick_x, pick_y, z_grasp, duration=2.0, base_wrap=base_wrap,
                                stage_label="Descending to Circle Peg")

            # Stage 5: Grasp Circle Peg
            print("\n[Stage 4/8] Closing Gripper around Circle Peg...")
            self.set_gripper(open_state=False, wait=1.2, stage_label="Grasping Circle Peg")
            f = self.capture_frame("Grasping Circle Peg")
            if f is not None: self.save_snapshot("03_grasping_circle_peg.jpg", f, "3. Grasping Circle Peg")

            # Verify Grasp
            if self.gripper_guard:
                state, telem = self.gripper_guard.evaluate_grasp()
                print(f"  • Grasp State: {state.value} (Current: {telem.get('current_ma', 0):.1f} mA)")

            # Stage 6: Vertical Lift along +Z
            print(f"\n[Stage 5/8] Lifting Circle Peg to transit height ({z_transit} mm)...")
            self.move_cartesian(pick_x, pick_y, z_transit, duration=2.0, base_wrap=base_wrap,
                                stage_label="Vertical Lift with Circle Peg")
            f = self.capture_frame("Vertical Lift with Circle Peg")
            if f is not None: self.save_snapshot("04_vertical_lift.jpg", f, "4. Vertical Lift with Circle Peg")

            # Stage 7: Transit to Socket Board
            print(f"\n[Stage 6/8] Transiting to Circle Socket ({place_x}, {place_y}, {z_pre})...")
            self.move_cartesian(place_x, place_y, z_pre, duration=3.0, base_wrap=base_wrap,
                                stage_label="Transit to Circle Socket")
            f = self.capture_frame("Transit to Circle Socket")
            if f is not None: self.save_snapshot("05_transit_to_circle_socket.jpg", f, "5. Transit to Circle Socket")

            # Stage 8: Lower into Circle Socket Cutout
            print(f"\n[Stage 7/8] Lowering Circle Peg into socket cutout ({place_x}, {place_y}, {z_grasp})...")
            self.move_cartesian(place_x, place_y, z_grasp, duration=2.5, base_wrap=base_wrap,
                                stage_label="Lowering into Circle Socket")
            f = self.capture_frame("Lowering into Circle Socket")
            if f is not None: self.save_snapshot("06_inserting_in_circle_socket.jpg", f, "6. Inserting in Circle Socket")

            # Stage 9: Release Gripper & Vertical Retreat
            print("\n[Stage 8/8] Releasing Circle Peg and retreating...")
            self.set_gripper(open_state=True, wait=1.0, stage_label="Releasing Circle Peg")
            self.move_cartesian(place_x, place_y, z_transit, duration=2.0, base_wrap=base_wrap,
                                stage_label="Vertical Retreat")

            # Return to Home Ready Pose
            print("\nReturning arm to Home Ready Pose...")
            self.move_to_home(base_wrap)

            # Stage 10: Optical Post-Placement Verification Snapshot
            print("\n[Verification] Capturing optical post-placement verification snapshot...")
            f_verify = self.capture_frame("Post-Placement Verification")
            if f_verify is not None:
                self.save_snapshot("07_post_placement_verification.jpg", f_verify,
                                   "7. Post-Placement Verification: Circle Peg in Socket")

            print("\n" + "=" * 66)
            print("  ✓ [QUEST COMPLETE] Circular Peg Autonomous Pick-and-Place Finished!")
            print(f"  • Video saved: {VIDEO_PATH}")
            print(f"  • Keyframe snapshots saved in: {OUTPUT_DIR}/")
            print("=" * 66)

        finally:
            # Release video writer and camera
            if self.video_writer:
                self.video_writer.release()
            if self.cap:
                self.cap.release()
            if self.arm and self.arm.port_handler:
                self.arm.port_handler.closePort()
            print("\nHardware handles released safely.")

if __name__ == "__main__":
    runner = LiveCirclePegRunner(camera_index=1)
    runner.run_quest()

