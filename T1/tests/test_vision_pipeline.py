import os
import sys
import unittest
import cv2
import numpy as np

# Ensure project root is in sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from src.vision.synthetic_scene import SyntheticSceneGenerator, CameraIntrinsics
from src.vision.color_segmentation import ColorSegmenter
from src.vision.shape_detection import ShapeDetector, DetectedObject

class TestVisionPipeline(unittest.TestCase):
    def setUp(self):
        self.intrinsics = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0, width=640, height=480)
        self.scene_gen = SyntheticSceneGenerator(intrinsics=self.intrinsics, seed=123)
        self.segmenter = ColorSegmenter()
        self.detector = ShapeDetector(intrinsics=self.intrinsics)

    def test_end_to_end_peg_detection(self):
        """
        Tests that all 4 pegs are correctly segmented by color,
        classified by shape, and have accurate 2D/3D centroids and heights.
        """
        rgb, depth_map, gt_poses = self.scene_gen.generate_scene(add_noise=True)

        # Filter ground-truth pegs
        gt_pegs = [p for p in gt_poses if not p.is_socket]
        self.assertEqual(len(gt_pegs), 4, "Ground truth should have 4 pegs")

        all_detected = []
        # Segment all colors
        color_masks = self.segmenter.segment_all(rgb)

        for color_name, mask in color_masks.items():
            detected = self.detector.detect_objects_in_mask(mask, color_name, depth_map)
            all_detected.extend(detected)

        self.assertEqual(len(all_detected), 4, f"Expected 4 detected pegs, got {len(all_detected)}")

        # Match each detected peg with ground truth
        matched_shapes = set()
        for dt in all_detected:
            # Find matching ground truth peg by color
            matching_gt = next((p for p in gt_pegs if p.color == dt.color), None)
            self.assertIsNotNone(matching_gt, f"No ground truth peg found for color {dt.color}")

            # 1. Shape classification check
            self.assertEqual(dt.shape, matching_gt.shape,
                             f"Shape mismatch for {dt.color} peg: detected {dt.shape}, expected {matching_gt.shape}")
            matched_shapes.add(dt.shape)

            # 2. 2D Centroid accuracy (allow max 2.5 pixels error due to noise and discretization)
            c2d_err = np.linalg.norm(np.array(dt.centroid_2d) - np.array(matching_gt.centroid_2d))
            self.assertLess(c2d_err, 2.5, f"Centroid error too high: {c2d_err:.2f} px")

            # 3. Depth Z accuracy (within 3.0 mm)
            z_err = abs(dt.centroid_3d[2] - matching_gt.centroid_3d[2])
            self.assertLess(z_err, 3.0, f"Depth error too high: {z_err:.2f} mm")

            # 4. 3D Position accuracy in camera frame (within 3.0 mm)
            p3d_err = np.linalg.norm(np.array(dt.centroid_3d) - np.array(matching_gt.centroid_3d))
            self.assertLess(p3d_err, 3.5, f"3D Position error too high: {p3d_err:.2f} mm")

        # Verify all 4 shapes were recovered
        expected_shapes = {"square", "triangle", "circle", "hexagon"}
        self.assertEqual(matched_shapes, expected_shapes, "Did not recover all 4 required geometric shapes")

    def test_minimal_rotation_modulo_symmetry(self):
        """
        Validates that symmetry-modulo rotation never commands more than half-period twist.
        """
        # Square (4-fold symmetry, period 90 deg, max turn 45 deg)
        rot_sq = self.detector.calculate_minimal_rotation(target_yaw=10.0, current_yaw=95.0, symmetry_order=4)
        self.assertAlmostEqual(rot_sq, 5.0, delta=0.1)

        # Hexagon (6-fold symmetry, period 60 deg, max turn 30 deg)
        rot_hex = self.detector.calculate_minimal_rotation(target_yaw=15.0, current_yaw=65.0, symmetry_order=6)
        self.assertAlmostEqual(rot_hex, 10.0, delta=0.1)

        # Triangle (3-fold symmetry, period 120 deg, max turn 60 deg)
        rot_tri = self.detector.calculate_minimal_rotation(target_yaw=30.0, current_yaw=160.0, symmetry_order=3)
        self.assertAlmostEqual(rot_tri, -10.0, delta=0.1)

        # Circle (rotation invariant)
        rot_circ = self.detector.calculate_minimal_rotation(target_yaw=88.0, current_yaw=12.0, symmetry_order=0)
        self.assertEqual(rot_circ, 0.0)

    def test_socket_detection_and_matching(self):
        """
        Tests automated socket detection on the board and 1-to-1 matching with pegs.
        """
        rgb, depth_map, gt_poses = self.scene_gen.generate_scene(add_noise=True)
        board_roi = (350, 60, 250, 360)
        sockets = self.detector.detect_sockets(rgb, depth_map, board_roi=board_roi)

        self.assertEqual(len(sockets), 4, f"Expected 4 sockets detected, got {len(sockets)}")
        socket_shapes = {s.shape for s in sockets}
        self.assertEqual(socket_shapes, {"square", "triangle", "circle", "hexagon"})

        # Match pegs to sockets
        color_masks = self.segmenter.segment_all(rgb)
        pegs = []
        for color_name, mask in color_masks.items():
            pegs.extend(self.detector.detect_objects_in_mask(mask, color_name, depth_map))

        # Check 1-to-1 matching
        for peg in pegs:
            matched_socket = next((s for s in sockets if s.shape == peg.shape), None)
            self.assertIsNotNone(matched_socket, f"No matching socket for {peg.shape} peg")
            min_rot = self.detector.calculate_minimal_rotation(
                matched_socket.yaw_deg, peg.yaw_deg, peg.symmetry_order
            )
            max_allowed_rot = (180.0 / peg.symmetry_order) if peg.symmetry_order > 0 else 0.0
            self.assertLessEqual(abs(min_rot), max_allowed_rot + 0.01,
                                 f"Rotation {min_rot} exceeded half-period for shape {peg.shape}")

    def test_multi_seed_robustness(self):
        """
        Runs perception across 10 randomized scenes to verify zero false classifications.
        """
        total_pegs_tested = 0
        for seed in range(10):
            gen = SyntheticSceneGenerator(intrinsics=self.intrinsics, seed=seed)
            rgb, depth_map, gt_poses = gen.generate_scene(add_noise=True)
            gt_pegs = [p for p in gt_poses if not p.is_socket]
            masks = self.segmenter.segment_all(rgb)

            for p in gt_pegs:
                total_pegs_tested += 1
                dt = self.detector.detect_objects_in_mask(masks[p.color], p.color, depth_map)
                self.assertEqual(len(dt), 1, f"Seed {seed}: failed to detect single {p.color} peg")
                self.assertEqual(dt[0].shape, p.shape, f"Seed {seed}: shape mismatch for {p.color}")

        self.assertEqual(total_pegs_tested, 40)

    def test_generate_diagnostic_visualization(self):
        """
        Generates and saves a full visual diagnostic debug image with overlays.
        """
        rgb, depth_map, gt_poses = self.scene_gen.generate_scene(add_noise=True)
        vis = rgb.copy()

        # Detect pegs
        color_masks = self.segmenter.segment_all(rgb)
        pegs = []
        for color_name, mask in color_masks.items():
            pegs.extend(self.detector.detect_objects_in_mask(mask, color_name, depth_map))

        # Ground truth sockets
        sockets = [p for p in gt_poses if p.is_socket]

        # Draw overlays
        for peg in pegs:
            u, v = int(peg.centroid_2d[0]), int(peg.centroid_2d[1])
            X, Y, Z = peg.centroid_3d

            # Draw contour
            cv2.drawContours(vis, [peg.contour], -1, (0, 255, 255), 2)
            cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)

            # Draw orientation arrow
            rad = np.radians(peg.yaw_deg)
            arrow_len = 30
            end_u = int(u + arrow_len * np.cos(rad))
            end_v = int(v + arrow_len * np.sin(rad))
            cv2.arrowedLine(vis, (u, v), (end_u, end_v), (255, 255, 0), 2, tipLength=0.3)

            # Label
            label = f"{peg.shape.upper()} ({peg.color})"
            pos_label = f"({X:.0f}, {Y:.0f}, {Z:.0f})mm"
            cv2.putText(vis, label, (u - 40, v - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            cv2.putText(vis, label, (u - 40, v - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(vis, pos_label, (u - 45, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1)

            # Match to corresponding socket
            matching_socket = next((s for s in sockets if s.shape == peg.shape), None)
            if matching_socket:
                su, sv = int(matching_socket.centroid_2d[0]), int(matching_socket.centroid_2d[1])
                cv2.line(vis, (u, v), (su, sv), (200, 200, 200), 1, cv2.LINE_AA)
                delta_yaw = self.detector.calculate_minimal_rotation(
                    matching_socket.yaw_deg, peg.yaw_deg, peg.symmetry_order
                )
                mid_u, mid_v = (u + su) // 2, (v + sv) // 2
                cv2.putText(vis, f"dYaw:{delta_yaw:+.1f}deg", (mid_u - 20, mid_v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

        out_path = os.path.join(BASE_DIR, "tests", "output_debug_vision.png")
        cv2.imwrite(out_path, vis)
        self.assertTrue(os.path.exists(out_path))
        print(f"\n[Test Passed] Diagnostic visualization saved to: {out_path}")

if __name__ == "__main__":
    unittest.main()
