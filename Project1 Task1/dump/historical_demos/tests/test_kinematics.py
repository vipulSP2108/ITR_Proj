import os
import sys
import unittest
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from src.planning.kinematics import OpenManipulatorKinematics

class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.kin = OpenManipulatorKinematics()

    def test_forward_and_inverse_roundtrip(self):
        """
        Validates that IK followed by FK accurately reconstructs Cartesian targets.
        """
        # Test targets within realistic tabletop reach (150 ~ 220 mm radial distance, 40 ~ 100 mm height)
        test_points = [
            (180.0, 0.0, 60.0),
            (170.0, 40.0, 80.0),
            (160.0, -50.0, 50.0),
            (190.0, -20.0, 70.0),
            (150.0, 60.0, 50.0),
        ]
        top_grasp_pitch = -np.pi / 2.0 # -90 degrees (pointing straight down)

        for tx, ty, tz in test_points:
            sol = self.kin.inverse_kinematics(tx, ty, tz, pitch_rad=top_grasp_pitch)
            self.assertIsNotNone(sol, f"Target ({tx}, {ty}, {tz}) should be reachable")

            q1, q2, q3, q4 = sol
            fx, fy, fz, fpitch = self.kin.forward_kinematics(q1, q2, q3, q4)

            # Assert Cartesian position error is less than 0.2 mm
            pos_err = np.sqrt((tx - fx)**2 + (ty - fy)**2 + (tz - fz)**2)
            self.assertLess(pos_err, 0.2, f"Roundtrip error {pos_err:.3f} mm exceeded tolerance at ({tx}, {ty}, {tz})")

            # Assert pitch angle error is less than 0.01 rad (~0.5 deg)
            pitch_err = abs(top_grasp_pitch - fpitch)
            self.assertLess(pitch_err, 0.01, f"Pitch error {pitch_err:.4f} rad exceeded tolerance")

    def test_unreachable_target_handling(self):
        """
        Validates that points beyond mechanical reach return None gracefully.
        """
        # Point way too far (1000 mm)
        sol = self.kin.inverse_kinematics(1000.0, 0.0, 100.0)
        self.assertIsNone(sol, "Point at 1000mm must be unreachable")

        # Point inside base cylinder (0, 0, 0)
        sol_inside = self.kin.inverse_kinematics(10.0, 0.0, 0.0)
        self.assertIsNone(sol_inside, "Point at origin must be unreachable")

    def test_encoder_count_conversion(self):
        """
        Tests conversion between radians and raw 0-4095 encoder units.
        """
        zero_radians = [0.0, 0.0, 0.0, 0.0]
        raw_counts = self.kin.radians_to_raw(zero_radians, base_wrap_count=4096)
        self.assertEqual(raw_counts, [4096, 2048, 2048, 2048])

        back_radians = self.kin.raw_to_radians(raw_counts)
        for r in back_radians[1:]: # Joints 2-4
            self.assertAlmostEqual(r, 0.0, delta=0.001)

if __name__ == "__main__":
    unittest.main()
