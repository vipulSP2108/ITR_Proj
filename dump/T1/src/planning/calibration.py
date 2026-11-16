import os
import yaml
import numpy as np
from typing import List, Tuple, Optional, Dict

class EyeToHandCalibrator:
    """
    Computes and manages the rigid 3D transformation (R, t) from Camera 3D coordinates
    to Robot Base 3D coordinates using the Kabsch / Umeyama algorithm (SVD).
    
    Transformation Formula:
        P_robot = R @ P_camera + t
    """
    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            config_path = os.path.join(base_dir, "config", "camera_calib.yaml")
        self.config_path = config_path
        self.R: Optional[np.ndarray] = None # 3x3 rotation matrix
        self.t: Optional[np.ndarray] = None # 3x1 translation vector in mm

        if os.path.exists(config_path):
            self.load_calibration(config_path)

    @staticmethod
    def compute_rigid_transform(camera_pts: List[Tuple[float, float, float]],
                                robot_pts: List[Tuple[float, float, float]]) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Calculates optimal rotation R and translation t minimizing ||robot_pts - (R @ camera_pts + t)||.
        Returns: (R, t, rmse_error_mm)
        """
        assert len(camera_pts) >= 3, "At least 3 non-collinear point pairs required for 3D calibration"
        assert len(camera_pts) == len(robot_pts), "Point lists must have identical lengths"

        A = np.array(camera_pts, dtype=np.float64) # Nx3
        B = np.array(robot_pts, dtype=np.float64)  # Nx3

        # 1. Compute centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        # 2. Center the vectors
        AA = A - centroid_A
        BB = B - centroid_B

        # 3. Covariance matrix
        H = AA.T @ BB

        # 4. SVD
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Special reflection case handling
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        # 5. Translation
        t = centroid_B - R @ centroid_A

        # 6. Calculate Root Mean Square Error (RMSE)
        transformed_A = (R @ A.T).T + t
        errors = np.linalg.norm(transformed_A - B, axis=1)
        rmse = float(np.sqrt(np.mean(errors**2)))

        return R, t, rmse

    def save_calibration(self, R: np.ndarray, t: np.ndarray, rmse: float, path: Optional[str] = None):
        save_path = path or self.config_path
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        data = {
            "rotation_matrix": R.tolist(),
            "translation_vector_mm": t.tolist(),
            "rmse_mm": float(rmse),
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S") if "time" in globals() else "manual"
        }
        with open(save_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        self.R = R
        self.t = t
        print(f"[Calibration] Saved camera calibration to {save_path} (RMSE: {rmse:.2f} mm)")

    def load_calibration(self, path: Optional[str] = None) -> bool:
        load_path = path or self.config_path
        if not os.path.exists(load_path):
            return False
        with open(load_path, "r") as f:
            data = yaml.safe_load(f)
            self.R = np.array(data["rotation_matrix"], dtype=np.float64)
            self.t = np.array(data["translation_vector_mm"], dtype=np.float64)
        return True

    def camera_to_robot(self, p_cam: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Transforms a 3D point (X, Y, Z) from Camera frame into Robot Base frame in mm."""
        if self.R is None or self.t is None:
            raise ValueError("Camera calibration (R, t) not loaded. Please calibrate first.")
        p = np.array(p_cam, dtype=np.float64)
        p_robot = self.R @ p + self.t
        return (float(p_robot[0]), float(p_robot[1]), float(p_robot[2]))

