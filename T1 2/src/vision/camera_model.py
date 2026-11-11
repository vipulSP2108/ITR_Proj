"""
Calibrated camera <-> robot mapping (loads config/camera_robot_calib.yaml written by
calibration/01_calibrate_camera_robot.py).

    cm = CameraModel()
    x, y = cm.pixel_to_robot(u, v, z=cm.table_z + 40)   # a point 40 mm above the mat
    u, v = cm.robot_to_pixel(x, y, z)

The camera is a pinhole with rotation matrix R and translation t (robot frame -> camera frame):
    p_cam = R @ p_robot + t ,   u = f * p_cam.x / p_cam.z + cx ,   v = f * p_cam.y / p_cam.z + cy
pixel_to_robot needs the HEIGHT of the thing you look at: the camera is only ~0.5 m up, so the
top of a 40 mm peg appears shifted by several mm compared with the table under it.
"""
import os
import numpy as np
import cv2
import yaml

DEFAULT_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                            "config", "camera_robot_calib.yaml")


class CameraModel:
    def __init__(self, path: str = DEFAULT_YAML):
        if not os.path.exists(path):
            raise FileNotFoundError(f"{path} missing - run ./run_calibration.sh first")
        with open(path) as f:
            c = yaml.safe_load(f)
        self.cfg = c
        self.f, self.cx, self.cy = c["focal_px"], c["cx"], c["cy"]
        self.R, _ = cv2.Rodrigues(np.array(c["rvec_robot_to_cam"], float))
        self.t = np.array(c["tvec_robot_to_cam_mm"], float)
        self.cam_pos = -self.R.T @ self.t
        self.table_z = float(c["table_z_mm"])
        self.xy_corr = np.array(c.get("xy_correction_mm", [0.0, 0.0]), float)
        self.width, self.height = c["image_size"]

    def robot_to_pixel(self, x, y, z=None):
        z = self.table_z if z is None else z
        p = self.R @ (np.array([x, y, z], float) - np.append(self.xy_corr, 0.0)) + self.t
        return float(self.f * p[0] / p[2] + self.cx), float(self.f * p[1] / p[2] + self.cy)

    def pixel_to_robot(self, u, v, z=None):
        """Robot XY (mm) of the point seen at pixel (u, v) that lies at robot height z (default: mat surface)."""
        z = self.table_z if z is None else z
        d = self.R.T @ np.array([(u - self.cx) / self.f, (v - self.cy) / self.f, 1.0])
        p = self.cam_pos + (z - self.cam_pos[2]) / d[2] * d
        return float(p[0] + self.xy_corr[0]), float(p[1] + self.xy_corr[1])

    def mm_per_px(self, z=None):
        z = self.table_z if z is None else z
        return (self.cam_pos[2] - z) / self.f
