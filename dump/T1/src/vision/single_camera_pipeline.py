import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from src.vision.synthetic_scene import CameraIntrinsics
from src.vision.color_segmentation import ColorSegmenter
from src.vision.shape_detection import ShapeDetector, DetectedObject
from src.planning.calibration import EyeToHandCalibrator

@dataclass
class TargetPose:
    shape: str                           # "square", "triangle", "circle", "hexagon"
    color: str                           # color identifier
    centroid_2d: Tuple[float, float]     # (u, v) in image pixels
    camera_3d: Tuple[float, float, float]# (X, Y, Z) in mm (Camera Frame)
    robot_3d: Tuple[float, float, float] # (X, Y, Z) in mm (Robot Base Frame)
    yaw_deg: float                       # Orientation in degrees
    is_socket: bool                      # True for socket, False for peg

class SingleCameraPipeline:
    """
    Complete perception pipeline for the single overhead/oblique RealSense camera:
      1. HSV Color Segmentation
      2. Depth Slicing to isolate the top polygon face from perspective side-walls
      3. Shape classification (Square, Triangle, Hexagon, Circle) & Yaw estimation
      4. Socket board cutout localization
      5. Deprojection and Transformation to Robot Base Frame (X_r, Y_r, Z_r)
    """
    def __init__(self, intrinsics: Optional[CameraIntrinsics] = None,
                 calibrator: Optional[EyeToHandCalibrator] = None,
                 colors_config_path: Optional[str] = None):
        self.intrinsics = intrinsics or CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0)
        self.calibrator = calibrator or EyeToHandCalibrator()
        self.segmenter = ColorSegmenter(colors_config_path)
        self.detector = ShapeDetector(intrinsics=self.intrinsics)

    def depth_slice_top_face(self, mask: np.ndarray, depth_map: np.ndarray,
                             slice_thickness_mm: float = 20.0) -> np.ndarray:
        """
        Removes perspective side-wall distortion by taking only depth pixels within
        [Z_min, Z_min + slice_thickness_mm], isolating the pure top polygon face.
        """
        if depth_map is None:
            return mask

        # Extract depths belonging to this color mask
        masked_depths = depth_map[mask > 0]
        valid_depths = masked_depths[masked_depths > 0]
        if len(valid_depths) == 0:
            return mask

        # Z_min is the top face (closest to camera)
        z_min = float(np.percentile(valid_depths, 5.0)) # 5th percentile rejects outlier noise
        z_max = z_min + slice_thickness_mm

        # Keep only the top face slice
        top_slice_mask = np.zeros_like(mask)
        in_slice = (mask > 0) & (depth_map >= z_min) & (depth_map <= z_max)
        top_slice_mask[in_slice] = 255

        # Clean with morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        top_slice_mask = cv2.morphologyEx(top_slice_mask, cv2.MORPH_CLOSE, kernel)
        return top_slice_mask

    def process_scene(self, rgb_image: np.ndarray, depth_map: Optional[np.ndarray] = None,
                      board_roi: Optional[Tuple[int, int, int, int]] = None) -> Tuple[List[TargetPose], List[TargetPose]]:
        """
        Processes a single camera snapshot.
        Returns: (detected_pegs, detected_sockets) with 3D robot base coordinates.
        """
        detected_pegs: List[TargetPose] = []
        detected_sockets: List[TargetPose] = []

        # 1. Detect Pegs using Color Segmentation + Depth Slicing
        color_masks = self.segmenter.segment_all(rgb_image)

        for color_name, raw_mask in color_masks.items():
            # Apply depth slicing if depth map is available
            sliced_mask = raw_mask
            if depth_map is not None:
                sliced_mask = self.depth_slice_top_face(raw_mask, depth_map)

            objs = self.detector.detect_objects_in_mask(sliced_mask, color_name, depth_map)
            for obj in objs:
                # Coordinate transformation to Robot Base Frame
                x_r, y_r, z_r = 0.0, 0.0, 0.0
                if self.calibrator.R is not None and obj.centroid_3d[2] > 0:
                    x_r, y_r, z_r = self.calibrator.camera_to_robot(obj.centroid_3d)

                detected_pegs.append(TargetPose(
                    shape=obj.shape,
                    color=color_name,
                    centroid_2d=obj.centroid_2d,
                    camera_3d=obj.centroid_3d,
                    robot_3d=(x_r, y_r, z_r),
                    yaw_deg=obj.yaw_deg,
                    is_socket=False
                ))

        # 2. Detect Sockets on the Board
        socket_objs = self.detector.detect_sockets(rgb_image, depth_map, board_roi=board_roi)
        for s_obj in socket_objs:
            sx_r, sy_r, sz_r = 0.0, 0.0, 0.0
            if self.calibrator.R is not None and s_obj.centroid_3d[2] > 0:
                sx_r, sy_r, sz_r = self.calibrator.camera_to_robot(s_obj.centroid_3d)

            detected_sockets.append(TargetPose(
                shape=s_obj.shape,
                color="socket",
                centroid_2d=s_obj.centroid_2d,
                camera_3d=s_obj.centroid_3d,
                robot_3d=(sx_r, sy_r, sz_r),
                yaw_deg=s_obj.yaw_deg,
                is_socket=True
            ))

        return detected_pegs, detected_sockets

