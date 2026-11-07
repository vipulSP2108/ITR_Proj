import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
from src.vision.synthetic_scene import CameraIntrinsics

@dataclass
class DetectedObject:
    shape: str              # "square", "triangle", "circle", "hexagon", or "unknown"
    color: str              # color name
    centroid_2d: Tuple[float, float] # (u, v) in pixels
    centroid_3d: Tuple[float, float, float] # (X, Y, Z) in mm (camera frame)
    yaw_deg: float          # Raw detected orientation in degrees [0, 360)
    contour: np.ndarray     # Contour points
    confidence: float       # Classification confidence [0.0 - 1.0]
    symmetry_order: int     # 1 for asymmetric, 3 for triangle, 4 for square, 6 for hex, 0 for circle

class ShapeDetector:
    """
    Classifies geometric shapes (Square, Triangle, Circle, Hexagon),
    estimates 3D centroids with depth-noise rejection, and computes
    orientation with rotational symmetry awareness.
    """
    def __init__(self, intrinsics: Optional[CameraIntrinsics] = None,
                 min_area: float = 400.0, max_area: float = 60000.0):
        self.intrinsics = intrinsics or CameraIntrinsics()
        self.min_area = min_area
        self.max_area = max_area

        # Symmetry orders for normalization
        self.symmetry_orders = {
            "circle": 0,    # Continuous rotational symmetry
            "square": 4,    # 90-degree symmetry
            "hexagon": 6,   # 60-degree symmetry
            "triangle": 3,  # 120-degree symmetry
            "unknown": 1
        }

    def _get_robust_depth_at_centroid(self, depth_map: np.ndarray, u: int, v: int, patch_radius: int = 3) -> float:
        """
        Samples a central patch around (u, v) and returns the median of non-zero depths
        to reject edge noise and sensor dropouts.
        """
        h, w = depth_map.shape
        u_min = max(0, u - patch_radius)
        u_max = min(w, u + patch_radius + 1)
        v_min = max(0, v - patch_radius)
        v_max = min(h, v + patch_radius + 1)

        patch = depth_map[v_min:v_max, u_min:u_max]
        valid_depths = patch[patch > 0]

        if len(valid_depths) == 0:
            return 0.0
        return float(np.median(valid_depths))

    def _estimate_orientation(self, cnt: np.ndarray, shape: str, approx_pts: np.ndarray, cx: float, cy: float) -> float:
        """
        Calculates the orientation angle (in degrees [0, 360)) of the shape.
        """
        if shape == "circle":
            return 0.0

        if shape == "triangle" and len(approx_pts) == 3:
            # For triangle, find vector from centroid to apex
            # Find the vertex furthest or most distinct
            dists = [np.linalg.norm(pt[0] - np.array([cx, cy])) for pt in approx_pts]
            apex_idx = int(np.argmax(dists))
            vec = approx_pts[apex_idx][0] - np.array([cx, cy])
            angle = float(np.degrees(np.arctan2(vec[1], vec[0]))) % 360.0
            return angle

        if shape in ["square", "hexagon"]:
            # Use minimum area rectangle or primary contour axis
            rect = cv2.minAreaRect(cnt)
            (rcx, rcy), (rw, rh), angle = rect
            # OpenCV minAreaRect returns angle in [-90, 0)
            if rw < rh:
                angle += 90.0
            return float(angle % 360.0)

        # Fallback using PCA
        pts = cnt.reshape(-1, 2).astype(np.float64)
        mean, eigenvectors = cv2.PCACompute(pts, mean=None)
        primary_axis = eigenvectors[0]
        angle = float(np.degrees(np.arctan2(primary_axis[1], primary_axis[0]))) % 360.0
        return angle

    def classify_contour(self, cnt: np.ndarray) -> Tuple[str, float, np.ndarray]:
        """
        Identifies shape using enclosing circle area ratio and multi-resolution polygon approximation.
        Returns: (shape_name, confidence, approx_polygon)
        """
        area = cv2.contourArea(cnt)
        peri = cv2.arcLength(cnt, True)
        if peri == 0:
            return "unknown", 0.0, cnt

        # 1. Enclosing circle ratio: Area / (pi * R^2)
        # Circle ~ 1.0, Hexagon ~ 0.827, Square ~ 0.637, Equilateral Triangle ~ 0.413
        (center_x, center_y), radius = cv2.minEnclosingCircle(cnt)
        enclosing_area = np.pi * (radius ** 2)
        circle_ratio = (area / enclosing_area) if enclosing_area > 0 else 0.0

        # 2. Polygon approximation with tuned epsilon (2% of perimeter)
        epsilon = 0.020 * peri
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        num_vertices = len(approx)

        # Disambiguation logic:
        # A) Circle: Area/CircleArea > 0.92 and vertices >= 7
        if circle_ratio > 0.91 or (num_vertices >= 7 and circle_ratio > 0.88):
            return "circle", circle_ratio, approx

        # B) Triangle: exactly 3 vertices or very low circle ratio (< 0.58)
        if num_vertices == 3 or circle_ratio < 0.58:
            return "triangle", 0.95, approx

        # C) Square: 4 vertices and circle ratio around 0.62 - 0.75
        if num_vertices == 4 or (0.58 <= circle_ratio <= 0.76):
            return "square", 0.95, approx

        # D) Hexagon: 5 to 7 vertices and circle ratio around 0.77 - 0.90
        if num_vertices == 6 or (0.77 <= circle_ratio <= 0.91):
            return "hexagon", 0.92, approx

        return "unknown", 0.5, approx

    def detect_objects_in_mask(self, mask: np.ndarray, color_name: str,
                              depth_map: Optional[np.ndarray] = None) -> List[DetectedObject]:
        """
        Finds all shapes of a given color in a binary mask and estimates their 2D/3D poses.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            # Centroid (u, v)
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]

            # Classify shape
            shape, conf, approx = self.classify_contour(cnt)
            if shape == "unknown":
                continue

            # Orientation
            yaw_deg = self._estimate_orientation(cnt, shape, approx, cx, cy)

            # 3D Depth & Coordinates
            z_mm = 0.0
            x_mm, y_mm = 0.0, 0.0
            if depth_map is not None:
                z_mm = self._get_robust_depth_at_centroid(depth_map, int(cx), int(cy))
                if z_mm > 0:
                    x_mm, y_mm, z_mm = self.intrinsics.deproject_pixel_to_point(cx, cy, z_mm)

            detected.append(DetectedObject(
                shape=shape,
                color=color_name,
                centroid_2d=(cx, cy),
                centroid_3d=(x_mm, y_mm, z_mm),
                yaw_deg=yaw_deg,
                contour=cnt,
                confidence=conf,
                symmetry_order=self.symmetry_orders[shape]
            ))

        return detected

    def detect_sockets(self, rgb_image: np.ndarray, depth_map: Optional[np.ndarray] = None,
                       board_roi: Optional[Tuple[int, int, int, int]] = None) -> List[DetectedObject]:
        """
        Detects socket cutouts on the board using grayscale contrast / depth relief.
        board_roi: optional (x, y, w, h) bounding box of the board.
        """
        h_img, w_img = rgb_image.shape[:2]
        
        # If board ROI is specified, crop with a small inset to exclude outer boundary rims
        if board_roi:
            bx, by, bw, bh = board_roi
            inset = 6
            crop_x = max(0, bx + inset)
            crop_y = max(0, by + inset)
            crop_w = max(10, bw - 2 * inset)
            crop_h = max(10, bh - 2 * inset)
            crop_rgb = rgb_image[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w]
            gray = cv2.cvtColor(crop_rgb, cv2.COLOR_BGR2GRAY)
            offset_x, offset_y = crop_x, crop_y
        else:
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            offset_x, offset_y = 0, 0

        # Otsu threshold to separate cutouts from board surface
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Morphological opening to eliminate any thin scratch/noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_sockets = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            # Classify shape in local crop coordinates
            shape, conf, approx = self.classify_contour(cnt)
            if shape == "unknown":
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            local_cx = M["m10"] / M["m00"]
            local_cy = M["m01"] / M["m00"]

            # Orientation in local frame
            yaw_deg = self._estimate_orientation(cnt, shape, approx, local_cx, local_cy)

            # Global image coordinates
            global_cx = local_cx + offset_x
            global_cy = local_cy + offset_y
            global_cnt = cnt.copy()
            global_cnt[:, :, 0] += offset_x
            global_cnt[:, :, 1] += offset_y

            z_mm = 0.0
            x_mm, y_mm = 0.0, 0.0
            if depth_map is not None:
                z_mm = self._get_robust_depth_at_centroid(depth_map, int(global_cx), int(global_cy))
                if z_mm > 0:
                    x_mm, y_mm, z_mm = self.intrinsics.deproject_pixel_to_point(global_cx, global_cy, z_mm)

            detected_sockets.append(DetectedObject(
                shape=shape,
                color="socket",
                centroid_2d=(global_cx, global_cy),
                centroid_3d=(x_mm, y_mm, z_mm),
                yaw_deg=yaw_deg,
                contour=global_cnt,
                confidence=conf,
                symmetry_order=self.symmetry_orders[shape]
            ))

        return detected_sockets

    @staticmethod
    def calculate_minimal_rotation(target_yaw: float, current_yaw: float, symmetry_order: int) -> float:
        """
        Computes the minimal signed rotation (-period/2 to +period/2) needed
        to align current_yaw with target_yaw under symmetry_order.
        """
        if symmetry_order == 0:
            return 0.0 # Circle is rotation invariant

        period = 360.0 / symmetry_order
        half_period = period / 2.0
        diff = (target_yaw - current_yaw) % period
        if diff > half_period:
            diff -= period
        return diff
