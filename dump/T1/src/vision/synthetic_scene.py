import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional

@dataclass
class ObjectPose:
    shape: str         # "square", "triangle", "circle", "hexagon"
    color: str         # "red", "green", "blue", "yellow"
    centroid_2d: Tuple[float, float] # (u, v) in pixels
    centroid_3d: Tuple[float, float, float] # (X, Y, Z) in mm (camera frame)
    yaw_deg: float     # Orientation in degrees
    is_socket: bool    # True for board socket, False for peg

class CameraIntrinsics:
    def __init__(self, fx=615.0, fy=615.0, cx=320.0, cy=240.0, width=640, height=480):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height

    def deproject_pixel_to_point(self, u: float, v: float, depth_mm: float) -> Tuple[float, float, float]:
        """Convert pixel (u, v) and depth in mm to 3D camera coordinates (X, Y, Z) in mm."""
        x = (u - self.cx) * depth_mm / self.fx
        y = (v - self.cy) * depth_mm / self.fy
        z = depth_mm
        return (x, y, z)

    def project_point_to_pixel(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """Convert 3D camera coordinates (X, Y, Z) in mm to pixel (u, v)."""
        u = (x * self.fx / z) + self.cx
        v = (y * self.fy / z) + self.cy
        return (u, v)

class SyntheticSceneGenerator:
    """
    Generates synthetic RGB and 16-bit Depth frames of pegs and a socket board
    with ground-truth poses for validation prior to physical hardware testing.
    """
    def __init__(self, intrinsics: Optional[CameraIntrinsics] = None, seed: Optional[int] = 42):
        self.intrinsics = intrinsics or CameraIntrinsics()
        if seed is not None:
            np.random.seed(seed)

        # Heights in mm from camera (Z-axis pointing into table)
        self.table_depth = 500.0   # Table surface at 500 mm
        self.board_depth = 475.0   # 25 mm thick board sitting at 475 mm
        self.socket_depth = 500.0  # Socket cutouts reach table depth
        self.peg_top_depth = 400.0 # 100 mm tall pegs sitting on table (500 - 100 = 400 mm)

        # Standard BGR color mapping
        self.color_bgr = {
            "red": (30, 30, 220),
            "green": (40, 200, 40),
            "blue": (220, 50, 40),
            "yellow": (30, 215, 235),
            "board": (70, 70, 80),
            "socket_bg": (45, 45, 50),
            "table": (180, 185, 190)
        }

    def _get_regular_polygon_pts(self, cx: float, cy: float, radius: float, sides: int, angle_deg: float) -> np.ndarray:
        angle_rad = np.radians(angle_deg)
        angles = angle_rad + np.linspace(0, 2 * np.pi, sides, endpoint=False)
        xs = cx + radius * np.cos(angles)
        ys = cy + radius * np.sin(angles)
        return np.stack([xs, ys], axis=1).astype(np.int32)

    def generate_scene(self, add_noise: bool = True) -> Tuple[np.ndarray, np.ndarray, List[ObjectPose]]:
        """
        Generates RGB image, Depth map (uint16 in mm), and ground truth object poses.
        """
        w, h = self.intrinsics.width, self.intrinsics.height
        rgb = np.full((h, w, 3), self.color_bgr["table"], dtype=np.uint8)
        depth = np.full((h, w), self.table_depth, dtype=np.float32)

        poses: List[ObjectPose] = []

        # 1. Draw Socket Board (e.g. right side of workspace)
        board_x, board_y, board_w, board_h = 350, 60, 250, 360
        cv2.rectangle(rgb, (board_x, board_y), (board_x + board_w, board_y + board_h), self.color_bgr["board"], -1)
        cv2.rectangle(rgb, (board_x, board_y), (board_x + board_w, board_y + board_h), (40, 40, 45), 2)
        depth[board_y:board_y + board_h, board_x:board_x + board_w] = self.board_depth

        # Shapes to place: square, triangle, circle, hexagon
        shapes = ["square", "triangle", "circle", "hexagon"]
        colors = ["red", "green", "blue", "yellow"]
        radius = 28 # ~4.5 cm in image projection at 500mm depth

        # Fixed relative socket locations on the board
        socket_centers = [
            (board_x + 65, board_y + 80),
            (board_x + 185, board_y + 80),
            (board_x + 65, board_y + 260),
            (board_x + 185, board_y + 260),
        ]
        socket_angles = [15.0, 40.0, 0.0, 25.0]

        for i, shape in enumerate(shapes):
            scx, scy = socket_centers[i]
            s_angle = socket_angles[i]
            s_color = colors[i]

            # Render socket cutout (depressed into board)
            mask = np.zeros((h, w), dtype=np.uint8)
            if shape == "circle":
                cv2.circle(rgb, (scx, scy), radius, self.color_bgr["socket_bg"], -1)
                cv2.circle(mask, (scx, scy), radius, 255, -1)
            elif shape == "square":
                pts = self._get_regular_polygon_pts(scx, scy, radius * 1.25, 4, s_angle)
                cv2.fillPoly(rgb, [pts], self.color_bgr["socket_bg"])
                cv2.fillPoly(mask, [pts], 255)
            elif shape == "triangle":
                pts = self._get_regular_polygon_pts(scx, scy, radius * 1.3, 3, s_angle)
                cv2.fillPoly(rgb, [pts], self.color_bgr["socket_bg"])
                cv2.fillPoly(mask, [pts], 255)
            elif shape == "hexagon":
                pts = self._get_regular_polygon_pts(scx, scy, radius * 1.15, 6, s_angle)
                cv2.fillPoly(rgb, [pts], self.color_bgr["socket_bg"])
                cv2.fillPoly(mask, [pts], 255)

            depth[mask > 0] = self.socket_depth

            # Calculate 3D position
            p3d = self.intrinsics.deproject_pixel_to_point(scx, scy, self.board_depth)
            poses.append(ObjectPose(
                shape=shape,
                color=s_color,
                centroid_2d=(float(scx), float(scy)),
                centroid_3d=p3d,
                yaw_deg=s_angle,
                is_socket=True
            ))

        # 2. Draw Pegs (pickup area on left side of workspace)
        peg_centers = [
            (100, 100),
            (220, 120),
            (110, 280),
            (210, 320)
        ]
        # Peg orientations (different from socket orientations to test alignment)
        peg_angles = [65.0, 110.0, 0.0, 75.0]

        for i, shape in enumerate(shapes):
            pcx, pcy = peg_centers[i]
            p_angle = peg_angles[i]
            color_name = colors[i]
            color_val = self.color_bgr[color_name]

            mask = np.zeros((h, w), dtype=np.uint8)
            if shape == "circle":
                cv2.circle(rgb, (pcx, pcy), radius, color_val, -1)
                cv2.circle(mask, (pcx, pcy), radius, 255, -1)
                # Subtle rim / specular highlight
                cv2.circle(rgb, (pcx, pcy), radius, (20, 20, 20), 1)
            elif shape == "square":
                pts = self._get_regular_polygon_pts(pcx, pcy, radius * 1.25, 4, p_angle)
                cv2.fillPoly(rgb, [pts], color_val)
                cv2.polylines(rgb, [pts], True, (20, 20, 20), 1)
                cv2.fillPoly(mask, [pts], 255)
            elif shape == "triangle":
                pts = self._get_regular_polygon_pts(pcx, pcy, radius * 1.3, 3, p_angle)
                cv2.fillPoly(rgb, [pts], color_val)
                cv2.polylines(rgb, [pts], True, (20, 20, 20), 1)
                cv2.fillPoly(mask, [pts], 255)
            elif shape == "hexagon":
                pts = self._get_regular_polygon_pts(pcx, pcy, radius * 1.15, 6, p_angle)
                cv2.fillPoly(rgb, [pts], color_val)
                cv2.polylines(rgb, [pts], True, (20, 20, 20), 1)
                cv2.fillPoly(mask, [pts], 255)

            # Peg top depth (pegs stand ~100mm high)
            depth[mask > 0] = self.peg_top_depth

            p3d = self.intrinsics.deproject_pixel_to_point(pcx, pcy, self.peg_top_depth)
            poses.append(ObjectPose(
                shape=shape,
                color=color_name,
                centroid_2d=(float(pcx), float(pcy)),
                centroid_3d=p3d,
                yaw_deg=p_angle,
                is_socket=False
            ))

        # 3. Add Realistic Noise
        if add_noise:
            # Subtle lighting vignette / gradient
            y_coords, x_coords = np.mgrid[0:h, 0:w]
            gradient = 1.0 - 0.15 * ((x_coords - w/2)**2 + (y_coords - h/2)**2) / ((w/2)**2 + (h/2)**2)
            gradient = np.clip(gradient, 0.8, 1.0)[:, :, np.newaxis]
            rgb = np.clip(rgb.astype(np.float32) * gradient, 0, 255).astype(np.uint8)

            # Gaussian color noise
            noise = np.random.normal(0, 3.5, rgb.shape).astype(np.float32)
            rgb = np.clip(rgb.astype(np.float32) + noise, 0, 255).astype(np.uint8)

            # Depth sensor noise: edge dropout (convert boundary pixels to 0 / invalid)
            edges = cv2.Canny(rgb, 50, 150)
            dilated_edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
            depth[dilated_edges > 0] = 0.0  # RealSense zero-depth dropout on sharp depth discontinuities

            # Small random depth fluctuations (+- 1.5mm)
            depth_noise = np.random.normal(0, 1.2, depth.shape).astype(np.float32)
            valid_mask = depth > 0
            depth[valid_mask] += depth_noise[valid_mask]

        depth_uint16 = np.clip(depth, 0, 65535).astype(np.uint16)
        return rgb, depth_uint16, poses

if __name__ == "__main__":
    gen = SyntheticSceneGenerator()
    rgb, depth_u16, gt_poses = gen.generate_scene(add_noise=True)
    cv2.imwrite("synthetic_rgb_sample.png", rgb)
    # Save normalized depth for viewing
    depth_vis = cv2.normalize(depth_u16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    cv2.imwrite("synthetic_depth_sample.png", depth_vis)
    print(f"Generated synthetic scene with {len(gt_poses)} objects.")
    for p in gt_poses:
        print(f" - {'Socket' if p.is_socket else 'Peg   '} {p.shape:8s} ({p.color:6s}): 2D={p.centroid_2d}, 3D={p.centroid_3d}, Yaw={p.yaw_deg:.1f}°")

