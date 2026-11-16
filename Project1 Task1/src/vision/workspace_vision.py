"""
Unified workspace vision module — detects all 4 peg shapes and all board sockets
dynamically with ZERO hardcoded pixel coordinates.

Uses ColorSegmenter for peg detection and Otsu+shape classification for socket
detection. Converts pixel coords to robot-frame coords via overhead pinhole model
calibrated from workspace_dimensions.yaml or homography.
"""

import os
import cv2
import yaml
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict

from src.vision.color_segmentation import ColorSegmenter
from src.vision.shape_detection import ShapeDetector, DetectedObject


@dataclass
class WorkspaceObject:
    shape: str          # "circle", "square", "hexagon", "triangle"
    color: str          # color name or "socket"
    pixel_uv: Tuple[float, float]
    robot_xy: Tuple[float, float]   # (x_mm, y_mm) in robot base frame
    robot_z: float                  # z_mm for manipulation
    yaw_deg: float
    symmetry_order: int
    confidence: float
    contour: np.ndarray
    area_px: float = 0.0


@dataclass
class WorkspaceScan:
    pegs: List[WorkspaceObject] = field(default_factory=list)
    sockets: List[WorkspaceObject] = field(default_factory=list)
    board_roi: Optional[Tuple[int, int, int, int]] = None
    annotated_frame: Optional[np.ndarray] = None
    assignments: Dict[str, Tuple[WorkspaceObject, WorkspaceObject]] = field(default_factory=dict)


class WorkspaceVision:
    """
    Full workspace perception with no hardcoded coordinates.

    Pixel-to-robot conversion uses the overhead pinhole model:
        x_rob = (u_base - u) * scale
        y_rob = (v_base - v) * scale
    where u_base, v_base is the robot base pixel position and
    scale = camera_height / focal_length (mm/px).

    If a homography matrix is available (from calibration), it is used instead
    for higher accuracy.
    """

    def __init__(self, config_dir: Optional[str] = None):
        if config_dir is None:
            config_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
        config_dir = os.path.abspath(config_dir)

        self.segmenter = ColorSegmenter(os.path.join(config_dir, 'colors.yaml'))
        self.detector = ShapeDetector(min_area=400, max_area=60000)

        ws_path = os.path.join(config_dir, 'workspace_dimensions.yaml')
        with open(ws_path, 'r') as f:
            ws = yaml.safe_load(f)

        cam = ws['camera']
        self.cam_height = cam['height_above_board_mm']
        self.focal_px = cam['focal_length_px']
        self.scale = self.cam_height / self.focal_px
        self.img_w = cam['sensor_width_px']
        self.img_h = cam['sensor_height_px']

        mount = ws['robot_mounting']['base_center_pixel_estimate']
        self.base_u = mount['u']
        self.base_v = mount['v']

        heights = ws['manipulation_heights']
        self.z_hover = heights['z_hover_mm']
        self.z_grasp = heights['z_grasp_default_mm']
        self.z_transit = heights['z_transit_mm']
        self.z_insert = heights['z_insert_default_mm']
        self.z_sock_hover = heights['z_sock_hover_mm']

        self.color_shape_map = {}
        colors_path = os.path.join(config_dir, 'colors.yaml')
        with open(colors_path, 'r') as f:
            cdata = yaml.safe_load(f)
        for cname, cinfo in cdata.get('colors', {}).items():
            if 'shape' in cinfo:
                self.color_shape_map[cname] = cinfo['shape']

        self.homography = None
        hom_path = os.path.join(config_dir, 'homography_calib.yaml')
        if os.path.exists(hom_path):
            try:
                with open(hom_path, 'r') as f:
                    hdata = yaml.safe_load(f)
                if hdata and 'H' in hdata:
                    self.homography = np.array(hdata['H'], dtype=np.float64).reshape(3, 3)
            except Exception:
                pass

    def pixel_to_robot(self, u: float, v: float) -> Tuple[float, float]:
        """Convert pixel (u,v) to robot base frame (x_mm, y_mm)."""
        if self.homography is not None:
            pt = np.array([u, v, 1.0])
            rob = self.homography @ pt
            rob /= rob[2]
            return float(rob[0]), float(rob[1])
        x_rob = (self.base_u - u) * self.scale
        y_rob = (self.base_v - v) * self.scale
        return x_rob, y_rob

    def robot_to_pixel(self, x_rob: float, y_rob: float) -> Tuple[float, float]:
        """Convert robot (x_mm, y_mm) back to pixel (u,v)."""
        if self.homography is not None:
            H_inv = np.linalg.inv(self.homography)
            pt = np.array([x_rob, y_rob, 1.0])
            px = H_inv @ pt
            px /= px[2]
            return float(px[0]), float(px[1])
        u = self.base_u - x_rob / self.scale
        v = self.base_v - y_rob / self.scale
        return u, v

    def detect_pegs(self, frame: np.ndarray) -> List[WorkspaceObject]:
        """Detect all colored pegs in the frame using color segmentation + shape classification."""
        masks = self.segmenter.segment_all(frame)
        pegs = []

        for color_name, mask in masks.items():
            objects = self.detector.detect_objects_in_mask(mask, color_name)
            for obj in objects:
                u, v = obj.centroid_2d
                x_rob, y_rob = self.pixel_to_robot(u, v)
                expected_shape = self.color_shape_map.get(color_name, obj.shape)
                pegs.append(WorkspaceObject(
                    shape=expected_shape,
                    color=color_name,
                    pixel_uv=(u, v),
                    robot_xy=(x_rob, y_rob),
                    robot_z=self.z_grasp,
                    yaw_deg=obj.yaw_deg,
                    symmetry_order=obj.symmetry_order,
                    confidence=obj.confidence,
                    contour=obj.contour,
                    area_px=cv2.contourArea(obj.contour),
                ))
        return pegs

    def detect_board(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        Detect the wooden sorting board dynamically.
        Returns (x, y, w, h) bounding box or None.
        Uses grayscale + adaptive threshold to find a large rectangular region.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_board = None
        best_area = 0
        min_board_area = 3000
        max_board_area = self.img_w * self.img_h * 0.5

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_board_area or area > max_board_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = max(w, h) / max(min(w, h), 1)
            if 1.5 < aspect < 6.0 and area > best_area:
                best_area = area
                best_board = (x, y, w, h)

        return best_board

    def detect_sockets(self, frame: np.ndarray,
                       board_roi: Optional[Tuple[int, int, int, int]] = None) -> List[WorkspaceObject]:
        """Detect socket cutouts on the board using shape detection."""
        socket_objs = self.detector.detect_sockets(frame, board_roi=board_roi)
        sockets = []

        for obj in socket_objs:
            u, v = obj.centroid_2d
            x_rob, y_rob = self.pixel_to_robot(u, v)
            sockets.append(WorkspaceObject(
                shape=obj.shape,
                color="socket",
                pixel_uv=(u, v),
                robot_xy=(x_rob, y_rob),
                robot_z=self.z_insert,
                yaw_deg=obj.yaw_deg,
                symmetry_order=obj.symmetry_order,
                confidence=obj.confidence,
                contour=obj.contour,
                area_px=cv2.contourArea(obj.contour),
            ))
        return sockets

    def match_pegs_to_sockets(self, pegs: List[WorkspaceObject],
                               sockets: List[WorkspaceObject]) -> Dict[str, Tuple[WorkspaceObject, WorkspaceObject]]:
        """
        Match each peg to its corresponding socket by shape.
        Returns dict keyed by shape name → (peg, socket).
        """
        assignments = {}
        socket_by_shape = {}
        for s in sockets:
            socket_by_shape.setdefault(s.shape, []).append(s)

        for peg in pegs:
            candidates = socket_by_shape.get(peg.shape, [])
            if candidates:
                socket = candidates.pop(0)
                assignments[peg.shape] = (peg, socket)

        return assignments

    def full_scan(self, frame: np.ndarray) -> WorkspaceScan:
        """
        Complete workspace scan: detect all pegs, board, sockets, and match them.
        """
        pegs = self.detect_pegs(frame)
        board_roi = self.detect_board(frame)
        sockets = self.detect_sockets(frame, board_roi)
        assignments = self.match_pegs_to_sockets(pegs, sockets)

        annotated = self.draw_annotations(frame.copy(), pegs, sockets, assignments, board_roi)

        return WorkspaceScan(
            pegs=pegs,
            sockets=sockets,
            board_roi=board_roi,
            annotated_frame=annotated,
            assignments=assignments,
        )

    def quick_detect_obstacles(self, frame: np.ndarray,
                                arm_pixel_path: List[Tuple[int, int]],
                                corridor_width_px: int = 40) -> List[Tuple[int, int]]:
        """
        Check if any large dark object (camera stand, obstacles) intersects the
        planned arm trajectory corridor in pixel space.
        Returns list of obstacle pixel positions found in the corridor.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, dark_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)

        corridor_mask = np.zeros_like(dark_mask)
        if len(arm_pixel_path) >= 2:
            pts = np.array(arm_pixel_path, dtype=np.int32)
            cv2.polylines(corridor_mask, [pts], False, 255, corridor_width_px)

        overlap = cv2.bitwise_and(dark_mask, corridor_mask)
        obstacle_points = []
        contours, _ = cv2.findContours(overlap, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 200:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    obstacle_points.append((cx, cy))

        return obstacle_points

    def verify_peg_at_location(self, frame: np.ndarray,
                                expected_shape: str,
                                expected_pixel: Tuple[float, float],
                                search_radius_px: int = 50) -> Optional[WorkspaceObject]:
        """
        Re-detect a specific peg near an expected pixel location.
        Used for vision verification during trajectory execution.
        """
        pegs = self.detect_pegs(frame)
        eu, ev = expected_pixel
        best = None
        best_dist = float('inf')

        for p in pegs:
            if p.shape != expected_shape:
                continue
            pu, pv = p.pixel_uv
            dist = np.sqrt((pu - eu)**2 + (pv - ev)**2)
            if dist < search_radius_px and dist < best_dist:
                best_dist = dist
                best = p

        return best

    def verify_socket_empty(self, frame: np.ndarray,
                             socket: WorkspaceObject,
                             search_radius_px: int = 40) -> bool:
        """
        Check if a socket appears empty (no peg color detected near it).
        """
        pegs = self.detect_pegs(frame)
        su, sv = socket.pixel_uv
        for p in pegs:
            pu, pv = p.pixel_uv
            if np.sqrt((pu - su)**2 + (pv - sv)**2) < search_radius_px:
                return False
        return True

    def draw_annotations(self, frame: np.ndarray,
                          pegs: List[WorkspaceObject],
                          sockets: List[WorkspaceObject],
                          assignments: Dict,
                          board_roi: Optional[Tuple[int, int, int, int]] = None) -> np.ndarray:
        """Draw detection overlays on the frame."""
        color_map = {
            "green": (0, 255, 0), "blue": (255, 0, 0),
            "red": (0, 0, 255), "yellow": (0, 255, 255),
            "socket": (200, 200, 200),
        }

        if board_roi:
            bx, by, bw, bh = board_roi
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (128, 128, 0), 2)
            cv2.putText(frame, "BOARD", (bx, by - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1)

        for peg in pegs:
            u, v = int(peg.pixel_uv[0]), int(peg.pixel_uv[1])
            clr = color_map.get(peg.color, (255, 255, 255))
            cv2.drawContours(frame, [peg.contour], -1, clr, 2)
            cv2.circle(frame, (u, v), 4, clr, -1)
            x_r, y_r = peg.robot_xy
            label = f"{peg.shape}({x_r:.0f},{y_r:.0f})"
            cv2.putText(frame, label, (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, clr, 1)

        for sock in sockets:
            u, v = int(sock.pixel_uv[0]), int(sock.pixel_uv[1])
            cv2.drawContours(frame, [sock.contour], -1, (200, 200, 200), 2)
            x_r, y_r = sock.robot_xy
            label = f"[{sock.shape}]({x_r:.0f},{y_r:.0f})"
            cv2.putText(frame, label, (u + 8, v + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        for shape_name, (peg, sock) in assignments.items():
            pu, pv = int(peg.pixel_uv[0]), int(peg.pixel_uv[1])
            su, sv = int(sock.pixel_uv[0]), int(sock.pixel_uv[1])
            cv2.arrowedLine(frame, (pu, pv), (su, sv), (0, 255, 255), 1, tipLength=0.08)

        cv2.circle(frame, (int(self.base_u), int(self.base_v)), 6, (0, 0, 255), -1)
        cv2.putText(frame, "BASE", (int(self.base_u) + 8, int(self.base_v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        return frame
