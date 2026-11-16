"""
Real-time trajectory visualization overlays for the hardware pipeline.

Draws on camera frames:
- Planned trajectory path
- Current and completed waypoints
- Peg/socket detection annotations
- Gripper state indicator
- Robot base marker and reachability circle
"""

import cv2
import math
import numpy as np
from typing import List, Tuple, Optional, Dict


class TrajectoryVisualizer:
    """Draws real-time overlays on camera frames during trajectory execution."""

    COLORS = {
        "path_planned": (255, 200, 0),      # cyan-ish
        "path_completed": (0, 255, 0),       # green
        "waypoint_pending": (180, 180, 180), # gray
        "waypoint_active": (0, 255, 255),    # yellow
        "waypoint_done": (0, 200, 0),        # dark green
        "obstacle": (0, 0, 255),             # red
        "gripper_open": (0, 255, 0),
        "gripper_closed": (255, 0, 255),
        "base": (0, 0, 255),
        "reach_circle": (60, 60, 60),
    }

    def __init__(self, robot_to_pixel_fn, base_u: float, base_v: float, scale: float):
        self.robot_to_pixel = robot_to_pixel_fn
        self.base_u = base_u
        self.base_v = base_v
        self.scale = scale

    def draw_full_overlay(self, frame: np.ndarray,
                           waypoints: list,
                           current_step: int,
                           peg_shape: str = "",
                           peg_color: str = "",
                           gripper_open: bool = True,
                           obstacles: Optional[List[Tuple[int, int]]] = None) -> np.ndarray:
        """Draw complete trajectory overlay on a frame."""
        overlay = frame.copy()

        self._draw_base_and_reach(overlay)
        self._draw_trajectory_path(overlay, waypoints, current_step)
        self._draw_waypoint_markers(overlay, waypoints, current_step)
        self._draw_hud(overlay, current_step, len(waypoints), peg_shape, peg_color, gripper_open)

        if obstacles:
            for ox, oy in obstacles:
                cv2.circle(overlay, (ox, oy), 12, self.COLORS["obstacle"], 2)
                cv2.putText(overlay, "!", (ox - 4, oy + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS["obstacle"], 2)

        return overlay

    def _draw_base_and_reach(self, frame: np.ndarray):
        bu, bv = int(self.base_u), int(self.base_v)
        cv2.circle(frame, (bu, bv), 6, self.COLORS["base"], -1)

        for radius_mm, label in [(215, "V"), (290, "A"), (340, "S")]:
            r_px = int(radius_mm / self.scale)
            cv2.circle(frame, (bu, bv), r_px, self.COLORS["reach_circle"], 1)

    def _draw_trajectory_path(self, frame: np.ndarray, waypoints: list, current_step: int):
        points = []
        for wp in waypoints:
            px = self.robot_to_pixel(wp.x, wp.y)
            points.append((int(px[0]), int(px[1])))

        for i in range(len(points) - 1):
            color = self.COLORS["path_completed"] if i < current_step else self.COLORS["path_planned"]
            thickness = 2 if i < current_step else 1
            cv2.line(frame, points[i], points[i + 1], color, thickness)

    def _draw_waypoint_markers(self, frame: np.ndarray, waypoints: list, current_step: int):
        for i, wp in enumerate(waypoints):
            px = self.robot_to_pixel(wp.x, wp.y)
            pt = (int(px[0]), int(px[1]))

            if i < current_step:
                color = self.COLORS["waypoint_done"]
                cv2.circle(frame, pt, 5, color, -1)
            elif i == current_step:
                color = self.COLORS["waypoint_active"]
                cv2.circle(frame, pt, 8, color, 2)
                cv2.circle(frame, pt, 3, color, -1)
            else:
                color = self.COLORS["waypoint_pending"]
                cv2.circle(frame, pt, 4, color, 1)

            short_name = wp.name.split()[0] if hasattr(wp, 'name') else str(i)
            cv2.putText(frame, short_name, (pt[0] + 10, pt[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

    def _draw_hud(self, frame: np.ndarray, step: int, total: int,
                   shape: str, color: str, gripper_open: bool):
        h, w = frame.shape[:2]

        cv2.rectangle(frame, (0, 0), (w, 32), (0, 0, 0), -1)

        step_text = f"Step {step + 1}/{total}"
        cv2.putText(frame, step_text, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        shape_text = f"{shape} ({color})"
        cv2.putText(frame, shape_text, (160, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        grip_text = "OPEN" if gripper_open else "HOLD"
        grip_color = self.COLORS["gripper_open"] if gripper_open else self.COLORS["gripper_closed"]
        cv2.putText(frame, f"Grip: {grip_text}", (400, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, grip_color, 1)

        cv2.rectangle(frame, (w - 120, 2), (w - 2, 30), (40, 40, 40), -1)
        pct = int((step / max(total, 1)) * 100)
        bar_w = int(116 * step / max(total, 1))
        cv2.rectangle(frame, (w - 118, 4), (w - 118 + bar_w, 28), (0, 200, 0), -1)
        cv2.putText(frame, f"{pct}%", (w - 90, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
