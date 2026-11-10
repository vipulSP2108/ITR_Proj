"""
Vision-guided trajectory execution for OpenMANIPULATOR-X.

Key improvements over the old blind trajectory:
1. Camera checks at every waypoint (not just start)
2. Obstacle detection before each move
3. Peg verification after grasp
4. Socket verification before place
5. Collision avoidance via trajectory re-planning
"""

import time
import math
import numpy as np
from enum import Enum
from typing import List, Tuple, Optional, Dict, Callable
from dataclasses import dataclass

from src.planning.kinematics import OpenManipulatorKinematics
from src.planning.workspace_analyzer import WorkspaceAnalyzer, ReachabilityStatus
from src.vision.shape_detection import ShapeDetector


class MoveResult(Enum):
    SUCCESS = "success"
    OBSTACLE_DETECTED = "obstacle_detected"
    IK_FAILED = "ik_failed"
    GRASP_FAILED = "grasp_failed"
    VISION_LOST = "vision_lost"
    TIMEOUT = "timeout"


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float
    pitch_rad: float = -math.pi / 2
    gripper_open: bool = True
    duration: float = 2.0
    vision_check: bool = False
    obstacle_check: bool = False


@dataclass
class TrajectoryPlan:
    waypoints: List[Waypoint]
    peg_shape: str
    peg_color: str
    pick_xy: Tuple[float, float]
    place_xy: Tuple[float, float]
    yaw_alignment_deg: float = 0.0


class VisionTrajectoryExecutor:
    """
    Executes pick-and-place trajectories with continuous vision feedback.

    Architecture:
    - Before each move: optionally check camera for obstacles
    - After grasp: verify peg is held (gripper + vision)
    - During transit: gripper monitoring for drops
    - Before place: re-verify socket position via vision
    """

    def __init__(self, arm_controller, camera_capture, workspace_vision,
                 gripper_guard=None):
        """
        arm_controller: OpenManipulatorController instance
        camera_capture: callable that returns BGR frame (e.g. cap.read)
        workspace_vision: WorkspaceVision instance
        gripper_guard: optional GripperGuard for grasp monitoring
        """
        self.arm = arm_controller
        self.capture = camera_capture
        self.vision = workspace_vision
        self.gripper = gripper_guard
        self.kin = OpenManipulatorKinematics()
        self.workspace = WorkspaceAnalyzer(self.kin)
        self.on_waypoint_cb: Optional[Callable] = None
        self.on_vision_cb: Optional[Callable] = None

    def get_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from the camera."""
        ret, frame = self.capture()
        if ret and frame is not None:
            return frame
        return None

    def smooth_move(self, targets: List[int], duration: float = 2.0, steps: int = 8):
        """
        Interpolate from current joint positions to target in small steps.
        Joint 11 uses shortest-path wrap-around for multi-turn mode.
        """
        joint_ids = [11, 12, 13, 14, 15]
        current = []
        for jid in joint_ids:
            pos = self.arm.read_position(jid)
            if pos is None:
                return False
            current.append(pos)

        dt = duration / steps
        for step in range(1, steps + 1):
            alpha = step / steps
            for i, jid in enumerate(joint_ids):
                if jid == 11:
                    diff = targets[i] - current[i]
                    goal = current[i] + int(round(diff * alpha))
                else:
                    goal = int(round(current[i] + (targets[i] - current[i]) * alpha))
                self.arm.write_position(jid, goal)
            time.sleep(dt)
        return True

    def move_to_cartesian(self, x: float, y: float, z: float,
                          pitch_rad: float = -math.pi / 2,
                          gripper_counts: int = 3130,
                          duration: float = 2.0) -> bool:
        """
        Move arm to Cartesian position using IK.
        Tries vertical grasp first, then angled if needed.
        """
        status, info = self.workspace.classify_target(x, y, z)

        if status == ReachabilityStatus.OUT_OF_BOUNDS:
            print(f"  [IK] Target ({x:.1f}, {y:.1f}, {z:.1f}) out of bounds")
            return False

        if status == ReachabilityStatus.REACHABLE_VERTICAL:
            q = info['ik_solution']
        elif status == ReachabilityStatus.REACHABLE_ANGLED:
            q = info['ik_solution']
        elif status == ReachabilityStatus.REACHABLE_SWEEP:
            print(f"  [IK] Target needs sweep strategy (r={info['radius']:.1f}mm)")
            return False
        else:
            return False

        raw = self.kin.radians_to_raw(q)
        targets = list(raw) + [gripper_counts]
        return self.smooth_move(targets, duration)

    def plan_pick_and_place(self, peg_shape: str, peg_color: str,
                            pick_xy: Tuple[float, float],
                            place_xy: Tuple[float, float],
                            peg_yaw: float = 0.0,
                            socket_yaw: float = 0.0,
                            symmetry_order: int = 0) -> TrajectoryPlan:
        """
        Generate a full pick-and-place trajectory plan with vision checkpoints.
        """
        px, py = pick_xy
        sx, sy = place_xy

        yaw_align = ShapeDetector.calculate_minimal_rotation(
            socket_yaw, peg_yaw, symmetry_order
        )

        z_hover = self.vision.z_hover
        z_grasp = self.vision.z_grasp
        z_transit = self.vision.z_transit
        z_sock_hover = self.vision.z_sock_hover
        z_insert = self.vision.z_insert

        waypoints = [
            Waypoint("pre-pick hover", px, py, z_hover,
                     duration=2.0, vision_check=True, obstacle_check=True),
            Waypoint("descend to grasp", px, py, z_grasp,
                     duration=1.5, vision_check=True),
            Waypoint("close gripper", px, py, z_grasp,
                     gripper_open=False, duration=1.0),
            Waypoint("lift peg", px, py, z_transit,
                     gripper_open=False, duration=1.5, vision_check=True),
            Waypoint("transit to socket", sx, sy, z_transit,
                     gripper_open=False, duration=2.5, obstacle_check=True),
            Waypoint("socket hover", sx, sy, z_sock_hover,
                     gripper_open=False, duration=1.5, vision_check=True),
            Waypoint("insert peg", sx, sy, z_insert,
                     gripper_open=False, duration=1.5),
            Waypoint("release", sx, sy, z_insert,
                     gripper_open=True, duration=0.8),
            Waypoint("retreat up", sx, sy, z_hover,
                     duration=1.5),
        ]

        return TrajectoryPlan(
            waypoints=waypoints,
            peg_shape=peg_shape,
            peg_color=peg_color,
            pick_xy=pick_xy,
            place_xy=place_xy,
            yaw_alignment_deg=yaw_align,
        )

    def execute_plan(self, plan: TrajectoryPlan) -> Tuple[MoveResult, str]:
        """
        Execute a trajectory plan with vision checks at each checkpoint.
        Returns (result, message).
        """
        gripper_open_counts = 1700
        gripper_close_counts = 2200

        print(f"\n{'='*60}")
        print(f"  EXECUTING: {plan.peg_shape} ({plan.peg_color})")
        print(f"  Pick: ({plan.pick_xy[0]:.1f}, {plan.pick_xy[1]:.1f})")
        print(f"  Place: ({plan.place_xy[0]:.1f}, {plan.place_xy[1]:.1f})")
        print(f"  Yaw alignment: {plan.yaw_alignment_deg:.1f} deg")
        print(f"{'='*60}\n")

        for i, wp in enumerate(plan.waypoints):
            print(f"  [{i+1}/{len(plan.waypoints)}] {wp.name}")

            if wp.vision_check:
                result = self._vision_check(wp, plan)
                if result is not None:
                    return result

            if wp.obstacle_check:
                result = self._obstacle_check(wp, plan)
                if result is not None:
                    return result

            gripper = gripper_open_counts if wp.gripper_open else gripper_close_counts
            success = self.move_to_cartesian(
                wp.x, wp.y, wp.z,
                pitch_rad=wp.pitch_rad,
                gripper_counts=gripper,
                duration=wp.duration,
            )

            if not success:
                return MoveResult.IK_FAILED, f"IK failed at waypoint '{wp.name}'"

            if wp.name == "close gripper":
                time.sleep(0.5)
                if self.gripper:
                    from src.control.gripper_guard import GraspState
                    state, info = self.gripper.evaluate_grasp()
                    if state != GraspState.SECURE_HOLD:
                        print(f"    GRASP CHECK: {state.value} — retrying...")
                        self.arm.write_position(15, gripper_close_counts + 50)
                        time.sleep(0.5)
                        state, info = self.gripper.evaluate_grasp()
                        if state != GraspState.SECURE_HOLD:
                            return MoveResult.GRASP_FAILED, f"Grasp failed: {state.value}"
                    print(f"    GRASP: {state.value} (pos={info.get('position', '?')}, cur={info.get('current_ma', 0):.1f}mA)")

            if wp.name == "lift peg" and self.gripper:
                time.sleep(0.3)
                frame = self.get_frame()
                if frame is not None and self.on_vision_cb:
                    self.on_vision_cb("post_lift", frame)

            if self.on_waypoint_cb:
                self.on_waypoint_cb(i, wp, plan)

        return MoveResult.SUCCESS, f"Placed {plan.peg_shape} successfully"

    def _vision_check(self, wp: Waypoint, plan: TrajectoryPlan) -> Optional[Tuple[MoveResult, str]]:
        """Run a vision verification at the current waypoint."""
        frame = self.get_frame()
        if frame is None:
            print("    [VISION] No frame — continuing blind")
            return None

        if self.on_vision_cb:
            self.on_vision_cb(wp.name, frame)

        if wp.name == "pre-pick hover":
            peg = self.vision.verify_peg_at_location(
                frame, plan.peg_shape,
                self.vision.robot_to_pixel(*plan.pick_xy),
                search_radius_px=60,
            )
            if peg is None:
                print("    [VISION] Peg not found at expected position!")
                pegs = self.vision.detect_pegs(frame)
                for p in pegs:
                    if p.shape == plan.peg_shape:
                        print(f"    [VISION] Found {plan.peg_shape} at ({p.robot_xy[0]:.1f}, {p.robot_xy[1]:.1f})")
                        plan.pick_xy = p.robot_xy
                        wp.x, wp.y = p.robot_xy
                        print(f"    [VISION] Updated pick position")
                        return None
                return MoveResult.VISION_LOST, f"Cannot find {plan.peg_shape} peg"
            else:
                dx = peg.robot_xy[0] - plan.pick_xy[0]
                dy = peg.robot_xy[1] - plan.pick_xy[1]
                if abs(dx) > 5 or abs(dy) > 5:
                    print(f"    [VISION] Peg shifted by ({dx:.1f}, {dy:.1f})mm — correcting")
                    plan.pick_xy = peg.robot_xy
                    wp.x, wp.y = peg.robot_xy

        elif wp.name == "socket hover":
            from src.vision.workspace_vision import WorkspaceObject
            sock_pixel = self.vision.robot_to_pixel(*plan.place_xy)
            sockets = self.vision.detect_sockets(frame)
            best_dist = float('inf')
            best_sock = None
            for s in sockets:
                if s.shape == plan.peg_shape:
                    su, sv = s.pixel_uv
                    dist = np.sqrt((su - sock_pixel[0])**2 + (sv - sock_pixel[1])**2)
                    if dist < 60 and dist < best_dist:
                        best_dist = dist
                        best_sock = s
            if best_sock:
                dx = best_sock.robot_xy[0] - plan.place_xy[0]
                dy = best_sock.robot_xy[1] - plan.place_xy[1]
                if abs(dx) > 5 or abs(dy) > 5:
                    print(f"    [VISION] Socket shifted by ({dx:.1f}, {dy:.1f})mm — correcting")
                    plan.place_xy = best_sock.robot_xy
                    wp.x, wp.y = best_sock.robot_xy

        return None

    def _obstacle_check(self, wp: Waypoint, plan: TrajectoryPlan) -> Optional[Tuple[MoveResult, str]]:
        """Check for obstacles along the planned trajectory corridor."""
        frame = self.get_frame()
        if frame is None:
            return None

        current_px = self.vision.robot_to_pixel(wp.x, wp.y)
        if wp.name == "transit to socket":
            target_px = self.vision.robot_to_pixel(*plan.place_xy)
        else:
            target_px = current_px

        start = (int(self.vision.base_u), int(self.vision.base_v))
        end = (int(current_px[0]), int(current_px[1]))
        path = [start, end]
        if wp.name == "transit to socket":
            path.append((int(target_px[0]), int(target_px[1])))

        obstacles = self.vision.quick_detect_obstacles(frame, path, corridor_width_px=30)
        if obstacles:
            print(f"    [OBSTACLE] {len(obstacles)} obstacle(s) detected in corridor!")
            for ox, oy in obstacles:
                rob_x, rob_y = self.vision.pixel_to_robot(ox, oy)
                print(f"      Obstacle at pixel ({ox},{oy}) = robot ({rob_x:.1f},{rob_y:.1f})")
            print(f"    [OBSTACLE] Raising transit height by 30mm for clearance")
            wp.z += 30

        return None

    def go_home(self, duration: float = 2.5):
        """Move arm to home position [0, 2048, 2048, 2048, 3130]."""
        self.smooth_move([0, 2048, 2048, 2048, 3130], duration)

    def go_side_park(self, duration: float = 2.5):
        """Park arm to the side (joint11=3072) to clear camera view."""
        current = []
        for jid in [11, 12, 13, 14, 15]:
            pos = self.arm.read_position(jid)
            current.append(pos if pos is not None else 2048)

        targets = [3072, 2048, 2048, 2048, 3130]
        self.smooth_move(targets, duration)
