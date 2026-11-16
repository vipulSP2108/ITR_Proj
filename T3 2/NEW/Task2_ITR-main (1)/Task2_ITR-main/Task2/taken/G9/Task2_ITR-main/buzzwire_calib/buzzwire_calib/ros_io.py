"""ROS 2 glue: joint states + KDL forward kinematics + optional ROS image topics.

FK is built exactly like your heal_end_effector_pub_node.py (URDF from /robot_description,
KDL chain base_link -> tip_link) so the poses match what the rest of your stack uses.
"""
from __future__ import annotations
import threading, time
from dataclasses import dataclass
from typing import Optional
import numpy as np

from .cameras import PushSource
from .transforms import make_T, quat_to_T

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                           QoSHistoryPolicy, qos_profile_sensor_data)
    from sensor_msgs.msg import JointState, Image, CompressedImage
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    HAVE_ROS = True
except Exception:  # pragma: no cover - lets the math/tests run on a laptop without ROS
    HAVE_ROS = False


@dataclass
class RobotSample:
    q: np.ndarray                 # joint positions, in cfg joint order
    qd: Optional[np.ndarray]      # joint velocities if published
    T_base_ee: Optional[np.ndarray]
    wall_time: float


class RosHub:
    """One node + one executor thread for everything."""

    def __init__(self, node_name="buzzwire_calib"):
        if not HAVE_ROS:
            raise RuntimeError("rclpy not importable - source your ROS 2 + workspace setup.bash first")
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.exec = MultiThreadedExecutor(num_threads=3)
        self.exec.add_node(self.node)
        self._th = threading.Thread(target=self.exec.spin, daemon=True)
        self._th.start()

    def shutdown(self):
        try:
            self.exec.shutdown()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


class RosRobotState:
    def __init__(self, hub: RosHub, robot_cfg: dict):
        self.hub, self.cfg = hub, robot_cfg
        self.joint_names = list(robot_cfg["joint_names"])
        self.base_link, self.tip_link = robot_cfg["base_link"], robot_cfg["tip_link"]
        self._lock = threading.Lock()
        self._sample: Optional[RobotSample] = None
        self._pose_topic_T: Optional[np.ndarray] = None
        self._kdl = None
        n = hub.node
        n.create_subscription(JointState, robot_cfg.get("joint_state_topic", "/joint_states"),
                              self._on_js, qos_profile_sensor_data)
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self._rd_sub = n.create_subscription(String, robot_cfg.get("robot_description_topic", "/robot_description"),
                                             self._on_urdf, qos)
        if robot_cfg.get("ee_pose_topic"):
            n.create_subscription(PoseStamped, robot_cfg["ee_pose_topic"], self._on_pose, 10)

    # ---- KDL ----
    def _on_urdf(self, msg):
        if self._kdl is not None:
            return
        try:
            import PyKDL as kdl
            from urdf_parser_py.urdf import URDF
            from kdl_parser_py.urdf import treeFromUrdfModel
            ok, tree = treeFromUrdfModel(URDF.from_xml_string(msg.data.strip()))
            if not ok:
                return
            chain = tree.getChain(self.base_link, self.tip_link)
            if chain.getNrOfJoints() == 0:
                self.hub.node.get_logger().error(f"KDL chain {self.base_link}->{self.tip_link} has 0 joints")
                return
            names = []
            for i in range(chain.getNrOfSegments()):
                j = chain.getSegment(i).getJoint()
                if j.getTypeName() != "None":
                    names.append(j.getName())
            self._kdl = (kdl, chain, kdl.ChainFkSolverPos_recursive(chain), names)
            self.hub.node.get_logger().info(f"KDL FK ready: {names}")
            self.hub.node.destroy_subscription(self._rd_sub)
        except Exception as e:
            self.hub.node.get_logger().warn(f"KDL setup failed ({e}); will rely on ee_pose_topic if set")

    def _fk(self, name_to_pos):
        if self._kdl is None:
            return None
        kdl, chain, solver, names = self._kdl
        q = kdl.JntArray(len(names))
        for i, nm in enumerate(names):
            if nm not in name_to_pos:
                return None
            q[i] = name_to_pos[nm]
        fr = kdl.Frame()
        if solver.JntToCart(q, fr) < 0:
            return None
        R = np.array([[fr.M[r, c] for c in range(3)] for r in range(3)])
        return make_T(R, [fr.p[0], fr.p[1], fr.p[2]])

    # ---- kinematics API used by teleop / replay (q in cfg joint order)
    @property
    def kinematics_ready(self) -> bool:
        return self._kdl is not None

    def fk_q(self, q):
        return self._fk(dict(zip(self.joint_names, [float(x) for x in q])))

    def jac_q(self, q):
        """6xN geometric Jacobian in the BASE frame at the tip origin (columns in cfg joint order)."""
        kdl, chain, _, names = self._kdl
        if not hasattr(self, "_jac_solver"):
            self._jac_solver = kdl.ChainJntToJacSolver(chain)
            self._jac_perm = [names.index(n) for n in self.joint_names]
        pos = dict(zip(self.joint_names, q))
        ja = kdl.JntArray(len(names))
        for i, nm in enumerate(names):
            ja[i] = float(pos[nm])
        jac = kdl.Jacobian(len(names))
        self._jac_solver.JntToJac(ja, jac)
        J = np.array([[jac[r, c] for c in range(len(names))] for r in range(6)])
        return J[:, self._jac_perm]

    def _on_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._pose_topic_T = quat_to_T([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    def _on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in idx for n in self.joint_names) or len(msg.position) < len(msg.name):
            return
        pos = {n: msg.position[idx[n]] for n in self.joint_names}
        q = np.array([pos[n] for n in self.joint_names], float)
        qd = None
        if len(msg.velocity) == len(msg.name):
            qd = np.array([msg.velocity[idx[n]] for n in self.joint_names], float)
        T = self._fk(pos)
        with self._lock:
            if T is None:
                T = self._pose_topic_T
            self._sample = RobotSample(q, qd, None if T is None else T.copy(), time.time())

    def get(self) -> Optional[RobotSample]:
        with self._lock:
            return self._sample

    def age(self) -> float:
        s = self.get()
        return float("inf") if s is None else time.time() - s.wall_time


def _decode_image(msg) -> Optional[np.ndarray]:
    import cv2
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return img.copy() if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ("bgra8", "rgba8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 4].reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    if enc in ("mono8", "8uc1"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return None


def make_ros_image_source(hub: RosHub, name: str, topic: str, compressed: bool = False) -> PushSource:
    import cv2
    src = PushSource(name)
    if compressed:
        def cb(msg):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                src.push(img)
        hub.node.create_subscription(CompressedImage, topic, cb, qos_profile_sensor_data)
    else:
        def cb(msg):
            img = _decode_image(msg)
            if img is not None:
                src.push(img)
        hub.node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    return src
