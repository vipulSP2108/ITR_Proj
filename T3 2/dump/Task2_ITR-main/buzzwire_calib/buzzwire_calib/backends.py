"""Robot back-ends shared by teleop and replay: real ROS 2 robot, or a simulated arm (--sim)."""
import sys, time
import numpy as np


class RosBackend:
    def __init__(self, cfg):
        from buzzwire_calib.ros_io import RosHub, RosRobotState
        from buzzwire_calib.gripper import GripperHelper
        from std_msgs.msg import Float64MultiArray
        self.cfg, self._msg = cfg, Float64MultiArray
        self.hub = RosHub("buzzwire_teleop")
        self.robot = RosRobotState(self.hub, cfg["robot"])
        self.pub = self.hub.node.create_publisher(Float64MultiArray, cfg["robot"]["velocity_command_topic"], 1)
        self.gripper = GripperHelper(cfg, self.hub)
        t0 = time.time()
        while not (self.robot.get() and self.robot.kinematics_ready):
            if time.time() - t0 > 20:
                sys.exit("No /joint_states or no KDL chain after 20 s. Is ./heal_server running? Is the workspace sourced?")
            time.sleep(0.05)
        self.n = len(cfg["robot"]["joint_names"])
        self.fk, self.jac = self.robot.fk_q, self.robot.jac_q

    def state(self):
        s = self.robot.get()
        return s.q.copy(), self.robot.age(), s.qd

    def send(self, qd):
        m = self._msg(); m.data = [float(x) for x in qd]; self.pub.publish(m)

    def stop(self):
        for _ in range(10):
            self.send(np.zeros(self.n)); time.sleep(0.01)

    def shutdown(self):
        self.hub.shutdown()


class SimBackend:
    def __init__(self, cfg):
        from buzzwire_calib import simarm
        self.q = simarm.Q_HOME.copy(); self.n = 6
        self.fk, self.jac, self.gripper = simarm.fk, simarm.jac, simarm.SimGripper()
        self._t = time.monotonic()

    def state(self):
        return self.q.copy(), 0.0, None

    def send(self, qd):
        now = time.monotonic(); self.q = self.q + np.asarray(qd) * min(now - self._t, 0.05); self._t = now

    def stop(self): pass
    def shutdown(self): pass
