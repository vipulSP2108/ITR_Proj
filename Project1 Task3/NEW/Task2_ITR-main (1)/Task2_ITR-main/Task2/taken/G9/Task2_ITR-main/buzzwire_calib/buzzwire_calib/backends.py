"""Robot back-ends shared by teleop and replay: real ROS 2 robot, or a simulated arm (--sim)."""
import sys, time
import numpy as np


class RosBackend:
    QD_CEIL = 0.40      # rad/s absolute per-joint ceiling (HEAL safety trips at ~0.50 rad/s)
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

        self._q_good = None          # last plausible joint vector
        self._bad_reported = False
        self._q_cand, self._cand_n = None, 0
        self._qd_last = np.zeros(self.n)
        self._t_send = None
        self.qdd_max = float(cfg.get("teleop", {}).get("max_joint_accel", 1.5))   # rad/s^2 command slew

    def state(self):
        """Joint state. Implausible readings are reported as STALE (age = inf) so every caller holds still:
        * all joints exactly 0  -> the HEAL driver publishes this once the robot has gone into ERROR
        * any joint jumping > 0.25 rad between two reads (impossible at our speeds) -> glitch / error
        """
        s = self.robot.get()
        q = s.q.copy()
        bad = bool(np.all(np.abs(q) < 1e-9))
        if not bad and self._q_good is not None and np.max(np.abs(q - self._q_good)) > 0.25:
            # a real re-position (e.g. error recovery drove the arm home) is accepted once the new
            # reading has stayed consistent for ~50 reads; a one-off glitch never is
            if self._q_cand is not None and np.max(np.abs(q - self._q_cand)) < 0.01:
                self._cand_n += 1
            else:
                self._q_cand, self._cand_n = q.copy(), 0
            if self._cand_n >= 50:
                self._q_good, self._q_cand, self._cand_n = q.copy(), None, 0
            else:
                bad = True
        if bad:
            if not self._bad_reported:
                sys.stderr.write("\n!! implausible /joint_states (all zeros or jump) - robot in ERROR? Holding, not recording.\n")
                self._bad_reported = True
            self._qd_last[:] = 0.0
            return (self._q_good.copy() if self._q_good is not None else q), float("inf"), s.qd
        if self._bad_reported:
            # recovered: accept the new reading as the reference only after it is consistent again
            self._bad_reported = False
        self._q_good = q
        return q.copy(), self.robot.age(), s.qd

    def send(self, qd):
        """Publish joint velocities with a per-joint acceleration (slew) limit, so the drive never sees a
        velocity step (the HEAL safety layer trips 'Large Velocity for Joint N' on sudden commands)."""
        qd = np.asarray(qd, float)
        m = float(np.max(np.abs(qd))) if qd.size else 0.0
        if m > self.QD_CEIL:                   # uniform scale keeps the Cartesian direction
            qd = qd * (self.QD_CEIL / m)
        now = time.monotonic()
        dt = 0.01 if self._t_send is None else min(max(now - self._t_send, 1e-3), 0.05)
        self._t_send = now
        step = self.qdd_max * dt
        qd = self._qd_last + np.clip(qd - self._qd_last, -step, step)
        if np.all(np.asarray(qd) == 0.0) or np.max(np.abs(qd)) < 1e-6:
            qd = np.zeros(self.n)
        self._qd_last = qd
        m = self._msg(); m.data = [float(x) for x in qd]; self.pub.publish(m)

    def stop(self):
        for _ in range(25):                    # ramp down through the slew limiter, then hard zero
            self.send(np.zeros(self.n)); time.sleep(0.02)
        self._qd_last[:] = 0.0
        m = self._msg(); m.data = [0.0] * self.n; self.pub.publish(m)
        time.sleep(0.1)

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
