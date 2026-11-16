"""6-DOF Cartesian pose controller (pure numpy - no ROS, fully unit-testable).

One controller is used by BOTH keyboard teleop and trajectory replay, so the thing you tune
while teleoperating is exactly what later replays your recording.

Idea ("carrot on a leash")
--------------------------
  * A *desired pose* T_des (position + orientation) is what the operator / replay moves.
    Keys integrate T_des; the robot chases it.  Because T_des is a full pose, the orientation is
    HELD automatically while you translate (no tool drift), and pitch/yaw/roll keys simply rotate it.
  * Control law:   v_cmd = feed_forward_twist + Kp * pose_error(T_des, T_actual)
  * Resolved-rate with damped least squares (Nakamura-style adaptive damping near singularities):
        qdot = J^T (J J^T + lambda^2 I)^-1 v_cmd
  * Leash: T_des may lead the real pose by at most (lead_lin, lead_ang). If the arm cannot follow
    (collision, fault, joint limit) T_des stops running away and `lag_s` grows -> caller can abort.
  * Safety shaping: acceleration (slew) limiting, workspace box, optional joint limits, uniform joint-velocity
    scaling (keeps the Cartesian DIRECTION when it has to slow down), slow-down near singularities.
  * Backtrack: T_des steps back along the path it travelled (1 mm breadcrumb history) - the retreat that
    got you into a contact is retraced exactly.
  * Auto-tangent (optional): tilts the tool about the wire-plane normal so a chosen tool axis follows the
    direction of travel - this is the "pitch" that the 77-degree wire legs need, done for you.
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional
import numpy as np
from scipy.spatial.transform import Rotation as Rot


@dataclass
class Limits:
    max_joint_vel: float = 0.35      # rad/s (largest joint)
    lin_accel: float = 0.10          # m/s^2
    ang_accel: float = 0.6           # rad/s^2
    lead_lin: float = 0.012          # m   max lead of T_des over actual
    lead_ang: float = 0.12           # rad
    kp_lin: float = 4.0
    kp_ang: float = 3.0
    sigma_eps: float = 0.03          # below this smallest singular value -> slow down + extra damping
    lam_max: float = 0.08
    box: Optional[list] = None       # [xmin,xmax,ymin,ymax,zmin,zmax] for T_des position (base frame)
    joint_limits: Optional[list] = None   # [[lo,hi],...] radians
    joint_margin: float = 0.05


def numeric_jacobian(fk: Callable, q, h: float = 1e-6) -> np.ndarray:
    """6xn geometric Jacobian (base frame, [v; w]) by central differences. Fallback if KDL's is unavailable."""
    q = np.asarray(q, float)
    J = np.zeros((6, len(q)))
    for i in range(len(q)):
        qp, qm = q.copy(), q.copy()
        qp[i] += h; qm[i] -= h
        Tp, Tm = fk(qp), fk(qm)
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)
        J[3:, i] = Rot.from_matrix(Tp[:3, :3] @ Tm[:3, :3].T).as_rotvec() / (2 * h)
    return J


def _limit_norm(v, m):
    n = np.linalg.norm(v)
    return v if n <= m else v * (m / n)


def _slew(cur, tgt, a_lin, a_ang, dt):
    out = cur.copy()
    out[:3] = cur[:3] + _limit_norm(tgt[:3] - cur[:3], a_lin * dt)
    out[3:] = cur[3:] + _limit_norm(tgt[3:] - cur[3:], a_ang * dt)
    return out


def _exp_rot(w, R):
    return Rot.from_rotvec(w).as_matrix() @ R


class PoseController:
    def __init__(self, fk: Callable, jac: Callable, limits: Limits = None,
                 tool_axis=(0, 0, 1), plane_normal=(0, 1, 0), auto_tangent: bool = False):
        self.fk, self.jac = fk, jac
        self.lim = limits or Limits()
        self.tool_axis = np.asarray(tool_axis, float) / np.linalg.norm(tool_axis)
        n = np.asarray(plane_normal, float)
        self.plane_normal = n / np.linalg.norm(n)
        self.auto_tangent = auto_tangent
        self.T_des: Optional[np.ndarray] = None
        self.twist = np.zeros(6)
        self.history: deque = deque(maxlen=40000)
        self._last_hist_p = None
        self._dir_f = np.zeros(3)
        self.lag_s = 0.0
        self.last_info: dict = {}

    # ------------------------------------------------------------------ state
    def sync(self, q):
        """Snap the desired pose to the real pose (use at start, after a pause/stall, or on demand)."""
        self.T_des = self.fk(np.asarray(q, float)).copy()
        self.twist[:] = 0
        self.lag_s = 0.0
        self._last_hist_p = self.T_des[:3, 3].copy()

    def clear_history(self):
        self.history.clear()
        if self.T_des is not None:
            self._last_hist_p = self.T_des[:3, 3].copy()

    # ------------------------------------------------------------------ teleop step
    def step(self, q, twist_target, dt, backtrack: bool = False):
        """One control tick. twist_target = desired [v(3), w(3)] in the BASE frame. Returns (qdot, info)."""
        q = np.asarray(q, float)
        T_cur = self.fk(q)
        if self.T_des is None:
            self.sync(q)
        tgt = np.asarray(twist_target, float).copy()

        if backtrack:
            self.twist = _slew(self.twist, np.zeros(6), self.lim.lin_accel, self.lim.ang_accel, dt)
            speed = max(np.linalg.norm(tgt[:3]), 0.004)
            self._retrace(speed * dt)
            ff = np.zeros(6)
        else:
            if self.auto_tangent:
                tgt[3:] += self._auto_omega(tgt[:3], dt)
            self.twist = _slew(self.twist, tgt, self.lim.lin_accel, self.lim.ang_accel, dt)
            p_new = self.T_des[:3, 3] + self.twist[:3] * dt
            if self.lim.box is not None:
                b = self.lim.box
                clipped = np.clip(p_new, [b[0], b[2], b[4]], [b[1], b[3], b[5]])
                hit = clipped != p_new
                self.twist[:3][hit] = 0.0
                p_new = clipped
            self.T_des[:3, 3] = p_new
            self.T_des[:3, :3] = _exp_rot(self.twist[3:] * dt, self.T_des[:3, :3])
            ff = self.twist.copy()

        # leash
        e_p = self.T_des[:3, 3] - T_cur[:3, 3]
        n_p = np.linalg.norm(e_p)
        lagging = False
        if n_p > self.lim.lead_lin:
            self.T_des[:3, 3] = T_cur[:3, 3] + e_p * (self.lim.lead_lin / n_p)
            lagging = True
        e_r = Rot.from_matrix(self.T_des[:3, :3] @ T_cur[:3, :3].T).as_rotvec()
        n_r = np.linalg.norm(e_r)
        if n_r > self.lim.lead_ang:
            self.T_des[:3, :3] = _exp_rot(e_r * (self.lim.lead_ang / n_r), T_cur[:3, :3])
            lagging = True
        self.lag_s = self.lag_s + dt if lagging else max(0.0, self.lag_s - 2 * dt)

        # breadcrumbs (forward motion only)
        if not backtrack and self._last_hist_p is not None and \
                np.linalg.norm(self.T_des[:3, 3] - self._last_hist_p) >= 0.001:
            self.history.append((self.T_des[:3, 3].copy(), self.T_des[:3, :3].copy()))
            self._last_hist_p = self.T_des[:3, 3].copy()

        e_p = self.T_des[:3, 3] - T_cur[:3, 3]
        e_r = Rot.from_matrix(self.T_des[:3, :3] @ T_cur[:3, :3].T).as_rotvec()
        v_cmd = ff + np.concatenate([self.lim.kp_lin * e_p, self.lim.kp_ang * e_r])
        qd, info = self._solve(q, v_cmd)
        info.update(lagging=lagging, lag_s=self.lag_s, err_lin=float(np.linalg.norm(e_p)),
                    err_ang=float(np.linalg.norm(e_r)), backtracking=backtrack,
                    history=len(self.history))
        self.last_info = info
        return qd, info

    # ------------------------------------------------------------------ replay step
    def track(self, q, T_target, twist_ff, dt=None):
        """Track an externally supplied pose (+ feed-forward twist). Used by replay."""
        q = np.asarray(q, float)
        T_cur = self.fk(q)
        e_p = T_target[:3, 3] - T_cur[:3, 3]
        e_r = Rot.from_matrix(T_target[:3, :3] @ T_cur[:3, :3].T).as_rotvec()
        v_cmd = np.asarray(twist_ff, float) + np.concatenate([self.lim.kp_lin * e_p, self.lim.kp_ang * e_r])
        qd, info = self._solve(q, v_cmd)
        info.update(err_lin=float(np.linalg.norm(e_p)), err_ang=float(np.linalg.norm(e_r)))
        self.last_info = info
        return qd, info

    # ------------------------------------------------------------------ internals
    def _solve(self, q, v6):
        L = self.lim
        J = self.jac(q)
        S = np.linalg.svd(J, compute_uv=False)
        smin = float(S[-1]) if J.shape[1] >= 6 else float(S.min())
        if smin >= L.sigma_eps:
            scale, lam2 = 1.0, 1e-6
        else:
            r = max(smin, 0.0) / L.sigma_eps
            scale, lam2 = r, L.lam_max ** 2 * (1 - r ** 2) + 1e-6
        qd = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), v6 * scale)
        flags = []
        if L.joint_limits is not None:
            for i, (lo, hi) in enumerate(L.joint_limits[:len(qd)]):
                if (q[i] <= lo + L.joint_margin and qd[i] < 0) or (q[i] >= hi - L.joint_margin and qd[i] > 0):
                    qd[i] = 0.0
                    flags.append(i)
        m = float(np.max(np.abs(qd)))
        vel_scaled = m > L.max_joint_vel
        if vel_scaled:
            qd = qd * (L.max_joint_vel / m)
        return qd, {"sigma_min": smin, "sing_scale": scale, "vel_scaled": vel_scaled, "joint_limit_hit": flags,
                    "joint_scale": (L.max_joint_vel / m) if vel_scaled else 1.0}

    def _retrace(self, dist):
        while dist > 1e-9 and self.history:
            p, R = self.history[-1]
            d = float(np.linalg.norm(p - self.T_des[:3, 3]))
            if d <= dist:
                self.T_des[:3, 3], self.T_des[:3, :3] = p, R
                self.history.pop()
                dist -= d
            else:
                self.T_des[:3, 3] += (p - self.T_des[:3, 3]) * (dist / d)
                return
        self._last_hist_p = self.T_des[:3, 3].copy()

    def _auto_omega(self, v_lin, dt, k=2.0, w_max=0.5):
        """Angular rate about the plane normal that swings the tool axis toward the direction of travel."""
        n = self.plane_normal
        v = v_lin - n * float(n @ v_lin)
        sp = float(np.linalg.norm(v))
        if sp < 5e-4:
            return np.zeros(3)
        self._dir_f += (v / sp - self._dir_f) * min(1.0, 4.0 * dt)
        nf = np.linalg.norm(self._dir_f)
        if nf < 0.3:
            return np.zeros(3)
        d = self._dir_f / nf
        a = self.T_des[:3, :3] @ self.tool_axis
        a = a - n * float(n @ a)
        na = np.linalg.norm(a)
        if na < 1e-3:
            return np.zeros(3)
        a /= na
        ang = float(np.arctan2(n @ np.cross(a, d), a @ d))
        if abs(ang) > np.pi / 2:                      # tool axis is undirected: take the shorter swing
            ang = ang - np.sign(ang) * np.pi
        return float(np.clip(k * ang, -w_max, w_max)) * n
