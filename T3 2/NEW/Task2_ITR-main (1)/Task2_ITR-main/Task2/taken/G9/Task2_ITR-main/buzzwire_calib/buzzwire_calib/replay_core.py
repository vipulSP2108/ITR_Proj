"""Path follower for replay (pure numpy, unit-testable).

A virtual 'carrot' moves along the recorded path (arc-length parameter s). Its speed comes from a
time-optimal-style profile that respects, at every point:
    * v_max                       (global speed cap you choose)
    * sqrt(a_lat / curvature)     (lateral acceleration in the tight U-bends)
    * w_max / (rotation per metre)(tool pitch/yaw rate limit on the steep legs)
    * a_max forward/backward pass (smooth acceleration into a bend and braking out of it)
The carrot additionally WAITS when the arm lags (tracking error gating) so the robot is never dragged.
"""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def _dedupe(P, Q, rot_metric=0.0, eps=1e-6):
    keep = [0]
    R = Rot.from_quat(Q)
    for i in range(1, len(P)):
        k = keep[-1]
        dr = float(np.linalg.norm((R[k].inv() * R[i]).as_rotvec()))
        if np.linalg.norm(P[i] - P[k]) > eps or rot_metric * dr > eps:
            keep.append(i)
    return P[keep], Q[keep]


class PathFollower:
    def __init__(self, P, Q, v_max=0.06, a_max=0.15, a_lat=0.10, w_max=0.5, v_floor=0.004,
                 e_lo=0.004, e_hi=0.010, rot_metric=0.0):
        """rot_metric (m/rad): >0 makes pure-rotation moves have length (used for approaches); 0 = positional arc."""
        P, Q = _dedupe(np.asarray(P, float), np.asarray(Q, float), rot_metric)
        if len(P) < 2:
            raise ValueError("path needs at least 2 distinct points")
        self.P, self.R = P, Rot.from_quat(Q)
        self.rel = np.array([(self.R[i].inv() * self.R[i + 1]).as_rotvec() for i in range(len(P) - 1)])
        dp = np.linalg.norm(np.diff(P, axis=0), axis=1)
        d = np.sqrt(dp ** 2 + (rot_metric * np.linalg.norm(self.rel, axis=1)) ** 2)
        self.ds = d
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.length = float(self.s[-1])
        self.tan = np.diff(P, axis=0) / d[:, None]
        # curvature at nodes (smoothed turning of the tangent per metre)
        kap = np.zeros(len(P))
        for i in range(1, len(P) - 1):
            kap[i] = np.linalg.norm(self.tan[i] - self.tan[i - 1]) / max(0.5 * (d[i] + d[i - 1]), 1e-9)
        k = 5
        kap = np.convolve(np.pad(kap, k // 2, mode="edge"), np.ones(k) / k, mode="valid")
        th = np.linalg.norm(self.rel, axis=1) / d                       # rad per metre per segment
        th_n = np.concatenate([[th[0]], 0.5 * (th[1:] + th[:-1]), [th[-1]]])
        v = np.full(len(P), v_max)
        v = np.minimum(v, np.sqrt(a_lat / np.maximum(kap, 1e-9)))
        v = np.minimum(v, w_max / np.maximum(th_n, 1e-9))
        v = np.maximum(v, v_floor)
        v[0] = v[-1] = min(v[0], v_floor * 1.5)
        for i in range(1, len(v)):                                       # forward: accel limit
            v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_max * d[i - 1]))
        for i in range(len(v) - 2, -1, -1):                              # backward: braking limit
            v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * d[i]))
        self.v_prof = v
        self.pos = 0.0                                                   # carrot arc position
        self.e_lo, self.e_hi = e_lo, e_hi

    # ----------------------------------------------------------------- sampling
    def _seg(self, s):
        i = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.ds) - 1))
        return i, (s - self.s[i]) / self.ds[i]

    def pose_at(self, s):
        i, f = self._seg(s)
        T = np.eye(4)
        T[:3, 3] = self.P[i] + f * (self.P[i + 1] - self.P[i])
        T[:3, :3] = (self.R[i] * Rot.from_rotvec(f * self.rel[i])).as_matrix()
        return T

    def speed_at(self, s):
        i, f = self._seg(s)
        return float(self.v_prof[i] + f * (self.v_prof[i + 1] - self.v_prof[i]))

    def twist_per_mps(self, s):
        """Feed-forward [v; w] (base frame) per 1 m/s of path speed."""
        i, _ = self._seg(s)
        return np.concatenate([self.tan[i], self.R[i].as_matrix() @ self.rel[i] / self.ds[i]])

    def nearest_s(self, p):
        """Arc position of the path point closest to p (used for resume)."""
        d = np.linalg.norm(self.P - p, axis=1)
        return float(self.s[int(np.argmin(d))])

    # ----------------------------------------------------------------- stepping
    def advance(self, dt, err_lin=0.0, scale=1.0):
        """Move the carrot; returns (T_target, twist_ff, done, speed)."""
        gate = float(np.clip((self.e_hi - err_lin) / (self.e_hi - self.e_lo), 0.0, 1.0))
        v = self.speed_at(self.pos) * scale * gate
        self.pos = min(self.length, self.pos + v * dt)
        done = self.pos >= self.length - 1e-9
        v_ff = 0.0 if done else v
        return self.pose_at(self.pos), self.twist_per_mps(self.pos) * v_ff, done, v

    def duration_estimate(self):
        return float(np.sum(2 * self.ds / (self.v_prof[:-1] + self.v_prof[1:])))


def line_path(T0, T1, step=0.002, rot_metric=0.05):
    """Straight Cartesian approach path from pose T0 to T1 as (P, Q) arrays (position and orientation blended)."""
    rel = Rot.from_matrix(T0[:3, :3].T @ T1[:3, :3]).as_rotvec()
    d = float(np.hypot(np.linalg.norm(T1[:3, 3] - T0[:3, 3]), rot_metric * np.linalg.norm(rel)))
    n = max(2, int(np.ceil(d / step)) + 1)
    f = np.linspace(0, 1, n)
    P = T0[:3, 3][None] + f[:, None] * (T1[:3, 3] - T0[:3, 3])[None]
    Q = np.array([(Rot.from_matrix(T0[:3, :3]) * Rot.from_rotvec(fi * rel)).as_quat() for fi in f])
    return P, Q
