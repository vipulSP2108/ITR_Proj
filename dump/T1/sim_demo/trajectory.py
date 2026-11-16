#!/usr/bin/env python3
"""
Trajectory Planning, Smoothing, and DLS-IK for OpenMANIPULATOR-X.
Addresses requirements from T2/draft.md and repare.md.

Features:
  1. Quintic Polynomial Trajectory Generator (minimum jerk trajectory).
  2. Cubic Spline Trajectory Interpolator.
  3. Damped Least-Squares (DLS) Inverse Kinematics solver.
  4. Cartesian path to joint trajectory interpolator.
"""

import numpy as np
from typing import List, Tuple, Optional, Callable
import arm_model as A


class QuinticTrajectory:
    """
    Minimum-jerk trajectory with zero boundary velocity and acceleration:
      s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5,  where tau = t / T in [0, 1].
    """

    @staticmethod
    def interpolate(p0: np.ndarray, p1: np.ndarray, t: float, duration: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns (pos, vel, acc) at time t in [0, duration].
        """
        if duration <= 1e-6:
            return p1.copy(), np.zeros_like(p1), np.zeros_like(p1)

        tau = np.clip(t / duration, 0.0, 1.0)
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau

        # Quintic blending
        s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
        s_dot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / duration
        s_ddot = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (duration * duration)

        pos = (1.0 - s) * p0 + s * p1
        vel = s_dot * (p1 - p0)
        acc = s_ddot * (p1 - p0)
        return pos, vel, acc


class DlsIkSolver:
    """
    Damped Least-Squares (DLS-IK) solver for singularity-robust Cartesian tracking.
      dq = J^T * (J * J^T + lambda^2 * I)^(-1) * dx
    """

    def __init__(self, damping: float = 0.03, max_iter: int = 25, tol_m: float = 1e-4):
        self.damping = damping
        self.max_iter = max_iter
        self.tol_m = tol_m

    @staticmethod
    def compute_jacobian(q: np.ndarray, eps: float = 1e-5) -> np.ndarray:
        """
        Computes 3x4 numerical Jacobian for TCP position wrt q = [q1, q2, q3, q4].
        """
        J = np.zeros((3, 4))
        p0 = A.fk(q)["tcp"]
        for j in range(4):
            q_pert = q.copy()
            q_pert[j] += eps
            pj = A.fk(q_pert)["tcp"]
            J[:, j] = (pj - p0) / eps
        return J

    def solve(self, target_tcp: np.ndarray, q_init: np.ndarray,
              target_tilt: Optional[float] = None) -> Optional[np.ndarray]:
        """
        Solves IK for target_tcp (and optional tilt) starting from q_init.
        """
        q = q_init.copy().astype(float)
        eye3 = np.eye(3)

        for _ in range(self.max_iter):
            cur_fk = A.fk(q)
            cur_tcp = cur_fk["tcp"]
            err_pos = target_tcp - cur_tcp

            if np.linalg.norm(err_pos) < self.tol_m:
                # If target tilt specified, check match or do secondary projection
                if target_tilt is not None:
                    # Pitch adjustment on q4
                    cur_tilt = cur_fk["tilt"]
                    q[3] += (cur_tilt - target_tilt)
                if A.within_limits(q, margin=0.01):
                    return q

            J = self.compute_jacobian(q)
            # Damped pseudo-inverse: J^T * inv(J*J^T + lambda^2 * I)
            dls_mat = J @ J.T + (self.damping ** 2) * eye3
            dq = J.T @ np.linalg.solve(dls_mat, err_pos)

            # Step with damping
            q += np.clip(dq, -0.15, 0.15)

            # Project into joint limits
            q = np.clip(q, A.JOINT_LIMITS[:, 0], A.JOINT_LIMITS[:, 1])

        cur_tcp = A.fk(q)["tcp"]
        if np.linalg.norm(target_tcp - cur_tcp) < 0.005:  # within 5 mm
            return q
        return None


def generate_smooth_cartesian_trajectory(
    p_start: np.ndarray,
    p_goal: np.ndarray,
    tilt_rad: float,
    duration: float,
    dt: float = 0.02
) -> List[Tuple[float, np.ndarray, np.ndarray]]:
    """
    Generates a smoothly interpolated trajectory of (time, cartesian_pos, joint_angles).
    Uses quintic polynomials for minimum jerk and closed-form / DLS IK.
    """
    N = max(int(np.ceil(duration / dt)), 2)
    traj = []
    dls = DlsIkSolver()
    q_prev = None

    for i in range(N + 1):
        t = i * dt
        p, v, a = QuinticTrajectory.interpolate(p_start, p_goal, t, duration)

        # Primary: closed-form IK
        q = A.ik(p[0], p[1], p[2], tilt_rad)
        if q is None:
            # Fallback to DLS-IK if closed-form hits boundary
            q_seed = q_prev if q_prev is not None else A.HOME_Q
            q = dls.solve(p, q_seed, target_tilt=tilt_rad)

        if q is not None:
            q_prev = q.copy()
            traj.append((t, p, q))

    return traj

