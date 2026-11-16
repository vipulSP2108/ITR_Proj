"""
Analytic kinematics of the ROBOTIS OpenMANIPULATOR-X, built ONLY from its published dimensions.

This module is the "what we know about the arm" part of the pipeline: it never reads MuJoCo ground
truth.  It is used (a) to command the arm (IK), (b) to compute where the arm *should* be (FK) for
camera<->robot calibration and path verification, and (c) for reachability analysis.

Robot frame: origin = centre of the base on the table surface, +X forward (toward the board),
+Y to the left of the arm, +Z up.  Joint angles in radians, zero pose = the "L" home pose.

Planar arm geometry (r = horizontal distance from the base axis, z = height above table):
    shoulder (joint 2 axis)      at (0, 0.077)
    elbow    (joint 3 axis)      shoulder + Ry(q2) * (0.024, 0.128)
    wrist    (joint 4 axis)      elbow    + Ry(q2+q3) * (0.124, 0)
    TCP (fingertip centre)       wrist    + Ry(q2+q3+q4) * (0.126, 0)
    tool marker (colored ball)   wrist    + Ry(q2+q3+q4) * (0, 0.030)   [local z of link4]
where Ry(a)*(x,z) = (x cos a + z sin a, -x sin a + z cos a).
"""
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

# ----- published OpenMANIPULATOR-X dimensions (metres) -----
SHOULDER_Z = 0.077          # joint-2 axis height above the base origin
ELBOW_OFF = (0.024, 0.128)  # joint-3 axis relative to joint-2 axis (x, z) at q2 = 0
FOREARM = 0.124             # joint-3 -> joint-4
TOOL = 0.126                # joint-4 -> end-effector (TCP)
MARKER_LOCAL = (0.0, 0.030) # tool-marker position in link-4 frame (x along tool, z perpendicular)

# ----- joint limits (rad), from the open_manipulator_x URDF -----
JOINT_LIMITS = np.array([
    [-np.pi,         np.pi        ],   # q1 pan
    [-np.pi * 0.57,  np.pi * 0.50 ],   # q2 shoulder
    [-np.pi * 0.30,  np.pi * 0.44 ],   # q3 elbow
    [-np.pi * 0.57,  np.pi * 0.65 ],   # q4 wrist
])

L2_EFF = float(np.hypot(*ELBOW_OFF))             # shoulder -> elbow length
GAMMA = float(np.arctan2(ELBOW_OFF[0], ELBOW_OFF[1]))  # forward lean of the upper arm at q2 = 0


def _ry(a: float, v: Tuple[float, float]) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([v[0] * c + v[1] * s, -v[0] * s + v[1] * c])


def fk_planar(q2: float, q3: float, q4: float):
    """Return dict of planar points (r, z) + tool direction angle sigma = q2+q3+q4."""
    S = np.array([0.0, SHOULDER_Z])
    E = S + _ry(q2, ELBOW_OFF)
    W = E + _ry(q2 + q3, (FOREARM, 0.0))
    sig = q2 + q3 + q4
    T = W + _ry(sig, (TOOL, 0.0))
    M = W + _ry(sig, MARKER_LOCAL)
    return dict(shoulder=S, elbow=E, wrist=W, tcp=T, marker=M, sigma=sig)


def _to3d(rz: np.ndarray, q1: float) -> np.ndarray:
    return np.array([rz[0] * np.cos(q1), rz[0] * np.sin(q1), rz[1]])


def fk(q) -> dict:
    """3-D forward kinematics. q = (q1, q2, q3, q4). Returns points in the robot frame (metres)."""
    q1, q2, q3, q4 = [float(v) for v in q[:4]]
    p = fk_planar(q2, q3, q4)
    out = {k: _to3d(v, q1) for k, v in p.items() if k != "sigma"}
    out["tilt"] = float(np.pi / 2 - p["sigma"])     # tool tilt from straight-down (0 = vertical, + = forward)
    return out


def within_limits(q, margin: float = 0.0) -> bool:
    q = np.asarray(q, float)
    return bool(np.all(q >= JOINT_LIMITS[:, 0] + margin) and np.all(q <= JOINT_LIMITS[:, 1] - margin))


def ik(x: float, y: float, z: float, tilt: float = 0.0, elbow_up: bool = True,
       limit_margin: float = 0.02) -> Optional[np.ndarray]:
    """
    Closed-form IK for the TCP at (x, y, z) with the tool tilted `tilt` rad from straight-down
    (0 = vertical top grasp, positive = tool points forward/down, wrist pulled back toward the base).
    Returns (q1..q4) or None if unreachable / outside joint limits.
    """
    q1 = float(np.arctan2(y, x)) if np.hypot(x, y) > 1e-9 else 0.0
    r = float(np.hypot(x, y))
    sig = np.pi / 2 - tilt                                   # q2+q3+q4
    # wrist = TCP - TOOL * tool_dir ;  tool_dir = (cos sig, -sin sig)
    rw = r - TOOL * np.cos(sig)
    zw = z + TOOL * np.sin(sig)
    Pr, Pz = rw, zw - SHOULDER_Z
    D2 = Pr * Pr + Pz * Pz
    L2, L3 = L2_EFF, FOREARM
    c = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c) > 1.0:
        return None
    delta = -np.arccos(c) if elbow_up else np.arccos(c)      # relative angle forearm vs upper arm
    a2 = np.arctan2(Pz, Pr) - np.arctan2(L3 * np.sin(delta), L2 + L3 * np.cos(delta))
    q2 = np.pi / 2 - GAMMA - a2
    q3 = -np.pi / 2 + GAMMA - delta
    q4 = sig - q2 - q3
    q = np.array([q1, q2, q3, q4])
    return q if within_limits(q, limit_margin) else None


def max_vertical_reach(z: float = 0.045) -> float:
    """Largest horizontal TCP distance reachable with the tool pointing straight down at height z."""
    best = 0.0
    for r in np.arange(0.05, 0.40, 0.002):
        if ik(r, 0.0, z, 0.0) is not None:
            best = r
    return float(best)


HOME_Q = np.zeros(4)   # the "L" pose from the screenshot / CAD

# arm folded out to the -Y side of the table: keeps the mat and board unoccluded for a clean snapshot
SIDE_PARK_Q = np.array([-np.pi / 2, 0.0, 0.0, 0.0])
