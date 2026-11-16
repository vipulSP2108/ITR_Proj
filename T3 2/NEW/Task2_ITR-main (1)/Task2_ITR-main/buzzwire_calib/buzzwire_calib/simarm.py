"""Tiny simulated 6-axis arm (UR5-like DH) so you can practise the keyboard teleop with NO robot:
    python3 scripts/teleop.py --sim
"""
import time
import numpy as np
from .teleop_core import numeric_jacobian

_D = [0.0892, 0, 0, 0.1093, 0.09475, 0.0825]
_A = [0, -0.425, -0.392, 0, 0, 0]
_AL = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]
Q_HOME = np.array([0.3, -1.3, 1.7, -0.4, 1.57, 0.0])


def fk(q):
    T = np.eye(4)
    for qi, d, a, al in zip(q, _D, _A, _AL):
        ct, st, ca, sa = np.cos(qi), np.sin(qi), np.cos(al), np.sin(al)
        T = T @ np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d], [0, 0, 0, 1]])
    return T


def jac(q):
    return numeric_jacobian(fk, q)


class SimGripper:
    def __init__(self):
        self.state = "open"

    @property
    def value(self):
        return 1.0 if self.state == "open" else 0.0

    def toggle(self):
        self.state = "closed" if self.state == "open" else "open"
        print(f"\n[sim gripper] {self.state.upper()}")
