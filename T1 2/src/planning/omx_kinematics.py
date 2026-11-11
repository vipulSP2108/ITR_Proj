"""
Hardware-verified kinematics for the ROBOTIS OpenMANIPULATOR-X (RM-X52-TNM).

Replaces src/planning/kinematics.py for REAL hardware. The old module assumed the arm is
straight up when joints 12-14 are at 2048 counts; the real arm is L-shaped at that pose
(upper arm up, forearm + gripper horizontal) - verified by overhead camera on 2026-11-10.

Robot base frame (mm):  origin on the base-pan axis at table level, +Z up,
                        +X = direction the arm points when joint 11 is at count 0.
Joint angles a1..a4 (rad):
    a1 base pan, counter-clockwise seen from above
    a2 shoulder, a3 elbow, a4 wrist: positive = tip moves forward/down (ROBOTIS convention)
"down" (rad) is the tool direction below horizontal: 0 = horizontal, pi/2 = straight down.
"""
import math
from typing import List, Optional, Tuple

H_SHOULDER = 77.0                     # table -> shoulder axis
L2_UP, L2_FWD = 128.0, 24.0           # shoulder -> elbow (dog-leg link)
L2 = math.hypot(L2_UP, L2_FWD)        # 130.23
BETA = math.atan2(L2_FWD, L2_UP)      # 10.62 deg
L3 = 124.0                            # elbow -> wrist
L4 = 126.0                            # wrist -> grip centre (TCP)

COUNTS_PER_RAD = 4096.0 / (2.0 * math.pi)
CENTER = 2048

# ROBOTIS joint limits (rad) with a small safety margin
LIMITS = {2: (-1.45, 1.45), 3: (-1.45, 1.35), 4: (-1.65, 1.90)}
BASE_LIMIT = math.radians(170.0)


def raw_to_angles(raw4: List[int]) -> List[float]:
    """Raw counts of joints [11, 12, 13, 14] -> [a1, a2, a3, a4] (rad)."""
    a1 = raw4[0] / COUNTS_PER_RAD
    a1 = (a1 + math.pi) % (2 * math.pi) - math.pi
    return [a1] + [(r - CENTER) / COUNTS_PER_RAD for r in raw4[1:4]]


def angles_to_raw(a: List[float]) -> List[int]:
    """[a1..a4] (rad) -> raw counts for joints [11, 12, 13, 14]. Base stays in -2048..2048."""
    return [int(round(a[0] * COUNTS_PER_RAD))] + [CENTER + int(round(x * COUNTS_PER_RAD)) for x in a[1:4]]


def fk(a: List[float], tool_len: float = L4) -> Tuple[float, float, float, float]:
    """Joint angles -> (x, y, z, down). tool_len lets you query other points on the tool axis."""
    a1, a2, a3, a4 = a
    down = a2 + a3 + a4
    r = L2 * math.sin(a2 + BETA) + L3 * math.cos(a2 + a3) + tool_len * math.cos(down)
    z = H_SHOULDER + L2 * math.cos(a2 + BETA) - L3 * math.sin(a2 + a3) - tool_len * math.sin(down)
    return r * math.cos(a1), r * math.sin(a1), z, down


def fk_raw(raw4: List[int], tool_len: float = L4) -> Tuple[float, float, float, float]:
    return fk(raw_to_angles(raw4), tool_len)


def ik(x: float, y: float, z: float, down: float = math.pi / 2, tool_len: float = L4) -> Optional[List[float]]:
    """Tool point (TCP by default) + tool 'down' angle -> [a1..a4], elbow-up solution. None if unreachable."""
    a1 = math.atan2(y, x)
    if abs(a1) > BASE_LIMIT:
        return None
    r = math.hypot(x, y)
    rw = r - tool_len * math.cos(down)
    zw = z - H_SHOULDER + tool_len * math.sin(down)
    d2 = rw * rw + zw * zw
    c = (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c) > 1.0:
        return None
    gamma = math.acos(c)                                   # bend between upper arm and forearm
    alpha = math.atan2(zw, rw) + math.atan2(L3 * math.sin(gamma), L2 + L3 * math.cos(gamma))
    a2 = (math.pi / 2 - alpha) - BETA
    b = gamma - alpha                                      # a2 + a3
    a3 = b - a2
    a4 = down - b
    for i, v in ((2, a2), (3, a3), (4, a4)):
        lo, hi = LIMITS[i]
        if not (lo <= v <= hi):
            return None
    return [a1, a2, a3, a4]


def ik_raw(x: float, y: float, z: float, down_deg: float = 90.0, tool_len: float = L4) -> Optional[List[int]]:
    sol = ik(x, y, z, math.radians(down_deg), tool_len)
    return None if sol is None else angles_to_raw(sol)


def ik_best(x: float, y: float, z: float, prefer_down_deg: float = 90.0,
            min_down_deg: float = 30.0, step: float = 5.0, tool_len: float = L4):
    """Closest-to-preferred tool angle that is reachable. Returns (raw4, down_deg) or (None, None)."""
    d = prefer_down_deg
    while d >= min_down_deg - 1e-6:
        raw = ik_raw(x, y, z, d, tool_len)
        if raw is not None:
            return raw, d
        d -= step
    return None, None


if __name__ == "__main__":
    # self-test: IK -> FK round trip
    for (x, y, z, dn) in [(274, 0, 205, 0), (180, 0, 60, 90), (150, 100, 40, 90), (250, -80, 50, 60)]:
        raw = ik_raw(x, y, z, dn)
        print((x, y, z, dn), "->", raw, "->", None if raw is None else [round(v, 1) for v in fk_raw(raw)[:3]])
