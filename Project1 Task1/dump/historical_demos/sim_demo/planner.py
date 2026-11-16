"""
Pick-and-place planner for ONE peg (circle).

The circle has continuous rotational symmetry, so only translation matters: no wrist-roll / orientation
alignment.  The planner chooses ONE constant tool tilt for the whole task (the peg stays vertical in the
gripper, so the same tilt is needed when it is lowered into the socket) as the smallest tilt for which
every waypoint and every straight-line segment is reachable.  The result carries a feasibility table so
the choice is explainable.
"""
from dataclasses import dataclass, field
import numpy as np
import arm_model as A

GRASP_DEPTH = 0.045       # TCP this far BELOW the peg's top face (pads overlap the upper 4-5 cm of the peg)
HOVER_ABOVE_TOP = 0.015   # fingertips this far above the peg top when hovering
CARRY_CLEARANCE = 0.030   # extra height above the higher of (grasp, place) heights for the transit
RETREAT_CLEAR = 0.022     # after release, fingertips must end this far ABOVE the peg top (open fingers straddle the peg)
PLACE_CLEARANCE = 0.004   # peg bottom this far above the board when released
TILTS_DEG = list(range(0, 61, 5))
TESTED_MAX_TILT_DEG = 65  # grasp physics verified up to here (see README)


@dataclass
class Waypoint:
    name: str
    p: np.ndarray            # TCP target (robot frame, m); for 'joint' moves it is the FK of q
    kind: str                # 'joint' | 'line'
    gripper: str = None      # None | 'open' | 'close'  (applied after arriving)
    duration: float = 1.5
    q: np.ndarray = None


@dataclass
class Plan:
    tilt_deg: float
    waypoints: list
    z_grasp: float
    z_place: float
    z_carry: float
    hold_offset: float
    z_hover: float = 0.0
    feasibility: list = field(default_factory=list)
    notes: list = field(default_factory=list)


def _line_ok(p0, p1, tilt, n=30, max_jump=0.22):
    prev = None
    for s in np.linspace(0, 1, n):
        p = (1 - s) * p0 + s * p1
        q = A.ik(*p, tilt=tilt)
        if q is None:
            return False, f"IK fails at ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})"
        if prev is not None and np.max(np.abs(q - prev)) > max_jump:
            return False, "joint branch jump"
        prev = q
    return True, ""


def _pt_ok(p, tilt, margin=0.008):
    r = np.hypot(p[0], p[1]); ux, uy = p[0] / r, p[1] / r
    for dr in (-margin, 0.0, margin):
        if A.ik((r + dr) * ux, (r + dr) * uy, p[2], tilt) is None:
            return False
    return True


def build_waypoints(peg, hole, board_z, tilt_deg, shape="circle"):
    t = np.radians(tilt_deg)
    px, py, ptop = peg["x"], peg["y"], peg["z_top"]
    z_grasp = ptop - GRASP_DEPTH
    z_bottom = ptop - peg["height"]
    hold_offset = z_grasp - z_bottom                          # TCP height above the peg bottom
    z_place = board_z + hold_offset + PLACE_CLEARANCE
    z_carry = max(z_grasp, z_place) + CARRY_CLEARANCE
    z_hover = ptop + HOVER_ABOVE_TOP
    hx, hy = hole["x"], hole["y"]
    P = lambda x, y, z: np.array([x, y, z], float)
    wps = [
        Waypoint("pick_hover",   P(px, py, z_hover), "joint", "open", 2.0),
        Waypoint("pick_descend", P(px, py, z_grasp), "line", None, 1.6),
        Waypoint("grasp_close",  P(px, py, z_grasp), "line", "close", 0.1),
        Waypoint("pick_lift",    P(px, py, z_carry), "line", None, 1.4),
        Waypoint("transit",      P(hx, hy, z_carry), "line", None, 2.4),
    ]
    # For non-circle shapes (square, hexagon, triangle), include dedicated alignment phase
    if shape != "circle":
        wps.append(Waypoint("align_with_hole", P(hx, hy, z_place + 0.015), "line", None, 1.2))
    wps += [
        Waypoint("place_descend", P(hx, hy, z_place), "line", None, 1.6),
        Waypoint("release_open", P(hx, hy, z_place), "line", "open", 0.1),
        Waypoint("place_retreat", P(hx, hy, max(z_carry, board_z + peg["height"] + PLACE_CLEARANCE + RETREAT_CLEAR)), "line", None, 1.6),
    ]
    for w in wps:
        w.q = A.ik(*w.p, tilt=t)
    return wps, dict(z_grasp=z_grasp, z_place=z_place, z_carry=z_carry, hold_offset=hold_offset, z_hover=z_hover)


def plan_pick_place(peg, hole, board_z, shape=None) -> Plan:
    table = []
    chosen = None
    target_shape = shape or peg.get("shape", "circle")
    for td in TILTS_DEG:
        t = np.radians(td)
        wps, z = build_waypoints(peg, hole, board_z, td, shape=target_shape)
        reason = None
        for w in wps:
            if w.q is None or not _pt_ok(w.p, t, margin=0.0 if w.name == 'place_retreat' else 0.008):
                reason = f"'{w.name}' at r={np.hypot(w.p[0], w.p[1])*100:.1f} cm, z={w.p[2]*100:.1f} cm not reachable"
                break
        if reason is None:
            prev = None
            for w in wps:
                if w.kind == "line" and prev is not None and np.linalg.norm(w.p - prev.p) > 1e-6:
                    ok, why = _line_ok(prev.p, w.p, t)
                    if not ok:
                        reason = f"segment ->'{w.name}': {why}"; break
                prev = w
        table.append(dict(tilt_deg=td, feasible=reason is None, reason=reason or "all waypoints & segments reachable"))
        if reason is None and chosen is None:
            chosen = (td, wps, z)
    if chosen is None:
        return None
    td, wps, z = chosen
    notes = []
    if td > TESTED_MAX_TILT_DEG:
        notes.append(f"tilt {td} deg exceeds the physics-verified range (<= {TESTED_MAX_TILT_DEG} deg)")
    if td > 0:
        notes.append("vertical top-grasp cannot reach every waypoint: using a tilted grasp (peg stays vertical in the gripper)")
    return Plan(tilt_deg=td, waypoints=wps, feasibility=table, notes=notes, **z)
