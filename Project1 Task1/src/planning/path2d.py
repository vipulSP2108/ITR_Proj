"""
Collision-free 2D path planning on the table plane: A* on a grid with inflated obstacles,
a soft "keep your distance" cost, and line-of-sight shortcutting.

Why 2D is enough here: every peg is 100 mm tall and the arm cannot lift a carried peg above the
others, so a carried peg (and the fingers around it) must go AROUND standing pegs. The arm links
stay above the pegs whenever the tool points down. So the moving thing is a disc (peg + fingers,
radius mover_r) and the obstacles are discs (other pegs).

Clearances are edge-to-edge gaps between the mover disc and an obstacle disc:
    hard  - never closer than this (cells are blocked)
    soft  - preferred; cells closer than this cost extra, so the path keeps a wide berth if it can
    end   - reduced hard gap inside escape_r of start and goal (a neighbour may legitimately
            stand near the pick / release spot; the caller checks the fingers there separately)

All coordinates are robot-frame millimetres. The planning area is given by the caller (it comes
from the mat seen by the camera) - nothing here is tied to a particular table layout.
"""
import heapq
import math
from typing import Callable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Disc = Tuple[float, float, float]          # x, y, radius
CELL = 5.0


def gap_to_obstacles(p: Point, obstacles: Sequence[Disc], mover_r: float) -> float:
    """Edge-to-edge gap between the mover disc centred at p and the nearest obstacle."""
    return min((math.hypot(p[0] - ox, p[1] - oy) - orad - mover_r for ox, oy, orad in obstacles), default=1e9)


def plan(start: Point, goal: Point, obstacles: Sequence[Disc], mover_r: float,
         reachable: Callable[[float, float], bool], bounds: Tuple[float, float, float, float],
         hard: float = 20.0, soft: float = 45.0, end: float = 8.0, escape_r: float = 70.0) -> Optional[List[Point]]:
    """Cheapest collision-free path start -> goal, or None. bounds = (xmin, xmax, ymin, ymax)."""
    x0, x1, y0, y1 = bounds
    nx, ny = int((x1 - x0) / CELL) + 1, int((y1 - y0) / CELL) + 1

    def to_cell(p):
        return (int(round((p[0] - x0) / CELL)), int(round((p[1] - y0) / CELL)))

    def to_xy(c):
        return (x0 + c[0] * CELL, y0 + c[1] * CELL)

    cache = {}

    def info(c):
        """(allowed, extra cost per mm) for a cell."""
        if c not in cache:
            if not (0 <= c[0] < nx and 0 <= c[1] < ny):
                cache[c] = (False, 0.0)
            else:
                p = to_xy(c)
                g = gap_to_obstacles(p, obstacles, mover_r)
                near_end = min(math.hypot(p[0] - start[0], p[1] - start[1]),
                               math.hypot(p[0] - goal[0], p[1] - goal[1])) < escape_r
                need = end if near_end else hard
                cache[c] = (g >= need and reachable(*p), 3.0 * max(0.0, (soft - g) / soft))
        return cache[c]

    s, g_ = to_cell(start), to_cell(goal)
    cache[s] = cache[g_] = (True, 0.0)
    moves = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    openq, came, cost = [(0.0, s)], {s: None}, {s: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == g_:
            break
        for dx, dy, d in moves:
            nb = (cur[0] + dx, cur[1] + dy)
            allowed, extra = info(nb)
            if not allowed:
                continue
            nc = cost[cur] + d * (1.0 + extra)
            if nc < cost.get(nb, 1e18):
                cost[nb], came[nb] = nc, cur
                heapq.heappush(openq, (nc + math.hypot(nb[0] - g_[0], nb[1] - g_[1]), nb))
    if g_ not in came:
        return None
    cells, c = [], g_
    while c is not None:
        cells.append(c)
        c = came[c]
    pts = [start] + [to_xy(c) for c in reversed(cells)][1:-1] + [goal]

    def seg_cost(a, b):
        """Cost of flying straight a -> b, or None if it is not allowed."""
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (CELL / 2)))
        tot = 0.0
        for i in range(n + 1):
            allowed, extra = info(to_cell((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)))
            if not allowed:
                return None
            tot += (1.0 + extra)
        return tot / (n + 1) * math.hypot(b[0] - a[0], b[1] - a[1])

    # shortcut only where the straight line is not more expensive than the detour it replaces
    # (so a shortcut never drags the path back towards an obstacle)
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            direct = seg_cost(pts[i], pts[j])
            via = sum(seg_cost(pts[k], pts[k + 1]) or 0.0 for k in range(i, j))
            if direct is not None and direct <= via * 1.02 + 1e-6:
                break
            j -= 1
        out.append(pts[j])
        i = j
    return out


def path_length(path: Sequence[Point]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def min_gap(path: Sequence[Point], obstacles: Sequence[Disc], mover_r: float) -> float:
    """Smallest edge-to-edge gap (mm) between the mover disc and any obstacle along the whole path."""
    best = 1e9
    for a, b in zip(path, path[1:]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 2))
        for i in range(n + 1):
            best = min(best, gap_to_obstacles((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n), obstacles, mover_r))
    return best
