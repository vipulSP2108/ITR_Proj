# `trajectory.py` — Quintic Trajectory & DLS IK Solver

**Module**: `sim_demo/trajectory.py`
**Purpose**: Smooth trajectory generation and numerical IK for Cartesian straight-line motion.

---

## Class: `QuinticTrajectory`

### Minimum-Jerk Profile
```
s(τ) = 10τ³ - 15τ⁴ + 6τ⁵     τ = t/T ∈ [0, 1]
ṡ(τ) = 30τ² - 60τ³ + 30τ⁴
s̈(τ) = 60τ - 180τ² + 120τ³
```

| Property | At τ=0 | At τ=1 |
|----------|--------|--------|
| Position | 0 | 1 |
| Velocity | 0 | 0 |
| Acceleration | 0 | 0 |

**Why quintic?** 6th-degree polynomial is the minimum degree that satisfies all 6 boundary conditions (position, velocity, acceleration at both endpoints). Results in the smoothest possible motion.

---

## Class: `DlsIkSolver` — Damped Least-Squares IK

| Parameter | Value |
|-----------|-------|
| Damping (λ) | 0.03 |
| Max iterations | 25 |
| Convergence tolerance | 0.1 mm |

### Algorithm
```
At each iteration:
    e = target_pose - current_pose     (position error)
    J = jacobian(current_q)            (6×4 Jacobian)
    Δq = J.T @ (J @ J.T + λ²I)⁻¹ @ e  (damped pseudo-inverse)
    q ← q + Δq
    if ||e|| < 0.1mm: converged
```

### Why Damped Least-Squares?

Near singularities, the standard pseudo-inverse `J.T @ (J @ J.T)⁻¹` produces infinite joint velocities. The damping term `λ²I` bounds the solution:

- **λ = 0**: Pure pseudo-inverse (fast convergence, unstable near singularities)
- **λ = 0.03**: Good balance — converges in ~5 iterations for typical waypoints, stable near workspace boundaries
- **λ → large**: Always stable but slow/inaccurate

---

## Usage in Pipeline

```
For each segment (waypoint_A → waypoint_B):
    quintic = QuinticTrajectory(duration)
    for t in range(0, duration, CTRL_DT):
        τ = t / duration
        s = quintic.evaluate(τ)
        target_xyz = waypoint_A + s × (waypoint_B - waypoint_A)  # linear Cartesian interpolation
        q = DlsIkSolver.solve(target_xyz, current_q)             # IK at each step
        command(q)
```

This produces straight-line Cartesian motion with smooth velocity profiles — no jerky starts/stops, no arc-shaped detours.
