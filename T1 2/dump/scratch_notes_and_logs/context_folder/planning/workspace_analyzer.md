# `workspace_analyzer.py` — Reachability Analysis (EC-06)

**Module**: `src/planning/workspace_analyzer.py`
**Purpose**: Classifies target positions into reachability zones and generates alternative grasp strategies for out-of-reach targets.

---

## Concentric Reachability Zones

```
             ┌─────────────────────────────┐
             │   OUT OF BOUNDS (> 340mm)    │
             │  ┌───────────────────────┐   │
             │  │  SWEEP/RAKE (≤ 340mm) │   │
             │  │  ┌─────────────────┐  │   │
             │  │  │ ANGLED (≤ 290mm)│  │   │
             │  │  │ ┌─────────────┐ │  │   │
             │  │  │ │ PRIME       │ │  │   │
             │  │  │ │ (≤ 215mm)   │ │  │   │
             │  │  │ │  ●          │ │  │   │
             │  │  │ │ Base(95mm+) │ │  │   │
             │  │  │ └─────────────┘ │  │   │
             │  │  └─────────────────┘  │   │
             │  └───────────────────────┘   │
             └─────────────────────────────┘
```

| Zone | Radius Range | Grasp Strategy |
|------|-------------|----------------|
| **Too close** | < 95 mm | Unreachable (self-collision) |
| **Prime Vertical** | 95–215 mm | Standard vertical top-grasp (pitch = -90°) |
| **Angled Grasp** | 215–290 mm | Tilted grasp (pitch swept -85° to -60°) |
| **Sweep/Rake** | 290–340 mm | Drag peg inward, then grasp |
| **Out of Bounds** | > 340 mm | Unreachable |
| **Hard Limit** | > 380 mm | Absolute maximum reach |

---

## Key Methods

### `classify_target(x, y, z) → (ReachabilityStatus, telemetry_dict)`

Computes radial distance `r = √(x² + y²)`, returns zone classification plus:
- Distance from base
- Nearest zone boundary
- Recommended strategy

### `solve_optimal_angled_grasp(x, y, z) → (q1, q2, q3, q4) or None`

For targets in the Angled Grasp zone (215–290mm):
1. Sweep pitch from **-85° to -60°** in 2.5° increments
2. Call `inverse_kinematics(x, y, z, pitch_rad)` at each angle
3. Return first valid solution (most vertical possible)

**Why sweep from -85° toward -60°?** More vertical = more stable grasp. Only tilt as much as necessary.

### `generate_inward_sweep_trajectory(x, y, z) → list[waypoints]`

4-step sweep/rake for targets at 290–340mm:
1. **Position behind target** (r + 30mm, elevated)
2. **Lower** to table surface
3. **Drag inward** toward base (linear Cartesian motion)
4. **Elevate** — peg is now in Prime/Angled zone

---

## Actual Reach Values (from sim demo runs)

| Configuration | Vertical Reach |
|--------------|----------------|
| z=45mm (grasp height) | **218 mm** |
| z=0mm (table surface) | ~250 mm |
| Tilted at 50° | ~295 mm |

From demo run `demo_20261108_043748/summary.md`:
> "vertical top-grasp reach at grasp height: 21.8 cm"
> "chosen constant tool tilt: 50 deg"

---

## Design Decisions

- **Concentric zones**: Simple radial classification is fast (<1μs) and covers all cases. No need for full IK workspace sampling at runtime.
- **2.5° pitch increments**: Fine enough to find solutions (IK boundary is smooth) but only 10 iterations maximum.
- **Sweep/rake as last resort**: Moving objects on the table risks bumping other pegs. Only used when direct grasp is impossible.
- **95mm minimum radius**: Prevents self-collision. Arm links would interfere below this distance.
