# Challenge: Workspace Reachability & Tilt Strategy

**Problem**: OpenMANIPULATOR-X has limited reach (~218mm vertical) but board sockets can be up to ~300mm away.

---

## The Core Constraint

With the tool pointing straight down (vertical top-grasp, pitch = -90°):
- **Maximum reach at z=45mm: 218mm**
- Many socket positions on the board are at 250–300mm from base

This means pure vertical grasping CANNOT reach all targets.

---

## Solution: Tilted Grasp Strategy

By tilting the tool away from vertical, the effective reach extends:

| Tilt Angle | Approximate Reach | Trade-off |
|------------|-------------------|-----------|
| 0° (vertical) | 218 mm | Most stable grasp |
| 30° | ~260 mm | Good stability |
| 50° | ~295 mm | Reduced grasp quality |
| 60° | ~310 mm | Minimum acceptable stability |

### How Tilt Extends Reach

The TCP is at the end of L4 (126mm). When tilted by angle θ from vertical:
```
reach_extension = L4 × sin(θ)
At 50°: 126 × sin(50°) = 96.5mm additional reach
```
Total: 218 + 96.5 ≈ 315mm — sufficient for all board positions.

---

## Implementation

### Simulation (`planner.py`)
- Tests tilt 0° to 60° in 5° steps
- Uses first (most vertical) valid solution
- **Constant tilt** for entire trajectory (chosen at planning time)
- Demo run chose 50° tilt for target at 297mm

### Hardware (`run_live_hardware_sorter.py`)
- Tries pitch from -90° to -40° in 5° steps
- First valid IK solution is used
- More aggressive range (-40° = 50° tilt) for hardware reliability

---

## Real Numbers from Demo Runs

From `demo_20261108_043748/summary.md`:
```
square peg at (15.0, -6.0) cm, 16.1 cm from base
square socket at (29.5, 2.8) cm, 29.7 cm from base
vertical top-grasp reach at grasp height: 21.8 cm
chosen constant tool tilt: 50 deg
result: SUCCESS - vision error peg→socket 6.4 mm
```

The socket at 297mm required 50° tilt. Vertical reach (218mm) was insufficient.

---

## Key Insight

The choice of tilt angle is the single most critical planning decision. It determines:
1. Whether the target is reachable at all
2. Grasp stability (vertical = best, tilted = worse)
3. Peg orientation relative to socket (tilted grasp may rotate the peg)
4. Joint limit margins (tilted configurations are closer to joint limits)
