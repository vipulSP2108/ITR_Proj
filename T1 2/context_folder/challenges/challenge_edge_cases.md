# Edge Cases — 15 Cases Across 4 Priority Tiers

**Source**: `EDGE_CASES.md`
**Purpose**: Comprehensive deterministic recovery protocols for every identified failure mode.

---

## Priority Tier System

| Tier | Name | Response Time | Examples |
|------|------|--------------|---------|
| **P0** | Safety | Immediate (<50ms) | Over-current, collision, e-stop |
| **P1** | Task-Critical | < 2 seconds | Drop, empty grasp, unreachable, jam |
| **P2** | Perception | < 500ms | Camera bump, occlusion, color drift |
| **P3** | Tolerances | Next planning cycle | Depth offset, insertion depth, threshold |

---

## P1 Edge Cases (Task-Critical)

### EC-05: Peg Drop/Slip During Transit
- **Detection**: `GripperGuard.monitor_transit_trajectory()` (proprioceptive) + `VisualDropDetector.verify_post_lift()` (visual)
- **Recovery**: `RecoveryManager.recover_from_drop()` — ascend, HOME, re-scan, re-pick
- **Root causes**: Insufficient grip force, vibration, peg surface texture

### EC-06: Target Out of Reach
- **Detection**: `WorkspaceAnalyzer.classify_target()` returns OUT_OF_BOUNDS
- **Recovery**: 
  - Angled grasp (tilt -85° to -60°) for 215–290mm range
  - Sweep/rake trajectory for 290–340mm range
- **Root cause**: Peg placed beyond vertical reach radius (218mm at z=45mm)

### EC-08: Empty Grasp (Missed Pickup)
- **Detection**: `GripperGuard.evaluate_grasp()` returns EMPTY_AIR
- **Recovery**: Re-attempt with adjusted z_grasp (-2mm lower), re-scan if second attempt fails
- **Root causes**: Peg height variation, calibration drift, peg shifted between scan and grasp

### EC-09: Socket Insertion Jamming
- **Detection**: Motor load spike without position convergence at socket depth
- **Recovery**: `RecoveryManager.resolve_insertion_jam()` — micro-spiral search at z=38mm
- **Root causes**: Calibration error, peg orientation mismatch, socket edge catch

---

## P2 Edge Cases (Perception)

### EC-07: Camera Bump During Operation
- **Detection**: `CameraDriftMonitor.compute_drift()` — translation > 4mm OR rotation > 1.5°
- **Recovery**: `CameraDriftMonitor.recalibrate_online()` — Kabsch SVD in <2ms
- **Root cause**: Physical contact with camera mount

---

## Design Philosophy

1. **Deterministic over probabilistic**: Every edge case has a fixed recovery sequence, not a retry loop with random perturbation
2. **Sensor fusion**: Proprioceptive (encoder + current) AND visual (camera) checks for critical states
3. **Graceful degradation**: P3 issues don't halt operation, they adjust parameters for next cycle
4. **Bounded retries**: Maximum 3 recovery attempts before escalating to human operator
5. **Always return HOME first**: Known-safe configuration before any recovery action
