# `recovery_manager.py` — Master Fault Recovery Engine

**Module**: `src/planning/recovery_manager.py`
**Purpose**: Orchestrates autonomous recovery from all P1 (task-critical) edge cases during pick-and-place operations.

---

## Recovery Protocols

### `recover_from_drop()` — EC-05: Peg Dropped During Transit

**5-Step Recovery**:

1. **Vertical ascent** to z=120mm — clear the drop zone
2. **Return to HOME** pose — known-safe configuration
3. **Camera re-scan** — `SingleCameraPipeline.process_scene()` to re-locate peg
4. **Reachability check** — `WorkspaceAnalyzer.classify_target()` on new peg position
5. **Re-execute pick-and-place** — full pipeline from new position

**Trigger**: `GripperGuard.monitor_transit_trajectory()` raises `PegDroppedException`, or `VisualDropDetector.verify_post_lift()` returns `DROPPED_ON_TABLE`.

### `resolve_insertion_jam()` — EC-09: Socket Insertion Failure

**Micro-Spiral Search**:
- Height: z=38mm (slightly above board surface)
- Pattern: `GripperGuard.generate_spiral_search_offsets(max_radius=2.0, num_steps=12)`
- At each offset (dx, dy): attempt controlled descent to insertion depth
- If gripper load indicates seating → success
- If all 12 positions fail → retreat, re-scan, retry

---

## Priority Hierarchy

| Priority | Category | Example | Recovery |
|----------|----------|---------|----------|
| **P0** | Safety | Over-current, collision | Immediate torque disable |
| **P1** | Task-critical | Drop (EC-05), unreachable (EC-06), empty grasp (EC-08), jam (EC-09) | Autonomous recovery |
| **P2** | Perception | Camera bump (EC-07), occlusion | Online recalibration |
| **P3** | Tolerances | Insertion depth offset, color threshold drift | Parameter adjustment |

---

## Integration Points

```
GripperGuard ──→ PegDroppedException ──→ recover_from_drop()
VisualDropDetector ──→ DROPPED_ON_TABLE ──→ recover_from_drop()
GripperGuard ──→ EMPTY_AIR ──→ retry pickup (adjust z_grasp)
Insertion failure ──→ resolve_insertion_jam()
CameraDriftMonitor ──→ drift detected ──→ recalibrate_online()
WorkspaceAnalyzer ──→ OUT_OF_BOUNDS ──→ generate_inward_sweep_trajectory()
```

---

## Design Decisions

- **Always return HOME before recovery**: Ensures arm is in known configuration. Prevents compound errors.
- **Camera re-scan after drop**: Don't assume peg is where it was. It may have bounced or rolled.
- **2mm spiral radius**: Matches typical calibration error (RMSE 0.85mm × 2σ ≈ 1.7mm). Covers alignment uncertainty.
- **12 spiral steps**: 30° angular spacing. Covers the full 2mm radius circle uniformly.
- **z=38mm for spiral**: Above board surface (socket depth ~28mm) + clearance. Avoids jamming while probing.
