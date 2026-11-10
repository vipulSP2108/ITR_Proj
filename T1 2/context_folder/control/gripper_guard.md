# `gripper_guard.py` — Proprioceptive Grasp Monitoring

**Module**: `src/control/gripper_guard.py`
**Purpose**: Real-time grasp state monitoring using gripper encoder position and motor current. Covers EC-05 (drop/slip), EC-08 (empty grasp), EC-09 (socket jamming).

---

## Gripper Position Thresholds (Motor 15, raw counts)

| Threshold | Value | Meaning |
|-----------|-------|---------|
| `OPEN_COUNT` | 1700 | Fully open gripper |
| `GRASP_MIN` | 1920 | Minimum position for valid object grasp |
| `GRASP_MAX` | 2240 | Maximum position for valid object grasp |
| `EMPTY_CLOSED` | 2310 | Closed on air (no object) |

**Interpretation**: Gripper position between 1920–2240 means fingers are stopped by an object. Above 2310 means fingers closed completely = nothing held.

---

## Current Thresholds

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `CURRENT_UNIT_MA` | 2.69 mA/count | XM430 current conversion |
| `MIN_HOLDING` | 25 mA | Minimum current indicating object resistance |
| `MAX_IDLE` | 15 mA | Maximum current with no load (empty grip) |

---

## Grasp States

| State | Condition |
|-------|-----------|
| `OPEN` | Position < OPEN_COUNT |
| `SECURE_HOLD` | GRASP_MIN ≤ position ≤ GRASP_MAX AND current ≥ MIN_HOLDING |
| `EMPTY_AIR` | Position > EMPTY_CLOSED OR current < MAX_IDLE |
| `SLIPPED_DROPPED` | Position changed significantly from last SECURE reading |

---

## Key Methods

### `evaluate_grasp(verify_cycles=3) → GraspState`

Multi-sample averaging over 3 cycles to reject transient readings:
1. Read gripper position (motor 15)
2. Read gripper current
3. Classify state from position + current thresholds
4. Majority vote across cycles

### `monitor_transit_trajectory(duration, poll_hz=20)`

Continuous monitoring during peg transit:
- Polls gripper state at 20 Hz (every 50ms)
- Raises `PegDroppedException` immediately if state transitions from SECURE_HOLD to SLIPPED_DROPPED
- Duration-limited: stops after specified seconds

### `generate_spiral_search_offsets(max_radius=2.0, num_steps=12) → list[(dx, dy)]`

**Archimedean spiral** for socket insertion compliance (EC-09):
```
for i in 0..num_steps:
    r = max_radius × (i / num_steps)
    θ = i × (2π / num_steps) × 3    # 3 full spiral turns
    dx = r × cos(θ)
    dy = r × sin(θ)
```
Used when peg doesn't seat cleanly — tries small XY offsets in spiral pattern.

---

## Design Decisions

- **Position + current dual check**: Position alone can't distinguish "object held loosely" from "fingers stopped by friction". Current confirms actual resistive load.
- **3-cycle majority vote**: Single reads are noisy during dynamic motion. Three samples at ~10ms intervals give reliable classification.
- **20Hz transit polling**: Matches the Dynamixel read cycle time (~5ms per read × 2 registers × 5 motors). Fast enough to catch drops within 50ms.
- **Archimedean spiral**: Systematic search pattern covers the error radius uniformly. Better than random perturbation for socket alignment.

---

## Edge Case Coverage

| Edge Case | Detection Method | Recovery Action |
|-----------|-----------------|-----------------|
| EC-05: Drop during transit | `monitor_transit_trajectory()` | `RecoveryManager.recover_from_drop()` |
| EC-08: Empty grasp | `evaluate_grasp()` → EMPTY_AIR | Re-attempt pickup with adjusted height |
| EC-09: Socket jamming | Spiral search offsets | Micro-compliance insertion at z=38mm |
