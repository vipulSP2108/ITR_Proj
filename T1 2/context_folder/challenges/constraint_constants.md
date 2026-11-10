# Constraint Constants — Definitive Reference

All physical constraints, thresholds, and magic numbers in one place.

---

## Robot Geometry (mm)

| Parameter | Value | Source |
|-----------|-------|--------|
| L1 (Base→Shoulder) | 77.0 | `kinematics.py`, `arm_model.py` |
| L2 (Shoulder→Elbow) | 130.0 | Same |
| L3 (Elbow→Wrist) | 124.0 | Same |
| L4 (Wrist→TCP) | 126.0 | Same |
| Shoulder offset (sim only) | (24, 128) mm | `arm_model.py` ELBOW_OFF |
| Max horizontal reach | ~380 mm | `workspace_analyzer.py` |
| Max vertical reach | ~450 mm | `device/ROBOT_INFO.md` |

---

## Workspace Radii (mm from base center)

| Zone | Radius | Strategy |
|------|--------|----------|
| Inner collision | < 95 | Unreachable (self-collision) |
| Vertical grasp | ≤ 215 | Top-down pick, pitch = -90° |
| Angled grasp | 215–290 | Tilted tool, pitch sweep -85° to -60° |
| Inward sweep | 290–340 | Drag peg inward, then pick |
| Hard limit | > 380 | Out of bounds |

---

## Manipulation Heights (mm)

### Hardware Pipeline (`workspace_dimensions.yaml`)

| Phase | Z (mm) |
|-------|--------|
| Hover | 105 |
| Transit | 105 |
| Socket hover | 85 |
| Grasp | 35 |
| Insert | 28 |

### Simulation Pipeline (`sim_demo/planner.py`)

| Phase | Z (m) | (mm) |
|-------|--------|------|
| GRASP_DEPTH below peg top | 0.045 | 45 |
| HOVER_ABOVE_TOP | 0.015 | 15 |
| CARRY_CLEARANCE | 0.030 | 30 |
| RETREAT_CLEAR | 0.022 | 22 |
| PLACE_CLEARANCE | 0.004 | 4 |

### Recovery Manager (`recovery_manager.py`)

| Phase | Z (mm) |
|-------|--------|
| Safe ascent | 120 |
| Pre-grasp hover | 80 |
| Grasp/insert | 40 |
| Transit | 85 |
| Seat depth | 38 |

---

## Encoder Constants

| Parameter | Value |
|-----------|-------|
| Counts per revolution | 4096 |
| Center position | 2048 counts = 0° |
| Counts per radian | 651.9 |
| Counts per degree | 11.378 |
| Degrees per count | 0.08789° |

---

## Joint Limits

| Joint | Legacy (IK enforcement) | Teleop (software clamp) |
|-------|------------------------|------------------------|
| J1 (Base) | ±π rad | 200–3900 counts (±160°) |
| J2 (Shoulder) | ±90° + 0.05 rad | 1000–3100 counts |
| J3 (Elbow) | ±118° | 900–3200 counts |
| J4 (Wrist) | ±110° | 900–3200 counts |
| J5 (Gripper) | — | 1400–2700 counts |

---

## Gripper Thresholds

| Parameter | Value | Source |
|-----------|-------|--------|
| Open (nominal) | 1700 counts | `gripper_guard.py` |
| Grasp engagement min | 1920 counts | `gripper_guard.py` |
| Grasp engagement max | 2240 counts | `gripper_guard.py` |
| Empty-closed limit | 2310 counts | `gripper_guard.py` |
| Min holding current | 25.0 mA | `gripper_guard.py` |
| Max idle current | 15.0 mA | `gripper_guard.py` |
| Current unit | 2.69 mA/raw | `gripper_guard.py` |

---

## Tracking Thresholds

| Parameter | Value | Source |
|-----------|-------|--------|
| Warning threshold | 10.0 mm | `tracker.py` |
| Abort threshold | 40.0 mm | `tracker.py` |
| Lag tolerance | 0.12 s | `tracker.py` |
| Consecutive abort frames | 4 | `tracker.py` |

---

## Camera Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Height above table | 470 mm | `workspace_dimensions.yaml` |
| Focal length | 344 px | `workspace_dimensions.yaml` |
| Principal point | (320, 240) | `autonomous_homography_calibrator.py` |
| Resolution | 640 × 480 | Pipeline default |
| Scale | 1.366 mm/px | 470 / 344 |

---

## Tilt Strategy

| Tilt (°) | Effective Reach (mm) | Use Case |
|----------|---------------------|----------|
| 0 | 218 | Standard top-down grasp |
| 30 | 290 | Extended reach |
| 45 | 330 | Far pegs |
| 50 | ~340 | Square peg (demo run 3) |
| 60 | 350 | Circle socket at 317mm |
| 65 | max tested | Upper limit |

---

## Symmetry Alignment

| Shape | Symmetry Order | Interval | Compliance Window |
|-------|---------------|----------|-------------------|
| Circle | ∞ | 0° | No alignment needed |
| Hexagon | 6 | 60° | ±12° |
| Square | 4 | 90° | ±12° |
| Triangle | 3 | 120° | ±12° |
