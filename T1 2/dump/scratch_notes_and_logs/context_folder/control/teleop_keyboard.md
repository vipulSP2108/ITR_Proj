# `teleop_keyboard.py` — Interactive Keyboard Teleoperation

**Module**: `src/control/teleop_keyboard.py`
**Purpose**: Manual keyboard control for calibration, testing, and demonstration.

---

## Control Mapping

| Key | Action |
|-----|--------|
| W/S | Joint 2 (Shoulder) up/down |
| A/D | Joint 1 (Base Pan) left/right |
| I/K | Joint 3 (Elbow) up/down |
| U/J | Joint 4 (Wrist) up/down |
| O/P | Joint 5 (Gripper) open/close |
| 1/2/3 | Step size: 1°/2.5°/5° |
| E | Emergency stop (disable all torque) |
| T | Toggle torque on/off |
| H | Return to home pose |
| Q | Quit |

---

## Joint Limits (raw counts)

| Joint | Min | Max |
|-------|-----|-----|
| 11 (Base) | 200 | 3900 |
| 12 (Shoulder) | 1000 | 3100 |
| 13 (Elbow) | 900 | 3200 |
| 14 (Wrist) | 900 | 3200 |
| 15 (Gripper) | 1400 | 2700 |

---

## Design Decisions

- **Three step sizes**: Fine (1°) for precision alignment during calibration, medium (2.5°) for normal operation, coarse (5°) for quick repositioning.
- **Emergency stop**: Immediate torque disable on 'E'. Arm falls under gravity but prevents motor damage.
- **Wider Joint 1 range**: 200–3900 (vs. other joints ~1000–3200) because base pan needs near-full rotation for workspace coverage.
