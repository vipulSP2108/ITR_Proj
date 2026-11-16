# Test Suite Overview

**Location**: `tests/`

---

## Test Files

### 1. `test_vision_pipeline.py` — End-to-End Vision Tests

| Test | What It Validates | Pass Criteria |
|------|-------------------|---------------|
| 4-peg detection | All 4 shapes detected with correct color/shape | < 2.5 px 2D error, < 3 mm depth, < 3.5 mm 3D |
| Symmetry rotation | Modulo-symmetry yaw angle extraction | Square=4-fold, Hex=6-fold, Tri=3-fold, Circle=invariant |
| Socket detection | 4 sockets detected and matched 1-to-1 with pegs | All 4 matched correctly |
| Multi-seed robustness | 10 random scenes (40 pegs total) | All detected across varied positions |
| Diagnostic visualization | Debug overlay image generation | File created, non-empty |

**Uses**: `SyntheticSceneGenerator`, `ColorSegmenter`, `ShapeDetector`

---

### 2. `test_edge_case_recovery.py` — P1 Edge Case Tests

| Test | Edge Case | What It Validates |
|------|-----------|-------------------|
| Workspace reachability | EC-06 | Correct classification: vertical (r<215mm), angled (215-290mm), sweep (290-340mm), out-of-bounds (>380mm) |
| Gripper secure hold | EC-05/EC-08 | Position 2100 counts + current 40mA → SECURE_HOLD |
| Gripper empty air | EC-08 | Position 2350 counts → EMPTY_AIR |
| In-transit drop | EC-05 | Snap-closed + low current → raises PegDroppedException |
| Camera drift detection | EC-07 | Simulates 16mm translation + 3.5° rotation → drift detected → auto-recalibrated |
| Spiral search offsets | EC-09 | 12-step Archimedean spiral, all offsets within 2mm radius |
| RecoveryManager orchestration | All P1 | End-to-end recovery sequence with mock arm |
| Visual drop detection | EC-05 | 4 scenarios: secure-in-hand, missed-at-pickup, tabletop-drop, socket-occupancy |

**Uses**: `MockDynamixelArm`, `WorkspaceAnalyzer`, `GripperGuard`, `CameraDriftMonitor`, `VisualDropDetector`, `RecoveryManager`

---

### 3. `test_kinematics.py` — FK/IK Roundtrip Tests

| Test | What It Validates | Pass Criteria |
|------|-------------------|---------------|
| 5-point roundtrip | FK(IK(target)) ≈ target | < 0.2 mm position error, < 0.01 rad pitch error |
| Unreachable targets | IK returns None for impossible positions | 3 unreachable points all return None |
| Encoder conversion | raw↔radian roundtrip | Zero radians → [4096, 2048, 2048, 2048] counts |

**Test points**:
- (200, 0, 40, -π/2) — straight ahead, vertical down
- (150, 100, 60, -π/2) — diagonal left
- (150, -100, 60, -π/2) — diagonal right
- (180, 0, 80, -π/3) — tilted 60°
- (120, 50, 100, -π/4) — tilted 45°

---

## Running Tests

```bash
cd "T1 2"
python -m pytest tests/ -v
```

All tests use synthetic data (no hardware or camera required).

---

## Simulation Benchmarks (`sim_demo/evaluate_timing.py`)

Automated benchmark across N randomized trials:

| Metric | Measured |
|--------|---------|
| Stage timing | Home, Calibration, Workspace, Pick-and-Place (independently timed) |
| Success rate | % of trials with successful placement |
| Mean placement error | mm from peg center to socket center |
| Cycle time | Total pipeline execution time |

Output: `benchmark_summary.md` + `benchmark_results.json` with rubric verification checkboxes.
