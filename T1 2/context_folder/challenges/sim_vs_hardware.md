# Challenge: Simulation-to-Hardware Gap

**The critical merge hazard in this project.**

---

## The Two Lineages

| Property | Legacy (`src/`) | Sim (`sim_demo/`) |
|----------|----------------|-------------------|
| **Units** | Millimetres, degrees, encoder counts | Metres, radians |
| **Joint zero** | Straight vertical arm (q=0 → all links collinear upward) | L-pose (q=0 → forearm horizontal) |
| **q3 zero offset** | ~0° | ~1.386 rad (~79.4°) difference |
| **Shoulder offset** | Ignored (L1 = pure vertical) | Modeled: ELBOW_OFF = (0.024, 0.128) m |
| **Camera assumption** | 45–55° angled (from setup notes) | Straight down at 0.61 m |
| **HSV thresholds** | Tuned for real RGB stream | Tuned for MuJoCo render |
| **Gripper gap** | ~60 mm (measured) | 70 mm (modeled) |
| **Calibration marker** | None physical | Magenta sphere on mast |
| **Depth** | RealSense D435i | MuJoCo rendered + noise model |

---

## Critical Merge Hazards

### 1. Unit System Collision
Legacy code uses mm everywhere. Sim uses metres. A bare `x = 0.15` means 0.15 mm in legacy, 0.15 m (150 mm) in sim. **Convert only at module boundaries**, never inline.

### 2. Joint-Zero Convention Conflict
This is the most dangerous one. The IK solver in `src/planning/kinematics.py` and `sim_demo/arm_model.py` solve the same physical arm but define q=0 differently:

```
Legacy:  q = [0, 0, 0, 0]  → arm straight vertical (all links up)
Sim:     q = [0, 0, 0, 0]  → L-pose (forearm horizontal, "ready" position)
```

**The q3 offset is ~1.386 radians.** Sending sim joint angles directly to real servos would slam the arm into the wrong position.

### 3. Camera Angle Uncertainty
- Sim: overhead at 470mm, looking straight down
- Real setup notes: 45–55° angled (unverified)
- **Impact**: Changes all pixel-to-robot projections, occlusion patterns, and depth-to-distance mapping

### 4. Socket Distance Uncertainty
- Legacy assumption: ~160 mm from base (vertical grasp suffices)
- Sim layout: ~317 mm from base (requires 60° tilt)
- **This is the biggest unknown for hardware deployment**

### 5. Gripper Gap Mismatch
- Sim: 70 mm open
- Real: ~60 mm open
- Peg cross-section: ~45 mm → fits both, but margin is tighter on hardware

### 6. Missing Physical Marker
Sim calibration uses a magenta ball on the wrist. The real robot has no such marker. Options:
- Attach a physical marker (tape/3D-print)
- Use the gripper-flick technique (already implemented in `autonomous_homography_calibrator.py`)

---

## Bridge Strategy: `RobotInterface` HAL

The `sim_demo/robot_interface.py` abstraction layer handles conversion:

```
Pipeline Code (metres, radians, sim conventions)
        │
        ▼
   RobotInterface (ABC)
        │
    ┌───┴───┐
    ▼       ▼
SimRobot  RealRobot
(MuJoCo)  (Dynamixel + conversion)
```

`RealRobot` converts:
- metres → encoder counts (via COUNTS_PER_RAD)
- sim joint convention → hardware convention (offset + sign)
- webcam capture → sim-format RGB-D pair

---

## Hardware Bring-Up Checklist (from `repare.md`)

1. Power safety check (voltage, USB isolator)
2. Ping all 5 motors
3. Read joint positions → FK → verify TCP location makes sense
4. Gentle nudge test (±5° on each joint)
5. Go to sim HOME_Q equivalent in hardware counts
6. Camera stream verification
7. Single gripper-flick calibration (homography)
8. Verify pixel_to_robot on known landmark
9. Single pick test (circle peg, vertical grasp, short reach)
10. Single place test (same peg, nearby socket)
11. Full pipeline dry run (elevated Z, no contact)
12. Full pipeline live run (circle only)
13. Multi-shape run

---

## Hard-Won Findings (from commit history)

1. **Retreat must clear peg top**: Open fingers straddle the peg after release. Lateral swing topples it. Fix: lift ≥22mm straight up before any sideways move.
2. **Recovery lift must go straight up first**: First version swept open fingers through the peg horizontally.
3. **Self-collision exclusions in MJCF**: Base ↔ link2/link3 collisions must be excluded or shoulder cannot swing backwards.
4. **Calibration must happen with table clear**: Arm sweeps at 7cm height during calibration poses — will knock over any peg.
5. **Home pose hides the table**: At home, the arm occludes the camera view. Must park aside for perception scans.
