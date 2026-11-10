# Project Timeline & Evolution

**Duration**: 6 days (2026-11-03 to 2026-11-08)

---

## Phase 1: Hardware Foundation (Nov 3)

**Commit**: `03a933a` — Initial commit

- Established U2D2 controller wrapper (motor IDs 11–15, 1 Mbps, macOS auto-detection)
- Built incremental test scripts: scan → gripper → joint nudge → teleop → home → pick-and-place
- Dynamixel SDK integration with Protocol 2.0
- Keyboard teleoperation with E-stop
- Conservative velocity profiles (20–25 ≈ 1.5 RPM)

**Lesson**: Start with the most basic "can I talk to the motors" test, then build up incrementally.

---

## Phase 2: Vision & Perception (Nov 4)

**Commit**: `9d9d406` — Vision pipeline + calibration + edge cases

- HSV color segmentation (4 colors with red hue wrap-around)
- Shape detection (enclosing-circle ratio + vertex count)
- Depth slicing to isolate peg top faces (top 12mm)
- Single-camera pipeline
- Eye-to-hand calibration via Kabsch SVD
- Workspace analyzer with 4-tier reachability
- GripperGuard (proprioceptive slip/drop detection)
- RecoveryManager for all P1 edge cases
- **15 edge cases documented across 4 priority tiers** (before code was written for most of them)

**Lesson**: Document edge cases upfront — they drive the architecture.

---

## Phase 3: MuJoCo Simulation (Nov 6)

**Commit**: `6f8b566` — 3D physics simulation

- MJCF XML scenes (multiple scales and configurations)
- Interactive 3D viewer with autonomous sorting
- Offscreen headless renderer for video generation
- 3D trajectory visualization (Matplotlib)
- Discovered that SCALE=0.78 factor needed between sim and expected dimensions

**Lesson**: Simulation caught the self-collision bug and reach constraints before hardware testing.

---

## Phase 4: Live Hardware Demo Prep (Nov 7, morning)

**Commits**: `023161c`, `70a6e9b`, `020467d`

- Multi-view workspace analysis (4 survey poses)
- Circle peg trajectory tracker with 10 visual checkpoints
- Random circle peg placement with autonomous sorting
- macOS `mjpython` trampoline for Cocoa main-thread MuJoCo viewer

---

## Phase 5: Autonomous Sim Pipeline (Nov 7, afternoon)

**Commits**: `af1321f`, `4d83d27`, `c8cbd8a`

This is the **critical build phase**:
- Self-contained `sim_demo/` package (perception, calibration, workspace, planner, tracker)
- `run_pipeline.py` — full 3-stage pipeline (HOME → CALIBRATE + WORKSPACE → PICK & PLACE)
- Quintic trajectory generator + DLS-IK solver
- Multi-shape symmetry planner
- Benchmark evaluation suite
- Full test suite
- **`repare.md`** — 452-line merge guide documenting all conventions, hazards, and hardware bring-up steps

**Lesson**: The pipeline was designed sim-first but with a HAL (`RobotInterface`) that cleanly separates sim and hardware.

---

## Phase 6: Grid Sweep & Multi-Shape (Nov 8)

**Commits**: `3772244`, `1a81ac8`, `60e843f`, `bdcf822`, `40cbe21`, `e8937cd`

- 11×11 grid simulation runner (ASCII workspace map)
- Batch grid sweep (144 positions) with heatmap + CSV logging
- Interactive MuJoCo 3D viewer enabled by default
- Automatic storyboard recording with HTML gallery
- **Square peg sorting** with dedicated hole alignment phase and 90° symmetry handling

**Lesson**: Systematic grid testing reveals the workspace boundaries — which positions succeed, which need tilt, which are unreachable.

---

## Key Technical Pivots

| When | What Changed | Why |
|------|-------------|-----|
| Phase 2 | Documented edge cases before implementation | De-risks architecture decisions |
| Phase 3 | Introduced constant tool tilt | Vertical grasp can't reach distant sockets |
| Phase 5 | Created separate sim_demo/ package | mm/m and joint-zero conflicts too dangerous to merge |
| Phase 5 | HAL abstraction (RobotInterface) | Allows identical pipeline on sim and hardware |
| Phase 6 | Circle-first ordering | Validates Cartesian accuracy before attempting orientation |

---

## What Exists vs. What's Pending

| Component | Status |
|-----------|--------|
| Hardware motor control | ✅ Complete and tested |
| Vision pipeline (real camera) | ✅ Complete |
| Vision pipeline (sim) | ✅ Complete |
| Sim pipeline (full) | ✅ Complete + grid-validated |
| Circle peg (sim) | ✅ Demonstrated |
| Square peg (sim) | ✅ Demonstrated |
| Hexagon/Triangle (sim) | ⚠️ Framework exists, not yet demonstrated |
| Hardware pipeline (circle) | ✅ Demonstrated (video exists) |
| Hardware pipeline (all 4) | ⚠️ Script exists, not yet run |
| Sim-to-real joint mapping | ❌ Not verified on hardware |
| Physical calibration marker | ❌ Not attached |
| Camera angle measurement | ❌ Not verified |
| Socket distance measurement | ❌ Not verified |
