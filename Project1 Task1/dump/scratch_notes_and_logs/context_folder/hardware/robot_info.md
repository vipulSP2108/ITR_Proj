# Robot Hardware — ROBOTIS OpenMANIPULATOR-X (RM-X52-TNM)

**Reference**: `device/ROBOT_INFO.md`

---

## Specifications

| Property | Value |
|----------|-------|
| **Model** | OpenMANIPULATOR-X (RM-X52-TNM) |
| **DOF** | 4 arm joints + 1 linear gripper = 5 total |
| **Actuators** | DYNAMIXEL XM430-W350-T (Model #1020) × 5 |
| **Voltage** | 12.0V DC (operating range 10.0–14.8V) |
| **Communication** | Protocol 2.0, 1 Mbps baud rate |
| **Interface** | ROBOTIS U2D2 USB-to-RS485 adapter |
| **macOS Port** | `/dev/cu.usbserial-FTA2W88F` |

---

## Kinematic Chain

```
Base (J1, ID=11) ─── 77mm ───> Shoulder (J2, ID=12) ─── 130mm ───>
Elbow (J3, ID=13) ─── 124mm ───> Wrist (J4, ID=14) ─── 126mm ───> TCP
                                                                      │
                                                             Gripper (J5, ID=15)
```

| Link | Length (mm) | From → To |
|------|-------------|-----------|
| L1 | 77 | Base center → Shoulder pitch axis |
| L2 | 130 | Shoulder → Elbow |
| L3 | 124 | Elbow → Wrist |
| L4 | 126 | Wrist → TCP (gripper center) |
| **Total** | **457** | Theoretical max reach (fully extended) |

**Practical horizontal reach**: ~380 mm (joint limits prevent full extension)
**Practical vertical reach**: ~450 mm

---

## Joint Configuration

| ID | Joint | Mode | Range | Home (counts) | Home (°) |
|----|-------|------|-------|---------------|----------|
| 11 | Base Pan | 4 (Multi-turn) | ±160° | 4096 (or 0) | 180° (forward) |
| 12 | Shoulder | 3 (Position) | ±90° | 2048 | 0° (straight up) |
| 13 | Elbow | 3 (Position) | ±100° | 2048 | 0° |
| 14 | Wrist | 3 (Position) | ±105° | 2048 | 0° |
| 15 | Gripper | 3 (Position) | 1400–2600 | 3100 | ~90° (open) |

**Why Mode 4 for Joint 11?** Standard Position mode wraps at 0/4095 boundary. Multi-turn mode allows continuous rotation past 360° without encoder discontinuity — essential for reaching all workspace quadrants without cable-wrap tracking.

---

## Gripper Specifications

| Parameter | Value |
|-----------|-------|
| Open gap | ~60 mm (real hardware) |
| Sim open gap | 70 mm (MuJoCo model) |
| Open position | 1700 counts (nominal) |
| Close position | 2150–2400 counts (varies by context) |
| Grasp engagement | 1920–2240 counts |
| Empty-closed threshold | 2310 counts |

**Discrepancy note**: Three different gripper ranges exist across the codebase:
- **Teleop**: open=1600, close=2400 (wide range for manual control)
- **GripperGuard**: open=1700, grasp=1920–2240, empty=2310 (precise thresholds)
- **Calibrator**: open=3130, close=2400 (maximally wide for optical contrast during gripper-flick)

---

## Home Pose

| Joint | Degrees | Counts | Meaning |
|-------|---------|--------|---------|
| J1 | 180° | 4096 | Forward-facing |
| J2 | 0° | 2048 | Straight up |
| J3 | 0° | 2048 | Collinear with upper arm |
| J4 | 0° | 2048 | Collinear with forearm |
| J5 | ~90° | 3100 | Open |

**Calibrated from**: `config/calibrated_home.yaml` (recorded 2026-11-07 via teach-mode)

---

## Electrical Safety

| Parameter | Safe Range | Trip Action |
|-----------|-----------|-------------|
| Voltage | 10.5V – 14.6V | Cut torque + close port |
| Temperature | < 65°C | Cut torque + close port |
| Load | < 85% | Cut torque + close port |
| HW Error | Bitmask = 0 | Cut torque + close port |

**Protection layers**:
1. `HardwareProtector` daemon thread polls all 5 motors every 0.5s
2. Gentle velocity profiles (default 50 ≈ 2.9 RPM)
3. Torque kept on at disconnect to prevent gravity collapse
4. E-stop key (ESC/X) in teleop mode
5. USB galvanic isolator recommended (ADuM3160/4160)
