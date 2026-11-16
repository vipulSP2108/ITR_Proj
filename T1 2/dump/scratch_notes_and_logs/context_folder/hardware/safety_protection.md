# Safety & Electrical Protection

**References**: `device/SAFETY_AND_PROTECTION.md`, `device/hardware_protector.py`

---

## Voltage Domains

| Domain | Voltage | Purpose |
|--------|---------|---------|
| Mac USB | 5V | Communication (U2D2 adapter) |
| Robot DC | 12V (10.0–14.8V) | Motor power |

**Primary hazards**: Overvoltage spikes (inductive kickback from motors), undervoltage brownouts (simultaneous multi-joint moves), USB ground loops.

---

## Software Watchdog — `HardwareProtector`

**File**: `device/hardware_protector.py`

Daemon thread polling all 5 motors at 0.5s intervals.

### Monitored Parameters

| Parameter | Safe Range | Unit |
|-----------|-----------|------|
| Voltage | 10.5V – 14.6V | V (read from servo) |
| Temperature | < 65°C | °C |
| Load | < 85% | % of stall torque |
| HW Error | Bitmask = 0 | Bits: overload, overheat, input voltage |

### Trip Action (Emergency Auto-Disconnect)

1. Disable torque on all 5 servos
2. Zero all velocity commands
3. Close serial bus
4. Raise `ElectricalSafetyException`

### API

| Method | Purpose |
|--------|---------|
| `get_motor_health(dxl_id)` | Returns voltage, temp, load, hw_error for one motor |
| `check_all_safety_limits()` | Checks all motors against thresholds |
| `trigger_emergency_auto_disconnect(reason)` | Full shutdown sequence |
| `start_background_guardian()` | Spawns daemon thread for continuous monitoring |
| `run_health_check_cli()` | One-shot health table with green/red status |

---

## Physical Safety Recommendations

1. **USB galvanic isolator** (ADuM3160/4160, ~$10–15) between Mac and U2D2
2. **Powered USB hub** as buffer between Mac and robot
3. **Power-on sequence**: USB first → then 12V DC (prevents uninitialized servo state)
4. **Power-off sequence**: 12V off → then USB (prevents inductive kickback through USB)

---

## Software Safety Layers

| Layer | Protection | Trigger |
|-------|-----------|---------|
| E-Stop key | ESC/X in teleop | Immediate torque cut |
| Torque on disconnect | Default behavior | Prevents gravity collapse |
| Position clamping | J11: ±3000 counts, J12–15: 0–4095 | Prevents cable wrap / overtravel |
| Velocity profiles | Default 50 (~2.9 RPM) | No instantaneous jumps |
| `finally:` blocks | All demo scripts | Guaranteed torque release on crash |
| HardwareProtector | Background daemon | Voltage/temp/load watchdog |
| GripperGuard | Transit monitoring | Drop detection (EC-05) |
