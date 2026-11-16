# `dynamixel_wrapper.py` — Safe Dynamixel Motor Interface

**Module**: `src/control/dynamixel_wrapper.py`
**Purpose**: Low-level safe interface to all 5 Dynamixel XM430-W350-T actuators via U2D2 USB-to-RS485 adapter.

---

## Hardware Specifications

| Property | Value |
|----------|-------|
| Actuator model | Dynamixel XM430-W350-T |
| Protocol | 2.0 |
| Baudrate | 1,000,000 bps (1 Mbps) |
| Interface | ROBOTIS U2D2 (USB → RS485) |
| Encoder | 4096 counts = 360° (0.0879°/count) |
| Center position | 2048 counts = 0° |

---

## Joint ID Map

| ID | Joint | Role |
|----|-------|------|
| 11 | Joint 1 | Base Pan (yaw) |
| 12 | Joint 2 | Shoulder Pitch |
| 13 | Joint 3 | Elbow Pitch |
| 14 | Joint 4 | Wrist Pitch |
| 15 | Joint 5 | Gripper |

---

## Control Table Addresses (XM430)

| Address | Size | Register |
|---------|------|----------|
| 11 | 1 byte | Operating Mode (3=Position, 4=Extended Multi-turn) |
| 64 | 1 byte | Torque Enable |
| 108 | 4 bytes | Profile Acceleration |
| 112 | 4 bytes | Profile Velocity |
| 116 | 4 bytes | Goal Position |
| 132 | 4 bytes | Present Position |
| 126 | 2 bytes | Present Load |

---

## Class: `OpenManipulatorController`

### `find_u2d2_port() → str or None`
Auto-detects FTDI/U2D2 USB serial adapter on macOS by scanning `/dev/cu.usbserial-*` ports.

### `connect() → bool`
Opens serial port, sets baudrate. Prints macOS-specific troubleshooting if failed.

### `disconnect(disable_torque=False)`
Closes port. **Default: keeps torque enabled** to prevent arm from collapsing under gravity.

### `initialize_all_joints(profile_vel=50, profile_acc=15) → bool`

For each joint:
1. Motor 11 → **Mode 4** (Extended Multi-turn Position Control)
2. Motors 12–15 → **Mode 3** (Position Control)
3. Set velocity/acceleration profiles
4. Enable torque

**Why Mode 4 for Joint 1?** Standard mode wraps at 0/4095. Multi-turn allows continuous rotation through 360° without position discontinuity. Essential for reaching all board quadrants.

### `read_position(dxl_id) → int or None`

Reads 4-byte present position. **Special handling for Motor 11**: Raw value ≥ 0x80000000 is interpreted as negative (signed 32-bit) for multi-turn mode.

### `read_load(dxl_id) → float or None`

Reads 2-byte load as signed percentage (-100% to +100%). Unit: 0.1% per count. Used by `GripperGuard` for grasp verification.

### `write_position(dxl_id, goal_position) → bool`

**Safety bounds**:
- Motor 11: Clamped to ±3000 counts (±263°). Written as unsigned 32-bit (two's complement for negatives).
- Motors 12–15: Clamped to 0–4095.

### `gentle_move_relative_deg(dxl_id, delta_deg, profile_vel=30) → bool`

Relative motion: reads current position, adds delta, writes new goal. Uses slow profile (vel=30 ≈ 1.7 RPM) for safety.

---

## Unit Conversions

```
degrees = (raw_counts - 2048) × (360 / 4096)
raw_counts = degrees × (4096 / 360) + 2048
1 count = 0.08789°
```

---

## Design Decisions

- **Torque kept on by default**: Arm would collapse under gravity if torque disabled. User must explicitly request `disable_torque=True`.
- **±3000 count limit on Joint 1**: Prevents cable wrap. ±263° covers all reachable workspace quadrants.
- **Signed 32-bit for Motor 11**: Multi-turn mode uses signed position values. Without the 0x80000000 check, negative positions read as large positive numbers.
- **Profile velocity as safety**: All moves use configurable velocity profiles (default 50 ≈ 2.9 RPM). Never commands instantaneous position jumps.
