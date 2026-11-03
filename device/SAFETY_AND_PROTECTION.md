# Electrical Safety, Surge Protection & Auto-Disconnector Guide

This document outlines both the **Software Auto-Disconnector Watchdog** and **Physical Hardware Isolation Strategies** to protect your Mac and the OpenMANIPULATOR-X robot from electrical faults, voltage spikes, thermal overheating, and ground loops.

---

## 1. Electrical Risk Analysis

When running robotic arms with external power supplies alongside a computer, two distinct voltage domains exist:
1. **Mac Domain**: 5.0V DC USB logic (sensitive microcontroller and Thunderbolt/USB-C host controller).
2. **Robot Domain**: 12.0V DC high-current motor drive (DYNAMIXEL XM430-W350 actuators drawing up to 2.3A stall current each).

### Primary Hazards:
* **Overvoltage & Power Supply Spikes**: If a 12V power supply surges past 15V, it can permanently damage actuator H-bridges.
* **Undervoltage Brownouts**: When voltage drops below 10.0V (e.g., loose DC plug or sagging supply), actuators stall and draw maximum stall current.
* **Inductive Kickback**: Rapid deceleration or motor reversal generates voltage spikes on the power rail.
* **USB Ground Loops**: Potential difference between wall-adapter ground and Mac chassis ground.

---

## 2. Software Auto-Disconnector Layer (`device/hardware_protector.py`)

We have implemented an active software guardian (`HardwareProtector`) that runs continuous real-time telemetry on the actuators:

```
┌────────────────────────────────────────────────────────┐
│             HardwareProtector Guardian                │
│                                                        │
│  [Voltage Sensor] ──► Range: 10.5V ~ 14.6V             │
│  [Thermal Sensor] ──► Max Threshold: 65.0°C            │
│  [Load Sensor]    ──► Max Threshold: 85.0% effort      │
│  [Error Bitmask]  ──► Shock / Overload / Voltage Fault │
└──────────────────────────┬─────────────────────────────┘
                           │
                 Breach Detected?
                  ├── NO  ──► Continue normal operation
                  └── YES ──► [AUTO-DISCONNECT TRIPPED]
                              1. Cut torque to all servos
                              2. Zero all velocities
                              3. Close serial bus to isolate Mac USB
                              4. Raise ElectricalSafetyException
```

### Running the Diagnostic:
```bash
python3 device/hardware_protector.py
```

---

## 3. Physical Hardware Protection Recommendations (Physical Barrier)

To provide 100% physical immunity against high-voltage faults reaching your Mac, we recommend the following hardware safety layers:

### A. USB Galvanic Isolator (Highest Recommendation)
* **What it is**: An in-line USB dongle based on the **Analog Devices ADuM3160 / ADuM4160** optical/magnetic isolation chip (~$10–$15 on Amazon/AliExpress).
* **How it works**: It completely severs the physical copper connection between the Mac's USB port and the robot's U2D2 adapter, transmitting data across an air/magnetic gap with **2,500V to 5,000V galvanic isolation**.
* **Benefit**: Even in a catastrophic direct short circuit where 12V touches the data lines, **0V can physically cross to your Mac**.

### B. Powered USB Hub Buffer
* If a dedicated USB isolator is not immediately on hand, plug both the **U2D2** and the **RealSense D435i** into a **powered external USB-C hub** rather than directly into the Mac's built-in ports.
* The hub acts as a sacrificial electrical buffer with its own internal overcurrent and ESD protection diodes.

### C. Power-On Sequencing Best Practice
1. **Always plug USB cables in FIRST** before powering on 12V DC.
2. **Power on 12V DC switch**.
3. When shutting down: **Turn 12V DC switch OFF FIRST**, then unplug USB cables.

