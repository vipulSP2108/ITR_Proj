# Buzz Wire Autonomous Operations Runbook (Method 2: M2 Velocity Control Only)

> **Notice**: This runbook covers **Method 2 (M2 Autonomous Velocity Control)** exclusively. Manual hand-teaching (Method 1 Free-Drive) is omitted. All trajectories described here have been calibrated, SVD planar-projected, and multi-constraint time-optimal profiled.

---

## 1. System Coordinate Frames & Hardware Setup

```
+-----------------------------------------------------------------------------------------+
|                                    Robot Base Frame {B}                                 |
+-----------------------------------------------------------------------------------------+
                                             │
                        T^B_G (Kabsch SVD Calibrated Transform)
                                             v
+-----------------------------------------------------------------------------------------+
|                                   Buzz Wire Plane                                       |
|   Length Axis (L) : In-plane longitudinal axis connecting Start Post L -> Goal Post R   |
|   Height Axis (H) : In-plane vertical axis spanning Valleys (V1-V4) & Peaks (P1-P3)    |
|   Normal Axis (N) : Out-of-plane transverse normal (deviation strictly 0.00 mm)         |
+-----------------------------------------------------------------------------------------+
                                             │
                                     Tool Frame {T}
                                             v
+-----------------------------------------------------------------------------------------+
|                              Wand Loop Tool Center Point (TCP)                          |
|   Tool Z (+Z_T)   : Aligned with local 3D wire tangent vector t(s)                      |
|   Tool Y (+Y_T)   : Fixed along -N (wand handle pointing outward toward operator)       |
|   Radial Clearance: 10.5 mm uniform air gap inside 9.0 mm safety envelope                |
+-----------------------------------------------------------------------------------------+
```

---

## 2. Infrastructure Startup (Terminal 1 & Terminal 2)

### Terminal 1: Robot Hardware Controller (SSH)
Connect to the cobot controller over Ethernet and start the low-level motor drives:

```bash
ssh cobot@192.168.1.25
# Password: cobot

cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
sudo env HOME=/home/cobot ./cobot run
# Sudo Password: cobot
# When prompted, type 1 and press ENTER

# Start M2 Velocity Interface:
./heal_server
```

### Terminal 2: Control Laptop (ROS 2 Velocity Controller Node)
Open a fresh terminal on your control laptop and launch the resolved-rate controller:

```bash
cd ~/Debojit_WS/addverb_ws
source install/setup.bash
ros2 launch addverb_cobot_control velocity_control.launch.py
```

*Keep Terminal 1 and Terminal 2 running in the background.*

---

## 3. Free-Space Arm Positioning (`goto L`, `goto R`, `goto home`)

Use these commands when the arm is in **free air** (outside the wire) to drive the end-effector smoothly from **ANY starting posture or orientation** into the calibrated start or exit poses:

```bash
cd /Users/aai/Desktop/Task2/taken/G9/Task2_ITR-main/buzzwire_calib
```

### Commands

| Objective | Command | Details |
| :--- | :--- | :--- |
| **Move to Start Post L** | `./buzzwire_run.sh goto L` | Smooth $C^2$ quintic joint trajectory from ANY orientation to Start Post L. |
| **Move to Goal Post R** | `./buzzwire_run.sh goto R` | Smooth $C^2$ quintic joint trajectory from ANY orientation to Goal Post R. |
| **Move to Home** | `./buzzwire_run.sh goto home` | Drives all 6 joint angles to $0.0^\circ$ (safe home). |
| **Bypass Confirmation** | `./buzzwire_run.sh goto L --yes` | Immediately starts motion without asking for ENTER. |
| **Simulation Mode** | `./buzzwire_run.sh goto L --sim` | Tests the motion trajectory in virtual simulation (no robot needed). |

### Calibrated Target Configurations

- **Start Post L**:  
  $\mathbf{q}_L = [-11.9^\circ, -21.2^\circ, -30.9^\circ, -118.8^\circ, 30.1^\circ, -0.3^\circ]$  
  Cartesian: $\mathbf{p} = [148.4, 252.0, 568.2]\text{ mm}$, Quaternion: $[-0.0662, -0.7017, 0.1013, 0.7021]$
- **Goal Post R**:  
  $\mathbf{q}_R = [0.0^\circ, 38.6^\circ, 23.2^\circ, -93.3^\circ, 0.5^\circ, 78.2^\circ]$  
  Cartesian: $\mathbf{p} = [101.7, 548.4, 580.8]\text{ mm}$, Quaternion: $[0.0471, 0.7042, -0.0823, -0.7036]$

> [!CAUTION]
> **Operational Rule**: If the wand loop is **already threaded through the copper wire** at Post R, do NOT use `goto L`. Instead, use the wire-guided return command (`./buzzwire_run.sh rl`) described in Section 4.

---

## 4. Returning from Post R to Post L (Current State)

Because the robot completed the run and is currently resting at **Goal Post R**, it must safely return to **Start Post L**.

### Option A: Standard Wire-Guided Return (Earlier Calibration, Without `-smooth`)
Follows the demonstrated contour with out-of-plane tremor projected away ($1229\text{ mm}$ arc):
```bash
./buzzwire_run.sh rl --speed 0.35
```

### Option B: Smooth CAD-Grade Wire-Guided Return (With `-smooth`)
Follows the silky-smooth, CAD-conforming wire centerline ($1096\text{ mm}$ arc):
```bash
./buzzwire_run.sh rl -smooth --speed 0.35
```

*Duration: ~70–80 seconds. The arm glides backward along the copper wire with a guaranteed $10.5\text{ mm}$ radial safety air gap and stops safely at Post L.*

---

## 5. Autonomous Wire Traversal: Post L $\to$ Post R

Once the arm is positioned at **Start Post L**, you can run autonomous wire traversal using either the standard or smooth calibrated trajectory:

```
                  =======================================================
                               TRAVERSAL PROFILE SELECTION
                  =======================================================
                      WITHOUT -smooth            WITH -smooth (CAD-Grade)
                  -------------------------   -----------------------------
                  Standard contour fit        Silky-smooth straight legs
                  Arc: 1229.6 mm              Arc: 1096.3 mm (CAD match)
                  Show    : ~134 s            Show    : ~69 s
                  Balanced: ~116 s            Balanced: ~55 s
                  Fast    : ~97 s             Fast    : ~43 s
                  =======================================================
```

### A. Traversal Without `-smooth` (Standard Calibration, What We Did Earlier)
Preserves the natural demonstrated contours while enforcing SVD planar centering and contact culling:

```bash
# 1. Show Presentation Pass (~134 s, ultra-smooth for live evaluation)
./buzzwire_run.sh lr show

# 2. Balanced Operational Pass (~116 s)
./buzzwire_run.sh lr balanced

# 3. Fast Competition Pass (~97 s)
./buzzwire_run.sh lr fast

# 4. Custom Speed Scale (e.g., 50% speed)
./buzzwire_run.sh lr --speed 0.50
```

### B. Traversal With `-smooth` (CAD-Grade Multi-Stage Smoothing)
Applies cascading Savitzky-Golay filtering + regularized $C^2$ natural spline. Straight legs are perfectly straight lines, U-bends are circular arcs:

```bash
# 1. Show Presentation Pass (~69 s, silky-smooth presentation run)
./buzzwire_run.sh lr show -smooth

# 2. Balanced Operational Pass (~55 s)
./buzzwire_run.sh lr balanced -smooth

# 3. Fast Competition Pass (~43 s, maximum safe competition speed)
./buzzwire_run.sh lr fast -smooth

# 4. Custom Speed Scale (e.g., 65% speed)
./buzzwire_run.sh lr -smooth --speed 0.65
```

---

## 6. Speed Control & Live Runtime Overrides

### Recommended Speed Values

| Value | Cruise Speed ($v_{\max}$) | Bend Speed ($v_{\min}$) | Duration (`-smooth`) | Operational Intent |
| :---: | :---: | :---: | :---: | :--- |
| **`0.25`** | $15.0\text{ mm/s}$ | $4.0\text{ mm/s}$ | **~105 s** | **Safe Return (`rl`)**: Guaranteed clearance buffer. |
| **`0.40`** | $24.0\text{ mm/s}$ | $4.0\text{ mm/s}$ | **~69 s** | **Show Profile**: Best for formal demonstrations and judges. |
| **`0.65`** | $39.0\text{ mm/s}$ | $4.0\text{--}8.0\text{ mm/s}$ | **~55 s** | **Balanced Profile**: Standard testing cruise speed. |
| **`1.00`** | **$60.0\text{ mm/s}$** | **$4.0\text{--}13.5\text{ mm/s}$** | **~43 s** | **Fast Competition Max**: Actuator acceleration limit. |

> [!WARNING]
> **Advised Speed Limit**: `--speed 1.0` ($60.0\text{ mm/s}$).  
> Do not command `--speed > 1.0`. At speeds above $60\text{ mm/s}$, Joint 2 and Joint 3 saturate against their physical velocity limits ($\dot{q}_{\max} = 0.50\text{ rad/s} \approx 28.6^\circ/\text{s}$) during rapid pitch adjustments into Valley 1 and Valley 3.

### Live Keyboard Overrides During Replay
While the robot is in motion:
- **`]`**: Increase traversal speed by $+20\%$.
- **`[`**: Decrease traversal speed by $-20\%$.
- **`p`**: Pause / Resume motion (virtual carrot freezes in place).
- **`q`**: Immediate soft stop (commands zero joint velocity and exits cleanly).

---

## 7. Inspection, Simulation & Verification Tools

### A. Dry-Run Profile Inspection (No Arm Movement)
Audits the speed profile and identifies slowest bend bottlenecks before commanding motor motion:

```bash
# Standard trajectory audit
./buzzwire_run.sh dry-run lr

# Smooth trajectory audit
./buzzwire_run.sh dry-run lr -smooth
```

### B. Virtual Simulation Mode (No Physical Hardware)
Practices execution on a simulated 6-axis arm:

```bash
./buzzwire_run.sh sim lr -smooth
./buzzwire_run.sh sim rl -smooth
```

### C. Visual Dashboard & Plot Inspection
- **Interactive 3D WebGL Dashboard** (Open in browser):  
  [buzz_wire_3d_calibration_and_averaging_dashboard.html](file:///Users/aai/Desktop/Task2/taken/buzz_wire_3d_calibration_and_averaging_dashboard.html)
- **High-Resolution 4-Panel Analysis Plot**:  
  [buzz_wire_waypoints_recorded_fit_profile.png](file:///Users/aai/Desktop/Task2/taken/buzz_wire_waypoints_recorded_fit_profile.png)
- **Technical Report Artifact**:  
  [trajectory_calibration_and_velocity_optimization_report.md](file:///Users/aai/.gemini/antigravity/brain/4fc16566-e7dd-4b99-9df4-e9230b93a9a1/trajectory_calibration_and_velocity_optimization_report.md)

---

## 8. Quick Command Reference Table

```bash
# -----------------------------------------------------------------
# 1. POSITIONING IN FREE AIR (From any orientation)
# -----------------------------------------------------------------
./buzzwire_run.sh goto L                  # Move arm to Start Post L
./buzzwire_run.sh goto R                  # Move arm to Goal Post R
./buzzwire_run.sh goto home               # Move arm to Home position

# -----------------------------------------------------------------
# 2. SAFE RETURN THROUGH WIRE (When arm is at Post R)
# -----------------------------------------------------------------
./buzzwire_run.sh rl                      # Standard return (earlier calibration)
./buzzwire_run.sh rl -smooth              # Smooth CAD-grade return

# -----------------------------------------------------------------
# 3. FORWARD TRAVERSAL (L -> R)
# -----------------------------------------------------------------
# Earlier Standard Calibration (without -smooth):
./buzzwire_run.sh lr show                 # Standard presentation run (~134 s)
./buzzwire_run.sh lr fast                 # Standard fast run (~97 s)
./buzzwire_run.sh lr --speed 0.50         # Standard custom speed

# Smooth CAD-Grade Averaging (with -smooth):
./buzzwire_run.sh lr show -smooth         # Smooth presentation run (~69 s)
./buzzwire_run.sh lr fast -smooth         # Smooth competition run (~43 s)
./buzzwire_run.sh lr -smooth --speed 0.65 # Smooth custom speed (~55 s)

# -----------------------------------------------------------------
# 4. RE-CALIBRATE FROM DATA FOLDER
# -----------------------------------------------------------------
./buzzwire_run.sh calibrate               # Re-generates standard trajectories
./buzzwire_run.sh calibrate -smooth       # Re-generates smooth trajectories
```
