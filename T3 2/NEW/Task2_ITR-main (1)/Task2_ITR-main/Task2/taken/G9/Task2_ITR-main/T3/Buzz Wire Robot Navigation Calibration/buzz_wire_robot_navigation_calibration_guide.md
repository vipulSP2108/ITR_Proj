# Buzz Wire Game: Complete Geometric Calibration & Robot Navigation Specification

This document explains every section, mathematical convention, and kinematic constraint encoded in `buzz_wire_calibration.json`. It is structured so a robot navigation stack (e.g., ROS 2, MoveIt 2, or a custom Cartesian trajectory controller) can parse the calibrated geometry, transform coordinates into the robot base frame, and execute a collision-free traversal from start to goal without triggering the continuity buzzer.

---

## 1. Multi-View Photogrammetric Calibration Summary

All dimensions in `buzz_wire_calibration.json` were extracted and cross-validated across 14 reference images containing a $300\text{ mm}$ stainless steel ruler ($1\text{ mm}$ graduations):

1. **Longitudinal Span & Board Fiducials (`3.47.43 PM.jpeg`)**:
   - The ruler placed flat on the base plate from the inner face of the Left MDF post ($X = 0.0\text{ mm}$) to the inner face of the Right MDF post measures an exact inner span of $L_{\text{inner}} = 288.0\text{ mm}$.
   - Four laser-cut cross (`+`) fiducials sit at the four corners of the playable span ($X \in \{0.0, 288.0\}\text{ mm}$, $Y \in \{-48.0, +48.0\}\text{ mm}$, $Z = 0.0\text{ mm}$) for repeatable hand-eye or probe calibration.
2. **Mid-Height Leg Cross-Section & Wire Gauge (`3.52.38 PM.jpeg`)**:
   - Measuring across all 8 vertical legs at $Z = 88.0\text{ mm}$ confirms a uniform copper track wire diameter of $d_w = 4.0\text{ mm}$ ($r_w = 2.0\text{ mm}$).
   - Inner leg-to-leg air gaps range from $22.0\text{ mm}$ at the tightest bend ($V_4$) up to $34.0\text{ mm}$ at the widest crests ($P_1, P_2$).
3. **End-Effector Wand Loop Orthogonality (`3.47.46 PM.jpeg` & `3.47.47 PM.jpeg`)**:
   - Because the copper wand loop is hand-formed, its inner opening is slightly elliptical:
     - **Major inner diameter** (parallel to wand handle): $D_{\text{loop,major}} = 27.0\text{ mm}$ ($a_{\text{inner}} = 13.5\text{ mm}$)
     - **Minor inner diameter** (orthogonal to wand handle): $D_{\text{loop,minor}} = 25.0\text{ mm}$ ($b_{\text{inner}} = 12.5\text{ mm}$)
     - **Loop wire thickness**: $d_{\text{loop}} = 4.0\text{ mm}$ ($r_{\text{loop,wire}} = 2.0\text{ mm}$)
4. **Vertical Profile Heights (`3.47.39 PM` through `3.47.42 PM`)**:
   - Wire entry anchors at both posts sit at $Z = 144.0\text{ mm}$ (below the $154.0\text{ mm}$ top edge of the MDF posts).
   - The lowest trough ($V_3$, above the red rocker switch) sits at centerline height $Z = 25.5\text{ mm}$, and the tallest crest ($P_2$) reaches centerline height $Z = 157.0\text{ mm}$.

---

## 2. Coordinate Frames & Rigid Transformations (`coordinate_frames`)

### 2.1 Ground Frame $\{G\}$ (`buzz_wire_ground`)

The JSON defines a right-handed Cartesian frame $\{G\}$ rigidly attached to the Buzz Wire wooden base:

- **Origin $O_G = (0, 0, 0)$**: Intersection of the inner face of the Left MDF post ($X = 0$), the nominal vertical plane of the copper wire ($Y = 0$), and the top surface of the MDF base board ($Z = 0$).
- **$+X$ Axis**: Points longitudinally from the Left post ($X = 0.0\text{ mm}$) toward the Right post ($X = 288.0\text{ mm}$).
- **$+Y$ Axis**: Points transversely from the front engraved `"BUZZ WIRE GAME"` text edge ($-Y$) toward the rear deck containing the red rocker switch and piezo buzzer ($+Y$).
- **$+Z$ Axis**: Points vertically upward normal to the top surface of the base board.

### 2.2 Robot Base Transformation ($T^B_G$)

Before executing motion, the robot controller must populate `robot_base_to_ground_transform` ($T^B_G \in SE(3)$) using at least three of the four laser-cut corner fiducials (`FID_FL`, `FID_RL`, `FID_FR`, `FID_RR`). Any waypoint $\mathbf{p}_G \in \mathbb{R}^3$ and orientation quaternion $\mathbf{q}_G$ in `buzz_wire_calibration.json` maps into the robot base frame $\{B\}$ via:

$$\mathbf{p}_B = R^B_G \, \mathbf{p}_G + \mathbf{t}^B_G, \qquad \mathbf{q}_B = \mathbf{q}^B_G \otimes \mathbf{q}_G$$

### 2.3 Tool Center Point Frame $\{T\}$ (`wand_loop_tcp`)

The Tool Center Point (TCP) is placed at the **geometric center of the closed copper loop**:

- **$\hat{\mathbf{z}}_T$ (Loop Plane Normal)**: Aligned with the local unit tangent vector $\hat{\mathbf{t}}(s) = [\cos\alpha, \; 0, \; \sin\alpha]^T$ of the copper track, where $\alpha = \operatorname{atan2}(t_z, t_x)$ is the local wire elevation angle in the $X\text{-}Z$ plane.
- **$\hat{\mathbf{y}}_T$ (Wand Handle Axis)**: Fixed along $[0, -1, 0]^T$ in $\{G\}$ so the wand handle and its flexible electrical lead extend cleanly out the front ($-Y$) side of the apparatus, completely clear of the rear rocker switch (`OBS_POWER_SWITCH`), piezo buzzer (`OBS_PIEZO_BUZZER`), and rear grommet (`OBS_REAR_CABLE_PORT`).
- **$\hat{\mathbf{x}}_T$ (In-Plane Orthogonal Axis)**: Completed by the right-hand rule $\hat{\mathbf{x}}_T = \hat{\mathbf{y}}_T \times \hat{\mathbf{z}}_T = [-\sin\alpha, \; 0, \; \cos\alpha]^T$.

This yields the exact proper rotation matrix $R^G_T(\alpha) \in SO(3)$:

$$R^G_T(\alpha) = \begin{bmatrix} -\sin\alpha & 0 & \cos\alpha \\ 0 & -1 & 0 \\ \cos\alpha & 0 & \sin\alpha \end{bmatrix}$$

In scalar-last ROS quaternion format $[q_x, q_y, q_z, q_w]$, every waypoint orientation in `navigation_waypoints` is computed analytically from $\alpha$ as:

$$q_x = \cos\!\left(\frac{\pi}{4} + \frac{\alpha}{2}\right), \quad q_y = 0.0, \quad q_z = \sin\!\left(\frac{\pi}{4} + \frac{\alpha}{2}\right), \quad q_w = 0.0$$

---

## 3. Collision Envelope & Kinematic Clearance Analysis

### 3.1 Radial Air Gap Budget

Because the copper wand loop is slightly elliptical ($27.0\text{ mm} \times 25.0\text{ mm}$ inner opening) and the track wire has radius $r_w = 2.0\text{ mm}$, the nominal radial air gap $\delta_r$ around the wire centerline satisfies:

$$\delta_{r,\min} = \frac{D_{\text{loop,minor}} - d_w}{2} = \frac{25.0 - 4.0}{2} = 10.5\text{ mm}$$

$$\delta_{r,\max} = \frac{D_{\text{loop,major}} - d_w}{2} = \frac{27.0 - 4.0}{2} = 11.5\text{ mm}$$

Reserving a $1.5\text{ mm}$ safety margin for hand-bent wire tolerances and robot path tracking error yields a strict **cylindrical collision-free tube radius** of:

$$r_{\text{safe}} = 9.0\text{ mm}$$

### 3.2 Why Continuous Pitch Orientation ($\alpha$) Is Mandatory

If the robot held the loop at a fixed vertical orientation ($\alpha_{\text{tool}} = 0^\circ$) while traversing a near-vertical leg ($\alpha_{\text{wire}} \approx 85^\circ$), the effective horizontal aperture of the loop would collapse by $\cos(\Delta\alpha)$:

$$\delta_{\text{eff}}(\Delta\alpha) = b_{\text{inner}} \cos(\Delta\alpha) - r_w$$

Setting $\delta_{\text{eff}}(\Delta\alpha) = 0$ gives the hard geometric collision angle:

$$\Delta\alpha_{\text{crit}} = \arccos\!\left(\frac{r_w}{b_{\text{inner}}}\right) = \arccos\!\left(\frac{2.0}{12.5}\right) \approx 80.8^\circ$$

Accounting for the $4.0\text{ mm}$ loop wire cross-section thickness at tight U-bends ($R_{\text{inner}} = 9.5\text{ mm}$ at $V_1$ and $10.0\text{ mm}$ at $V_4$), the orientation tracking error must remain strictly bounded by:

$$|\Delta\alpha| \le 22.0^\circ$$

### 3.3 Post & Base Clearance Limits

- **Post Buffer**: Do not command the TCP closer than $6.0\text{ mm}$ to either MDF support post ($X \in [6.0, 282.0]\text{ mm}$). Because the loop wire cross-section extends $\pm 2.0\text{ mm}$ along $\hat{\mathbf{z}}_T$, starting at `WP_00_START_SAFE` ($X = 8.0\text{ mm}$) and ending at `WP_30_GOAL_SAFE` ($X = 280.0\text{ mm}$) guarantees at least $6.0\text{ mm}$ physical clearance from both posts.
- **Lowest Trough ($V_3$)**: At `WP_19_V3_APEX` ($X = 196.0\text{ mm}, Z = 25.5\text{ mm}$), the bottom outer edge of the wand loop reaches $Z = 25.5 - 16.5 = 9.0\text{ mm}$, clearing the MDF base board by $9.0\text{ mm}$ and staying $20.0\text{ mm}$ in $+X$ past the red rocker switch (`OBS_POWER_SWITCH`, $X \in [160.0, 176.0]\text{ mm}$).

---

## 4. Waypoint Structure & Trajectory Execution (`navigation_waypoints`)

The 31 waypoints (`index` $0$ through $30$) decompose the $982.4\text{ mm}$ wire course into four semantic segment types:

| Segment Type | Role in Trajectory | Speed Profile ($v_{\max}$) | Interpolation Guidance |
| :--- | :--- | :---: | :--- |
| `linear_approach` / `linear_goal` | Safe entry/exit inside MDF posts ($6\text{–}8\text{ mm}$ margin) | $10\text{–}12\text{ mm/s}$ | Linear Cartesian with smooth ramp |
| `linear_leg` | Near-vertical straight traverses ($L_1 \dots L_8$) | $25\text{–}28\text{ mm/s}$ | Cubic Hermite spline; constant $\mathbf{q}$ |
| `bend_entry` / `bend_exit` | Tangent transition points entering/exiting circular U-bends | $9\text{–}11\text{ mm/s}$ | $C^1$-continuous tangent match |
| `valley_apex` / `peak_apex` | Bottom troughs ($V_1 \dots V_4$) and top crests ($P_1 \dots P_3$) | $8\text{–}10\text{ mm/s}$ | SLERP orientation through $\alpha = 0^\circ$ |

### Pre-Run Verification Checklist for Robot Deployment

1. **Dry-Run Offset Test**: Execute the spline trajectory once with a $+30.0\text{ mm}$ offset in $-Y$ (loop disengaged in front of the wire) to verify smooth SLERP wrist rotations without joint-limit singularities.
2. **3-Point Touch Registration**: Touch a pointer probe to `FID_FL`, `FID_FR`, and `FID_RL` on the wooden base to compute $T^B_G$ to sub-millimeter accuracy.
3. **Closed-Loop Buzzer Interrupt**: Wire the Buzz Wire continuity output into a digital input interrupt on the robot controller (`contact_feedback_loop`). If triggered, immediately halt Cartesian feed and command a small recovery step along the local loop radial normal toward the wire centerline.

---

## 5. Local ZIP Archive Packaging Script

Once you save `buzz_wire_calibration.json` and `buzz_wire_calibration_guide.md` into the same directory as your 14 reference images, run the following Python snippet to bundle all 16 files into `buzz_wire_calibration_package.zip`:

```python
import glob
import zipfile

output_zip = "buzz_wire_calibration_package.zip"
files_to_bundle = [
    "buzz_wire_calibration.json",
    "buzz_wire_calibration_guide.md",
    *sorted(glob.glob("WhatsApp Image 2026-11-12 at 3.*.jpeg")),
]

with zipfile.ZipFile(output_zip, "w", compression=zipfile.ZIP_DEFLATED) as zf:
    for file_path in files_to_bundle:
        zf.write(file_path)

print(f"Created {output_zip} with {len(files_to_bundle)} files.")
```