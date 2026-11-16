# 05. Buzz Wire Navigation Task Integration & Trajectory Execution

> **Trial 3 Execution Guide**: Bridging the dual-camera hand-eye calibration into autonomous collision-free traversal of the 3D Buzz Wire game without triggering the contact buzzer.

---

## 1. System Coordinate Frames & Transformations

```
 +-------------------------------------------------------------------------------+
 |                              Robot Base Frame {B}                             |
 +-------------------------------------------------------------------------------+
         |                                           |
         | T_base_cam1 (Eye-to-Hand)                 | T_base_ee (Robot FK)
         v                                           v
 +----------------------+                    +-----------------------+
 | Fixed Camera {cam1}  |                    |   End-Effector {ee}   |
 +----------------------+                    +-----------------------+
         |                                           |
         |                                           | T_ee_cam0 (Eye-in-Hand)
         |                                           v
         |                                   +-----------------------+
         |                                   |  Hand Camera {cam0}   |
         |                                   +-----------------------+
         |                                           |
         | T_cam1_G                                  | T_cam0_G
         v                                           v
 +-------------------------------------------------------------------------------+
 |                        Buzz Wire Ground Frame {G}                             |
 |  Origin: Inner face of Left MDF post (X=0), wire plane (Y=0), base plate (Z=0)|
 +-------------------------------------------------------------------------------+
                                         |
                                         v
                         +-------------------------------+
                         |   31 Navigation Waypoints     |
                         |     WP_00  -->  WP_30         |
                         +-------------------------------+
```

---

## 2. Buzz Wire Apparatus Geometry (Photogrammetrically Calibrated)

All dimensions were extracted from the 14 high-resolution reference photographs containing a 300 mm stainless steel ruler:

| Component | Specification | Geometric Value | Physical Significance |
| :--- | :--- | :---: | :--- |
| **Playable Span** | Left post to Right post ($L_{\text{inner}}$) | **$288.0\text{ mm}$** | Longitudinal bounds: $X \in [0.0, 288.0]\text{ mm}$. |
| **Track Wire** | Diameter ($d_w$), Radius ($r_w$) | **$4.0\text{ mm}$**, $2.0\text{ mm}$ | Solid copper rod forming the 3D track. |
| **Wand Loop (TCP)** | Inner Major $\times$ Minor ID | **$27.0 \times 25.0\text{ mm}$** | Slightly elliptical copper ring. Minor ID sets clearance. |
| **Safe Tube Margin** | Radial air gap radius ($r_{\text{safe}}$) | **$9.0\text{ mm}$** | Maximum allowable tracking error: $e_r < 9.0\text{ mm}$. |
| **Wire Entry / Exit** | Height at MDF post anchors | **$Z = 144.0\text{ mm}$** | Horizontal start/goal sections inside the wooden posts. |
| **Lowest Trough ($V_3$)** | Centerline height above base | **$Z = 25.5\text{ mm}$** | Bottom outer loop clears MDF base plate by $9.0\text{ mm}$. |
| **Tallest Crest ($P_2$)** | Centerline height above base | **$Z = 157.0\text{ mm}$** | Maximum vertical extension of course. |

### Laser-Cut Registration Fiducials
Four cross fiducials (`+`) are laser-engraved on the wooden base plate at $Z = 0.0\text{ mm}$:
- **`FID_FL`** (Front-Left): $[0.0, -48.0, 0.0]\text{ mm}$
- **`FID_FR`** (Front-Right): $[288.0, -48.0, 0.0]\text{ mm}$
- **`FID_RL`** (Rear-Left): $[0.0, 48.0, 0.0]\text{ mm}$
- **`FID_RR`** (Rear-Right): $[288.0, 48.0, 0.0]\text{ mm}$

---

## 3. Registering the Apparatus to the Robot ($T^B_G$)

To execute trajectories, we must map the Ground Frame $\{G\}$ into the Robot Base Frame $\{B\}$:
$$\mathbf{p}_B = R^B_G \cdot \mathbf{p}_G + \mathbf{t}^B_G, \qquad \mathbf{q}_B = \mathbf{q}^B_G \otimes \mathbf{q}_G$$

We provide three registration methods in `scripts/register_buzzwire.py`:

### Method A: 3-Point Touch Registration (Most Accurate: < 0.5 mm)
1. Switch robot to `./free_drive`.
2. Gently guide a pointer probe attached to the robot tip to touch three corner fiducials:
   - Touch `FID_FL` -> note robot FK pose $(x_1, y_1, z_1)$ in meters from `/end_effector_pose`
   - Touch `FID_FR` -> note $(x_2, y_2, z_2)$
   - Touch `FID_RL` -> note $(x_3, y_3, z_3)$
3. Run the registration script:
   ```bash
   python3 scripts/register_buzzwire.py \
     --touch-fl 0.420 -0.192 0.050 \
     --touch-fr 0.708 -0.192 0.050 \
     --touch-rl 0.420 -0.096 0.050
   ```
   The script computes the optimal Kabsch rigid transform via SVD, prints registration RMSE, and transforms all 31 waypoints.

### Method B: Vision-Based Registration via Fixed Camera (`cam1`)
Since $T^{\text{base}}_{\text{cam1}}$ is solved by our calibration:
$$T^B_G = T^{\text{base}}_{\text{cam1}} \cdot T^{\text{cam1}}_G$$
If `cam1` observes a calibration board placed flat on the buzz wire base, $T^{\text{cam1}}_G$ is obtained from PnP, completely automating registration without manual touching.

### Method C: Nominal Offset Test
```bash
python3 scripts/register_buzzwire.py --nominal-offset 0.40 -0.144 0.05 --yaw-deg 0.0
```

---

## 4. Trajectory Generation & Dynamic Pitch Orientation ($\alpha$)

### Why Dynamic Pitch ($\alpha$) Is Mandatory
The track wire has steep vertical sections (slopes up to $\pm 77^\circ$). If the loop is held fixed horizontally, the effective clearance aperture collapses:
$$\delta_{\text{eff}}(\Delta \alpha) = b_{\text{inner}} \cos(\Delta \alpha) - r_w$$
At $\Delta \alpha \approx 80.8^\circ$, collision is mathematically guaranteed!

Therefore, every waypoint in `results/buzz_wire_robot_trajectory.json` includes the analytical tangent orientation quaternion $\mathbf{q}_T$:
$$q_x = \cos\left(\frac{\pi}{4} + \frac{\alpha}{2}\right), \quad q_y = 0.0, \quad q_z = \sin\left(\frac{\pi}{4} + \frac{\alpha}{2}\right), \quad q_w = 0.0$$
where $\alpha = \operatorname{atan2}(t_z, t_x)$ is the local elevation angle.

### Trajectory Segment Hierarchy

| Waypoint Range | Segment Type | Speed ($v_{\max}$) | Motion Style |
| :--- | :--- | :---: | :--- |
| `WP_00` ($X=8.0\text{ mm}$) | `linear_approach` | $12\text{ mm/s}$ | Safe entry clearing left MDF post by $8\text{ mm}$. |
| `WP_01` to `WP_03` ($L_1$) | `linear_leg` | $25\text{ mm/s}$ | Near-vertical descent into Valley 1. |
| `WP_04` ($V_1$, Apex) | `valley_apex` | $8\text{ mm/s}$ | Tightest U-bend ($R=13.75\text{ mm}$); wrist pitch rotate $-77^\circ \to +77^\circ$. |
| `WP_07` ($P_1$, Crest) | `peak_apex` | $10\text{ mm/s}$ | Top crest traversal. |
| `WP_15` ($P_2$, Apex) | `peak_apex` | $8\text{ mm/s}$ | Tallest point on course ($Z=157.0\text{ mm}$). |
| `WP_19` ($V_3$, Apex) | `valley_apex` | $8\text{ mm/s}$ | Lowest point ($Z=25.5\text{ mm}$); clears red rocker switch. |
| `WP_30` ($X=280.0\text{ mm}$) | `linear_goal` | $10\text{ mm/s}$ | Safe exit inside right MDF post. |

---

## 5. Trajectory Execution Using Damped Least Squares IK (DLS-IK)

Trajectory replay connects to `Project3 Instructions/utils/dls_velocity_commander.py`:

1. **Orientation Tracking**: Computes quaternion logarithmic error:
   $$\mathbf{e}_o = 2 \cdot \operatorname{acos}(w_{\text{err}}) \cdot \frac{\mathbf{v}_{\text{err}}}{\|\mathbf{v}_{\text{err}}\|}$$
2. **DLS Pseudo-Inverse**:
   $$J^\dagger = J^T \cdot \left( J J^T + \lambda^2 I \right)^{-1}$$
   where $\lambda \approx 0.05$ prevents excessive joint speeds near singularities.
3. **Command Generation**:
   $$\dot{\mathbf{q}} = J^\dagger \begin{bmatrix} K_p (\mathbf{p}_{\text{target}} - \mathbf{p}) \\ K_o \mathbf{e}_o \end{bmatrix}$$
4. **Safety Verification**:
   - Run first trial with a $+30\text{ mm}$ offset in $-Y$ (air run in front of the wire).
   - If continuity buzzer is wired to a GPIO / ROS topic, trigger an immediate E-stop halt on rising edge.

