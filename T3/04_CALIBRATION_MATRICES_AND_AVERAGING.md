# 04. Calibration Mathematics, Averaging & Matrix Boards

> **Mathematical Specification**: Burst frame averaging, SE(3) chordal pose averaging, dual-camera joint non-linear optimization, and calibration board layouts (including `new_ref` matrices).

---

## 1. Multi-Stage Averaging Architecture

Calibration accuracy for precision robotics (sub-millimeter wire navigation) requires filtering out sensor discretization noise, hand-jitter, and visual blur before solving for transformations.

```
[Key Press SPACE]
       |
       v
+---------------------------------------------------------------------------------+
| 1. Settle Time (0.4 s): Wait for arm vibration and hand release jitter to decay  |
+---------------------------------------------------------------------------------+
       |
       v
+---------------------------------------------------------------------------------+
| 2. Burst Acquisition (12 frames per camera + 12 robot joint/FK state samples)   |
+---------------------------------------------------------------------------------+
       |
       +------------------------------------+------------------------------------+
       |                                                                         |
       v                                                                         v
+------------------------------------------+  +------------------------------------------+
| 3. Robot Averaging & Jitter Check        |  | 4. Visual Sharpness & Corner Detection   |
|  - Check: max ptp(q) <= 0.004 rad        |  |  - Detect ChArUco corners per frame      |
|  - Joint states: Median q_med across burst| |  - Sharpness score: S = Var(Laplacian(I))|
|  - FK Poses: Chordal SE(3) pose mean     |  |  - Select & save top sharpest frames     |
+------------------------------------------+  +------------------------------------------+
       |                                                                         |
       +------------------------------------+------------------------------------+
                                            |
                                            v
+---------------------------------------------------------------------------------+
| 5. Board PnP Pose Estimation & Per-Capture Averaging                             |
|  - solvePnP with IPPE / Levenberg-Marquardt refinement per burst frame          |
|  - Filter outliers (> 3 * median distance)                                      |
|  - Final burst-averaged camera-to-board pose: T_cam_board                       |
+---------------------------------------------------------------------------------+
```

---

## 2. Pose Averaging on SE(3)

A single capture collects $N = 12$ samples of the robot forward kinematics pose $T_i = \begin{bmatrix} R_i & \mathbf{t}_i \\ \mathbf{0} & 1 \end{bmatrix} \in SE(3)$.

### Translation Centroid
Translations are averaged over inlier samples:
$$\mathbf{t}_{\text{mean}} = \frac{1}{|K_{\text{inliers}}|} \sum_{i \in K_{\text{inliers}}} \mathbf{t}_i$$

### Rotation Mean on SO(3)
Rotations $R_i$ are mapped to unit quaternions $\mathbf{q}_i = [x_i, y_i, z_i, w_i]^T$. The chordal mean rotation $R_{\text{mean}}$ minimizes the sum of squared chordal distances:
$$\bar{\mathbf{q}} = \arg\max_{\mathbf{q}, \|\mathbf{q}\|=1} \sum_{i=1}^N (\mathbf{q}^T \mathbf{q}_i)^2 = \arg\max_{\mathbf{q}, \|\mathbf{q}\|=1} \mathbf{q}^T \left( \sum_{i=1}^N \mathbf{q}_i \mathbf{q}_i^T \right) \mathbf{q}$$
The optimal quaternion $\bar{\mathbf{q}}$ is the eigenvector corresponding to the **maximum eigenvalue** of the $4 \times 4$ covariance matrix:
$$M = \sum_{i=1}^N \mathbf{q}_i \mathbf{q}_i^T$$

### Two-Pass Outlier Filtering
Before computing the final mean, samples with translation deviation $> \max(0.01\text{ m}, 3 \times \text{median})$ or angular deviation $> \max(3^\circ, 3 \times \text{median})$ are pruned.

---

## 3. Dual-Camera Joint Hand-Eye Formulation

In our setup:
- **`cam0`** (`hand`): Mounted on the robot end-effector ($T^{\text{ee}}_{\text{cam0}} = X_0 \in SE(3)$ unknown).
- **`cam1`** (`fixed`): Statically mounted in the room facing the workspace ($T^{\text{base}}_{\text{cam1}} = X_1 \in SE(3)$ unknown).
- **Board** (`world`): Placed on the table at matrix placement $k \in \{0, 1, \dots, K-1\}$ ($T^{\text{base}}_{\text{board}, k} = B_k \in SE(3)$ unknown).

### Kinematic Loop Equations
For any capture $i$ belonging to matrix placement group $k$:
1. For the **hand camera** (`cam0`):
   $$T^{\text{base}}_{\text{cam0}, i} = T^{\text{base}}_{\text{ee}, i} \cdot X_0$$
   $$\hat{T}^{\text{cam0}}_{\text{board}, i} = \left( T^{\text{base}}_{\text{ee}, i} \cdot X_0 \right)^{-1} \cdot B_k$$
2. For the **fixed camera** (`cam1`):
   $$T^{\text{base}}_{\text{cam1}} = X_1$$
   $$\hat{T}^{\text{cam1}}_{\text{board}, i} = X_1^{-1} \cdot B_k$$

### Joint Loss Function
Rather than solving each camera independently, all parameters $(X_0, X_1, B_0, \dots, B_{K-1})$ are optimized **simultaneously**. Because both cameras view the same board $B_k$, the fixed camera constrains the hand camera and eliminates cumulative drift:
$$\min_{X_0, X_1, \{B_k\}} \sum_{i} \rho\left( \left\| \log_{SE(3)}\left( \hat{T}_{i}^{-1} \cdot T^{\text{measured}}_{i} \right) \right\| \right)$$
where $\rho(\cdot)$ is the Soft-$L_1$ robust loss function:
$$\rho(z) = 2 \cdot (\sqrt{1 + z^2} - 1)$$
This automatically down-weights occluded or noisy observations.

### Stereo Baseline Consistency Cross-Check
Whenever both cameras view the board simultaneously in capture $i$, the camera-to-camera transform can be measured directly without involving the robot kinematics:
$$T^{\text{cam1}}_{\text{cam0}, \text{direct}} = T^{\text{cam1}}_{\text{board}, i} \cdot \left( T^{\text{cam0}}_{\text{board}, i} \right)^{-1}$$
The solver compares this direct visual measurement against the model prediction:
$$T^{\text{cam1}}_{\text{cam0}, \text{model}} = \left( X_1 \right)^{-1} \cdot \left( T^{\text{base}}_{\text{ee}, i} \cdot X_0 \right)$$
The residual discrepancy $(\Delta \theta, \Delta d)$ is reported in the calibration summary. For a healthy calibration:
- Rotation error $\Delta \theta < 0.5^\circ$
- Translation error $\Delta d < 1.0\text{ mm}$

---

## 4. Calibration Matrices & Boards (ChArUco vs `new_ref` Layout)

### Standard ChArUco Board (`charuco`)
The toolkit includes `scripts/make_board.py` which renders a standard ChArUco board:
- **Grid Size**: $7 \times 5$ squares (24 internal corners).
- **Square Length**: $0.040\text{ m}$ ($40.0\text{ mm}$).
- **Marker Length**: $0.030\text{ m}$ ($30.0\text{ mm}$).
- **Dictionary**: `DICT_5X5_100`.

To generate a 300 DPI print-ready image:
```bash
python3 scripts/make_board.py --out charuco_board.png --dpi 300
```
> [!IMPORTANT]
> **Measure with Calipers**:
> Standard printers often scale images slightly (e.g., 98% or 101%). Use digital calipers to measure the physical printed black square edge. If it measures $39.4\text{ mm}$, update `config/calib_config.yaml`:
> ```yaml
> board:
>   square_length: 0.0394
>   marker_length: 0.02955
> ```

### Layout of Matrices in `new_ref`
The photos in `new_ref/` illustrate how the lab has laid out calibration boards across the workspace:
- In `WhatsApp Image 2026-11-12 at 7.57.08 AM.jpeg`, multiple ArUco markers (`DICT_6X6_250`, IDs `[0, 1, 2, 4, 5, 6, 7]`) are positioned across the surface.
- In `WhatsApp Image 2026-11-12 at 7.57.10 AM.jpeg`, markers `[2, 3, 4, 6, 7]` are visible at different table locations.
- **Why multiple matrices matter**: Moving the board across $\ge 4$ locations ensures the robot observes the board at varying depths and perspectives, breaking depth-scale ambiguity in monocular cameras.

