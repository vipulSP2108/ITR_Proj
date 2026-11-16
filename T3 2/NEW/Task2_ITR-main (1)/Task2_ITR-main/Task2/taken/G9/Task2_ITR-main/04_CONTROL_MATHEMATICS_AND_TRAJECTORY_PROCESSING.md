# 04. Control Mathematics, Kinematics & Trajectory Processing

> **Rigorous Mathematical Specification**: Formulation of 6-DOF resolved-rate Cartesian control, Nakamura-style singularity-adaptive DLS, spatial segment chaining, Savitzky-Golay filtering, and multi-constraint time-optimal velocity profiling.

---

## 1. 6-DOF Resolved-Rate Cartesian Control Law

The Addverb HEAL cobot is commanded via Cartesian task-space twists $v_{\text{cmd}} = [v^T, \omega^T]^T \in \mathbb{R}^6$ in the robot base frame.

```
       Desired Pose T_des                    Actual Pose T_cur (from FK)
               │                                      │
               └──────────────┬───────────────────────┘
                              v
                  Pose Error: e = [e_p; e_R]
                              │
                              v
           v_cmd = v_ff + [Kp_lin * e_p; Kp_ang * e_R]
                              │
                              v
        Singular Value Decomposition of J: J = U Σ V^T
                              │
                              v
       Nakamura DLS: qdot = J^T (J J^T + λ^2 I)^(-1) (v_cmd * s_sing)
                              │
                              v
      Uniform Scaling: qdot_scaled = qdot * min(1, qdot_max / max(|qdot|))
                              │
                              v
            Send to /velocity_controller/commands
```

### A. Task-Space Pose Error Formulation
Given the desired pose $T_{\text{des}} = (R_{\text{des}}, p_{\text{des}})$ and the actual pose $T_{\text{cur}} = (R_{\text{cur}}, p_{\text{cur}})$:
1. **Translational Error**:
   $$e_p = p_{\text{des}} - p_{\text{cur}} \in \mathbb{R}^3$$
2. **Rotational Error (Lie Algebra $\mathfrak{so}(3)$)**:
   $$R_{\text{err}} = R_{\text{des}} R_{\text{cur}}^T$$
   $$e_R = \text{Log}(R_{\text{err}}) = \frac{\theta}{2\sin\theta} (R_{\text{err}} - R_{\text{err}}^T)^\vee \in \mathbb{R}^3$$
   where $\theta = \arccos\left(\frac{\text{tr}(R_{\text{err}}) - 1}{2}\right)$.

3. **Composite Command Twist**:
   $$v_{\text{cmd}} = v_{\text{ff}} + \begin{bmatrix} K_{p,\text{lin}} e_p \\ K_{p,\text{ang}} e_R \end{bmatrix}$$
   with default proportional gains $K_{p,\text{lin}} = 4.0\text{ s}^{-1}$ and $K_{p,\text{ang}} = 3.0\text{ s}^{-1}$.

---

## 2. Nakamura Singularity-Adaptive DLS Solver

Direct matrix inversion $J^{-1}$ fails near kinematic singularities where $\det(J) \to 0$, causing infinite joint velocities. We employ Nakamura-style singularity-adaptive Damped Least Squares (DLS).

### Singular Value Decomposition (SVD)
Let the geometric Jacobian SVD be:
$$J = U \Sigma V^T, \quad \Sigma = \text{diag}(\sigma_1, \sigma_2, \dots, \sigma_6)$$
The smallest singular value $\sigma_{\text{min}} = \sigma_6$ measures the distance to the nearest singularity.

### Adaptive Damping Formulation
We define a threshold $\sigma_\epsilon = 0.03$. The damping factor $\lambda^2$ and Cartesian scaling factor $s_{\text{sing}}$ are computed as:
- **Far from Singularity** ($\sigma_{\text{min}} \ge \sigma_\epsilon$):
  $$s_{\text{sing}} = 1.0, \quad \lambda^2 = 10^{-6}$$
- **Near Singularity** ($\sigma_{\text{min}} < \sigma_\epsilon$):
  $$r = \frac{\sigma_{\text{min}}}{\sigma_\epsilon}$$
  $$s_{\text{sing}} = r$$
  $$\lambda^2 = \lambda_{\text{max}}^2 (1 - r^2) + 10^{-6} \quad (\lambda_{\text{max}} = 0.08)$$

The regularized inverse velocity is then computed:
$$\dot{q} = J^T \left( J J^T + \lambda^2 I \right)^{-1} \left( v_{\text{cmd}} \cdot s_{\text{sing}} \right)$$

### Direction-Preserving Uniform Velocity Scaling
If any individual joint exceeds the physical threshold $\dot{q}_{\text{max}} = 0.35\text{ rad/s}$:
$$m = \max_{i \in \{1,\dots,6\}} |\dot{q}_i|$$
$$\dot{q}_{\text{safe}} = \dot{q} \cdot \left(\frac{\dot{q}_{\text{max}}}{m}\right) \quad \text{if } m > \dot{q}_{\text{max}}$$
This uniform scalar scaling ensures that the Cartesian tool direction of motion is strictly maintained without distortion.

---

## 3. Trajectory Processing & Multi-Segment Merging

A typical Buzz Wire teaching session produces multiple partial segment CSVs (`seg_0000.csv`, `seg_0001.csv`, etc.). The merger pipeline executes a 7-stage deterministic transformation:

```
[Raw Segments] ──> [Filter Short] ──> [Trim Contacts] ──> [Greedy Ordering]
                                                                  │
                                                                  v
[Final Trajectory] <── [Arc Resample 2mm] <── [Savitzky-Golay] <── [Stitch / Bridge]
```

### Stage 1: Contact-Region Trimming
Any point recorded within $r_{\text{trim}} = 8.0\text{ mm}$ of an operator-marked or buzzer-detected contact is culled:
$$\mathcal{W}_{\text{clean}} = \{ w \in \mathcal{W} \mid \min_{c \in \mathcal{C}} \|p(w) - p(c)\| > r_{\text{trim}} \}$$
This automatically cuts out the collision and re-entry tremor.

### Stage 2: Spatial Greedy Ordering
Segments may be recorded out of order (e.g. recording the difficult third valley before the second). The algorithm sorts segment indices using a greedy nearest-neighbour heuristic:
$$\text{next\_idx} = \arg\min_{j \notin \mathcal{O}} \| p_{\text{start}}^{(j)} - p_{\text{end}}^{(\mathcal{O}_{\text{last}})} \|$$

### Stage 3: Stitching & Cubic SLERP Bridging
For adjacent segments in the ordered sequence:
- **Overlap** ($\text{gap} < 15\text{ mm}$): The overlapping tail of the previous segment is trimmed, and the new segment is spliced directly.
- **Small Gap** ($15\text{ mm} \le \text{gap} \le 120\text{ mm}$): A smooth bridge is inserted. Position is interpolated via linear/cubic polynomial, and orientation is interpolated via spherical linear interpolation (SLERP):
  $$q(\tau) = \text{Slerp}(q_{\text{last}}, q_{\text{next}}, \tau), \quad \tau \in [0, 1]$$
  Bridged points are tagged with `gap=True` for operator auditing.
- **Large Gap** ($\text{gap} > 120\text{ mm}$): Bridged with an explicit operator warning that a wire section was omitted.

### Stage 4: Savitzky-Golay Position Smoothing
Hand-guided teaching introduces high-frequency motor tremor ($\approx 6\text{--}10\text{ Hz}$). We apply a third-order Savitzky-Golay filter across a sliding window of $N=7$ samples:
$$p_{\text{smooth}}(k) = \sum_{j=-3}^{3} c_j p(k+j)$$
Boundary conditions are pinned: $p_{\text{smooth}}(0) \equiv p(0)$ and $p_{\text{smooth}}(-1) \equiv p(-1)$ to preserve the exact entry and exit coordinates.

### Stage 5: Uniform Arc-Length Resampling
To ensure consistent velocity tracking, cumulative arc-length $s_k$ is computed:
$$s_0 = 0, \quad s_k = s_{k-1} + \|p_k - p_{k-1}\|$$
A 3D natural cubic spline parameterizes position $p(s)$ at fixed $\Delta s = 2.0\text{ mm}$ steps. Orientations are resampled via continuous SLERP along the new arc parameter.

---

## 4. Multi-Constraint Time-Optimal Velocity Profiling

Autonomous replay maximizes traversal speed along the wire while strictly preventing tool contact in tight U-bends. The velocity profile $v(s)$ is shaped by four physical constraints:

$$\begin{aligned}
v_{\text{cap}} &= v_{\text{max}} \cdot \text{speed\_fraction} && \text{(Global speed cap)} \\
v_{\text{lat}}(s) &= \sqrt{\frac{a_{\text{lat}}}{\kappa(s)}} && \text{(Centripetal / lateral acceleration limit)} \\
v_{\text{rot}}(s) &= \frac{\omega_{\text{max}}}{\theta'(s)} && \text{(Tool angular rate limit on steep legs)}
\end{aligned}$$

Where:
- $\kappa(s) = \|\frac{d^2 p}{ds^2}\|$ is the path curvature.
- $\theta'(s) = \|\text{Log}(R(s)^{-1} R(s + \Delta s))\| / \Delta s$ is the orientation rotation per metre.

### Bidirectional Integration Passes
1. **Forward Pass (Acceleration Limit $a_{\text{max}} = 0.15\text{ m/s}^2$)**:
   $$v_i \leftarrow \min\left(v_i, \sqrt{v_{i-1}^2 + 2 a_{\text{max}} \Delta s}\right)$$
2. **Backward Pass (Braking / Deceleration Limit)**:
   $$v_i \leftarrow \min\left(v_i, \sqrt{v_{i+1}^2 + 2 a_{\text{max}} \Delta s}\right)$$

### Closed-Loop Joint Saturation Feedback
During live execution, if motor joint velocities saturate against physical limits:
$$\text{scale}_{\text{joint}} = \min\left(1.0, \frac{\dot{q}_{\text{max}}}{\max_i |\dot{q}_i|}\right)$$
The virtual carrot advances at speed:
$$v_{\text{carrot}} = v(s) \cdot \text{scale}_{\text{joint}} \cdot \text{gate}(e_{\text{track}})$$
Where $\text{gate}(e) = \text{clip}\left(\frac{e_{\text{hi}} - e}{e_{\text{hi}} - e_{\text{lo}}}, 0, 1\right)$ dynamically slows the carrot if tracking error exceeds $4.0\text{ mm}$, and halts it if error reaches $10.0\text{ mm}$.
