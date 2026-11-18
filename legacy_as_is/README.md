# Project 4: Survivable Multi-UAV Autonomy Under Uncertainty

## 1. Executive Summary & Problem Statement
Cooperative multi-UAV teams deployed in contested, GPS-degraded, or communication-denied environments face severe threats from environmental hazards, intermittent packet dropouts, actuator degradation, and state estimation drift. Traditional centralized autonomy fails when communication links drop or when agents enter high-risk hazard zones.

This project implements a multi-tier survivable autonomy stack for a team of $N = 3$ cooperative fixed-wing UAVs operating in a dynamic risk field with intermittent GPS access and packet loss:
1. **Part A (Core Dynamics & State Estimation)**: Implements 2D point-mass vehicle kinematics, an error-state Extended Kalman Filter (EKF), simulated GPS denial ($t \in [50, 150]\,\text{s}$), and stochastic communications.
2. **Part B (Belief-Space Planning with Chance Constraints)**: Navigates over joint state-covariance beliefs $(\boldsymbol{\mu}, \boldsymbol{\Sigma})$. Enforces $3\sigma$ positional chance constraints and dynamic risk field gradient avoidance.
3. **Part C (Markov Decision Process Autonomy & Fault Tolerance)**: Formulates an MDP discrete health-state abstraction to select operational modes (`NOMINAL`, `DEGRADED`, `CONSERVATIVE`, `EMERGENCY`), detects actuator degradation ($t > 140\,\text{s}$), and applies adaptive control compensation.

---

## 2. Theoretical Architecture & Mathematical Models

### 2.1 Vehicle Kinematics & Environmental Risk Field
Each UAV $u \in \{1, 2, 3\}$ is modeled in state space:
$$\mathbf{x} = \begin{bmatrix} x & y & v_x & v_y & \psi & b_g & b_a \end{bmatrix}^T \in \mathbb{R}^7$$

- **Point-Mass Kinematics**:
  $$\mathbf{p}_{k} = \mathbf{p}_{k-1} + \Delta t \cdot \mathbf{v}_{k-1}$$
  $$\mathbf{v}_{k} = v_{\text{cmd}} \frac{\mathbf{p}_{\text{task}} - \mathbf{p}}{\|\mathbf{p}_{\text{task}} - \mathbf{p}\|}$$
- **Continuous 2D Environmental Risk Field**:
  $$\mathcal{R}(x, y) = 0.0006(x^2 + y^2) + 5 \exp\left(-\frac{(x - 80)^2 + (y - 60)^2}{2600}\right) + 4 \exp\left(-\frac{(x + 120)^2 + (y + 90)^2}{2200}\right)$$

### 2.2 Error-State Extended Kalman Filter (EKF)
- **Kinematic State Transition**:
  $$\mathbf{F} = \mathbf{I}_7, \quad F_{1,3} = \Delta t, \quad F_{2,4} = \Delta t$$
  $$\hat{\mathbf{x}}_{k|k-1} = \mathbf{F} \hat{\mathbf{x}}_{k-1}, \quad \mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$$
  where $\mathbf{Q} = \operatorname{diag}([0, 0, 0.35, 0.35, 0.02, 2\times 10^{-4}, 2\times 10^{-4}])$.
- **GPS Measurement Model**:
  When GPS is active ($t \notin [50, 150]\,\text{s}$):
  $$\mathbf{z} = \mathbf{p}_{\text{true}} + \boldsymbol{\nu}, \quad \boldsymbol{\nu} \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_{\text{gps}}), \quad \mathbf{R}_{\text{gps}} = \operatorname{diag}([8, 8])$$
  $$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}$$

### 2.3 Belief-Space Planning & Chance Constraints (Part B)
Navigation operates on the positional belief distribution $\mathcal{N}(\boldsymbol{\mu}_p, \boldsymbol{\Sigma}_p)$:
$$\sigma_{\text{pos}} = \sqrt{\operatorname{Tr}(\boldsymbol{\Sigma}_p)}$$
- **Chance Constraint Violation Check**:
  $$\sigma_{\text{pos}} \cdot \kappa_{\sigma} > d_{\max} \quad \text{or} \quad \mathcal{R}(\mu_x, \mu_y) > \mathcal{R}_{\text{thresh}}$$
  where $\kappa_{\sigma} = 3.0$ ($99.7\%$ confidence), $d_{\max} = 20\,\text{m}$, and $\mathcal{R}_{\text{thresh}} = 15.0$.
- When a constraint is violated, the guidance vector executes an orthogonal avoidance maneuver:
  $$\mathbf{d}_{\text{avoid}} = \begin{bmatrix} -d_y \\ d_x \end{bmatrix}, \quad v_{\text{cmd}} = v_{\max,\text{safe}} = 2.5\,\text{m/s}$$

### 2.4 Discrete MDP Autonomy & Adaptive Control (Part C)
- **State Representation**: $2^3 = 8$ discrete states derived from 3 binary health indicators:
  - $s_1$: High Uncertainty ($\operatorname{Tr}(\mathbf{P}) > 120$)
  - $s_2$: GPS Available ($t \notin [50, 150]$)
  - $s_3$: Energy High ($\text{Energy} > 0.3$)
- **Actuator Fault & Adaptive Gain**:
  For $t > 140\,\text{s}$, actuator effectiveness degrades by $40\%$ ($\eta_{\text{act}} = 0.6$).
  $$\dot{\hat{\boldsymbol{\theta}}} = \gamma (\mathbf{v}_{\text{est}} - \mathbf{v}_{\text{true}})$$
  $$\mathbf{u}_{\text{ctrl}} = \operatorname{diag}(\hat{\boldsymbol{\theta}}) (\mathbf{v}_{\text{est}} - \mathbf{v}_{\text{true}})$$

---

## 3. Simulation Parameters & Mission Tasks

| Parameter | Symbol | Value | Unit | Description |
|:---|:---:|:---:|:---:|:---|
| Time Step | $\Delta t$ | $0.1$ | $\text{s}$ | Simulation step interval |
| Simulation Horizon | $T_{\text{end}}$ | $200.0$ | $\text{s}$ | Total execution time |
| Team Size | $N_{\text{UAV}}$ | $3$ | — | Cooperative agents |
| Task 1 Location | $\mathbf{p}_{\text{task}, 1}$ | $[160, 140]$ | $\text{m}$ | Target assigned to UAV 1 |
| Task 2 Location | $\mathbf{p}_{\text{task}, 2}$ | $[-150, 170]$ | $\text{m}$ | Target assigned to UAV 2 |
| Task 3 Location | $\mathbf{p}_{\text{task}, 3}$ | $[180, -120]$ | $\text{m}$ | Target assigned to UAV 3 |
| GPS Dropout Window | — | $[50, 150]$ | $\text{s}$ | Complete GPS denial |
| Comms Loss Rate | $P_{\text{loss}}$ | $0.25$ | — | Stochastic packet loss |

---

## 4. Execution & Usage

```bash
# Execute headless multi-UAV autonomy simulation (saves figures to output_figures/)
python3 project4_as_is.py

# Execute with interactive plot display
python3 project4_as_is.py --show
```

### Generated Outputs
The script generates the following diagnostics in `output_figures/`:
1. `fig_part_a_trajectories.png`: 3D time-space trajectories under baseline naive guidance.
2. `fig_part_a_uncertainty.png`: Covariance trace growth $\operatorname{Tr}(\mathbf{P})$ during GPS denial.
3. `fig_part_b_risk.png`: Temporal risk exposure $\mathcal{R}(\boldsymbol{\mu})$ under belief planning.
4. `fig_part_b_sigma_position.png`: $1\sigma$ positional uncertainty bounds over mission time.
5. `fig_part_c_mdp_modes.png`: Action timeline selected across the UAV fleet.
6. `fig_part_c_faults.png`: Flagged actuator degradation events.
7. `fig_part_c_control_effort.png`: Adaptive control effort $\|\mathbf{u}\|$ compensating for actuator faults.

Console Output Summary:
```text
================ FINAL SUMMARY ================
 UAV  FinalEnergy  FinalTraceP  FaultCount  TaskReassignments
   1         -3.0    50.764692         400                  0
   2         -3.0    50.764692         393                  0
   3         -3.0    50.764692         389                  0
================================================
```

---

## 5. Critical Engineering Audit: Holes, Faults & Simulation Limitations

> [!WARNING]
> The following architectural and mathematical flaws exist in the original **as-is** MATLAB implementation (`project4.mlx`) and are faithfully reproduced here for baseline assessment.

1. **Sequential Loop Overwriting & Trajectory Decoupling**:
   - The script is structured in three separate sequential loops: Part A, Part B, and Part C.
   - Part B re-runs from $k = 2 \dots N$ and completely overwrites `xTrue` and `xEst` generated by Part A.
   - In Part C, an adaptive control correction is calculated and applied to velocity (`xTrue[2:4] += uCtrl`), but **position is never integrated** (`xTrue[0:2]` is not updated in Part C).
   - **Result**: The adaptive control and fault-tolerant adjustments in Part C have **zero physical effect** on the aircraft trajectories; the flight paths plotted are purely the frozen trajectories from Part B.
2. **Ground Truth Contamination in Feedback Controller ("Cheating" Observer)**:
   - In lines 482 and 491, fault detection and adaptive control compute:
     $$\mathbf{e}_{\text{vel}} = \mathbf{x}_{\text{Est}} - \mathbf{x}_{\text{True}}$$
   - In operational flight software, the true ground truth $\mathbf{x}_{\text{True}}$ is unobservable. Feeding $\mathbf{x}_{\text{True}}$ directly into the feedback control law violates estimator-controller separation.
3. **Degenerate MDP Reward Table**:
   - The reward matrix $R(s, a)$ is populated without referencing the state index $s$:
     $$R(s, a) = -w_{\text{Energy}}\mathbf{1}_{a=4} - w_{\text{Risk}}\mathbf{1}_{a=1} - w_{\text{Unc}}\mathbf{1}_{a=1}$$
   - For all $s \in \{0, \dots, 7\}$: $R(s, 1) = -14$, $R(s, 2) = 0$, $R(s, 3) = 0$, and $R(s, 4) = -5$.
   - **Result**: $\operatorname{argmax}_a R(s, a) \equiv 2$ (`DEGRADED`) for all states, all agents, and all time steps. The MDP state encoding has zero effect on mode selection.
4. **Variable Name Typo in Task Reallocation**:
   - In Section 5.5, line 513 sets `taskAssign(u) = 0`, whereas the array allocated and checked elsewhere is `taskAssigned`.
5. **Dormant EKF States**:
   - States 5, 6, and 7 ($\psi$, gyro bias $b_g$, accelerometer bias $b_a$) have no measurement equations ($H$ is $2 \times 7$) and no cross-coupling into $\mathbf{F}$. They remain dormant and unobservable throughout the entire simulation.
6. **Energy Underflow**:
   - Energy rate is subtracted at every step without lower bounds, causing final energy to report negative values ($-3.0$).
