# Literature Benchmarks and Theoretical Foundation

This document summarizes the peer-reviewed aerospace, robotics, estimation, and fault-tolerant control literature used to audit the legacy Project 4 baseline (`legacy_as_is/project4.mlx` and `legacy_as_is/project4_as_is.py`) and formulate the upgraded multi-UAV survivable autonomy architecture. Every identifier below has been verified against CrossRef or DataCite metadata registries.

## 1. Cooperative State Estimation and GNSS-Denied Navigation

In contested electromagnetic environments where GNSS is denied across extended time windows or geographic jamming regions, standalone dead-reckoning inertial navigation accumulates unbounded position and velocity covariance. Rigorous multi-agent estimation requires two properties that were absent in the baseline model: first, nonlinear coupling between inertial sensor biases, attitude, and translational kinematics inside an Error-State Extended Kalman Filter (ES-EKF); and second, conservative decentralized fusion of inter-agent relative range and shared state beliefs without double-counting unknown cross-correlations.

1. **Solà, J. (2017).** *Quaternion kinematics for the error-state Kalman filter.* `arXiv:1711.02508`.
   - Local file: `Sola_2017_Error_State_Kalman_Filter.pdf`
   - Architectural mapping: Establishes the strapdown IMU propagation equations, error-state Jacobian coupling for gyroscope and accelerometer biases, and Joseph-form covariance updates that guarantee positive semi-definiteness across long GNSS outages.

2. **Luft, L., Schubert, T., Roumeliotis, S. I., & Burgard, W. (2018).** *Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication.* *The International Journal of Robotics Research*, 37(10), 1152–1167. `DOI: 10.1177/0278364918760698`.
   - Architectural mapping: Benchmarks asynchronous pairwise communication updates when swarm links drop intermittently, guiding how inter-UAV relative range and shared-pose packets are assimilated without requiring synchronous all-to-all broadcasting.

3. **Tu, C., Cui, X., Liu, G., & Lu, M. (2025).** *Weighted Covariance Intersection for Range-based Distributed Cooperative Localization of Multi-Vehicle Systems.* `arXiv:2508.12207`.
   - Local file: `Tu_2025_Weighted_Covariance_Intersection_Multi_Vehicle_DCL.pdf`
   - Architectural mapping: Provides the Weighted Covariance Intersection (WCI) fusion law used to combine local INS error covariances with neighbor-shared beliefs over UWB range links during GNSS denial while preserving filter consistency (NEES bounds).

## 2. Decentralized Task Allocation Under Communication Impairments and Attrition

Static one-to-one task assignment collapses whenever an individual aircraft experiences propulsion degradation, rapid battery depletion, or mission attrition. Auction-based consensus protocols allow healthy teammates to absorb orphaned tasks automatically over intermittent communication graphs while bounding bandwidth consumption.

4. **Choi, H.-L., Brunet, L., & How, J. P. (2009).** *Consensus-Based Decentralized Auctions for Robust Task Allocation.* *IEEE Transactions on Robotics*, 25(4), 912–926. `DOI: 10.1109/TRO.2009.2022423`.
   - Architectural mapping: Defines the canonical two-phase Consensus-Based Bundle Algorithm (CBBA) combining marginal-score bundle construction with timestamped conflict resolution over arbitrary connected and intermittently connected topologies.

5. **Sao, V., Ho, T. D., Bhore, S., & Sujit, P. B. (2025).** *Event Driven CBBA with Reduced Communication.* *IEEE International Conference on Unmanned Aircraft Systems (ICUAS 2025)*. `DOI: 10.1109/icuas65942.2025.11007849` / `arXiv:2509.06481`.
   - Local file: `Sao_Sujit_2025_Event_Driven_CBBA_Reduced_Communication.pdf`
   - Architectural mapping: Introduces event-triggered consensus broadcasts so UAVs only transmit auction updates when marginal task bids shift meaningfully (for example, following an actuator fault, SOC drop, or task completion), cutting communication traffic by over 40 percent without sacrificing convergence guarantees.

## 3. Belief-Space Chance Constraints and Multi-Agent Safety Barrier Certificates

Navigating a multi-UAV formation through a spatially varying hazard field under growing positional uncertainty cannot rely on blind fixed-angle turns. Safety-critical planning requires probabilistic obstacle inflation paired with forward-invariant control barrier constraints that guarantee inter-agent separation in closed loop.

6. **Blackmore, L., Ono, M., & Williams, B. C. (2011).** *Chance-Constrained Optimal Path Planning With Obstacles.* *IEEE Transactions on Robotics*, 27(6), 1080–1094. `DOI: 10.1109/TRO.2011.2161160`.
   - Architectural mapping: Formulates analytical Gaussian chance constraints $\Pr(\mathbf{p}_k \in \mathcal{O}) \le \alpha$ by inflating hazard boundaries with the inverse error function of the positional covariance $\boldsymbol{\Sigma}_p$.

7. **Ames, A. D., Xu, X., Grizzle, J. W., & Tabuada, P. (2017).** *Control Barrier Function Based Quadratic Programs for Safety Critical Systems.* *IEEE Transactions on Automatic Control*, 62(8), 3861–3876. `DOI: 10.1109/TAC.2016.2638961`.
   - Architectural mapping: Establishes the Control Barrier Function Quadratic Program (CBF-QP) safety filter minimizing $\|\mathbf{u} - \mathbf{u}_{\text{nom}}\|^2$ subject to $\dot{h}(\mathbf{x}, \mathbf{u}) + \gamma h(\mathbf{x}) \ge 0$.

8. **Wang, L., Ames, A. D., & Egerstedt, M. (2017).** *Safety Barrier Certificates for Collisions-Free Multirobot Systems.* *IEEE Transactions on Robotics*, 33(3), 661–674. `DOI: 10.1109/TRO.2017.2659727`.
   - Architectural mapping: Extends CBF-QP to decentralized pairwise multi-robot safety barrier certificates guaranteeing zero inter-UAV collisions across crossing mission corridors.

9. **Chriat, A. E., & Sun, C. (2023).** *Wasserstein Distributionally Robust Control Barrier Function using Conditional Value-at-Risk with Differentiable Convex Programming.* `arXiv:2309.08700`.
   - Local file: `Chriat_Sun_2023_Distributionally_Robust_CBF_CVaR.pdf`
   - Architectural mapping: Bridges state estimation uncertainty and tail-risk metrics (CVaR) into robust barrier margins so non-Gaussian drift during GNSS outages does not erode safety invariance.

10. **Li, M., Sun, Z., Liao, Z., & Weiland, S. (2023).** *Moving Obstacle Collision Avoidance via Chance-Constrained MPC with CBF.* `arXiv:2304.01639`.
    - Local file: `Li_2023_Chance_Constrained_MPC_CBF_Collision_Avoidance.pdf`
    - Architectural mapping: Guides the uncertainty-inflated inter-agent separation margin $d_{\text{eff}} = d_{\text{safe}} + \kappa_\alpha(\sigma_i + \sigma_j)$ under noisy neighbor pose broadcasts.

11. **Yin, J., Tsiotras, P., & Berntorp, K. (2024).** *Chance-Constrained Information-Theoretic Stochastic Model Predictive Control with Safety Shielding.* `arXiv:2408.00494`.
    - Local file: `Yin_Tsiotras_2024_Chance_Constrained_MPC_Safety_Shielding.pdf`
    - Architectural mapping: Informs the belief-space risk-gradient guidance law combined with real-time CBF safety shielding under stochastic wind and sensor disturbances.

## 4. Separation-Principle-Compliant Actuator FDI and Fault-Tolerant Control

Operational flight software strictly obeys the estimation-control separation principle: fault detection and adaptive control laws may consume only onboard sensor measurements, estimator innovations, and commanded inputs—never unobservable simulation ground truth.

12. **Zhang, Y., & Jiang, J. (2008).** *Bibliographical review on reconfigurable fault-tolerant control systems.* *Annual Reviews in Control*, 32(2), 229–252. `DOI: 10.1016/j.arcontrol.2008.03.008`.
    - Architectural mapping: Defines the active FDI/FTC architecture separating residual-based fault detection (CUSUM / Generalized Likelihood Ratio on innovation residuals) from online parameter identification (Recursive Least Squares loss-of-effectiveness estimation) and reconfigurable control allocation.

13. **Karahan, M., Inal, M., & Kasnakoglu, C. (2023).** *Fault Tolerant Super Twisting Sliding Mode Control of a Quadrotor UAV Using Control Allocation.* *International Journal of Robotics and Control Systems*, 3(2). `DOI: 10.31763/ijrcs.v3i2.994` / `arXiv:2304.14350`.
    - Local file: `Karahan_2023_Fault_Tolerant_Control_Quadrotor_Allocation.pdf`
    - Architectural mapping: Benchmarks closed-loop quadrotor trajectory recovery under 40% actuator loss-of-effectiveness using online control allocation and disturbance rejection.
