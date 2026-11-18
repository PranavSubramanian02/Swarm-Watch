# Literature Benchmarks and Theoretical Foundation

This document summarizes the peer-reviewed aerospace, flight-mechanics, navigation, adaptive-control, cyber-EW, and swarm-autonomy literature used to audit the legacy Project 4 baseline (`legacy_as_is/project4.mlx`, `legacy_as_is/project4.m`, and `legacy_as_is/project4_as_is.py`) and formulate the upgraded survivable fixed-wing multi-UAV architecture. Every identifier below has been verified against CrossRef, DataCite, or arXiv metadata registries, and 12 open-access full-text PDFs are archived locally in this directory.

## 1. 15-State Quaternion ESKF, Non-Holonomic Dead Reckoning, and Cooperative Localization

In GNSS-denied contested airspace, standalone dead-reckoning INS drifts rapidly unless inertial biases are properly coupled to attitude on $SO(3)$, non-holonomic aerodynamic constraints are fused with air-relative velocity, and inter-UAV relative range/belief updates are combined conservatively without double-counting cross-correlations.

1. **Solà, J. (2017).** *Quaternion kinematics for the error-state Kalman filter.* `arXiv:1711.02508`.
   - Local PDF: `Sola_2017_Error_State_Kalman_Filter.pdf`
   - Architectural mapping: Foundation of the 15-state Hamilton quaternion ESKF (`[p, v, q, b_a, b_g]` nominal, `[δp, δv, δθ, δb_a, δb_g]` tangent-space error), exact $15 \times 15$ strapdown Jacobian, Joseph-form covariance update, and tangent-space reset $\mathbf{P} \leftarrow \mathbf{G}\mathbf{P}\mathbf{G}^T$.

2. **Luft, L., Schubert, T., Roumeliotis, S. I., & Burgard, W. (2018).** *Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication.* *The International Journal of Robotics Research*, 37(10), 1152–1167. `DOI: 10.1177/0278364918760698`.
   - Architectural mapping: Guides asynchronous pairwise communication fusion under bursty packet dropouts and clock-skew latency compensation.

3. **Tu, C., Cui, X., Liu, G., & Lu, M. (2025).** *Weighted Covariance Intersection for Range-based Distributed Cooperative Localization of Multi-Vehicle Systems.* `arXiv:2508.12207`.
   - Local PDF: `Tu_2025_Weighted_Covariance_Intersection_Multi_Vehicle_DCL.pdf`
   - Architectural mapping: Provides the Weighted Covariance Intersection (WCI) fusion law combining local INS error covariances with Master/peer UWB range and shared beliefs during GNSS denial while preserving NEES consistency.

4. **Lei, G., Ping, Y., Liang, T., Ding, H., & Zhang, T. (2025).** *Relative Localization of UAV Swarms in GNSS-Denied Conditions.* `arXiv:2509.04412`.
   - Local PDF: `Lei_2025_Relative_Localization_UAV_Swarms_GNSS_Denied.pdf`
   - Architectural mapping: Benchmarks multi-UAV UWB relative localization under packet loss and GDOP-sensitive swarm geometry.

## 2. Cyber-EW Integrity: GNSS Spoofing Detection and Escape Margins

Contested electromagnetic environments combine signal blackout with adversarial GNSS spoofing ramps and jumps. Resilient autonomy requires innovation-based spoofing detection, immediate rejection of corrupted GNSS fixes, and safe mode switching.

5. **Wan, W., Kim, H., Hovakimyan, N., Sha, L., & Voulgaris, P. G. (2020).** *A Safety Constrained Control Framework for UAVs in GPS Denied Environment.* `arXiv:1910.10826`.
   - Local PDF: `Wan_Hovakimyan_2020_Safety_Constrained_UAV_GPS_Spoofing_Denied.pdf`
   - Architectural mapping: Motivates innovation-based GNSS spoofing detection (Mahalanobis $\chi^2$ gating), safety escape-time / time-to-hazard margins, and automated transition between nominal and degraded/emergency modes.

## 3. Adaptive Fault-Tolerant Control for Fixed-Wing Aircraft

Strict flight-software design enforces the estimation–control separation principle: fault detection and adaptive control may consume only observable estimator states and onboard sensor readings, never simulation ground truth.

6. **Hovakimyan, N., Cao, C., Kharisov, E., Xargay, E., & Gregory, I. M. (2011).** *$\mathcal{L}_1$ adaptive control for safety-critical systems.* *IEEE Control Systems Magazine*, 31(5), 54–104. `DOI: 10.1109/MCS.2011.941961`.
   - Architectural mapping: Establishes the $\mathcal{L}_1$ adaptive flight-control architecture (state predictor, projection-based fast adaptation for unknown actuator effectiveness $\hat{\boldsymbol{\Lambda}}$ and matched disturbance $\hat{\boldsymbol{\sigma}}$, and bandwidth-limited low-pass filter $C(s)$) with guaranteed transient bounds under actuator saturation.

7. **Lee, J., Spencer, J., Shao, S., Paredes, J. A., Bernstein, D. S., & Goel, A. (2022).** *Experimental Flight Testing of a Fault-Tolerant Adaptive Autopilot for Fixed-Wing Aircraft.* `arXiv:2210.13621`.
   - Local PDF: `Lee_Bernstein_2022_Fixed_Wing_Adaptive_FTC_Flight_Test.pdf`
   - Architectural mapping: Benchmarks fixed-wing digital adaptive autopilot recovery under degraded control-surface effectiveness and stuck/biased surface faults.

8. **Zhang, Y., & Jiang, J. (2008).** *Bibliographical review on reconfigurable fault-tolerant control systems.* *Annual Reviews in Control*, 32(2), 229–252. `DOI: 10.1016/j.arcontrol.2008.03.008`.
   - Architectural mapping: Defines active FDI/FTC separation between residual-based fault detection (CUSUM / $\chi^2$) and online adaptive reconfiguration.

9. **Karahan, M., Inal, M., & Kasnakoglu, C. (2023).** *Fault Tolerant Super Twisting Sliding Mode Control of a Quadrotor UAV Using Control Allocation.* *IJRCS*, 3(2). `DOI: 10.31763/ijrcs.v3i2.994` / `arXiv:2304.14350`.
   - Local PDF: `Karahan_2023_Fault_Tolerant_Control_Quadrotor_Allocation.pdf`
   - Architectural mapping: Reference control-allocation benchmark for 40% propulsion loss-of-effectiveness recovery.

## 4. Non-Holonomic Fixed-Wing Guidance, Chance Constraints, and Multi-Agent CBF-QP

Fixed-wing aircraft cannot hover or brake to zero speed; collision avoidance and risk-field navigation must respect minimum airspeed ($V_{\text{stall}}(\phi)$), bounded bank angle, Dubins turning radii, wind drift, and 3D vertical/horizontal deconfliction.

10. **Feurgard, M., Hattenberger, G., Durand, N., & Lacroix, S. (2026).** *Safe and wind-aware synchronous path planning for a fleet of fixed-wing constant speed aircraft.* `arXiv:2603.21789`.
    - Local PDF: `Feurgard_2026_Wind_Aware_Fixed_Wing_Fleet_Dubins_Planning.pdf`
    - Architectural mapping: Guides wind-aware non-holonomic fixed-wing fleet routing, Dubins turn constraints, and conflict-free multi-aircraft separation in formation transitions.

11. **Blackmore, L., Ono, M., & Williams, B. C. (2011).** *Chance-Constrained Optimal Path Planning With Obstacles.* *IEEE Transactions on Robotics*, 27(6), 1080–1094. `DOI: 10.1109/TRO.2011.2161160`.
    - Architectural mapping: Analytical Gaussian chance constraints inflating spatial risk boundaries with positional covariance $\boldsymbol{\Sigma}_p$.

12. **Ames, A. D., Xu, X., Grizzle, J. W., & Tabuada, P. (2017).** *Control Barrier Function Based Quadratic Programs for Safety Critical Systems.* *IEEE Transactions on Automatic Control*, 62(8), 3861–3876. `DOI: 10.1109/TAC.2016.2638961`.
    - Architectural mapping: Canonical CBF-QP safety shield formulation.

13. **Wang, L., Ames, A. D., & Egerstedt, M. (2017).** *Safety Barrier Certificates for Collisions-Free Multirobot Systems.* *IEEE Transactions on Robotics*, 33(3), 661–674. `DOI: 10.1109/TRO.2017.2659727`.
    - Architectural mapping: Pairwise multi-agent safety barrier certificates extended to 3D fixed-wing bank + altitude deconfliction.

14. **Chriat, A. E., & Sun, C. (2023).** *Wasserstein Distributionally Robust Control Barrier Function using Conditional Value-at-Risk with Differentiable Convex Programming.* `arXiv:2309.08700`.
    - Local PDF: `Chriat_Sun_2023_Distributionally_Robust_CBF_CVaR.pdf`
    - Architectural mapping: Incorporates estimation tail-risk uncertainty into safety barrier margins during GNSS outages.

15. **Li, M., Sun, Z., Liao, Z., & Weiland, S. (2023).** *Moving Obstacle Collision Avoidance via Chance-Constrained MPC with CBF.* `arXiv:2304.01639`.
    - Local PDF: `Li_2023_Chance_Constrained_MPC_CBF_Collision_Avoidance.pdf`
    - Architectural mapping: Uncertainty-inflated inter-agent separation margins under noisy neighbor broadcasts.

16. **Yin, J., Tsiotras, P., & Berntorp, K. (2024).** *Chance-Constrained Information-Theoretic Stochastic Model Predictive Control with Safety Shielding.* `arXiv:2408.00494`.
    - Local PDF: `Yin_Tsiotras_2024_Chance_Constrained_MPC_Safety_Shielding.pdf`
    - Architectural mapping: Belief-space risk-gradient guidance integrated with real-time safety shielding under stochastic wind disturbances.

## 5. Time-Based Rotating Master–Slave Swarming and Event-Driven Task Allocation

Static leader–follower formations create a single point of failure for battery depletion, EW jamming, and covariance drift. Rotating Master authority across epochs with health-weighted preemptive failover and event-driven auction reallocation preserves fleet survivability.

17. **Zhou, Y., Cheng, G., Du, K., Chen, Z., Qin, T., & Zhao, Y. (2025).** *From Static to Adaptive Defense: Federated Multi-Agent Deep Reinforcement Learning-Driven Moving Target Defense Against DoS Attacks in UAV Swarm Networks.* `arXiv:2506.07392`.
    - Local PDF: `Zhou_2025_Dynamic_Leader_Switching_MTD_UAV_Swarm.pdf`
    - Architectural mapping: Motivates dynamic Master–Slave leader rotation as Moving Target Defense (MTD) against jamming/DoS targeting and instant leader failover on degradation.

18. **Choi, H.-L., Brunet, L., & How, J. P. (2009).** *Consensus-Based Decentralized Auctions for Robust Task Allocation.* *IEEE Transactions on Robotics*, 25(4), 912–926. `DOI: 10.1109/TRO.2009.2022423`.
    - Architectural mapping: Canonical Consensus-Based Bundle Algorithm (CBBA) for decentralized task allocation over intermittent graphs.

19. **Sao, V., Ho, T. D., Bhore, S., & Sujit, P. B. (2025).** *Event Driven CBBA with Reduced Communication.* *IEEE ICUAS 2025*. `DOI: 10.1109/icuas65942.2025.11007849` / `arXiv:2509.06481`.
    - Local PDF: `Sao_Sujit_2025_Event_Driven_CBBA_Reduced_Communication.pdf`
    - Architectural mapping: Event-triggered auction broadcasting to cut radio overhead by over 40% while guaranteeing task handoff on UAV faults, low SOC, or attrition.
