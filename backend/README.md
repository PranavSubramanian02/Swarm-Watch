# Pure C++20 Swarm Backend (`backend/`)

The `backend/` subsystem implements the complete multi-UAV flight-mechanics plant, navigation estimator, adaptive fault-tolerant autopilot, decentralized swarm coordination, cyber-EW security layer, and embedded HTTP/WebSocket server strictly in zero-dependency C++20 with OpenMP parallelization.

## Module Decomposition

- **`backend/core/`**: Stack-allocated `Mat<R,C>` and Hamilton `Quat` math (`math_types.hpp`), 6-DOF fixed-wing aerodynamics with ISA density scaling, Dryden continuous turbulence, adverse yaw / Dutch-Roll coupling, wake-vortex trailing cones, and 1-RC battery electro-thermal voltage sag (`aero_plant`), Solà (2017) 15-state quaternion ESKF with wind-aware non-holonomic constraint (NHC) aiding and latency-compensated Weighted Covariance Intersection (`eskf_15state`), strictly observable $\mathcal{L}_1$ adaptive FTC with Pseudo-Control Hedging and TECS speed-altitude coupling (`l1_adaptive_ftc`), Park–Deyst–How $L_1$ path/loiter guidance, 3D CBF-QP + TCAS altitude deconfliction, 36-state discounted Bellman Value-Iteration MDP, and trajectory deviation / 15-second drift forecasting (`guidance_cbf_mdp`), Time-Based Rotating Master–Slave with preemptive health failover, multi-hop RF / Gilbert–Elliott burst loss, Fiedler algebraic connectivity $\lambda_2$, and Event-Driven CBBA (`swarm_coordination`), plus the unified engine and OpenMP ablation runner (`swarm_engine`).
- **`backend/security/`**: Constant-time LAN shared-secret token verification and sliding-window rate limiter (`auth_guard`), trust-boundary JSON payload bounds clamping and path-traversal rejection (`input_sanitizer`), and 3-DOF Mahalanobis $\chi^2$ GNSS spoofing rejection paired with monotonic TDMA epoch-lease verification (`cyber_ew_defense`).
- **`backend/transport/`**: Embedded multi-threaded POSIX HTTP/1.1 + RFC 6455 WebSocket server (`http_ws_server`, `main_server`) streaming 20 Hz telemetry and tactical alerts to both desktop and mobile clients.
- **`backend/tests/`**: CTest verification suite (`test_swarm_core.cpp`) validating filter norm/RMSE, spoof rejection, observable $\mathcal{L}_1$ detection, Master rotation/failover, disconnect alerts, CBBA savings, and security guards.

## Build and Verification

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/swarm_server --port 8080
```
