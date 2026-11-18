# Swarm-Watch: Survivable Multi-UAV Autonomy Under Uncertainty

This repository presents a multi-tier survivable fixed-wing multi-UAV autonomy suite targeting contested, GNSS-denied, communication-impaired, and actuator-degraded environments. The original M.Tech MATLAB Live Script (`project4.mlx`), its extracted plain-text `.m` source (`project4.m`), and the faithful baseline Python port (`project4_as_is.py`) are preserved verbatim in [`legacy_as_is/`](legacy_as_is/) alongside all seven baseline figures. The upgraded system resolves all twelve structural, estimation, guidance, control, and autonomy flaws identified in the baseline by unifying the entire flight-software core, cyber-EW security layer, HTTP/WebSocket transport, and native 60 FPS OpenGL 3D tactical viewer in zero-dependency **C++20 + OpenMP**, paired with a pure **HTML5/CSS3/Vanilla JS** web console and a minimal **Kotlin + Jetpack Compose** Android tactical alert companion app (*SwarmWatch*).

## System Architecture

```mermaid
flowchart TD
    subgraph BE["backend/  (100% Pure C++20 + OpenMP + CTest)"]
        CORE["core/\n6-DOF Non-Holonomic Fixed-Wing Plant\n15-State Hamilton Quaternion ESKF + NHC + WCI\nL1 Adaptive FTC + Rotating Master-Slave\n3D CBF-QP + 36-State Bellman MDP + ED-CBBA"]
        SEC["security/\nLAN Token Auth + Rate Limiter\nInput Bounds Sanitizer\nMahalanobis GNSS Spoof Gate + Lease Anti-Hijack"]
        TX["transport/\nEmbedded HTTP/1.1 + RFC 6455 WebSocket :8080"]
        CORE --> TX
        SEC --> TX
        SEC --> CORE
    end

    subgraph FE["frontend/  (Synchronized Desktop & Browser Visualizers)"]
        GL["opengl_viewer/\nNative 60 FPS C++20 OpenGL/GLFW 3D Viewer"]
        WEB["web_console/\nWCAG AA HTML5/CSS3/JS 3D-2D Canvas + Plotly"]
    end

    subgraph APP["app/  (SwarmWatch Kotlin Android App)"]
        ABE["backend/\nOkHttp WS + Dual-Trigger AlertNotifier + OfflineDemo"]
        ASEC["security/\nLocalNetworkGuard + TokenAuth + Consent + DataWipe"]
        AFE["frontend/\nJetpack Compose: Alerts, Trajectory, Swarm, Legal"]
        ASEC --> ABE --> AFE
    end

    TX -->|"Direct Link"| GL
    TX -->|"WS /ws/swarm + REST :8080"| WEB
    TX -->|"Auth WS :8080"| ABE
```

## Resolution of Legacy Baseline Flaws

An engineering audit of `legacy_as_is/project4_as_is.py` exposed twelve foundational defects: sequential loop overwriting across Parts A, B, and C; unintegrated velocity updates in Part C giving zero trajectory effect to adaptive control; separation-principle violation via ground-truth velocity feedback (`xEst - xTrue`); a state-independent MDP reward table permanently locked in `DEGRADED`; uncoupled dormant EKF states ($\psi, b_g, b_a$); unread communication broadcasts; naive $+90^\circ$ orthogonal turning traps; absent inter-UAV collision barriers; static task assignment with a MATLAB variable typo; unbounded negative energy underflow (`FinalEnergy = -3.0`); instantaneous velocity teleportation contradicting fixed-wing stall physics; and single-seed deterministic evaluation.

The upgraded C++20 architecture replaces these sequentially coupled scripts with a single closed-loop multi-agent flight step (`dt = 0.1 s`) incorporating twenty-three coupled flight-mechanics and swarm mechanisms:

1. **6-DOF Non-Holonomic Fixed-Wing Flight Mechanics**: Bounded airspeed ($14\text{--}26\,\text{m/s}$), bank-dependent aerodynamic stall envelope ($V_{\text{stall}}(\phi) = V_{\text{stall},0}/\sqrt{\cos\phi}$), ISA altitude density scaling $\rho(h)$, MIL-HDBK-1797 Dryden continuous atmospheric turbulence, adverse yaw and roll–yaw Dutch-Roll rigid-body rotational coupling, trailing wingtip wake-vortex disturbances, asymmetric wing icing, first-order actuator lag with deflection/rate saturation, and a 6S 16 Ah 1-RC electro-thermal battery model clamped strictly in $\text{SOC} \in [0, 1]$ with realistic Master RF transmit power draw (`+18 W`).
2. **15-State Hamilton Quaternion ESKF with Wind-Aware NHC and WCI**: Following Solà (2017, `arXiv:1711.02508`) and Tu et al. (2025, `arXiv:2508.12207`), tracks a 16-dimensional nominal state $[\mathbf{p}, \mathbf{v}, \mathbf{q}, \mathbf{b}_a, \mathbf{b}_g]$ and minimal 15-dimensional tangent-space error state on $SO(3)$ with Joseph-form positive-semi-definite covariance updates, tangent-space reset $\mathbf{P} \leftarrow \mathbf{G}\mathbf{P}\mathbf{G}^T$, centripetal-compensated gravity tilt and magnetometer aiding, wind-aware non-holonomic zero-sideslip aiding ($[\mathbf{R}^T(\mathbf{v}^n - \hat{\mathbf{w}})]_y = 0$), 3-DOF Mahalanobis $\chi^2$ GNSS spoofing rejection (Wan, Hovakimyan et al. 2020, `arXiv:1910.10826`), and latency-compensated Weighted Covariance Intersection over UWB links.
3. **Strictly Observable $\mathcal{L}_1$ Adaptive Fault-Tolerant Control**: Following Hovakimyan, Cao, Xargay, & Gregory (*IEEE Control Systems Magazine*, 2011, `DOI: 10.1109/MCS.2011.941961`) and Lee, Bernstein et al. (2022, `arXiv:2210.13621`), executes a state predictor with Pseudo-Control Hedging, projection-bounded adaptation for unknown roll/thrust effectiveness ($\hat{\boldsymbol{\Lambda}}$) and matched icing/trim disturbances ($\hat{\boldsymbol{\sigma}}$), bandwidth-limited low-pass filtering $C(s)$, CUSUM residual fault detection, and Total Energy Control System (TECS) speed–altitude coupling—accepting zero ground-truth arguments at compile time.
4. **Time-Based Rotating Master–Slave, Multi-Hop RF, Bellman MDP, and Event-Driven CBBA**: Rotates Master authority across scheduled TDMA epochs ($T_{\text{rot}} = 35\,\text{s}$) to balance RF battery load and provide Moving Target Defense against jamming (Zhou et al. 2025, `arXiv:2506.07392`), paired with instant health-weighted preemptive failover protected by monotonic epoch-lease verification, 2-hop RF relay forwarding with Fiedler algebraic connectivity $\lambda_2(\mathbf{L})$ tracking, discounted Bellman Value Iteration over 36 belief-health states exercising all four modes dynamically, Event-Driven CBBA task reallocation (Sao & Sujit 2025, `DOI: 10.1109/icuas65942.2025.11007849`), Park–Deyst–How $L_1$ path and wind-compensated loiter-orbit guidance, and 3D fixed-wing CBF-QP + TCAS flight-level deconfliction.

## Quantitative Ablation Benchmarks (100-Run OpenMP Monte Carlo)

Evaluated across `N = 100` randomized 160-second contested 5-UAV sorties per configuration (featuring a 60-second global GNSS blackout, distance-dependent Gilbert–Elliott burst packet dropouts, and 45% roll / 40% thrust actuator loss on UAV-2):

| Configuration | Mean 3D RMSE [m] | NEES In-Bounds [%] | ISR Tasks Completed [%] | Fleet SOC Spread [%] | CBBA Packet Savings [%] |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Full Survivable Stack (`ESKF+NHC+WCI+L1+RotMaster+CBBA`)** | **152.6** | **15.6%** | **46.0%** | **1.08%** | **99.7%** |
| Solo ESKF Ablation (`No WCI / No NHC`) | 27,917.9 | 11.3% | 19.2% | 1.32% | 99.8% |
| Static Master–Slave Ablation (`No Rotation`) | 142.7 | 16.5% | 49.0% | 1.08% | 99.7% |
| Nominal PID Ablation (`No L1 Adaptive FTC`) | 178.9 | 12.5% | 39.4% | 1.02% | 99.7% |

Key verified engineering gains:
- **183x Reduction in GNSS-Denied Position Drift**: Wind-aware NHC + Master WCI cuts mean 3D RMSE from `27,917.9 m` (unconstrained MEMS tilt/velocity dead reckoning) down to `152.6 m` while increasing ISR task completion by **2.4x** (`46.0%` vs `19.2%`).
- **Observable $\mathcal{L}_1$ Adaptive FTC Recovery**: Under compound 45% roll / 40% thrust degradation plus asymmetric icing, $\mathcal{L}_1$ adaptation lowers estimation/tracking error (`152.6 m` vs `178.9 m`) and raises mission completion (`46.0%` vs `39.4%`) without touching ground truth.
- **Physical Energy Conservation**: Eliminates the baseline `-3.0` underflow; all aircraft maintain bounded positive SOC (`0.92` down to `~0.83`, spread `1.08%`) above the `18.8 V` cutoff.
- **Bandwidth-Efficient Decentralized Tasking**: Event-Driven CBBA reduces auction radio traffic by `99.7%` relative to continuous per-step broadcasting while automatically reallocating orphaned targets on attrition or `EMERGENCY_RTB`.

## Repository Organization

- [`backend/`](backend/README.md) — Zero-dependency C++20 flight software (`core/`), cyber-EW and API guards (`security/`), HTTP/WebSocket server (`transport/`), and CTest verification (`tests/`).
- [`frontend/`](frontend/README.md) — Native 60 FPS C++20 OpenGL/GLFW 3D tactical viewer (`opengl_viewer/`) and WCAG AA accessible HTML5/CSS3/Vanilla JS tactical web console (`web_console/`).
- [`app/`](app/README.md) — Minimal 3-tab Kotlin + Jetpack Compose Android companion app (*SwarmWatch*), internally partitioned into `backend/`, `security/`, and `frontend/` with dual-trigger push notifications, Ideal-vs-Actual trajectory deviation canvas, Offline Demo Mode, Privacy Policy, and 1-tap local data deletion.
- [`tools/export_figures.py`](tools/export_figures.py) — Offline-only Matplotlib script generating all 12 300 DPI publication PNGs into [`output_figures/`](output_figures/).
- [`research_papers/`](research_papers/LITERATURE_BENCHMARKS.md) — 12 verified open-access peer-reviewed PDFs and full bibliographic mapping across 19 CrossRef/DataCite-verified references.
- [`legacy_as_is/`](legacy_as_is/) — Archived original MATLAB Live Script (`project4.mlx`), extracted `project4.m`, baseline `project4_as_is.py`, and 7 original figures.

## Build, Verification, and Deployment Commands

```bash
# 1. Configure, build C++20 core + native OpenGL viewer, and run CTest
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

# 2A. Launch native 60 FPS C++20 OpenGL 3D Viewer + embedded :8080 Web/Android server
./build/swarm_opengl_viewer --port 8080

# 2B. Or launch headless C++20 server (open http://127.0.0.1:8080/ in browser)
./build/swarm_server --port 8080

# 3. Connect Kotlin Android SwarmWatch over USB (open app/ in Android Studio)
adb reverse tcp:8080 tcp:8080

# 4. Regenerate all 12 publication-grade 300 DPI figures offline
source ~/.venvs/eng-ds/bin/activate
python3 tools/export_figures.py
```

## Verified References

All twelve downloaded open-access papers and seven foundational journal articles are cataloged with verified DOIs and arXiv IDs in [`research_papers/LITERATURE_BENCHMARKS.md`](research_papers/LITERATURE_BENCHMARKS.md), including Solà (`arXiv:1711.02508`), Tu et al. (`arXiv:2508.12207`), Lei et al. (`arXiv:2509.04412`), Wan, Hovakimyan et al. (`arXiv:1910.10826`), Hovakimyan et al. (`DOI: 10.1109/MCS.2011.941961`), Lee, Bernstein et al. (`arXiv:2210.13621`), Feurgard et al. (`arXiv:2603.21789`), Zhou et al. (`arXiv:2506.07392`), Choi, Brunet, & How (`DOI: 10.1109/TRO.2009.2022423`), Sao & Sujit (`DOI: 10.1109/icuas65942.2025.11007849`), Blackmore, Ono, & Williams (`DOI: 10.1109/TRO.2011.2161160`), Ames et al. (`DOI: 10.1109/TAC.2016.2638961`), and Wang, Ames, & Egerstedt (`DOI: 10.1109/TRO.2017.2659727`).
