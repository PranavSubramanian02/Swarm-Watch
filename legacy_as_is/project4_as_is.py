#!/usr/bin/env python3
"""
===========================================================================
SURVIVABLE MULTI-UAV AUTONOMY UNDER UNCERTAINTY
Parts A, B, and C: State Estimation, Belief-Space Planning, and MDP Autonomy

As-Is Python Migration of MATLAB Live Script (project4.mlx)
Author of Original Model: Pranav Subramanian (M.Tech)
Target Venue: ICAAV-2026 (ADE-DRDO & AeSI)

Architecture:
- Part A: Point-mass dynamics, GPS denial, intermittent comms, error-state EKF
- Part B: Belief-space planning, chance constraints, risk gradient avoidance
- Part C: Markov Decision Process (MDP) autonomy, actuator FTC, adaptive control
===========================================================================
"""

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def run_multi_uav(show_plots=False, save_dir="output_figures"):
    # Reproducibility (rng(42))
    np.random.seed(42)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    # =========================================================================
    # 1. TIME & GLOBAL PARAMETERS
    # =========================================================================
    dt    = 0.1                            # Time step [s]
    T_end = 200.0                          # Total simulation time [s]
    time  = np.arange(0, T_end + dt, dt)   # Time vector
    N     = len(time)

    numUAV = 3                             # Number of UAVs
    nx     = 7                             # State dimension: [x, y, vx, vy, psi, gyroBias, accelBias]

    # =========================================================================
    # 2. ENVIRONMENT & RISK FIELD
    # =========================================================================
    x_coords = np.arange(-250, 255, 5)
    y_coords = np.arange(-250, 255, 5)
    xg, yg = np.meshgrid(x_coords, y_coords)

    riskMap = (
        0.0006 * (xg ** 2 + yg ** 2) +
        5.0 * np.exp(-((xg - 80.0) ** 2 + (yg - 60.0) ** 2) / 2600.0) +
        4.0 * np.exp(-((xg + 120.0) ** 2 + (yg + 90.0) ** 2) / 2200.0)
    )

    # Interpolator accepts (y, x) queries matching MATLAB interp2(xg, yg, riskMap, x, y)
    risk_interp = RegularGridInterpolator((y_coords, x_coords), riskMap, bounds_error=False, fill_value=10.0)

    def riskFcn(x_val, y_val):
        return float(risk_interp([y_val, x_val])[0])

    # =========================================================================
    # 3. STATE STORAGE
    # =========================================================================
    xTrue = np.zeros((nx, N, numUAV))      # Ground truth
    xEst  = np.zeros((nx, N, numUAV))      # Estimated state
    P     = np.zeros((nx, nx, N, numUAV))  # State covariance
    traceP = np.zeros((N, numUAV))         # Uncertainty metric

    # =========================================================================
    # 4. INITIAL CONDITIONS
    # =========================================================================
    for u in range(numUAV):
        xTrue[:, 0, u] = [
            -200.0 + 70.0 * (u + 1),        # x (1-based index in MATLAB: u=1,2,3 -> -130, -60, 10)
            -200.0,                         # y
            4.0,                            # vx
            4.0,                            # vy
            0.0,                            # heading
            0.0,                            # gyro bias
            0.0                             # accel bias
        ]
        xEst[:, 0, u] = xTrue[:, 0, u] + np.concatenate([10.0 * np.random.randn(2), np.zeros(5)])
        P[:, :, 0, u] = np.diag([60.0, 60.0, 3.0, 3.0, 0.1, 0.02, 0.02])

    # =========================================================================
    # 5. PROCESS & MEASUREMENT MODELS
    # =========================================================================
    Q = np.diag([0.0, 0.0, 0.35, 0.35, 0.02, 2e-4, 2e-4])  # Process noise
    Rgps = np.diag([8.0, 8.0])                             # GPS measurement noise covariance
    cholRgps = np.linalg.cholesky(Rgps)

    gpsAvailable = np.ones(N, dtype=bool)
    gpsAvailable[(time > 50.0) & (time < 150.0)] = False    # GPS-denied window

    # =========================================================================
    # 6. COMMUNICATION MODEL
    # =========================================================================
    commAvailable = (np.random.rand(numUAV, N) > 0.25)
    sharedPosition = np.full((2, N, numUAV), np.nan)

    # =========================================================================
    # 7. ENERGY MODEL
    # =========================================================================
    energy = np.ones((N, numUAV))
    energyRateNominal = 0.0025
    energyRateSafe    = 0.0012

    # =========================================================================
    # 8. TASK LOCATIONS (STATIC FOR PART A)
    # =========================================================================
    tasks = np.array([
        [160.0, 140.0],
        [-150.0, 170.0],
        [180.0, -120.0]
    ])
    taskAssigned = np.zeros(numUAV, dtype=int)

    # =========================================================================
    # 9. MAIN SIMULATION LOOP (PART A: CORE ONLY)
    # =========================================================================
    for k in range(1, N):
        for u in range(numUAV):
            # 9.1 Simple Task Assignment
            if taskAssigned[u] == 0:
                taskAssigned[u] = u  # 0-indexed in python corresponds to task u
            goal = tasks[taskAssigned[u]]

            # 9.2 Simple Guidance Law
            pos = xTrue[0:2, k - 1, u]
            direction = goal - pos
            direction = direction / (np.linalg.norm(direction) + 1e-6)
            vCmd = 4.0 * direction

            # 9.3 True Dynamics
            xTrue[0:2, k, u] = xTrue[0:2, k - 1, u] + dt * xTrue[2:4, k - 1, u]
            xTrue[2:4, k, u] = vCmd
            xTrue[4, k, u]   = np.arctan2(vCmd[1], vCmd[0])
            xTrue[5:7, k, u] = xTrue[5:7, k - 1, u]

            # 9.4 Error-State EKF Prediction
            F = np.eye(nx)
            F[0, 2] = dt
            F[1, 3] = dt

            xPred = F @ xEst[:, k - 1, u]
            PPred = F @ P[:, :, k - 1, u] @ F.T + Q

            # 9.5 GPS Measurement Update
            if gpsAvailable[k]:
                H = np.zeros((2, nx))
                H[0, 0] = 1.0
                H[1, 1] = 1.0

                z = xTrue[0:2, k, u] + cholRgps.T @ np.random.randn(2)
                S_cov = H @ PPred @ H.T + Rgps
                K = PPred @ H.T @ np.linalg.inv(S_cov)

                xEst[:, k, u] = xPred + K @ (z - H @ xPred)
                P[:, :, k, u] = (np.eye(nx) - K @ H) @ PPred
            else:
                xEst[:, k, u] = xPred
                P[:, :, k, u] = PPred

            # 9.6 Communication Broadcast
            if commAvailable[u, k]:
                sharedPosition[:, k, u] = xEst[0:2, k, u]

            # 9.7 Energy Consumption
            energy[k, u] = energy[k - 1, u] - energyRateNominal

            # 9.8 Diagnostics
            traceP[k, u] = np.trace(P[:, :, k, u])

    # Save Part A figure results
    fig_a1 = plt.figure(figsize=(8, 6))
    ax_a1 = fig_a1.add_subplot(111, projection='3d')
    for u in range(numUAV):
        ax_a1.plot(xTrue[0, :, u], xTrue[1, :, u], time, linewidth=2, label=f'UAV {u+1}')
    ax_a1.set_xlabel('X [m]')
    ax_a1.set_ylabel('Y [m]')
    ax_a1.set_zlabel('Time [s]')
    ax_a1.set_title('PART A: 3D UAV Trajectories')
    ax_a1.legend(loc='best')
    ax_a1.grid(True)
    fig_a1.tight_layout()
    if save_dir:
        fig_a1.savefig(os.path.join(save_dir, "fig_part_a_trajectories.png"), dpi=300)

    fig_a2, ax_a2 = plt.subplots(figsize=(7, 4.5))
    for u in range(numUAV):
        ax_a2.plot(time, traceP[:, u], linewidth=2, label=f'UAV {u+1}')
    ax_a2.set_xlabel('Time [s]')
    ax_a2.set_ylabel('Trace(P)')
    ax_a2.set_title('PART A: State Uncertainty Growth')
    ax_a2.legend(loc='best')
    ax_a2.grid(True)
    fig_a2.tight_layout()
    if save_dir:
        fig_a2.savefig(os.path.join(save_dir, "fig_part_a_uncertainty.png"), dpi=300)

    # =========================================================================
    # PROJECT 4 – PART B: BELIEF-SPACE PLANNING WITH CHANCE CONSTRAINTS
    # (Faithfully executes Part B loop, overwriting state trajectories)
    # =========================================================================
    chanceSigma       = 3.0
    maxPosSigma       = 20.0
    maxRiskThreshold = 15.0

    vMaxNominal = 4.0
    vMaxSafe    = 2.5

    riskValue      = np.zeros((N, numUAV))
    sigmaPosition  = np.zeros((N, numUAV))
    chanceViolated = np.zeros((N, numUAV), dtype=bool)

    MODE_NOMINAL      = 1
    MODE_DEGRADED     = 2
    MODE_CONSERVATIVE = 3
    modeHistory       = np.zeros((N, numUAV), dtype=int)

    for k in range(1, N):
        for u in range(numUAV):
            mu    = xEst[0:2, k - 1, u]
            Sigma = P[0:2, 0:2, k - 1, u]

            sigmaPosition[k, u] = np.sqrt(np.trace(Sigma))
            riskValue[k, u]     = riskFcn(mu[0], mu[1])

            if not gpsAvailable[k]:
                mode = MODE_DEGRADED
            elif sigmaPosition[k, u] > 0.7 * maxPosSigma:
                mode = MODE_CONSERVATIVE
            else:
                mode = MODE_NOMINAL
            modeHistory[k, u] = mode

            goal = tasks[taskAssigned[u]]
            direction = goal - mu
            direction = direction / (np.linalg.norm(direction) + 1e-6)

            if (sigmaPosition[k, u] * chanceSigma > maxPosSigma) or (riskValue[k, u] > maxRiskThreshold):
                chanceViolated[k, u] = True
                # Orthogonal turn
                direction = np.array([-direction[1], direction[0]])
                vCmdMag = vMaxSafe
            else:
                vCmdMag = vMaxNominal

            vCmd = vCmdMag * direction

            # Overwriting Part A dynamics & estimation as in MATLAB
            xTrue[0:2, k, u] = xTrue[0:2, k - 1, u] + dt * xTrue[2:4, k - 1, u]
            xTrue[2:4, k, u] = vCmd
            xTrue[4, k, u]   = np.arctan2(vCmd[1], vCmd[0])
            xTrue[5:7, k, u] = xTrue[5:7, k - 1, u]

            F = np.eye(nx)
            F[0, 2] = dt
            F[1, 3] = dt

            xPred = F @ xEst[:, k - 1, u]
            PPred = F @ P[:, :, k - 1, u] @ F.T + Q

            if gpsAvailable[k]:
                H = np.zeros((2, nx))
                H[0, 0] = 1.0
                H[1, 1] = 1.0

                z = xTrue[0:2, k, u] + cholRgps.T @ np.random.randn(2)
                S_cov = H @ PPred @ H.T + Rgps
                K = PPred @ H.T @ np.linalg.inv(S_cov)

                xEst[:, k, u] = xPred + K @ (z - H @ xPred)
                P[:, :, k, u] = (np.eye(nx) - K @ H) @ PPred
            else:
                xEst[:, k, u] = xPred
                P[:, :, k, u] = PPred

            if mode == MODE_CONSERVATIVE:
                energy[k, u] = energy[k - 1, u] - energyRateSafe
            else:
                energy[k, u] = energy[k - 1, u] - energyRateNominal

            traceP[k, u] = np.trace(P[:, :, k, u])

    # Save Part B figures
    fig_b1, ax_b1 = plt.subplots(figsize=(7, 4.5))
    for u in range(numUAV):
        ax_b1.plot(time, riskValue[:, u], linewidth=2, label=f'UAV {u+1}')
    ax_b1.set_xlabel('Time [s]')
    ax_b1.set_ylabel('Risk Value')
    ax_b1.set_title('PART B: Risk Exposure Over Time')
    ax_b1.legend(loc='best')
    ax_b1.grid(True)
    fig_b1.tight_layout()
    if save_dir:
        fig_b1.savefig(os.path.join(save_dir, "fig_part_b_risk.png"), dpi=300)

    fig_b2, ax_b2 = plt.subplots(figsize=(7, 4.5))
    for u in range(numUAV):
        ax_b2.plot(time, sigmaPosition[:, u], linewidth=2, label=f'UAV {u+1}')
    ax_b2.set_xlabel('Time [s]')
    ax_b2.set_ylabel('Position Std Dev [m]')
    ax_b2.set_title('PART B: Belief Uncertainty (Position)')
    ax_b2.legend(loc='best')
    ax_b2.grid(True)
    fig_b2.tight_layout()
    if save_dir:
        fig_b2.savefig(os.path.join(save_dir, "fig_part_b_sigma_position.png"), dpi=300)

    # =========================================================================
    # PROJECT 4 – PART C: MDP AUTONOMY, FAULT-TOLERANT CONTROL
    # =========================================================================
    numStates  = 8
    numActions = 4
    ACTION = {'NOMINAL': 1, 'DEGRADED': 2, 'CONSERVATIVE': 3, 'EMERGENCY': 4}

    wEnergy = 5.0
    wRisk   = 8.0
    wUnc    = 6.0

    R_table = np.zeros((numStates, numActions))
    for s in range(numStates):
        for a_idx, a_val in enumerate([1, 2, 3, 4]):
            R_table[s, a_idx] = (
                -wEnergy * (a_val == ACTION['EMERGENCY'])
                - wRisk * (a_val == ACTION['NOMINAL'])
                - wUnc * (a_val == ACTION['NOMINAL'])
            )

    actuatorFaultTime = 140.0
    actuatorLoss      = 0.6

    faultDetected  = np.zeros((N, numUAV), dtype=bool)
    thetaHat       = np.ones((2, 1, numUAV))
    gamma          = 0.03
    controlEffort  = np.zeros((N, numUAV))
    taskReassigned = np.zeros((N, numUAV), dtype=bool)

    for k in range(1, N):
        for u in range(numUAV):
            uncHigh = (traceP[k - 1, u] > 120.0)
            gpsOn   = gpsAvailable[k]
            eHigh   = (energy[k - 1, u] > 0.3)

            s_state = int(uncHigh) * 4 + int(gpsOn) * 2 + int(eHigh) * 1  # 0 to 7
            action  = int(np.argmax(R_table[s_state, :])) + 1            # 1 to 4
            modeHistory[k, u] = action

            # Fault detection (reads ground truth velocity xTrue as in MATLAB)
            if time[k] > actuatorFaultTime:
                velErr = np.linalg.norm(xEst[2:4, k, u] - xTrue[2:4, k, u])
                if velErr > 1.0:
                    faultDetected[k, u] = True

            # Adaptive control (reads ground truth velocity xTrue as in MATLAB)
            velError = xEst[2:4, k, u] - xTrue[2:4, k, u]
            thetaHat[:, 0, u] += gamma * velError

            uCtrl = np.diag(thetaHat[:, 0, u]) @ velError
            controlEffort[k, u] = np.linalg.norm(uCtrl)

            if time[k] > actuatorFaultTime:
                uCtrl = actuatorLoss * uCtrl

            # Velocity is modified, but note position xTrue(1:2) is not integrated in Part C
            xTrue[2:4, k, u] += uCtrl

            # Communication-aware task reallocation
            if (not commAvailable[u, k]) and (modeHistory[k, u] == ACTION['CONSERVATIVE']):
                # taskAssign(u) = 0 in MATLAB
                taskReassigned[k, u] = True

            # Energy update
            if modeHistory[k, u] == ACTION['NOMINAL']:
                energy[k, u] = energy[k - 1, u] - energyRateNominal
            elif modeHistory[k, u] == ACTION['DEGRADED']:
                energy[k, u] = energy[k - 1, u] - 0.8 * energyRateNominal
            elif modeHistory[k, u] == ACTION['CONSERVATIVE']:
                energy[k, u] = energy[k - 1, u] - energyRateSafe
            elif modeHistory[k, u] == ACTION['EMERGENCY']:
                energy[k, u] = energy[k - 1, u] - 0.5 * energyRateSafe

    # Part C diagnostic plots
    fig_c1, ax_c1 = plt.subplots(figsize=(8, 4))
    im1 = ax_c1.imshow(modeHistory.T, aspect='auto', extent=[0, T_end, numUAV + 0.5, 0.5], cmap='Set2')
    fig_c1.colorbar(im1, ax=ax_c1, label='Action Mode')
    ax_c1.set_yticks(range(1, numUAV + 1))
    ax_c1.set_xlabel('Time [s]')
    ax_c1.set_ylabel('UAV Index')
    ax_c1.set_title('PART C: MDP-Based Autonomy Modes')
    fig_c1.tight_layout()
    if save_dir:
        fig_c1.savefig(os.path.join(save_dir, "fig_part_c_mdp_modes.png"), dpi=300)

    fig_c2, ax_c2 = plt.subplots(figsize=(8, 4))
    im2 = ax_c2.imshow(faultDetected.T, aspect='auto', extent=[0, T_end, numUAV + 0.5, 0.5], cmap='Reds')
    fig_c2.colorbar(im2, ax=ax_c2, label='Fault Flag')
    ax_c2.set_yticks(range(1, numUAV + 1))
    ax_c2.set_xlabel('Time [s]')
    ax_c2.set_title('PART C: Detected Actuator Faults')
    fig_c2.tight_layout()
    if save_dir:
        fig_c2.savefig(os.path.join(save_dir, "fig_part_c_faults.png"), dpi=300)

    fig_c3, ax_c3 = plt.subplots(figsize=(7, 4.5))
    for u in range(numUAV):
        ax_c3.plot(time, controlEffort[:, u], linewidth=2, label=f'UAV {u+1}')
    ax_c3.set_xlabel('Time [s]')
    ax_c3.set_ylabel('||u||')
    ax_c3.set_title('PART C: Adaptive Control Effort')
    ax_c3.legend(loc='best')
    ax_c3.grid(True)
    fig_c3.tight_layout()
    if save_dir:
        fig_c3.savefig(os.path.join(save_dir, "fig_part_c_control_effort.png"), dpi=300)

    # Summary Table
    summary_df = pd.DataFrame({
        'UAV': list(range(1, numUAV + 1)),
        'FinalEnergy': energy[-1, :],
        'FinalTraceP': traceP[-1, :],
        'FaultCount': np.sum(faultDetected, axis=0),
        'TaskReassignments': np.sum(taskReassigned, axis=0)
    })

    print("\n================ FINAL SUMMARY ================")
    print(summary_df.to_string(index=False))
    print("================================================\n")

    if show_plots:
        plt.show()

    return summary_df

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run Project 4 As-Is Multi-UAV Simulation")
    parser.add_argument("--show", action="store_true", help="Display interactive plots")
    parser.add_argument("--save-dir", type=str, default="output_figures", help="Directory to save figures")
    args = parser.parse_args()

    run_multi_uav(show_plots=args.show, save_dir=args.save_dir)
