# Offline 300 DPI ICAAV-2026 figure exporter reading C++20 swarm_server CSV and OpenMP ablation JSON

import json
import os
import subprocess
import tempfile
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def export_figures(repo_root: str = None, dpi: int = 300) -> str:
    if repo_root is None:
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    out_dir = os.path.join(repo_root, "output_figures")
    os.makedirs(out_dir, exist_ok=True)
    server_bin = os.path.join(repo_root, "build", "swarm_server")
    if not os.path.exists(server_bin):
        raise FileNotFoundError(f"Build C++20 target first: {server_bin}")

    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([server_bin, "--export-ablation", tmp], check=True)
        df = pd.read_csv(os.path.join(tmp, "contested_sortie.csv"))
        df_stat = pd.read_csv(os.path.join(tmp, "static_master_sortie.csv"))
        df_solo = pd.read_csv(os.path.join(tmp, "solo_eskf_sortie.csv"))
        with open(os.path.join(tmp, "ablation_results.json"), "r", encoding="utf-8") as f:
            abl = json.load(f)["variants"]

    colors = ["#0284c7", "#10b981", "#f59e0b", "#ef4444", "#8b5cf6"]

    # Fig 1: 3D Non-Holonomic Fixed-Wing Swarm Trajectories
    fig = plt.figure(figsize=(8.0, 5.4))
    ax = fig.add_subplot(111, projection="3d")
    for u in range(5):
        sub = df[df["uav_id"] == u]
        ax.plot(sub["true_e"], sub["true_n"], sub["true_alt"], color=colors[u], lw=1.8, label=f"UAV-{u+1}")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Altitude [m]")
    ax.set_title("3D Non-Holonomic Fixed-Wing Swarm Trajectories (Contested Corridor)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig01_3d_trajectories.png"), dpi=dpi)
    plt.close(fig)

    # Fig 2: Cooperative WCI+NHC 15-State ESKF vs Solo ESKF RMSE
    def mean_rmse_series(frame):
        err = np.sqrt((frame["true_n"] - frame["est_n"]) ** 2 + (frame["true_e"] - frame["est_e"]) ** 2 + (frame["true_alt"] - frame["est_alt"]) ** 2)
        return frame.assign(err=err).groupby("time_s")["err"].mean()

    r_coop = mean_rmse_series(df)
    r_solo = mean_rmse_series(df_solo)
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    ax.plot(r_coop.index, r_coop.values, color="#0284c7", lw=2.2, label="15-State Quaternion ESKF + NHC + Master WCI")
    ax.plot(r_solo.index, r_solo.values, color="#ef4444", ls="--", lw=1.8, label="Solo ESKF (No NHC / No WCI)")
    ax.axvspan(50.0, 145.0, color="#fbbf24", alpha=0.14, label="GNSS Blackout Window")
    ax.set_yscale("log")
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("Fleet Mean 3D Position RMSE [m] (log)")
    ax.set_title("GNSS-Denied Estimation: Cooperative WCI + NHC vs Solo ESKF")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig02_cooperative_vs_solo_rmse.png"), dpi=dpi)
    plt.close(fig)

    # Fig 3: 3-Sigma Positional Uncertainty Growth
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    for u in range(5):
        sub = df[df["uav_id"] == u]
        ax.plot(sub["time_s"], sub["pos_sigma3_m"], color=colors[u], lw=1.7, label=f"UAV-{u+1}")
    ax.axvspan(50.0, 145.0, color="#fbbf24", alpha=0.14, label="GNSS Denied")
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("3-Sigma Position Bound [m]")
    ax.set_title("15-State Quaternion ESKF 3-Sigma Uncertainty with Master WCI Updates")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=8, ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig03_3sigma_uncertainty_growth.png"), dpi=dpi)
    plt.close(fig)

    # Fig 4: Ideal-vs-Actual Cross-Track Trajectory Deviation (XTE)
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    for u in range(5):
        sub = df[df["uav_id"] == u]
        ax.plot(sub["time_s"], sub["xte_m"], color=colors[u], lw=1.6, label=f"UAV-{u+1}")
    ax.axhline(48.0, color="#ef4444", ls="--", lw=1.4, label="Alert Threshold (48 m)")
    ax.axvline(120.0, color="#991b1b", ls=":", lw=1.4, label="UAV-2 Actuator LoE")
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("Cross-Track Error XTE [m]")
    ax.set_title("Trajectory Deviation from Ideal Planned Corridor Under Faults and DR")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=8, ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig04_xte_trajectory_deviation.png"), dpi=dpi)
    plt.close(fig)

    # Fig 5: Observable L1 Adaptive FTC Effectiveness Tracking on UAV-2
    u2 = df[df["uav_id"] == 1]
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    ax.plot(u2["time_s"], u2["eta_roll_hat"] * 100.0, color="#0284c7", lw=2.2, label="Estimated Roll Effectiveness η̂_roll [%]")
    ax.plot(u2["time_s"], u2["eta_thrust_hat"] * 100.0, color="#10b981", lw=2.2, label="Estimated Thrust Effectiveness η̂_thrust [%]")
    ax.axvline(120.0, color="#ef4444", ls="--", lw=1.6, label="Fault Injected (55% Roll / 60% Thrust + Icing)")
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("Estimated Actuator Health [%]")
    ax.set_title("L1 Adaptive FTC Parameter Adaptation (Strictly Observable, Zero xTrue)")
    ax.set_ylim(30, 108)
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig05_l1_adaptive_ftc_health.png"), dpi=dpi)
    plt.close(fig)

    # Fig 6: Time-Based Rotating Master-Slave vs Static Master Battery SOC Balance
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.4, 4.2), sharey=True)
    for u in range(5):
        s_rot = df[df["uav_id"] == u]
        s_sta = df_stat[df_stat["uav_id"] == u]
        ax1.plot(s_rot["time_s"], s_rot["soc"] * 100.0, color=colors[u], lw=1.8, label=f"UAV-{u+1}")
        ax2.plot(s_sta["time_s"], s_sta["soc"] * 100.0, color=colors[u], lw=1.8, label=f"UAV-{u+1}")
    ax1.set_title("Time-Based Rotating Master-Slave")
    ax2.set_title("Static Master-Slave (UAV-1 Always Master)")
    ax1.set_xlabel("Time [s]")
    ax2.set_xlabel("Time [s]")
    ax1.set_ylabel("Battery SOC [%]")
    ax1.grid(True, ls=":", alpha=0.5)
    ax2.grid(True, ls=":", alpha=0.5)
    ax1.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig06_rotating_vs_static_master_soc.png"), dpi=dpi)
    plt.close(fig)

    # Fig 7: 1-RC Electro-Thermal Terminal Voltage Sag
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    for u in range(5):
        sub = df[df["uav_id"] == u]
        ax.plot(sub["time_s"], sub["v_term"], color=colors[u], lw=1.7, label=f"UAV-{u+1}")
    ax.axhline(18.8, color="#ef4444", ls="--", lw=1.6, label="Brownout Cutoff (18.8 V)")
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("Pack Terminal Voltage [V]")
    ax.set_title("6S 16Ah 1-RC Electro-Thermal Pack Voltage Under Flight & Master RF Load")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=8, ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig07_1rc_battery_voltage_sag.png"), dpi=dpi)
    plt.close(fig)

    # Fig 8: Non-Degenerate Bellman Value-Iteration MDP Mode Swimlanes
    piv = df.pivot(index="uav_id", columns="time_s", values="mode")
    fig, ax = plt.subplots(figsize=(8.2, 4.0))
    im = ax.imshow(piv.values, aspect="auto", cmap="viridis", vmin=0, vmax=3, extent=[piv.columns.min(), piv.columns.max(), 5.5, 0.5])
    cbar = fig.colorbar(im, ax=ax, ticks=[0, 1, 2, 3])
    cbar.ax.set_yticklabels(["NOMINAL", "DEGRADED_COOP", "CONSERVATIVE", "EMERGENCY_RTB"])
    ax.set_yticks([1, 2, 3, 4, 5])
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("UAV ID")
    ax.set_title("Discounted Bellman Value-Iteration Belief-MDP Operational Modes")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig08_bellman_mdp_modes.png"), dpi=dpi)
    plt.close(fig)

    # Fig 9: Rotating Master Timeline & Fiedler Algebraic Connectivity lambda2
    master_ts = df[df["is_master"] == 1][["time_s", "uav_id", "fiedler_l2"]].drop_duplicates("time_s")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.8, 5.2), sharex=True)
    ax1.step(master_ts["time_s"], master_ts["uav_id"] + 1, where="post", color="#f59e0b", lw=2.2)
    ax1.set_yticks([1, 2, 3, 4, 5])
    ax1.set_ylabel("Active Master UAV")
    ax1.set_title("TDMA Rotating Master-Slave Authority & Swarm Algebraic Connectivity (λ2)")
    ax1.grid(True, ls=":", alpha=0.5)
    ax2.plot(master_ts["time_s"], master_ts["fiedler_l2"], color="#0284c7", lw=2.0)
    ax2.axvspan(90.0, 105.0, color="#ef4444", alpha=0.18, label="UAV-3 Disconnected Window")
    ax2.set_xlabel("Mission Time [s]")
    ax2.set_ylabel("Fiedler λ2(L)")
    ax2.grid(True, ls=":", alpha=0.5)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig09_master_epoch_and_fiedler.png"), dpi=dpi)
    plt.close(fig)

    # Fig 10: 15-State ESKF NEES Consistency
    nees_mean = df.groupby("time_s")["nees"].mean()
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    ax.plot(nees_mean.index, np.minimum(nees_mean.values, 25.0), color="#0284c7", lw=1.9, label="Fleet Mean 3D Position NEES")
    ax.axhline(7.815, color="#ef4444", ls="--", lw=1.6, label="95% Chi-Squared Upper Bound (7.815)")
    ax.axvspan(155.0, 170.0, color="#ef4444", alpha=0.14, label="GNSS Spoofing Attack (Rejected)")
    ax.set_xlabel("Mission Time [s]")
    ax.set_ylabel("NEES")
    ax.set_title("15-State Quaternion ESKF Consistency Under Blackout & Spoofing")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig10_eskf_nees_consistency.png"), dpi=dpi)
    plt.close(fig)

    # Fig 11: 100-Run OpenMP Ablation — Position RMSE & Mission Completion
    short_labels = ["Full Stack", "Solo ESKF\n(No WCI/NHC)", "Static Master\n(No Rotation)", "Nominal PID\n(No L1 FTC)"]
    rmses = [v["rmse_m"] for v in abl]
    comps = [v["completion_pct"] for v in abl]
    x = np.arange(len(short_labels))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.4, 4.4))
    ax1.bar(x, rmses, color=["#10b981", "#ef4444", "#f59e0b", "#0284c7"])
    ax1.set_xticks(x)
    ax1.set_xticklabels(short_labels, fontsize=8)
    ax1.set_ylabel("Mean 3D RMSE [m]")
    ax1.set_title("100-Run OpenMP Ablation: Estimation RMSE")
    ax1.grid(True, axis="y", ls=":", alpha=0.5)
    ax2.bar(x, comps, color=["#10b981", "#ef4444", "#f59e0b", "#0284c7"])
    ax2.set_xticks(x)
    ax2.set_xticklabels(short_labels, fontsize=8)
    ax2.set_ylabel("ISR Task Completion [%]")
    ax2.set_title("100-Run OpenMP Ablation: Mission Completion")
    ax2.grid(True, axis="y", ls=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig11_icaav_ablation_rmse_completion.png"), dpi=dpi)
    plt.close(fig)

    # Fig 12: 100-Run OpenMP Ablation — SOC Spread & CBBA Bandwidth Savings
    spreads = [v["soc_spread_pct"] for v in abl]
    savings = [v["cbba_savings_pct"] for v in abl]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.4, 4.4))
    ax1.bar(x, spreads, color=["#10b981", "#0284c7", "#ef4444", "#f59e0b"])
    ax1.set_xticks(x)
    ax1.set_xticklabels(short_labels, fontsize=8)
    ax1.set_ylabel("Fleet Max-Min SOC Spread [%]")
    ax1.set_title("Battery Load Balance (Lower Is Better)")
    ax1.grid(True, axis="y", ls=":", alpha=0.5)
    ax2.bar(x, savings, color="#0284c7")
    ax2.set_xticks(x)
    ax2.set_xticklabels(short_labels, fontsize=8)
    ax2.set_ylabel("Event-Driven CBBA Packet Savings [%]")
    ax2.set_title("Radio Bandwidth Reduction vs Full Broadcast")
    ax2.set_ylim(0, 105)
    ax2.grid(True, axis="y", ls=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig12_icaav_ablation_soc_cbba.png"), dpi=dpi)
    plt.close(fig)

    print("Ablation summary:")
    for v in abl:
        print(f"  {v['label']}: RMSE={v['rmse_m']:.1f}m | NEES_OK={v['nees_ok_pct']:.1f}% | Comp={v['completion_pct']:.1f}% | Coll={v['collision_pct']:.1f}% | SOCSpread={v['soc_spread_pct']:.2f}% | CBBA_Saved={v['cbba_savings_pct']:.1f}%")
    return out_dir


if __name__ == "__main__":
    out = export_figures()
    print(f"Exported 12 300 DPI ICAAV-2026 figures to {out}")
