// Unified closed-loop multi-UAV swarm engine, Alert Engine, OpenMP ablation, and JSON/CSV export

#include "swarm_engine.hpp"
#include "aero_plant.hpp"
#include "eskf_15state.hpp"
#include "l1_adaptive_ftc.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <omp.h>
#include <sstream>

namespace swarm {

namespace {

const char* mode_to_str(MdpMode m) {
    switch (m) {
        case MdpMode::NOMINAL: return "NOMINAL";
        case MdpMode::DEGRADED_COOP: return "DEGRADED_COOP";
        case MdpMode::CONSERVATIVE_RISK: return "CONSERVATIVE_RISK";
        case MdpMode::EMERGENCY_RTB: return "EMERGENCY_RTB";
    }
    return "NOMINAL";
}

const char* sev_to_str(AlertSeverity s) {
    switch (s) {
        case AlertSeverity::INFO: return "INFO";
        case AlertSeverity::CAUTION: return "CAUTION";
        case AlertSeverity::CRITICAL: return "CRITICAL";
    }
    return "INFO";
}

std::string json_escape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        if (c == '"' || c == '\\') out.push_back('\\');
        if (c == '\n') out += "\\n";
        else out.push_back(c);
    }
    return out;
}

}  // namespace

SwarmEngine::SwarmEngine() {
    mdp_ = solve_bellman_mdp();
    reset(SimOptions{});
}

void SwarmEngine::reset(const SimOptions& opts) {
    std::lock_guard<std::mutex> lock(mtx_);
    opts_ = opts;
    opts_.n_uavs = std::max(1, std::min(kMaxUavs, opts.n_uavs));
    rng_.seed(opts_.seed);
    sim_time_s_ = 0.0;
    step_count_ = 0;
    alert_seq_ = 0;
    min_inter_uav_sep_m_ = 9999.0;
    max_trace_p_ = 0.0;
    rmse_accum_ = 0.0;
    rmse_samples_ = 0;
    nees_total_ = 0;
    nees_ok_ = 0;
    paused_ = false;
    alerts_.clear();
    csv_rows_.clear();
    csv_rows_.push_back("time_s,uav_id,is_master,epoch_id,true_n,true_e,true_alt,est_n,est_e,est_alt,xte_m,pos_sigma3_m,nees,soc,v_term,eta_roll_hat,eta_thrust_hat,dr_active,connected,mode,fiedler_l2");

    // Default contested 3000x3000 m Gaussian risk field
    rcfg_ = RiskFieldConfig{};
    rcfg_.peaks = {
        {420.0, 320.0, 6.2, 260.0},
        {-480.0, -340.0, 5.4, 240.0},
        {-200.0, 580.0, 4.8, 220.0},
    };
    ambient_wind_ = {3.5, -2.0, 0.0};
    comms_ = CommsChannelState{};
    formation_ = FormationType::V_WEDGE;

    // 8 distributed 3D ISR orbit targets across contested corridor
    tasks_.clear();
    const std::array<Vec3, 8> tpos = {
        Vec3{340.0, 320.0, -145.0},
        Vec3{-310.0, 360.0, -165.0},
        Vec3{420.0, -290.0, -135.0},
        Vec3{-350.0, -320.0, -155.0},
        Vec3{80.0, 560.0, -175.0},
        Vec3{590.0, 90.0, -140.0},
        Vec3{-90.0, -560.0, -160.0},
        Vec3{-560.0, 60.0, -150.0},
    };
    for (int i = 0; i < kMaxTasks; ++i) {
        IsrTask t{};
        t.id = i;
        t.pos_ned = tpos[i];
        t.loiter_radius_m = 68.0;
        t.dwell_required_s = 8.0;
        t.priority = (i < 4) ? 1.2 : 0.9;
        tasks_.push_back(t);
    }

    // Initialize fixed-wing fleet staggered cleanly in 3D NED with balanced initial SOC
    ms_ = MasterSlaveState{};
    ms_.active_master = 0;
    ms_.next_master = opts_.n_uavs > 1 ? 1 : 0;
    ms_.epoch_id = 1;
    ms_.rotation_enabled = opts_.enable_master_rotation;

    for (int u = 0; u < opts_.n_uavs; ++u) {
        UavTrueState st{};
        st.pos_ned = {-620.0 + 95.0 * u, -540.0 + 185.0 * u, -(105.0 + 26.0 * u)};
        double yaw0 = 0.65;
        st.q_nb = Quat::from_euler(0.0, 0.0, yaw0);
        st.vel_ned = Vec3{params_.v_cruise_ms * std::cos(yaw0), params_.v_cruise_ms * std::sin(yaw0), 0.0} + ambient_wind_;
        st.soc = 0.92;
        st.true_ba = {0.035 + 0.005 * u, -0.025, 0.04};
        st.true_bg = {0.0035, -0.0028, 0.0042};
        fleet_[u] = st;

        init_eskf(beliefs_[u], st.pos_ned + Vec3{3.0, -2.5, 0.0}, st.vel_ned, st.q_nb);
        init_l1_adaptive(l1_[u], params_.v_cruise_ms);
        modes_[u] = MdpMode::NOMINAL;
        leg_starts_[u] = st.pos_ned;
        prev_dr_[u] = false;
        prev_l1_fault_[u] = false;
        prev_connected_[u] = true;
        prev_xte_high_[u] = false;
        forced_disconnect_[u] = false;
        ms_.connected_to_swarm[u] = true;
        trails_[u].clear();
        trails_[u].push_back(st.pos_ned);
    }

    // Initial Event-Driven CBBA task allocation
    std::array<double, kMaxUavs> socs{};
    std::array<bool, kMaxUavs> alive{};
    for (int u = 0; u < opts_.n_uavs; ++u) { socs[u] = fleet_[u].soc; alive[u] = true; }
    std::string dummy;
    step_event_driven_cbba(tasks_, ms_, beliefs_, l1_, modes_, socs, alive, opts_.n_uavs, true, dummy);
    push_alert(AlertSeverity::INFO, "MISSION_INIT", -1,
               "Swarm Mission Initialized", "5 fixed-wing UAVs airborne. Master: UAV-1 (Epoch 1).");
}

void SwarmEngine::push_alert(AlertSeverity sev, const std::string& code, int uav_id,
                             const std::string& title, const std::string& detail) {
    TacticalAlert a{};
    a.seq = ++alert_seq_;
    a.sim_time_s = sim_time_s_;
    a.severity = sev;
    a.code = code;
    a.uav_id = uav_id;
    a.title = title;
    a.detail = detail;
    alerts_.push_front(a);
    while (static_cast<int>(alerts_.size()) > kMaxAlerts) alerts_.pop_back();
}

void SwarmEngine::step() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (paused_) return;

    const double dt = opts_.dt;
    sim_time_s_ += dt;
    step_count_ += 1;
    std::normal_distribution<double> n01(0.0, 1.0);

    // 1. Update RF multi-hop topology, Gilbert-Elliott burst loss, and Fiedler lambda2
    update_comms_topology(ms_, comms_, fleet_, rcfg_, opts_.n_uavs, rng_);
    for (int u = 0; u < opts_.n_uavs; ++u) {
        if (forced_disconnect_[u]) {
            ms_.connected_to_swarm[u] = false;
            ms_.missed_heartbeats[u] = 10;
            for (int j = 0; j < opts_.n_uavs; ++j) {
                ms_.link_hop_matrix[u][j] = 0;
                ms_.link_hop_matrix[j][u] = 0;
            }
        }
        // Alert immediately when any UAV disconnects or reconnects to the swarm
        if (fleet_[u].alive && prev_connected_[u] && !ms_.connected_to_swarm[u]) {
            push_alert(AlertSeverity::CRITICAL, "UAV_DISCONNECTED", u,
                       "UAV-" + std::to_string(u + 1) + " Disconnected from Swarm",
                       "RF/TDMA heartbeat lost; operating autonomously on onboard 15-state ESKF.");
        } else if (fleet_[u].alive && !prev_connected_[u] && ms_.connected_to_swarm[u]) {
            push_alert(AlertSeverity::INFO, "UAV_RECONNECTED", u,
                       "UAV-" + std::to_string(u + 1) + " Reconnected to Swarm",
                       "RF link restored; resuming Master WCI fusion and CBBA sync.");
        }
        prev_connected_[u] = ms_.connected_to_swarm[u];
    }

    // 2. Sample sensors and run 15-state Quaternion ESKF prediction + GNSS/spoof + NHC
    std::array<SensorPacket, kMaxUavs> packets{};
    std::array<double, kMaxUavs> socs{};
    std::array<bool, kMaxUavs> alive{};
    for (int u = 0; u < opts_.n_uavs; ++u) {
        alive[u] = fleet_[u].alive;
        socs[u] = fleet_[u].soc;
        if (!alive[u]) continue;

        packets[u] = sample_sensors(fleet_[u], params_, rcfg_, ambient_wind_, sim_time_s_, dt, rng_);
        eskf_predict(beliefs_[u], packets[u], dt);
        eskf_update_gnss(beliefs_[u], packets[u], opts_.enable_spoof_gate);
        eskf_update_nhc_airdata(beliefs_[u], packets[u], opts_.enable_nhc);

        if (beliefs_[u].spoof_rejected_step && (step_count_ % 10 == 0 || beliefs_[u].spoof_reject_count == 1)) {
            push_alert(AlertSeverity::CRITICAL, "GNSS_SPOOF_REJECTED", u,
                       "GNSS Spoofing Rejected (UAV-" + std::to_string(u + 1) + ")",
                       "Mahalanobis innovation exceeded chi2 gate; switched to NHC + WCI Dead Reckoning.");
        }
        if (!prev_dr_[u] && beliefs_[u].dr_active) {
            push_alert(AlertSeverity::CAUTION, "DR_ACTIVATED", u,
                       "Dead Reckoning Activated (UAV-" + std::to_string(u + 1) + ")",
                       "GNSS unavailable; 15-state Quaternion ESKF + wind-aware NHC engaged.");
        } else if (prev_dr_[u] && !beliefs_[u].dr_active) {
            push_alert(AlertSeverity::INFO, "DR_CLEARED", u,
                       "GNSS Lock Restored (UAV-" + std::to_string(u + 1) + ")",
                       "Dead reckoning cleared; full GNSS + ESKF fusion active.");
        }
        prev_dr_[u] = beliefs_[u].dr_active;
    }

    // 3. Cooperative Master-to-Slave & multi-peer UWB Weighted Covariance Intersection (WCI)
    if (opts_.enable_wci && (step_count_ % 2 == 0)) {
        for (int u = 0; u < opts_.n_uavs; ++u) {
            if (!alive[u] || !ms_.connected_to_swarm[u]) continue;
            for (int j = 0; j < opts_.n_uavs; ++j) {
                if (j == u || !alive[j] || ms_.link_hop_matrix[u][j] == 0) continue;
                bool is_master_anchor = (j == ms_.active_master);
                if (!is_master_anchor && !beliefs_[u].dr_active) continue;
                double true_range = (fleet_[u].pos_ned - fleet_[j].pos_ned).norm();
                double meas_range = std::max(1.0, true_range + 1.2 * n01(rng_));
                Mat<3, 3> peer_P{};
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        peer_P(r, c) = beliefs_[j].P(r, c);
                double delay = comms_.packet_delay_s[u][j] + std::abs(fleet_[u].clock_skew_s);
                eskf_update_wci_coop(beliefs_[u], beliefs_[j].p, beliefs_[j].v, peer_P, meas_range, delay);
            }
        }
    }

    // 4. Time-Based Rotating Master-Slave with preemptive health failover
    int old_m = ms_.active_master;
    std::string ms_reason;
    std::string ms_evt = step_rotating_master_slave(
        ms_, beliefs_, l1_, modes_, socs, alive, opts_.n_uavs, dt, old_m, ms_reason
    );
    if (ms_evt == "MASTER_ROTATED") {
        push_alert(AlertSeverity::INFO, "MASTER_ROTATED", ms_.active_master,
                   "Master Rotated: UAV-" + std::to_string(old_m + 1) + " -> UAV-" + std::to_string(ms_.active_master + 1),
                   "Epoch " + std::to_string(ms_.epoch_id) + " | " + ms_reason);
    } else if (ms_evt == "MASTER_FAILOVER") {
        push_alert(AlertSeverity::CRITICAL, "MASTER_FAILOVER", ms_.active_master,
                   "Master Failover: UAV-" + std::to_string(old_m + 1) + " -> UAV-" + std::to_string(ms_.active_master + 1),
                   "Epoch " + std::to_string(ms_.epoch_id) + " | " + ms_reason);
    }

    // 5. Bellman Value-Iteration MDP + Event-Driven CBBA + Guidance/CBF + L1 FTC + 6-DOF Plant
    bool trigger_cbba = false;
    for (int u = 0; u < opts_.n_uavs; ++u) {
        if (!alive[u]) continue;
        MdpMode prev_m = modes_[u];
        modes_[u] = query_bellman_mode(mdp_, beliefs_[u].pos_sigma_3, !beliefs_[u].dr_active, packets[u].soc_est, l1_[u].fault_flag);
        if (modes_[u] == MdpMode::EMERGENCY_RTB && prev_m != MdpMode::EMERGENCY_RTB) {
            trigger_cbba = true;
            push_alert(AlertSeverity::CRITICAL, "MDP_EMERGENCY_RTB", u,
                       "UAV-" + std::to_string(u + 1) + " Entered Emergency RTB",
                       "Bellman MDP triggered RTB due to low battery reserve / fault posture.");
        }
    }

    if (opts_.enable_ed_cbba) {
        std::string handoff;
        if (step_event_driven_cbba(tasks_, ms_, beliefs_, l1_, modes_, socs, alive, opts_.n_uavs, trigger_cbba, handoff) && !handoff.empty()) {
            push_alert(AlertSeverity::CAUTION, "CBBA_HANDOFF", -1, "CBBA Task Reallocated", handoff);
        }
    }

    // Compute per-UAV guidance, CBF-QP, L1 Adaptive FTC, and integrate 6-DOF fixed-wing plant
    std::array<ControlCommand, kMaxUavs> cmds{};
    for (int u = 0; u < opts_.n_uavs; ++u) {
        if (!alive[u]) continue;

        // Locate assigned ISR task or RTB home base
        Vec3 goal{-950.0, -880.0, -140.0};
        bool loiter = false;
        double loiter_r = 72.0;
        int active_task_idx = -1;
        if (modes_[u] != MdpMode::EMERGENCY_RTB) {
            for (int ti = 0; ti < static_cast<int>(tasks_.size()); ++ti) {
                if (!tasks_[ti].completed && tasks_[ti].assigned_uav == u) {
                    active_task_idx = ti;
                    goal = tasks_[ti].pos_ned;
                    loiter_r = tasks_[ti].loiter_radius_m;
                    double d_xy = std::hypot(beliefs_[u].p.x - goal.x, beliefs_[u].p.y - goal.y);
                    if (d_xy < loiter_r * 1.9) {
                        loiter = true;
                        tasks_[ti].dwell_accum_s += dt;
                        if (tasks_[ti].dwell_accum_s >= tasks_[ti].dwell_required_s) {
                            tasks_[ti].completed = true;
                            leg_starts_[u] = beliefs_[u].p;
                            std::string msg;
                            if (opts_.enable_ed_cbba) {
                                step_event_driven_cbba(tasks_, ms_, beliefs_, l1_, modes_, socs, alive, opts_.n_uavs, true, msg);
                            }
                        }
                    }
                    break;
                }
            }
            // If all tasks assigned to u are complete, hold staggered formation around active Master
            if (active_task_idx < 0 && ms_.active_master >= 0 && alive[ms_.active_master] && u != ms_.active_master) {
                Vec3 m_eul = beliefs_[ms_.active_master].q.to_euler();
                Vec3 off = compute_formation_offset(formation_, u, opts_.n_uavs, m_eul.z);
                goal = beliefs_[ms_.active_master].p + off;
                goal.z = clampd(goal.z, -240.0, -100.0);
            }
        }

        double bank_c = 0.0, climb_c = 0.0, va_c = params_.v_cruise_ms;
        compute_guidance_and_cbf(u, beliefs_, alive, opts_.n_uavs, goal, loiter, loiter_r, modes_[u],
                                 params_, rcfg_, opts_.enable_cbf, bank_c, climb_c, va_c);

        cmds[u] = step_l1_adaptive_ftc(l1_[u], beliefs_[u], packets[u], params_,
                                       bank_c, climb_c, va_c, opts_.enable_l1_ftc, sim_time_s_, dt);

        if (!prev_l1_fault_[u] && l1_[u].fault_flag) {
            push_alert(AlertSeverity::CRITICAL, "L1_FTC_ACTIVE", u,
                       "Actuator Fault Detected (UAV-" + std::to_string(u + 1) + ")",
                       "L1 Adaptive FTC active; compensating effectiveness drop without ground truth.");
        }
        prev_l1_fault_[u] = l1_[u].fault_flag;

        // Trajectory deviation & 15s drift forecast
        devs_[u] = compute_deviation_and_drift(beliefs_[u], leg_starts_[u], goal, loiter, loiter_r, rcfg_);
        bool xte_high = devs_[u].xte_m > 48.0;
        if (xte_high && !prev_xte_high_[u]) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << "XTE = " << devs_[u].xte_m << " m | Alt Dev = " << devs_[u].alt_dev_m << " m";
            push_alert(AlertSeverity::CAUTION, "TRAJECTORY_DEVIATION", u,
                       "UAV-" + std::to_string(u + 1) + " Deviating from Ideal Path", oss.str());
        }
        prev_xte_high_[u] = xte_high;
    }

    // Advance 6-DOF plant and update diagnostics
    for (int u = 0; u < opts_.n_uavs; ++u) {
        if (!alive[u]) continue;
        double wake_m = compute_wake_vortex_disturbance(u, fleet_, opts_.n_uavs);
        step_fixed_wing_plant(fleet_[u], cmds[u], params_, ambient_wind_, wake_m, (u == ms_.active_master), dt, rng_);
        if (beliefs_[u].dr_active) fleet_[u].clock_skew_s += 0.00015 * dt;
        else fleet_[u].clock_skew_s *= 0.9;

        eskf_update_diagnostics(beliefs_[u], fleet_[u]);
        double tr = beliefs_[u].P.trace();
        if (tr > max_trace_p_) max_trace_p_ = tr;
        rmse_accum_ += (fleet_[u].pos_ned - beliefs_[u].p).norm2();
        rmse_samples_ += 1;
        nees_total_ += 1;
        if (beliefs_[u].nees <= 9.35) nees_ok_ += 1;

        if (step_count_ % 5 == 0) {
            trails_[u].push_back(fleet_[u].pos_ned);
            while (static_cast<int>(trails_[u].size()) > kMaxTrail) trails_[u].pop_front();
        }
    }

    // Track inter-UAV minimum 3D separation
    for (int i = 0; i < opts_.n_uavs; ++i) {
        if (!fleet_[i].alive) continue;
        for (int j = i + 1; j < opts_.n_uavs; ++j) {
            if (!fleet_[j].alive) continue;
            double d = (fleet_[i].pos_ned - fleet_[j].pos_ned).norm();
            if (d < min_inter_uav_sep_m_) min_inter_uav_sep_m_ = d;
        }
    }

    // Record 1 Hz CSV telemetry rows for offline ICAAV plotting
    if (step_count_ % 10 == 0 && csv_rows_.size() < 4000) {
        for (int u = 0; u < opts_.n_uavs; ++u) {
            std::ostringstream r;
            r << std::fixed << std::setprecision(3)
              << sim_time_s_ << "," << u << "," << (u == ms_.active_master ? 1 : 0) << "," << ms_.epoch_id << ","
              << fleet_[u].pos_ned.x << "," << fleet_[u].pos_ned.y << "," << -fleet_[u].pos_ned.z << ","
              << beliefs_[u].p.x << "," << beliefs_[u].p.y << "," << -beliefs_[u].p.z << ","
              << devs_[u].xte_m << "," << beliefs_[u].pos_sigma_3 << "," << beliefs_[u].nees << ","
              << fleet_[u].soc << "," << fleet_[u].v_term_v << ","
              << l1_[u].omega_roll_hat << "," << l1_[u].omega_thrust_hat << ","
              << (beliefs_[u].dr_active ? 1 : 0) << "," << (ms_.connected_to_swarm[u] ? 1 : 0) << ","
              << static_cast<int>(modes_[u]) << "," << ms_.fiedler_lambda2;
            csv_rows_.push_back(r.str());
        }
    }
}

void SwarmEngine::apply_preset_scenario(const std::string& scenario_name) {
    if (scenario_name == "gnss_blackout") {
        set_ew_jamming(true, rcfg_.jam_north, rcfg_.jam_east, rcfg_.jam_radius_m);
    } else if (scenario_name == "gnss_spoof") {
        set_ew_jamming(false, rcfg_.jam_north, rcfg_.jam_east, 120.0);
        set_gnss_spoofing(true);
    } else if (scenario_name == "actuator_fault") {
        inject_actuator_fault(1, 0.55, 0.60, 0.09);
        inject_icing(1, 0.65);
    } else if (scenario_name == "master_attrition") {
        int m = ms_.active_master;
        kill_uav(m);
    } else if (scenario_name == "disconnect_uav") {
        disconnect_uav_comms(2, true);
    } else if (scenario_name == "nominal") {
        reset(SimOptions{});
    }
}

void SwarmEngine::inject_actuator_fault(int uav_id, double eta_roll, double eta_thrust, double roll_bias_rad) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (uav_id < 0 || uav_id >= opts_.n_uavs) return;
    fleet_[uav_id].eta_roll = clampd(eta_roll, 0.25, 1.0);
    fleet_[uav_id].eta_thrust = clampd(eta_thrust, 0.30, 1.0);
    fleet_[uav_id].roll_bias_rad = clampd(roll_bias_rad, -0.25, 0.25);
    push_alert(AlertSeverity::CAUTION, "FAULT_INJECTED", uav_id,
               "Actuator LoE Injected on UAV-" + std::to_string(uav_id + 1),
               "Roll eta=" + std::to_string(int(eta_roll * 100)) + "% | Thrust eta=" + std::to_string(int(eta_thrust * 100)) + "%.");
}

void SwarmEngine::inject_icing(int uav_id, double icing_level) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (uav_id < 0 || uav_id >= opts_.n_uavs) return;
    fleet_[uav_id].icing_level = clampd(icing_level, 0.0, 1.0);
}

void SwarmEngine::set_ew_jamming(bool global_blackout, double jam_n, double jam_e, double jam_r) {
    std::lock_guard<std::mutex> lock(mtx_);
    rcfg_.global_gnss_blackout = global_blackout;
    rcfg_.jam_north = jam_n;
    rcfg_.jam_east = jam_e;
    rcfg_.jam_radius_m = clampd(jam_r, 80.0, 900.0);
}

void SwarmEngine::set_gnss_spoofing(bool active) {
    std::lock_guard<std::mutex> lock(mtx_);
    rcfg_.gnss_spoof_active = active;
    rcfg_.spoof_start_s = active ? sim_time_s_ : -1.0;
}

void SwarmEngine::force_master_rotate() {
    std::lock_guard<std::mutex> lock(mtx_);
    ms_.epoch_elapsed_s = ms_.epoch_duration_s + 1.0;
}

void SwarmEngine::kill_uav(int uav_id) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (uav_id < 0 || uav_id >= opts_.n_uavs || !fleet_[uav_id].alive) return;
    fleet_[uav_id].alive = false;
    ms_.connected_to_swarm[uav_id] = false;
    push_alert(AlertSeverity::CRITICAL, "UAV_ATTRITION", uav_id,
               "UAV-" + std::to_string(uav_id + 1) + " Lost (Attrition)",
               "Swarm morphing formation and triggering CBBA task takeover.");
}

void SwarmEngine::disconnect_uav_comms(int uav_id, bool forced_disconnect) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (uav_id < 0 || uav_id >= opts_.n_uavs) return;
    forced_disconnect_[uav_id] = forced_disconnect;
}

void SwarmEngine::set_wind(double wn, double we, double wd) {
    std::lock_guard<std::mutex> lock(mtx_);
    ambient_wind_ = {clampd(wn, -14.0, 14.0), clampd(we, -14.0, 14.0), clampd(wd, -3.0, 3.0)};
}

void SwarmEngine::set_formation(FormationType fmt) {
    std::lock_guard<std::mutex> lock(mtx_);
    formation_ = fmt;
}

void SwarmEngine::add_dynamic_task(double north, double east, double alt_m) {
    std::lock_guard<std::mutex> lock(mtx_);
    IsrTask t{};
    t.id = static_cast<int>(tasks_.size());
    t.pos_ned = {clampd(north, -1400.0, 1400.0), clampd(east, -1400.0, 1400.0), -clampd(alt_m, 80.0, 260.0)};
    t.loiter_radius_m = 70.0;
    t.dwell_required_s = 12.0;
    t.priority = 1.4;
    tasks_.push_back(t);
    push_alert(AlertSeverity::INFO, "TASK_ADDED", -1,
               "New Dynamic ISR Target #" + std::to_string(t.id + 1),
               "Queued for Event-Driven CBBA auction.");
}

void SwarmEngine::set_paused(bool paused) { std::lock_guard<std::mutex> lock(mtx_); paused_ = paused; }
bool SwarmEngine::is_paused() const { std::lock_guard<std::mutex> lock(mtx_); return paused_; }

int SwarmEngine::completed_tasks_count() const {
    int c = 0;
    for (const auto& t : tasks_)
        if (t.completed) c++;
    return c;
}

double SwarmEngine::mean_position_rmse_m() const {
    return rmse_samples_ > 0 ? std::sqrt(rmse_accum_ / rmse_samples_) : 0.0;
}

double SwarmEngine::soc_spread_pct() const {
    double lo = 1.0, hi = 0.0;
    for (int u = 0; u < opts_.n_uavs; ++u) {
        lo = std::min(lo, fleet_[u].soc);
        hi = std::max(hi, fleet_[u].soc);
    }
    return (hi - lo) * 100.0;
}

double SwarmEngine::min_soc() const {
    double lo = 1.0;
    for (int u = 0; u < opts_.n_uavs; ++u) lo = std::min(lo, fleet_[u].soc);
    return lo;
}

double SwarmEngine::nees_in_bounds_pct() const {
    return nees_total_ > 0 ? (100.0 * nees_ok_ / nees_total_) : 100.0;
}

std::vector<AblationVariantResult> SwarmEngine::run_icaav_ablation(int n_runs_per_variant) {
    struct VariantCfg {
        std::string label;
        bool wci;
        bool nhc;
        bool rot;
        bool l1;
        bool cbf;
        bool cbba;
    };
    const std::vector<VariantCfg> variants = {
        {"Full Survivable Stack (ESKF+NHC+WCI+L1+RotMaster+CBBA)", true, true, true, true, true, true},
        {"Ablation: Solo ESKF (No WCI / No NHC)", false, false, true, true, true, true},
        {"Ablation: Static Master-Slave (No Rotation)", true, true, false, true, true, true},
        {"Ablation: No L1 Adaptive FTC (Nominal PID)", true, true, true, false, true, true},
    };

    std::vector<AblationVariantResult> results(variants.size());
    for (size_t vi = 0; vi < variants.size(); ++vi) {
        double sum_rmse = 0.0, sum_tr = 0.0, sum_nees = 0.0, sum_comp = 0.0;
        double sum_coll = 0.0, sum_spread = 0.0, sum_min_soc = 0.0, sum_fail = 0.0, sum_sav = 0.0, sum_xte = 0.0;

        #pragma omp parallel for reduction(+:sum_rmse,sum_tr,sum_nees,sum_comp,sum_coll,sum_spread,sum_min_soc,sum_fail,sum_sav,sum_xte)
        for (int r = 0; r < n_runs_per_variant; ++r) {
            SimOptions o{};
            o.n_uavs = 5;
            o.dt = 0.1;
            o.enable_wci = variants[vi].wci;
            o.enable_nhc = variants[vi].nhc;
            o.enable_master_rotation = variants[vi].rot;
            o.enable_l1_ftc = variants[vi].l1;
            o.enable_cbf = variants[vi].cbf;
            o.enable_ed_cbba = variants[vi].cbba;
            o.seed = 1000u + static_cast<uint32_t>(r * 17 + vi * 97);

            SwarmEngine sim;
            sim.reset(o);
            int steps = 1600;  // 160 s simulated contested sortie at 10 Hz
            for (int k = 0; k < steps; ++k) {
                if (k == 300) sim.set_ew_jamming(true, 220.0, 180.0, 460.0);
                if (k == 900) sim.set_ew_jamming(false, 220.0, 180.0, 460.0);
                if (k == 1000) sim.inject_actuator_fault(1, 0.55, 0.60, 0.08);
                sim.step();
            }
            sum_rmse += sim.mean_position_rmse_m();
            sum_tr += sim.max_trace_p();
            sum_nees += sim.nees_in_bounds_pct();
            sum_comp += 100.0 * sim.completed_tasks_count() / static_cast<double>(kMaxTasks);
            sum_coll += (sim.min_inter_uav_sep_m() < 15.0) ? 100.0 : 0.0;
            sum_spread += sim.soc_spread_pct();
            sum_min_soc += sim.min_soc() * 100.0;
            sum_fail += sim.master_slave_state().last_failover_latency_s * 1000.0;
            int tot_p = sim.master_slave_state().cbba_packets_sent + sim.master_slave_state().cbba_packets_saved;
            sum_sav += tot_p > 0 ? (100.0 * sim.master_slave_state().cbba_packets_saved / tot_p) : 0.0;
            double xte_avg = 0.0;
            for (int u = 0; u < 5; ++u) xte_avg += sim.deviations()[u].xte_m;
            sum_xte += xte_avg / 5.0;
        }

        AblationVariantResult out{};
        out.label = variants[vi].label;
        out.n_runs = n_runs_per_variant;
        out.mean_rmse_m = sum_rmse / n_runs_per_variant;
        out.peak_trace_p = sum_tr / n_runs_per_variant;
        out.nees_in_bounds_pct = sum_nees / n_runs_per_variant;
        out.mission_completion_pct = sum_comp / n_runs_per_variant;
        out.collision_rate_pct = sum_coll / n_runs_per_variant;
        out.soc_spread_pct = sum_spread / n_runs_per_variant;
        out.min_soc_pct = sum_min_soc / n_runs_per_variant;
        out.mean_failover_ms = sum_fail / n_runs_per_variant;
        out.cbba_packet_savings_pct = sum_sav / n_runs_per_variant;
        out.mean_xte_m = sum_xte / n_runs_per_variant;
        results[vi] = out;
    }
    return results;
}

std::string SwarmEngine::serialize_state_json() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream o;
    o << std::fixed << std::setprecision(3);
    o << "{";
    o << "\"sim_time_s\":" << sim_time_s_ << ",";
    o << "\"paused\":" << (paused_ ? "true" : "false") << ",";
    o << "\"wind\":[" << ambient_wind_.x << "," << ambient_wind_.y << "," << ambient_wind_.z << "],";
    o << "\"jamming\":{\"global\":" << (rcfg_.global_gnss_blackout ? "true" : "false")
      << ",\"spoof\":" << (rcfg_.gnss_spoof_active ? "true" : "false")
      << ",\"n\":" << rcfg_.jam_north << ",\"e\":" << rcfg_.jam_east << ",\"r\":" << rcfg_.jam_radius_m << "},";
    o << "\"master_slave\":{\"active_master\":" << ms_.active_master
      << ",\"next_master\":" << ms_.next_master
      << ",\"epoch_id\":" << ms_.epoch_id
      << ",\"epoch_rem_s\":" << std::max(0.0, ms_.epoch_duration_s - ms_.epoch_elapsed_s)
      << ",\"total_rotations\":" << ms_.total_rotations
      << ",\"total_failovers\":" << ms_.total_failovers
      << ",\"fiedler_lambda2\":" << ms_.fiedler_lambda2
      << ",\"avg_latency_ms\":" << ms_.avg_link_latency_ms
      << ",\"relay_hops\":" << ms_.relay_hops_used << "},";

    // Hazards
    o << "\"hazards\":[";
    for (size_t i = 0; i < rcfg_.peaks.size(); ++i) {
        if (i > 0) o << ",";
        o << "{\"n\":" << rcfg_.peaks[i].north << ",\"e\":" << rcfg_.peaks[i].east
          << ",\"amp\":" << rcfg_.peaks[i].amplitude << ",\"sig\":" << rcfg_.peaks[i].sigma << "}";
    }
    o << "],";

    // Tasks
    o << "\"tasks\":[";
    for (size_t i = 0; i < tasks_.size(); ++i) {
        if (i > 0) o << ",";
        const auto& t = tasks_[i];
        o << "{\"id\":" << t.id << ",\"n\":" << t.pos_ned.x << ",\"e\":" << t.pos_ned.y
          << ",\"alt\":" << -t.pos_ned.z << ",\"r\":" << t.loiter_radius_m
          << ",\"assigned\":" << t.assigned_uav << ",\"done\":" << (t.completed ? "true" : "false")
          << ",\"dwell_pct\":" << clampd(100.0 * t.dwell_accum_s / t.dwell_required_s, 0.0, 100.0) << "}";
    }
    o << "],";

    // UAV fleet array
    o << "\"uavs\":[";
    for (int u = 0; u < opts_.n_uavs; ++u) {
        if (u > 0) o << ",";
        const auto& f = fleet_[u];
        const auto& b = beliefs_[u];
        const auto& d = devs_[u];
        const auto& l = l1_[u];
        Vec3 eul = b.q.to_euler();
        o << "{\"id\":" << u
          << ",\"alive\":" << (f.alive ? "true" : "false")
          << ",\"is_master\":" << (u == ms_.active_master ? "true" : "false")
          << ",\"connected\":" << (ms_.connected_to_swarm[u] ? "true" : "false")
          << ",\"dr_active\":" << (b.dr_active ? "true" : "false")
          << ",\"mode\":\"" << mode_to_str(modes_[u]) << "\""
          << ",\"true_pos\":[" << f.pos_ned.x << "," << f.pos_ned.y << "," << -f.pos_ned.z << "]"
          << ",\"est_pos\":[" << b.p.x << "," << b.p.y << "," << -b.p.z << "]"
          << ",\"ideal_pos\":[" << d.ideal_pos_ned.x << "," << d.ideal_pos_ned.y << "," << -d.ideal_pos_ned.z << "]"
          << ",\"quat\":[" << f.q_nb.w << "," << f.q_nb.x << "," << f.q_nb.y << "," << f.q_nb.z << "]"
          << ",\"euler_deg\":[" << eul.x * 180.0 / kPi << "," << eul.y * 180.0 / kPi << "," << eul.z * 180.0 / kPi << "]"
          << ",\"va_ms\":" << (f.vel_ned - ambient_wind_).norm()
          << ",\"soc\":" << f.soc
          << ",\"v_term\":" << f.v_term_v
          << ",\"pos_sigma3\":" << b.pos_sigma_3
          << ",\"nees\":" << b.nees
          << ",\"xte_m\":" << d.xte_m
          << ",\"alt_dev_m\":" << d.alt_dev_m
          << ",\"tthi_s\":" << d.tthi_s
          << ",\"l1_roll_hat\":" << l.omega_roll_hat
          << ",\"l1_thrust_hat\":" << l.omega_thrust_hat
          << ",\"l1_fault\":" << (l.fault_flag ? "true" : "false")
          << ",\"health_score\":" << ms_.health_scores[u]
          << ",\"drift_cone\":[";
        for (int k = 0; k < 5; ++k) {
            if (k > 0) o << ",";
            o << "[" << d.drift_cone_pts[k].x << "," << d.drift_cone_pts[k].y << "," << d.drift_cone_rad[k] << "]";
        }
        o << "],\"trail\":[";
        int step_stride = std::max(1, static_cast<int>(trails_[u].size()) / 40);
        bool first_pt = true;
        for (size_t ti = 0; ti < trails_[u].size(); ti += step_stride) {
            if (!first_pt) o << ",";
            first_pt = false;
            o << "[" << trails_[u][ti].x << "," << trails_[u][ti].y << "," << -trails_[u][ti].z << "]";
        }
        o << "]}";
    }
    o << "],";

    // RF link edges
    o << "\"links\":[";
    bool first_link = true;
    for (int i = 0; i < opts_.n_uavs; ++i) {
        for (int j = i + 1; j < opts_.n_uavs; ++j) {
            if (ms_.link_hop_matrix[i][j] > 0) {
                if (!first_link) o << ",";
                first_link = false;
                o << "{\"a\":" << i << ",\"b\":" << j << ",\"hops\":" << ms_.link_hop_matrix[i][j] << "}";
            }
        }
    }
    o << "],";

    // Latest 20 alerts
    o << "\"alerts\":[";
    size_t lim = std::min<size_t>(alerts_.size(), 20);
    for (size_t i = 0; i < lim; ++i) {
        if (i > 0) o << ",";
        const auto& a = alerts_[i];
        o << "{\"seq\":" << a.seq << ",\"t\":" << a.sim_time_s
          << ",\"sev\":\"" << sev_to_str(a.severity) << "\""
          << ",\"code\":\"" << a.code << "\""
          << ",\"uav_id\":" << a.uav_id
          << ",\"title\":\"" << json_escape(a.title) << "\""
          << ",\"detail\":\"" << json_escape(a.detail) << "\"}";
    }
    o << "]}";
    return o.str();
}

std::string SwarmEngine::export_csv_log() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream oss;
    for (const auto& row : csv_rows_) oss << row << "\n";
    return oss.str();
}

void SwarmEngine::copy_render_snapshot(
    std::array<UavTrueState, kMaxUavs>& fleet_out,
    std::array<EskfState, kMaxUavs>& beliefs_out,
    std::array<DeviationMetrics, kMaxUavs>& devs_out,
    std::array<L1AdaptiveState, kMaxUavs>& l1_out,
    std::array<MdpMode, kMaxUavs>& modes_out,
    std::vector<IsrTask>& tasks_out,
    MasterSlaveState& ms_out,
    RiskFieldConfig& rcfg_out,
    std::array<std::deque<Vec3>, kMaxUavs>& trails_out,
    double& sim_time_out,
    int& n_uavs_out
) const {
    std::lock_guard<std::mutex> lock(mtx_);
    fleet_out = fleet_;
    beliefs_out = beliefs_;
    devs_out = devs_;
    l1_out = l1_;
    modes_out = modes_;
    tasks_out = tasks_;
    ms_out = ms_;
    rcfg_out = rcfg_;
    trails_out = trails_;
    sim_time_out = sim_time_s_;
    n_uavs_out = opts_.n_uavs;
}

}  // namespace swarm
