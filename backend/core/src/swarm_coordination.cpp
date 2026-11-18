// Rotating Master-Slave, multi-hop RF/Gilbert-Elliott topology, Fiedler lambda2, and Event-Driven CBBA

#include "swarm_coordination.hpp"
#include "cyber_ew_defense.hpp"
#include <algorithm>
#include <cmath>

namespace swarm {

namespace {

// Estimate algebraic connectivity (Fiedler eigenvalue lambda2) of symmetric N x N Laplacian
double compute_fiedler_lambda2(const std::array<std::array<int, kMaxUavs>, kMaxUavs>& adj,
                               const std::array<bool, kMaxUavs>& alive, int n_uavs) {
    std::vector<int> idx;
    for (int i = 0; i < n_uavs; ++i)
        if (alive[i]) idx.push_back(i);
    int m = static_cast<int>(idx.size());
    if (m <= 1) return 0.0;

    // Build graph Laplacian L and shifted matrix M = d_max * I - L
    std::array<std::array<double, kMaxUavs>, kMaxUavs> L{};
    double d_max = 0.0;
    for (int a = 0; a < m; ++a) {
        double deg = 0.0;
        for (int b = 0; b < m; ++b) {
            if (a != b && adj[idx[a]][idx[b]] > 0) {
                L[a][b] = -1.0;
                deg += 1.0;
            }
        }
        L[a][a] = deg;
        d_max = std::max(d_max, deg);
    }
    if (d_max < 1e-6) return 0.0;

    // Deflated power iteration on M = d_max*I - L orthogonal to constant vector 1
    std::array<double, kMaxUavs> v{};
    for (int a = 0; a < m; ++a) v[a] = (a % 2 == 0) ? 1.0 : -1.0;
    auto project_norm = [&](std::array<double, kMaxUavs>& vec) {
        double mean = 0.0;
        for (int a = 0; a < m; ++a) mean += vec[a];
        mean /= m;
        double nrm = 0.0;
        for (int a = 0; a < m; ++a) { vec[a] -= mean; nrm += vec[a] * vec[a]; }
        nrm = std::sqrt(std::max(1e-12, nrm));
        for (int a = 0; a < m; ++a) vec[a] /= nrm;
    };
    project_norm(v);

    for (int it = 0; it < 28; ++it) {
        std::array<double, kMaxUavs> w{};
        for (int a = 0; a < m; ++a) {
            double acc = d_max * v[a];
            for (int b = 0; b < m; ++b) acc -= L[a][b] * v[b];
            w[a] = acc;
        }
        v = w;
        project_norm(v);
    }
    double rayleigh = 0.0;
    for (int a = 0; a < m; ++a)
        for (int b = 0; b < m; ++b)
            rayleigh += v[a] * L[a][b] * v[b];
    return std::max(0.0, rayleigh);
}

}  // namespace

void update_comms_topology(
    MasterSlaveState& ms,
    CommsChannelState& ch,
    const std::array<UavTrueState, kMaxUavs>& fleet,
    const RiskFieldConfig& rcfg,
    int n_uavs,
    std::mt19937& rng
) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    // Gilbert-Elliott 2-state Markov burst jamming model
    if (ch.burst_jam_state) {
        if (u01(rng) < 0.28) ch.burst_jam_state = false;
    } else {
        if (u01(rng) < 0.04) ch.burst_jam_state = true;
    }

    std::array<std::array<int, kMaxUavs>, kMaxUavs> direct{};
    std::array<bool, kMaxUavs> alive{};
    double delay_sum = 0.0;
    int link_cnt = 0;

    for (int i = 0; i < n_uavs; ++i) {
        alive[i] = fleet[i].alive;
        for (int j = 0; j < n_uavs; ++j) {
            ms.link_hop_matrix[i][j] = 0;
            ch.packet_delay_s[i][j] = 0.0;
        }
    }

    for (int i = 0; i < n_uavs; ++i) {
        if (!alive[i]) continue;
        for (int j = i + 1; j < n_uavs; ++j) {
            if (!alive[j]) continue;
            double dist = (fleet[i].pos_ned - fleet[j].pos_ned).norm();
            if (dist > ch.max_range_m) continue;
            double p_loss = ch.base_loss_prob + 0.35 * std::pow(dist / ch.max_range_m, 2);
            if (ch.burst_jam_state) p_loss += 0.25;
            double d_jam_i = std::hypot(fleet[i].pos_ned.x - rcfg.jam_north, fleet[i].pos_ned.y - rcfg.jam_east);
            double d_jam_j = std::hypot(fleet[j].pos_ned.x - rcfg.jam_north, fleet[j].pos_ned.y - rcfg.jam_east);
            if (d_jam_i < rcfg.jam_radius_m || d_jam_j < rcfg.jam_radius_m) p_loss += 0.18;

            if (u01(rng) > clampd(p_loss, 0.02, 0.85)) {
                direct[i][j] = 1;
                direct[j][i] = 1;
                ms.link_hop_matrix[i][j] = 1;
                ms.link_hop_matrix[j][i] = 1;
                double dly = 0.025 + 0.00004 * dist + 0.04 * u01(rng);
                ch.packet_delay_s[i][j] = dly;
                ch.packet_delay_s[j][i] = dly;
                delay_sum += dly * 1000.0;
                link_cnt += 1;
            }
        }
    }

    // 2-hop TDMA relay forwarding for out-of-range or jammed edge nodes
    int relays = 0;
    for (int i = 0; i < n_uavs; ++i) {
        if (!alive[i]) continue;
        for (int j = i + 1; j < n_uavs; ++j) {
            if (!alive[j] || ms.link_hop_matrix[i][j] == 1) continue;
            for (int k = 0; k < n_uavs; ++k) {
                if (k != i && k != j && direct[i][k] == 1 && direct[k][j] == 1) {
                    ms.link_hop_matrix[i][j] = 2;
                    ms.link_hop_matrix[j][i] = 2;
                    ch.packet_delay_s[i][j] = ch.packet_delay_s[i][k] + ch.packet_delay_s[k][j] + 0.03;
                    ch.packet_delay_s[j][i] = ch.packet_delay_s[i][j];
                    relays += 1;
                    break;
                }
            }
        }
    }
    ms.relay_hops_used = relays;
    ms.avg_link_latency_ms = link_cnt > 0 ? (delay_sum / link_cnt) : 180.0;
    ms.fiedler_lambda2 = compute_fiedler_lambda2(ms.link_hop_matrix, alive, n_uavs);

    // Track per-UAV heartbeat connectivity to active Master/swarm mesh
    for (int i = 0; i < n_uavs; ++i) {
        if (!alive[i]) {
            ms.connected_to_swarm[i] = false;
            ms.missed_heartbeats[i] = 10;
            continue;
        }
        bool has_link = (i == ms.active_master);
        if (!has_link && ms.active_master >= 0 && ms.active_master < n_uavs && alive[ms.active_master]) {
            has_link = (ms.link_hop_matrix[i][ms.active_master] > 0);
        } else if (i == ms.active_master) {
            int peers = 0;
            for (int j = 0; j < n_uavs; ++j)
                if (j != i && ms.link_hop_matrix[i][j] > 0) peers++;
            has_link = (peers > 0 || n_uavs == 1);
        }
        if (has_link) {
            ms.missed_heartbeats[i] = 0;
            ms.connected_to_swarm[i] = true;
        } else {
            ms.missed_heartbeats[i] += 1;
            if (ms.missed_heartbeats[i] >= 5) ms.connected_to_swarm[i] = false;
        }
    }
}

std::string step_rotating_master_slave(
    MasterSlaveState& ms,
    const std::array<EskfState, kMaxUavs>& beliefs,
    const std::array<L1AdaptiveState, kMaxUavs>& l1s,
    const std::array<MdpMode, kMaxUavs>& modes,
    const std::array<double, kMaxUavs>& socs,
    const std::array<bool, kMaxUavs>& alive,
    int n_uavs,
    double dt,
    int& old_master_out,
    std::string& reason_out
) {
    old_master_out = ms.active_master;
    reason_out.clear();

    // Compute health + graph centrality election score S_i for every UAV
    for (int i = 0; i < n_uavs; ++i) {
        if (!alive[i]) {
            ms.health_scores[i] = -1.0;
            continue;
        }
        int deg = 0;
        for (int j = 0; j < n_uavs; ++j)
            if (j != i && ms.link_hop_matrix[i][j] > 0) deg++;
        double centrality = static_cast<double>(deg) / std::max(1, n_uavs - 1);
        double eta_min = std::min(l1s[i].omega_roll_hat, l1s[i].omega_thrust_hat);
        double cov_term = 1.0 / (1.0 + 0.04 * beliefs[i].pos_sigma_3);
        double gnss_bonus = beliefs[i].dr_active ? 0.0 : 0.20;
        double mode_pen = (modes[i] == MdpMode::EMERGENCY_RTB) ? -0.40 : 0.0;
        ms.health_scores[i] = 0.30 * socs[i] + 0.25 * eta_min + 0.15 * cov_term + 0.10 * centrality + gnss_bonus + mode_pen;
    }

    int cur = ms.active_master;
    bool cur_dead = (cur < 0 || cur >= n_uavs || !alive[cur]);
    bool cur_faulted = (!cur_dead) && (l1s[cur].fault_flag || modes[cur] == MdpMode::EMERGENCY_RTB);
    bool cur_lost_link = (!cur_dead) && (!ms.connected_to_swarm[cur] && ms.missed_heartbeats[cur] >= 5);

    // Check if current Master lost GNSS while a healthy Slave still has GNSS lock
    int best_gnss_slave = -1;
    if (!cur_dead && beliefs[cur].dr_active) {
        for (int i = 0; i < n_uavs; ++i) {
            if (i != cur && alive[i] && !beliefs[i].dr_active && !l1s[i].fault_flag && ms.health_scores[i] > 0.45) {
                if (best_gnss_slave < 0 || ms.health_scores[i] > ms.health_scores[best_gnss_slave]) {
                    best_gnss_slave = i;
                }
            }
        }
    }

    auto pick_best_candidate = [&](bool round_robin) -> int {
        if (round_robin) {
            for (int off = 1; off <= n_uavs; ++off) {
                int cand = (cur + off) % n_uavs;
                if (alive[cand] && !l1s[cand].fault_flag && modes[cand] != MdpMode::EMERGENCY_RTB) {
                    return cand;
                }
            }
        }
        int best = -1;
        for (int i = 0; i < n_uavs; ++i) {
            if (i == cur || !alive[i]) continue;
            if (best < 0 || ms.health_scores[i] > ms.health_scores[best]) best = i;
        }
        return best >= 0 ? best : cur;
    };

    ms.next_master = pick_best_candidate(true);

    // Preemptive health-weighted failover
    if (cur_dead || cur_faulted || cur_lost_link || best_gnss_slave >= 0) {
        int next_m = (best_gnss_slave >= 0) ? best_gnss_slave : pick_best_candidate(false);
        if (next_m != cur && next_m >= 0 && alive[next_m]) {
            int next_epoch = ms.epoch_id + 1;
            if (verify_master_lease_claim(ms.epoch_id, cur, next_epoch, next_m)) {
                ms.active_master = next_m;
                ms.epoch_id = next_epoch;
                ms.epoch_elapsed_s = 0.0;
                ms.total_failovers += 1;
                ms.last_failover_latency_s = 0.22;
                if (cur_dead) reason_out = "Master attrition";
                else if (cur_faulted) reason_out = "Master actuator/RTB degradation";
                else if (cur_lost_link) reason_out = "Master heartbeat timeout";
                else reason_out = "Slave holds superior GNSS lock";
                return "MASTER_FAILOVER";
            }
        }
    }

    // Scheduled time-based TDMA Master rotation
    ms.epoch_elapsed_s += dt;
    if (ms.rotation_enabled && ms.epoch_elapsed_s >= ms.epoch_duration_s) {
        int next_m = pick_best_candidate(true);
        ms.epoch_elapsed_s = 0.0;
        if (next_m != cur && next_m >= 0 && alive[next_m]) {
            int next_epoch = ms.epoch_id + 1;
            if (verify_master_lease_claim(ms.epoch_id, cur, next_epoch, next_m)) {
                ms.active_master = next_m;
                ms.epoch_id = next_epoch;
                ms.total_rotations += 1;
                reason_out = "Scheduled TDMA epoch rotation (SOC & MTD balance)";
                return "MASTER_ROTATED";
            }
        }
    }
    return "";
}

bool step_event_driven_cbba(
    std::vector<IsrTask>& tasks,
    MasterSlaveState& ms,
    const std::array<EskfState, kMaxUavs>& beliefs,
    const std::array<L1AdaptiveState, kMaxUavs>& l1s,
    const std::array<MdpMode, kMaxUavs>& modes,
    const std::array<double, kMaxUavs>& socs,
    const std::array<bool, kMaxUavs>& alive,
    int n_uavs,
    bool force_event,
    std::string& handoff_msg_out
) {
    handoff_msg_out.clear();

    // Check if an event triggers consensus broadcast (unassigned task, dead/RTB/faulted owner)
    bool event_triggered = force_event;
    for (const auto& t : tasks) {
        if (t.completed) continue;
        if (t.assigned_uav < 0 || t.assigned_uav >= n_uavs || !alive[t.assigned_uav] ||
            modes[t.assigned_uav] == MdpMode::EMERGENCY_RTB) {
            event_triggered = true;
            break;
        }
    }

    if (!event_triggered) {
        ms.cbba_packets_saved += n_uavs;
        return false;
    }

    ms.cbba_packets_sent += std::max(1, n_uavs);
    std::array<int, kMaxUavs> load_count{};
    for (const auto& t : tasks) {
        if (!t.completed && t.assigned_uav >= 0 && alive[t.assigned_uav] && modes[t.assigned_uav] != MdpMode::EMERGENCY_RTB) {
            load_count[t.assigned_uav] += 1;
        }
    }

    bool changed = false;
    for (auto& t : tasks) {
        if (t.completed) continue;
        bool need_rebid = (t.assigned_uav < 0 || !alive[t.assigned_uav] ||
                           modes[t.assigned_uav] == MdpMode::EMERGENCY_RTB || force_event);
        if (!need_rebid) continue;

        int old_owner = t.assigned_uav;
        int best_u = -1;
        double best_bid = -1e9;
        for (int u = 0; u < n_uavs; ++u) {
            if (!alive[u] || modes[u] == MdpMode::EMERGENCY_RTB) continue;
            double dist = (beliefs[u].p - t.pos_ned).norm();
            double eta_min = std::min(l1s[u].omega_roll_hat, l1s[u].omega_thrust_hat);
            double bid = 100.0 * t.priority * std::exp(-dist / 1400.0) + 25.0 * socs[u] + 20.0 * eta_min - 18.0 * load_count[u];
            if (bid > best_bid) {
                best_bid = bid;
                best_u = u;
            }
        }
        if (best_u >= 0 && best_u != old_owner) {
            if (old_owner >= 0 && old_owner < n_uavs && load_count[old_owner] > 0) load_count[old_owner] -= 1;
            t.assigned_uav = best_u;
            load_count[best_u] += 1;
            changed = true;
            if (handoff_msg_out.empty() && old_owner >= 0) {
                handoff_msg_out = "Task #" + std::to_string(t.id + 1) + " handed off: UAV-" +
                                  std::to_string(old_owner + 1) + " -> UAV-" + std::to_string(best_u + 1);
            }
        }
    }
    return changed;
}

Vec3 compute_formation_offset(FormationType fmt, int slot_idx, int active_count, double master_yaw) {
    // Non-collinear GDOP-friendly lateral & vertical offsets clear of teammate wake cones
    double local_x = 0.0, local_y = 0.0, local_z = 0.0;
    int side = (slot_idx % 2 == 1) ? 1 : -1;
    int rank = (slot_idx + 1) / 2;
    if (fmt == FormationType::V_WEDGE) {
        local_x = -85.0 * rank;
        local_y = 95.0 * rank * side;
        local_z = -16.0 * (slot_idx % 3);
    } else if (fmt == FormationType::ECHELON_RIGHT) {
        local_x = -75.0 * slot_idx;
        local_y = 85.0 * slot_idx;
        local_z = -15.0 * (slot_idx % 3);
    } else if (fmt == FormationType::DIAMOND) {
        static constexpr double dx[5] = {0.0, -85.0, -85.0, -170.0, -170.0};
        static constexpr double dy[5] = {0.0, 95.0, -95.0, 45.0, -45.0};
        int k = slot_idx % 5;
        local_x = dx[k]; local_y = dy[k]; local_z = -18.0 * (k % 3);
    } else {  // WIDE_ISR
        double ang = (2.0 * kPi * slot_idx) / std::max(1, active_count);
        local_x = 180.0 * std::cos(ang);
        local_y = 180.0 * std::sin(ang);
        local_z = -20.0 * (slot_idx % 3);
    }
    double cy = std::cos(master_yaw), sy = std::sin(master_yaw);
    return {local_x * cy - local_y * sy, local_x * sy + local_y * cy, local_z};
}

}  // namespace swarm
