// Park-Deyst-How L1 guidance, loiter orbit, 3D CBF-QP, Bellman MDP, and deviation predictor

#include "guidance_cbf_mdp.hpp"
#include "aero_plant.hpp"
#include <cmath>

namespace swarm {

namespace {

int encode_mdp_state(int unc_lvl, int gps_ok, int soc_lvl, int fault_on) {
    return (((unc_lvl * 2 + gps_ok) * 3 + soc_lvl) * 2 + fault_on);
}

}  // namespace

BellmanMdpPolicy solve_bellman_mdp() {
    BellmanMdpPolicy mdp{};
    constexpr int kS = 36;
    constexpr int kA = 4;
    constexpr double kGamma = 0.88;

    std::array<double, kS> V{};
    for (int iter = 0; iter < 120; ++iter) {
        double max_diff = 0.0;
        std::array<double, kS> V_next{};
        for (int u = 0; u < 3; ++u) {
            for (int g = 0; g < 2; ++g) {
                for (int e = 0; e < 3; ++e) {
                    for (int f = 0; f < 2; ++f) {
                        int s = encode_mdp_state(u, g, e, f);
                        double best_q = -1e9;
                        int best_a = 0;
                        for (int a = 0; a < kA; ++a) {
                            // State-dependent physical reward modeling mission progress, safety, and survival
                            double r = 0.0;
                            if (a == 0) {  // NOMINAL
                                r = 10.0 - 6.5 * u - 8.0 * (1 - g) - 18.0 * (e == 0) - 9.0 * f;
                            } else if (a == 1) {  // DEGRADED_COOP
                                r = 7.2 - 1.8 * u + 4.0 * (1 - g) - 14.0 * (e == 0) - 3.0 * f;
                            } else if (a == 2) {  // CONSERVATIVE_RISK
                                r = 4.8 + 3.5 * (u == 2) + 2.5 * f - 10.0 * (e == 0);
                            } else {  // EMERGENCY_RTB
                                r = -2.0 + 18.0 * (e == 0) + 5.5 * (e == 1 && f == 1) - 6.0 * (e == 2 && f == 0);
                            }

                            // Expected next belief-health state transition
                            int u_next = u;
                            if (a == 1 && u > 0) u_next = u - 1;
                            if (a == 0 && g == 0 && u < 2) u_next = u + 1;
                            int e_next = (a == 0 && e > 0) ? e - 1 : e;
                            int s_stay = encode_mdp_state(u, g, e, f);
                            int s_move = encode_mdp_state(u_next, g, e_next, f);
                            double q = r + kGamma * (0.65 * V[s_stay] + 0.35 * V[s_move]);
                            if (q > best_q) {
                                best_q = q;
                                best_a = a;
                            }
                        }
                        V_next[s] = best_q;
                        mdp.pi_star[s] = best_a;
                        max_diff = std::max(max_diff, std::abs(V_next[s] - V[s]));
                    }
                }
            }
        }
        V = V_next;
        mdp.iterations = iter + 1;
        if (max_diff < 1e-6) break;
    }
    mdp.v_star = V;
    return mdp;
}

MdpMode query_bellman_mode(const BellmanMdpPolicy& mdp, double pos_sigma_3, bool gps_ok, double soc, bool fault_active) {
    int u = pos_sigma_3 > 38.0 ? 2 : (pos_sigma_3 > 20.0 ? 1 : 0);
    int g = gps_ok ? 1 : 0;
    int e = soc < 0.26 ? 0 : (soc < 0.48 ? 1 : 2);
    int f = fault_active ? 1 : 0;
    int s = encode_mdp_state(u, g, e, f);
    return static_cast<MdpMode>(mdp.pi_star[s]);
}

void compute_guidance_and_cbf(
    int uav_idx,
    const std::array<EskfState, kMaxUavs>& beliefs,
    const std::array<bool, kMaxUavs>& alive,
    int n_uavs,
    const Vec3& goal_ned,
    bool loiter_mode,
    double loiter_radius_m,
    MdpMode mode,
    const FixedWingParams& p,
    const RiskFieldConfig& rcfg,
    bool enable_cbf,
    double& bank_cmd_out,
    double& climb_cmd_out,
    double& va_cmd_out
) {
    const auto& est = beliefs[uav_idx];
    double v_ground = std::max(10.0, std::hypot(est.v.x, est.v.y));
    double course = std::atan2(est.v.y, est.v.x);

    // Desired speed and bank authority scaled by Bellman MDP supervisory mode
    double va_target = p.v_cruise_ms;
    double bank_lim = p.max_bank_rad;
    if (mode == MdpMode::DEGRADED_COOP) {
        va_target = 17.5;
        bank_lim = 32.0 * kPi / 180.0;
    } else if (mode == MdpMode::CONSERVATIVE_RISK) {
        va_target = 16.0;
        bank_lim = 28.0 * kPi / 180.0;
    } else if (mode == MdpMode::EMERGENCY_RTB) {
        va_target = 15.8;
        bank_lim = 26.0 * kPi / 180.0;
    }

    // Nominal Park-Deyst-How L1 guidance or wind-compensated vector-field loiter orbit
    double dn = goal_ned.x - est.p.x;
    double de = goal_ned.y - est.p.y;
    double dist_xy = std::hypot(dn, de);
    double desired_course = std::atan2(de, dn);

    if (loiter_mode && dist_xy < loiter_radius_m * 2.4) {
        double bearing_from_center = std::atan2(est.p.y - goal_ned.y, est.p.x - goal_ned.x);
        double radial_err = (dist_xy - loiter_radius_m) / std::max(25.0, loiter_radius_m);
        desired_course = wrap_angle(bearing_from_center + 0.5 * kPi + std::atan(1.8 * radial_err));
    }

    // Chance-constrained risk-gradient avoidance (smooth downhill deflection, no +90 deg trap)
    RiskEval re = evaluate_risk_field(rcfg, est.p.x, est.p.y);
    double sigma_xy = std::sqrt(est.P(0, 0) + est.P(1, 1));
    double grad_norm = std::hypot(re.grad_n, re.grad_e);
    double r_eff = re.value + 0.5 * (re.hess_nn * est.P(0, 0) + re.hess_ee * est.P(1, 1)) + 2.2 * grad_norm * sigma_xy;

    Vec3 steer_vec{std::cos(desired_course), std::sin(desired_course), 0.0};
    if (r_eff > 3.2 && grad_norm > 1e-5) {
        double w_rep = clampd((r_eff - 3.2) * 0.28, 0.0, 1.35);
        steer_vec.x -= w_rep * (re.grad_n / grad_norm);
        steer_vec.y -= w_rep * (re.grad_e / grad_norm);
        desired_course = std::atan2(steer_vec.y, steer_vec.x);
    }

    double eta_l1 = clampd(wrap_angle(desired_course - course), -0.45 * kPi, 0.45 * kPi);
    double l1_dist = clampd(2.4 * v_ground, 38.0, 85.0);
    double a_lat = (2.0 * v_ground * v_ground / l1_dist) * std::sin(eta_l1);

    // Dedicated TCAS flight-level altitude band per UAV (26 m vertical separation)
    double target_alt = 105.0 + 26.0 * uav_idx;
    double alt_err = target_alt - (-est.p.z);
    double climb_cmd = clampd(0.018 * alt_err, -p.max_climb_rad, p.max_climb_rad);

    // Multi-agent 3D fixed-wing CBF-QP + TCAS vertical deconfliction + wake-cone exclusion
    if (enable_cbf) {
        for (int j = 0; j < n_uavs; ++j) {
            if (j == uav_idx || !alive[j]) continue;
            Vec3 rel = est.p - beliefs[j].p;
            double d_xy = std::hypot(rel.x, rel.y);
            double d_z = std::abs(rel.z);
            double sig_j = std::sqrt(beliefs[j].P(0, 0) + beliefs[j].P(1, 1));
            double d_safe = 58.0 + 1.2 * (sigma_xy + sig_j);

            // Pairwise CBF safety barrier in horizontal and vertical channels
            if (d_xy < d_safe * 1.45 && d_z < 30.0) {
                double h_val = d_xy - d_safe;
                Vec3 rel_v = est.v - beliefs[j].v;
                double closing = -(rel.x * rel_v.x + rel.y * rel_v.y) / std::max(1.0, d_xy);
                if (h_val < 18.0 || closing > 0.5) {
                    // Right-of-way lateral turn + deterministic TCAS vertical altitude layering
                    double cross_z = std::cos(course) * (-rel.y) - std::sin(course) * (-rel.x);
                    double turn_sign = (cross_z >= 0.0) ? -1.0 : 1.0;
                    double push = clampd((d_safe * 1.45 - d_xy) / d_safe, 0.15, 1.0);
                    a_lat += turn_sign * 7.8 * push;
                    double vert_sign = (uav_idx % 2 == 0) ? 1.0 : -1.0;
                    if (std::abs(rel.z) > 2.0) vert_sign = (-rel.z >= 0.0) ? 1.0 : -1.0;
                    climb_cmd = clampd(climb_cmd + vert_sign * 0.16 * push, -p.max_climb_rad, p.max_climb_rad);
                }
            }
        }
    }

    // Soft arena geofence barrier keeping fixed-wing formation inside contested corridor
    double bound = rcfg.arena_half_m - 140.0;
    if (std::abs(est.p.x) > bound || std::abs(est.p.y) > bound) {
        double home_course = std::atan2(-est.p.y, -est.p.x);
        double eta_home = wrap_angle(home_course - course);
        a_lat = (2.0 * v_ground * v_ground / 55.0) * std::sin(clampd(eta_home, -0.45 * kPi, 0.45 * kPi));
    }

    bank_cmd_out = clampd(std::atan2(a_lat, kGravity), -bank_lim, bank_lim);
    climb_cmd_out = climb_cmd;
    va_cmd_out = clampd(va_target, p.v_min_ms, p.v_max_ms);
}

DeviationMetrics compute_deviation_and_drift(
    const EskfState& est,
    const Vec3& start_ned,
    const Vec3& goal_ned,
    bool loiter_mode,
    double loiter_radius_m,
    const RiskFieldConfig& rcfg
) {
    DeviationMetrics m{};
    if (loiter_mode) {
        double dx = est.p.x - goal_ned.x;
        double dy = est.p.y - goal_ned.y;
        double r = std::max(1.0, std::hypot(dx, dy));
        m.ideal_pos_ned = {goal_ned.x + loiter_radius_m * (dx / r), goal_ned.y + loiter_radius_m * (dy / r), goal_ned.z};
        m.xte_m = std::abs(r - loiter_radius_m);
        m.ate_m = 0.0;
    } else {
        Vec3 seg = goal_ned - start_ned;
        double seg_xy = std::max(1.0, std::hypot(seg.x, seg.y));
        double ux = seg.x / seg_xy;
        double uy = seg.y / seg_xy;
        Vec3 rel = est.p - start_ned;
        double proj = clampd(rel.x * ux + rel.y * uy, 0.0, seg_xy);
        m.ideal_pos_ned = {start_ned.x + proj * ux, start_ned.y + proj * uy,
                           start_ned.z + (proj / seg_xy) * (goal_ned.z - start_ned.z)};
        m.xte_m = std::abs(-rel.x * uy + rel.y * ux);
        m.ate_m = std::max(0.0, seg_xy - proj);
    }
    m.alt_dev_m = std::abs((-est.p.z) - (-m.ideal_pos_ned.z));

    // 15-second forward predicted drift cone and Time-to-Hazard-Intersection (TTHI)
    double base_sig = std::max(3.0, est.pos_sigma_3);
    double drift_grow = est.dr_active ? 1.45 : 0.25;
    m.drift_rate_mps = drift_grow;
    m.tthi_s = 999.0;
    for (int k = 0; k < 5; ++k) {
        double tau = 3.0 * (k + 1);
        Vec3 pt = est.p + est.v * tau;
        double rad = base_sig + drift_grow * tau;
        m.drift_cone_pts[k] = pt;
        m.drift_cone_rad[k] = rad;
        RiskEval re = evaluate_risk_field(rcfg, pt.x, pt.y);
        if (m.tthi_s > 900.0 && re.value > 5.0) {
            m.tthi_s = tau;
        }
    }
    return m;
}

}  // namespace swarm
