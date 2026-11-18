// 6-DOF non-holonomic fixed-wing aerodynamics, Dryden turbulence, wake vortices, and 1-RC battery

#include "aero_plant.hpp"
#include <cmath>

namespace swarm {

RiskEval evaluate_risk_field(const RiskFieldConfig& cfg, double north, double east) {
    RiskEval out{};
    out.value = cfg.base_quad_coeff * (north * north + east * east);
    out.grad_n = 2.0 * cfg.base_quad_coeff * north;
    out.grad_e = 2.0 * cfg.base_quad_coeff * east;
    out.hess_nn = 2.0 * cfg.base_quad_coeff;
    out.hess_ee = 2.0 * cfg.base_quad_coeff;

    for (const auto& pk : cfg.peaks) {
        double dn = north - pk.north;
        double de = east - pk.east;
        double inv_2s2 = 1.0 / (2.0 * pk.sigma * pk.sigma);
        double g = pk.amplitude * std::exp(-(dn * dn + de * de) * inv_2s2);
        out.value += g;
        out.grad_n += g * (-2.0 * dn * inv_2s2);
        out.grad_e += g * (-2.0 * de * inv_2s2);
        out.hess_nn += g * (4.0 * dn * dn * inv_2s2 * inv_2s2 - 2.0 * inv_2s2);
        out.hess_ee += g * (4.0 * de * de * inv_2s2 * inv_2s2 - 2.0 * inv_2s2);
    }
    return out;
}

double isa_air_density(double alt_m) {
    double h = clampd(alt_m, 0.0, 8000.0);
    return 1.225 * std::pow(1.0 - 2.25577e-5 * h, 4.25588);
}

double compute_stall_speed(const FixedWingParams& p, double bank_rad, double icing_level) {
    double cos_phi = std::max(std::cos(std::abs(bank_rad)), 0.45);
    double cl_eff = p.cl_max * (1.0 - 0.14 * clampd(icing_level, 0.0, 1.0));
    double rho = isa_air_density(140.0);
    return std::sqrt((2.0 * p.mass_kg * kGravity) / (rho * p.wing_area_m2 * cl_eff * cos_phi));
}

double compute_wake_vortex_disturbance(
    int self_idx,
    const std::array<UavTrueState, kMaxUavs>& fleet,
    int n_uavs
) {
    const auto& self = fleet[self_idx];
    if (!self.alive) return 0.0;
    double wake_moment = 0.0;
    for (int j = 0; j < n_uavs; ++j) {
        if (j == self_idx || !fleet[j].alive) continue;
        Vec3 v_lead = fleet[j].vel_ned;
        double spd = std::hypot(v_lead.x, v_lead.y);
        if (spd < 1.0) continue;
        double hx = v_lead.x / spd;
        double hy = v_lead.y / spd;
        Vec3 rel = self.pos_ned - fleet[j].pos_ned;
        double trail_dist = -(rel.x * hx + rel.y * hy);
        double lat_dist = -rel.x * hy + rel.y * hx;
        double alt_dist = std::abs(rel.z);
        if (trail_dist > 8.0 && trail_dist < 120.0 && std::abs(lat_dist) < 22.0 && alt_dist < 14.0) {
            double decay = std::exp(-trail_dist / 65.0) * std::exp(-(lat_dist * lat_dist) / 160.0);
            wake_moment += (lat_dist >= 0.0 ? 1.0 : -1.0) * 0.45 * decay;
        }
    }
    return wake_moment;
}

void step_fixed_wing_plant(
    UavTrueState& st,
    const ControlCommand& cmd,
    const FixedWingParams& p,
    const Vec3& ambient_wind_ned,
    double wake_roll_moment,
    bool is_master,
    double dt,
    std::mt19937& rng
) {
    if (!st.alive) return;
    std::normal_distribution<double> n01(0.0, 1.0);

    // First-order actuator lag with deflection/rate saturation and loss of effectiveness
    double alpha_act = dt / (p.tau_actuator_s + dt);
    double cmd_ail = clampd(cmd.aileron_cmd, -p.max_aileron_rad, p.max_aileron_rad);
    double cmd_ele = clampd(cmd.elevator_cmd, -p.max_elevator_rad, p.max_elevator_rad);
    double cmd_rud = clampd(cmd.rudder_cmd, -0.35, 0.35);
    double cmd_thr = clampd(cmd.throttle_cmd, 0.05, 1.0);

    st.act_roll += clampd(alpha_act * (cmd_ail - st.act_roll), -p.max_rate_rad_s * dt, p.max_rate_rad_s * dt);
    st.act_pitch += clampd(alpha_act * (cmd_ele - st.act_pitch), -p.max_rate_rad_s * dt, p.max_rate_rad_s * dt);
    st.act_yaw += alpha_act * (cmd_rud - st.act_yaw);
    st.act_throttle += alpha_act * (cmd_thr - st.act_throttle);

    // Effective actuator output under LoE and stuck-surface bias
    double eff_ail = st.eta_roll * st.act_roll + st.roll_bias_rad;
    double eff_thr = st.eta_thrust * st.act_throttle;

    // MIL-HDBK-1797 Dryden continuous turbulence forming filters
    double alt_m = std::max(20.0, -st.pos_ned.z);
    Vec3 v_air_prev = st.vel_ned - ambient_wind_ned - Vec3{st.dryden_u, st.dryden_v, st.dryden_w};
    double va = std::max(p.v_min_ms * 0.85, v_air_prev.norm());
    double Lu = 120.0, Lv = 95.0, Lw = std::max(30.0, alt_m);
    double sig_u = 0.85, sig_v = 0.75, sig_w = 0.45;
    st.dryden_u += (-(va / Lu) * st.dryden_u + sig_u * std::sqrt(2.0 * va / Lu) * n01(rng) / std::sqrt(dt)) * dt;
    st.dryden_v += (-(va / Lv) * st.dryden_v + sig_v * std::sqrt(2.0 * va / Lv) * n01(rng) / std::sqrt(dt)) * dt;
    st.dryden_w += (-(va / Lw) * st.dryden_w + sig_w * std::sqrt(2.0 * va / Lw) * n01(rng) / std::sqrt(dt)) * dt;
    st.dryden_u = clampd(st.dryden_u, -4.0, 4.0);
    st.dryden_v = clampd(st.dryden_v, -4.0, 4.0);
    st.dryden_w = clampd(st.dryden_w, -2.5, 2.5);

    Vec3 total_wind = ambient_wind_ned + Vec3{st.dryden_u, st.dryden_v, st.dryden_w};
    Vec3 eul = st.q_nb.to_euler();
    double roll = eul.x;
    double pitch = eul.y;
    double yaw = eul.z;

    // 6-DOF rotational dynamics with adverse yaw, Dutch-Roll coupling, icing asymmetry, and wake vortex
    double rho = isa_air_density(alt_m);
    double qbar = 0.5 * rho * va * va;
    Vec3 v_air_body = st.q_nb.rotate_inv(st.vel_ned - total_wind);
    double beta = clampd(std::atan2(v_air_body.y, std::max(5.0, v_air_body.x)), -0.35, 0.35);

    double icing_roll_asym = 0.32 * st.icing_level;
    double tau_roll = qbar * p.wing_area_m2 * p.wingspan_m *
                      (0.24 * eff_ail - 0.06 * beta - 0.14 * st.omega_body.x * p.wingspan_m / (2.0 * va))
                      + wake_roll_moment + icing_roll_asym;
    double tau_pitch = qbar * p.wing_area_m2 * 0.40 *
                       (0.45 * st.act_pitch - 0.18 * pitch - 0.25 * st.omega_body.y)
                       - 0.12 * st.icing_level;
    double r_coord = (kGravity / std::max(10.0, va)) * std::tan(clampd(roll, -0.75, 0.75));
    double tau_yaw = qbar * p.wing_area_m2 * p.wingspan_m *
                     (-0.025 * eff_ail + 0.16 * st.act_yaw + 0.08 * beta - 0.16 * (st.omega_body.z - r_coord));

    Vec3 tau{tau_roll, tau_pitch, tau_yaw};
    Vec3 Jw{p.inertia_diag.x * st.omega_body.x, p.inertia_diag.y * st.omega_body.y, p.inertia_diag.z * st.omega_body.z};
    Vec3 gyro_cpl = st.omega_body.cross(Jw);
    st.omega_body.x += ((tau.x - gyro_cpl.x) / p.inertia_diag.x) * dt;
    st.omega_body.y += ((tau.y - gyro_cpl.y) / p.inertia_diag.y) * dt;
    st.omega_body.z += ((tau.z - gyro_cpl.z) / p.inertia_diag.z) * dt;
    st.omega_body.x = clampd(st.omega_body.x, -2.5, 2.5);
    st.omega_body.y = clampd(st.omega_body.y, -1.8, 1.8);
    st.omega_body.z = clampd(st.omega_body.z, -1.5, 1.5);

    // Coordinated-turn yaw rate enforcement and unit Hamilton quaternion integration
    Quat q_prev = st.q_nb;
    st.q_nb = (st.q_nb * Quat::from_rotvec(st.omega_body * dt)).normalized();
    Vec3 eul_new = st.q_nb.to_euler();
    double roll_new = clampd(eul_new.x, -0.95, 0.95);
    double pitch_new = clampd(eul_new.y, -0.42, 0.42);
    double yaw_new = wrap_angle(yaw + ((kGravity / std::max(10.0, va)) * std::tan(roll_new) + 0.25 * (st.omega_body.z - r_coord)) * dt);
    st.q_nb = Quat::from_euler(roll_new, pitch_new, yaw_new);
    // Exact effective body angular rate consistent with quaternion increment
    st.omega_eff_body = quat_log(q_prev.conjugate() * st.q_nb) / dt;

    // Aerodynamic lift, drag polar, icing drag, and advance-ratio propeller efficiency
    double load_n = 1.0 / std::max(std::cos(roll_new), 0.50);
    double cl = clampd((p.mass_kg * kGravity * load_n) / std::max(1.0, qbar * p.wing_area_m2), 0.05, p.cl_max);
    double cd = p.cd0 + p.induced_k * cl * cl + 0.018 * st.icing_level;
    double drag_n = qbar * p.wing_area_m2 * cd;
    double thrust_n = eff_thr * p.max_thrust_n;
    double ax_air = (thrust_n - drag_n - p.mass_kg * kGravity * std::sin(pitch_new)) / p.mass_kg;

    // Non-holonomic forward airspeed integration bounded above aerodynamic stall
    double v_stall = compute_stall_speed(p, roll_new, st.icing_level);
    double va_new = clampd(va + ax_air * dt, std::max(p.v_min_ms * 0.92, v_stall * 0.98), p.v_max_ms);

    // Smooth inertial acceleration from true aerodynamic velocity change
    Vec3 v_prev = st.vel_ned;
    Vec3 v_air_ned{
        va_new * std::cos(pitch_new) * std::cos(yaw_new),
        va_new * std::cos(pitch_new) * std::sin(yaw_new),
        -va_new * std::sin(pitch_new),
    };
    st.accel_ned_true = (v_air_ned - v_air_prev) / dt;
    st.vel_ned = v_air_ned + total_wind;
    st.pos_ned += 0.5 * (v_prev + st.vel_ned) * dt;
    st.pos_ned.z = clampd(st.pos_ned.z, -320.0, -55.0);

    // Gauss-Markov IMU bias drift
    double tau_b = 400.0;
    st.true_ba.x += (-st.true_ba.x / tau_b + 1.5e-4 * n01(rng)) * dt;
    st.true_ba.y += (-st.true_ba.y / tau_b + 1.5e-4 * n01(rng)) * dt;
    st.true_ba.z += (-st.true_ba.z / tau_b + 1.5e-4 * n01(rng)) * dt;
    st.true_bg.x += (-st.true_bg.x / tau_b + 4.0e-5 * n01(rng)) * dt;
    st.true_bg.y += (-st.true_bg.y / tau_b + 4.0e-5 * n01(rng)) * dt;
    st.true_bg.z += (-st.true_bg.z / tau_b + 4.0e-5 * n01(rng)) * dt;

    // 1-RC electro-thermal battery model with propeller advance-ratio efficiency and Master RF draw
    double eta_prop = clampd(0.76 - 0.012 * std::abs(va_new - p.v_cruise_ms) - 0.08 * (1.0 - st.eta_thrust), 0.45, 0.80);
    double p_prop = std::max(20.0, (thrust_n * va_new) / eta_prop);
    double p_rf = is_master ? p.master_rf_power_w : 4.0;
    st.power_w = p_prop + p.hotel_power_w + p_rf;

    double ocv = 6.0 * (3.25 + 0.88 * st.soc + 0.07 * std::pow(st.soc, 3));
    double v_guess = std::max(15.0, ocv - st.vp_v);
    double current_a = st.power_w / v_guess;
    double tau_rc = p.r1_ohm * p.c1_f;
    st.vp_v += (-(st.vp_v / tau_rc) + (current_a / p.c1_f)) * dt;
    st.v_term_v = ocv - current_a * p.r0_ohm - st.vp_v;
    st.soc = clampd(st.soc - (current_a * dt) / (p.capacity_ah * 3600.0), 0.0, 1.0);
    if (st.v_term_v < p.v_cutoff_v || st.soc <= 0.005) {
        st.alive = false;
    }
}

SensorPacket sample_sensors(
    const UavTrueState& st,
    const FixedWingParams& p,
    const RiskFieldConfig& rcfg,
    const Vec3& ambient_wind_ned,
    double sim_time_s,
    double dt,
    std::mt19937& rng
) {
    SensorPacket pkt{};
    std::normal_distribution<double> n01(0.0, 1.0);

    // Specific force in body frame from true inertial acceleration: f_b = R^T * (a_true - g_ned)
    Vec3 v_air = st.vel_ned - ambient_wind_ned;
    double va = std::max(8.0, v_air.norm());
    Vec3 g_ned{0.0, 0.0, kGravity};
    Vec3 f_body = st.q_nb.rotate_inv(st.accel_ned_true - g_ned);

    pkt.accel_body_meas = f_body + st.true_ba + Vec3{0.06 * n01(rng), 0.06 * n01(rng), 0.06 * n01(rng)};
    pkt.gyro_body_meas = st.omega_eff_body + st.true_bg + Vec3{0.003 * n01(rng), 0.003 * n01(rng), 0.003 * n01(rng)};
    pkt.airspeed_pitot_ms = std::max(5.0, va + 0.35 * n01(rng));
    pkt.baro_alt_m = std::max(10.0, -st.pos_ned.z + 0.7 * n01(rng));
    pkt.mag_yaw_rad = wrap_angle(st.q_nb.to_euler().z + 0.025 * n01(rng));
    pkt.v_term_meas = st.v_term_v + 0.04 * n01(rng);
    pkt.soc_est = clampd(st.soc + 0.003 * n01(rng), 0.0, 1.0);

    // Determine GNSS reception (blocked by global blackout or local EW jamming cylinder)
    double dist_jam = std::hypot(st.pos_ned.x - rcfg.jam_north, st.pos_ned.y - rcfg.jam_east);
    bool in_jam = rcfg.global_gnss_blackout || (dist_jam < rcfg.jam_radius_m);
    pkt.gnss_received = !in_jam;

    if (pkt.gnss_received) {
        Vec3 spoof_offset{0.0, 0.0, 0.0};
        if (rcfg.gnss_spoof_active && rcfg.spoof_start_s >= 0.0) {
            double dt_sp = std::max(0.0, sim_time_s - rcfg.spoof_start_s);
            spoof_offset = {rcfg.spoof_ramp_mps * dt_sp + 18.0, -0.7 * rcfg.spoof_ramp_mps * dt_sp - 12.0, 0.0};
        }
        pkt.gnss_pos_ned = st.pos_ned + spoof_offset + Vec3{2.2 * n01(rng), 2.2 * n01(rng), 3.0 * n01(rng)};
        pkt.gnss_vel_ned = st.vel_ned + Vec3{0.35 * n01(rng), 0.35 * n01(rng), 0.45 * n01(rng)};
    }
    return pkt;
}

}  // namespace swarm
