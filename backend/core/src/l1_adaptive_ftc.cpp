// L1 Adaptive Fault-Tolerant Autopilot with Pseudo-Control Hedging and TECS coupling
// Strictly obeys Separation Principle: zero access to UavTrueState

#include "l1_adaptive_ftc.hpp"
#include "aero_plant.hpp"
#include <cmath>

namespace swarm {

void init_l1_adaptive(L1AdaptiveState& l1, double v_cruise_ms) {
    l1 = L1AdaptiveState{};
    l1.va_hat = v_cruise_ms;
    l1.omega_roll_hat = 1.0;
    l1.omega_thrust_hat = 1.0;
    l1.u_thrust_filt = 0.48;
}

ControlCommand step_l1_adaptive_ftc(
    L1AdaptiveState& l1,
    const EskfState& est,
    const SensorPacket& sens,
    const FixedWingParams& p,
    double bank_cmd_rad,
    double climb_cmd_rad,
    double va_cmd_ms,
    bool enable_l1_adaptation,
    double sim_time_s,
    double dt
) {
    ControlCommand out{};
    Vec3 eul = est.q.to_euler();
    double roll_est = eul.x;
    double pitch_est = eul.y;
    double p_meas = sens.gyro_body_meas.x - est.bg.x;
    double q_meas = sens.gyro_body_meas.y - est.bg.y;
    double r_meas = sens.gyro_body_meas.z - est.bg.z;
    double va_meas = sens.airspeed_pitot_ms;

    // Bank-dependent stall protection and TECS speed-altitude energy coupling
    double v_stall = compute_stall_speed(p, roll_est, 0.0);
    double va_safe_cmd = std::max(va_cmd_ms, v_stall * 1.18);
    double stall_margin = va_meas - v_stall * 1.12;
    double tecs_climb_cmd = climb_cmd_rad;
    if (stall_margin < 0.0) {
        // Under thrust LoE, trade excess altitude smoothly to protect airspeed above stall
        tecs_climb_cmd = std::min(tecs_climb_cmd, -0.10 + 0.05 * stall_margin);
    }

    // Baseline nominal roll and thrust commands
    double p_des = clampd(2.4 * (bank_cmd_rad - roll_est), -1.6, 1.6);
    double u_roll_nom = clampd(0.32 * (p_des - p_meas) + 0.45 * (bank_cmd_rad - roll_est),
                               -p.max_aileron_rad, p.max_aileron_rad);
    double u_thr_nom = clampd(0.46 + 0.055 * (va_safe_cmd - va_meas) + 0.55 * tecs_climb_cmd, 0.10, 1.0);

    // L1 state predictor (with Pseudo-Control Hedging using saturated applied commands)
    double roll_err = l1.p_hat - p_meas;
    double va_err = l1.va_hat - va_meas;
    const double am_roll = -3.8;
    const double bm_roll = 4.6;
    const double am_va = -0.65;
    const double bm_va = 3.2;

    if (enable_l1_adaptation) {
        // Projection-bounded fast adaptation of actuator effectiveness and matched disturbance
        double gamma_w = 1.6;
        double gamma_sig = 2.8;
        if (std::abs(l1.u_roll_filt) > 0.025) {
            l1.omega_roll_hat = clampd(l1.omega_roll_hat - gamma_w * roll_err * bm_roll * l1.u_roll_filt * dt, 0.35, 1.05);
        }
        l1.sigma_roll_hat = clampd(l1.sigma_roll_hat - gamma_sig * roll_err * dt, -1.8, 1.8);

        if (l1.u_thrust_filt > 0.15) {
            l1.omega_thrust_hat = clampd(l1.omega_thrust_hat - 0.85 * va_err * bm_va * l1.u_thrust_filt * dt, 0.40, 1.05);
        }
        l1.sigma_thrust_hat = clampd(l1.sigma_thrust_hat - 1.2 * va_err * dt, -2.5, 2.5);

        // Observable CUSUM residual detector (zero ground-truth access)
        double resid = std::abs(roll_err) * 1.4 + std::abs(va_err) * 0.45 +
                       (1.0 - std::min(l1.omega_roll_hat, l1.omega_thrust_hat)) * 2.5;
        l1.cusum_stat = std::max(0.0, l1.cusum_stat + (resid - 0.55) * dt);
        if (!l1.fault_flag && (l1.cusum_stat > 0.45 || l1.omega_roll_hat < 0.82 || l1.omega_thrust_hat < 0.82)) {
            l1.fault_flag = true;
            l1.fault_detect_time_s = sim_time_s;
        }
    }

    // L1 control law passed through bandwidth-limited low-pass filter C(s) = wc / (s + wc)
    double u_roll_raw = u_roll_nom;
    double u_thr_raw = u_thr_nom;
    if (enable_l1_adaptation) {
        u_roll_raw = (u_roll_nom - l1.sigma_roll_hat / bm_roll) / std::max(0.38, l1.omega_roll_hat);
        u_thr_raw = (u_thr_nom - l1.sigma_thrust_hat / bm_va) / std::max(0.42, l1.omega_thrust_hat);
    }
    double wc_roll = 10.0;
    double wc_thr = 4.5;
    l1.u_roll_filt += wc_roll * (clampd(u_roll_raw, -p.max_aileron_rad, p.max_aileron_rad) - l1.u_roll_filt) * dt;
    l1.u_thrust_filt += wc_thr * (clampd(u_thr_raw, 0.08, 1.0) - l1.u_thrust_filt) * dt;

    // Advance L1 state predictor using hedged (saturated) control inputs
    double u_roll_hedged = clampd(l1.u_roll_filt, -p.max_aileron_rad, p.max_aileron_rad);
    double u_thr_hedged = clampd(l1.u_thrust_filt, 0.08, 1.0);
    l1.p_hat += (am_roll * (l1.p_hat - p_meas) + bm_roll * (l1.omega_roll_hat * u_roll_hedged) + l1.sigma_roll_hat) * dt;
    l1.va_hat += (am_va * (l1.va_hat - va_meas) + bm_va * (l1.omega_thrust_hat * (u_thr_hedged - 0.46)) + l1.sigma_thrust_hat) * dt;
    l1.roll_hat = roll_est;
    l1.gamma_hat = pitch_est;

    // Pitch TECS and Dutch-Roll yaw damper
    double ele_cmd = clampd(1.15 * (tecs_climb_cmd - pitch_est) - 0.35 * q_meas, -p.max_elevator_rad, p.max_elevator_rad);
    double r_coord = (kGravity / std::max(10.0, va_meas)) * std::tan(clampd(roll_est, -0.75, 0.75));
    double rud_cmd = clampd(-0.42 * (r_meas - r_coord) + 0.14 * u_roll_hedged, -0.35, 0.35);

    out.aileron_cmd = u_roll_hedged;
    out.elevator_cmd = ele_cmd;
    out.rudder_cmd = rud_cmd;
    out.throttle_cmd = u_thr_hedged;
    out.bank_cmd_rad = bank_cmd_rad;
    out.climb_cmd_rad = tecs_climb_cmd;
    out.va_cmd_ms = va_safe_cmd;
    return out;
}

}  // namespace swarm
