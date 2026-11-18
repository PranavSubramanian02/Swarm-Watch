// 15-State Hamilton Quaternion ESKF (Sola 2017) with wind-aware NHC, spoof gating, and WCI

#include "eskf_15state.hpp"
#include "cyber_ew_defense.hpp"
#include <cmath>

namespace swarm {

namespace {

// Inject 15D error state into nominal state and apply Sola (2017) tangent-space reset G*P*G^T
void inject_and_reset(EskfState& ekf, const std::array<double, 15>& dx) {
    ekf.p.x += dx[0]; ekf.p.y += dx[1]; ekf.p.z += dx[2];
    ekf.v.x += dx[3]; ekf.v.y += dx[4]; ekf.v.z += dx[5];
    Vec3 dtheta{clampd(dx[6], -0.25, 0.25), clampd(dx[7], -0.25, 0.25), clampd(dx[8], -0.25, 0.25)};
    ekf.q = (ekf.q * Quat::from_rotvec(dtheta)).normalized();
    ekf.ba.x = clampd(ekf.ba.x + dx[9], -0.8, 0.8);
    ekf.ba.y = clampd(ekf.ba.y + dx[10], -0.8, 0.8);
    ekf.ba.z = clampd(ekf.ba.z + dx[11], -0.8, 0.8);
    ekf.bg.x = clampd(ekf.bg.x + dx[12], -0.1, 0.1);
    ekf.bg.y = clampd(ekf.bg.y + dx[13], -0.1, 0.1);
    ekf.bg.z = clampd(ekf.bg.z + dx[14], -0.1, 0.1);

    // Exact tangent-space Jacobian G = diag(I, I, I - 0.5*[dtheta]_x, I, I)
    Mat<15, 15> G = Mat<15, 15>::identity();
    Mat<3, 3> half_sk = skew(dtheta) * 0.5;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            G(6 + r, 6 + c) -= half_sk(r, c);
    ekf.P = mat_mul(mat_mul(G, ekf.P), G.transpose());

    // Symmetrize covariance and compute 3-sigma position norm
    for (int r = 0; r < 15; ++r) {
        ekf.P(r, r) = std::max(ekf.P(r, r), 1e-8);
        for (int c = r + 1; c < 15; ++c) {
            double avg = 0.5 * (ekf.P(r, c) + ekf.P(c, r));
            ekf.P(r, c) = avg;
            ekf.P(c, r) = avg;
        }
    }
    ekf.pos_sigma_3 = 3.0 * std::sqrt(ekf.P(0, 0) + ekf.P(1, 1) + ekf.P(2, 2));
    ekf.quat_norm_err = std::abs(ekf.q.norm() - 1.0);
}

// Generic scalar Joseph-form Kalman update on 15-state ESKF
void scalar_joseph_update(EskfState& ekf, const std::array<double, 15>& h_row, double innov, double r_var) {
    std::array<double, 15> ph{};
    double s_var = r_var;
    for (int i = 0; i < 15; ++i) {
        double acc = 0.0;
        for (int j = 0; j < 15; ++j) acc += ekf.P(i, j) * h_row[j];
        ph[i] = acc;
        s_var += h_row[i] * acc;
    }
    if (s_var < 1e-10) return;
    std::array<double, 15> K{};
    std::array<double, 15> dx{};
    for (int i = 0; i < 15; ++i) {
        K[i] = ph[i] / s_var;
        dx[i] = K[i] * innov;
    }

    // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
    Mat<15, 15> I_KH = Mat<15, 15>::identity();
    for (int r = 0; r < 15; ++r)
        for (int c = 0; c < 15; ++c)
            I_KH(r, c) -= K[r] * h_row[c];
    Mat<15, 15> P_new = mat_mul(mat_mul(I_KH, ekf.P), I_KH.transpose());
    for (int r = 0; r < 15; ++r)
        for (int c = 0; c < 15; ++c)
            P_new(r, c) += K[r] * r_var * K[c];
    ekf.P = P_new;
    inject_and_reset(ekf, dx);
}

}  // namespace

void init_eskf(EskfState& ekf, const Vec3& p0, const Vec3& v0, const Quat& q0) {
    ekf = EskfState{};
    ekf.p = p0;
    ekf.v = v0;
    ekf.q = q0.normalized();
    ekf.P = Mat<15, 15>::zero();
    for (int i = 0; i < 3; ++i) ekf.P(i, i) = 16.0;
    for (int i = 3; i < 6; ++i) ekf.P(i, i) = 2.0;
    for (int i = 6; i < 9; ++i) ekf.P(i, i) = 0.04;
    for (int i = 9; i < 12; ++i) ekf.P(i, i) = 0.02;
    for (int i = 12; i < 15; ++i) ekf.P(i, i) = 0.002;
    ekf.pos_sigma_3 = 3.0 * std::sqrt(ekf.P(0, 0) + ekf.P(1, 1) + ekf.P(2, 2));
}

void eskf_predict(EskfState& ekf, const SensorPacket& sens, double dt) {
    Vec3 a_corr = sens.accel_body_meas - ekf.ba;
    Vec3 w_corr = sens.gyro_body_meas - ekf.bg;
    Mat<3, 3> R_nb = ekf.q.to_rot_matrix();
    Vec3 g_ned{0.0, 0.0, kGravity};
    Vec3 a_world = mat3_vec(R_nb, a_corr) + g_ned;

    // Strapdown nominal integration
    ekf.p += ekf.v * dt + a_world * (0.5 * dt * dt);
    ekf.v += a_world * dt;
    ekf.q = (ekf.q * Quat::from_rotvec(w_corr * dt)).normalized();

    // 15x15 error-state transition Jacobian F_x (Sola 2017 Eq. 269)
    Mat<15, 15> Fx = Mat<15, 15>::identity();
    Mat<3, 3> neg_R_sk_a = (mat_mul(R_nb, skew(a_corr))) * (-dt);
    Mat<3, 3> neg_R_dt = R_nb * (-dt);
    Mat<3, 3> rot_att = Quat::from_rotvec(-w_corr * dt).to_rot_matrix();

    for (int r = 0; r < 3; ++r) {
        Fx(r, 3 + r) = dt;
        Fx(6 + r, 12 + r) = -dt;
        for (int c = 0; c < 3; ++c) {
            Fx(3 + r, 6 + c) = neg_R_sk_a(r, c);
            Fx(3 + r, 9 + c) = neg_R_dt(r, c);
            Fx(6 + r, 6 + c) = rot_att(r, c);
        }
    }

    // Allan-variance-consistent discrete IMU + Dryden wind process noise Q
    Mat<15, 15> P_pred = mat_mul(mat_mul(Fx, ekf.P), Fx.transpose());
    double q_p = ekf.dr_active ? (4.8 * dt) : (1.1 * dt);
    double q_v = 1.8 * dt;
    double q_th = 4.0e-4 * dt;
    double q_ba = 6.0e-5 * dt;
    double q_bg = 6.0e-6 * dt;
    for (int i = 0; i < 3; ++i) {
        P_pred(i, i) += (i < 2 ? q_p : 0.25 * dt);
        P_pred(3 + i, 3 + i) += q_v;
        P_pred(6 + i, 6 + i) += q_th;
        P_pred(9 + i, 9 + i) += q_ba;
        P_pred(12 + i, 12 + i) += q_bg;
    }
    ekf.P = P_pred;
    ekf.pos_sigma_3 = 3.0 * std::sqrt(ekf.P(0, 0) + ekf.P(1, 1) + ekf.P(2, 2));
    ekf.quat_norm_err = std::abs(ekf.q.norm() - 1.0);
}

bool eskf_update_gnss(EskfState& ekf, const SensorPacket& sens, bool enable_spoof_gate) {
    ekf.spoof_rejected_step = false;
    if (!sens.gnss_received) {
        ekf.dr_active = true;
        return false;
    }

    // 3D position innovation and Mahalanobis chi-squared spoof gate
    Vec3 innov = sens.gnss_pos_ned - ekf.p;
    Mat<3, 3> S_pos{};
    const double r_pos = 14.0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) S_pos(r, c) = ekf.P(r, c);
        S_pos(r, r) += r_pos;
    }

    SpoofGateResult gate = check_gnss_spoof_gate(innov, S_pos, 11.345);
    ekf.nis = gate.mahalanobis_sq;
    if (enable_spoof_gate && !gate.accepted) {
        ekf.spoof_rejected_step = true;
        ekf.spoof_reject_count += 1;
        ekf.dr_active = true;
        return false;
    }

    ekf.dr_active = false;
    for (int axis = 0; axis < 3; ++axis) {
        std::array<double, 15> h{};
        h[axis] = 1.0;
        scalar_joseph_update(ekf, h, sens.gnss_pos_ned[axis] - ekf.p[axis], r_pos);
    }
    for (int axis = 0; axis < 3; ++axis) {
        std::array<double, 15> h{};
        h[3 + axis] = 1.0;
        scalar_joseph_update(ekf, h, sens.gnss_vel_ned[axis] - ekf.v[axis], 0.30);
    }

    // Update ambient wind estimate while GNSS velocity is trusted
    Vec3 air_ned = ekf.q.rotate(Vec3{sens.airspeed_pitot_ms, 0.0, 0.0});
    Vec3 w_obs = ekf.v - air_ned;
    ekf.wind_est = ekf.wind_est * 0.94 + w_obs * 0.06;
    return true;
}

void eskf_update_nhc_airdata(EskfState& ekf, const SensorPacket& sens, bool enable_nhc) {
    // Barometric altitude update (p_z = -alt)
    {
        std::array<double, 15> h{};
        h[2] = -1.0;
        scalar_joseph_update(ekf, h, sens.baro_alt_m - (-ekf.p.z), 1.8);
    }

    // Coordinated-turn gravity tilt & magnetometer heading aiding (PX4 EKF2 style, prevents tilt-to-g blowup)
    {
        Vec3 eul = ekf.q.to_euler();
        double yaw_innov = wrap_angle(sens.mag_yaw_rad - eul.z);
        std::array<double, 15> h_yaw{};
        h_yaw[8] = 1.0;
        scalar_joseph_update(ekf, h_yaw, yaw_innov, 0.008);

        // Centripetal-compensated gravity vector in body frame: g_obs = -(f_b - [0, Va*r, 0])
        double r_meas = sens.gyro_body_meas.z - ekf.bg.z;
        Vec3 a_cent{0.0, sens.airspeed_pitot_ms * r_meas, 0.0};
        Vec3 g_meas_b = (a_cent - (sens.accel_body_meas - ekf.ba)).normalized();
        Vec3 g_pred_b = ekf.q.rotate_inv(Vec3{0.0, 0.0, 1.0});
        // Correct sign: g_meas x g_pred aligns estimated body frame to measured gravity
        Vec3 tilt_err = g_meas_b.cross(g_pred_b);
        std::array<double, 15> h_r{}; h_r[6] = 1.0;
        std::array<double, 15> h_p{}; h_p[7] = 1.0;
        scalar_joseph_update(ekf, h_r, clampd(tilt_err.x, -0.15, 0.15), 0.025);
        scalar_joseph_update(ekf, h_p, clampd(tilt_err.y, -0.15, 0.15), 0.025);
    }

    if (!enable_nhc) return;

    // Wind-aware air-relative velocity in body frame: v_b = R_nb^T * (v_ned - wind_est)
    Mat<3, 3> R_bn = ekf.q.to_rot_matrix().transpose();
    Vec3 v_air_ned = ekf.v - ekf.wind_est;
    Vec3 v_air_body = mat3_vec(R_bn, v_air_ned);

    // Pitot forward airspeed measurement: v_air_body.x = Va_pitot
    {
        std::array<double, 15> h{};
        h[3] = R_bn(0, 0); h[4] = R_bn(0, 1); h[5] = R_bn(0, 2);
        scalar_joseph_update(ekf, h, sens.airspeed_pitot_ms - v_air_body.x, 1.0);
    }
    // Non-holonomic lateral zero-sideslip constraint: v_air_body.y = 0
    {
        std::array<double, 15> h{};
        h[3] = R_bn(1, 0); h[4] = R_bn(1, 1); h[5] = R_bn(1, 2);
        scalar_joseph_update(ekf, h, 0.0 - v_air_body.y, 1.2);
    }
}

void eskf_update_wci_coop(
    EskfState& ekf,
    const Vec3& peer_p_tx,
    const Vec3& peer_v_tx,
    const Mat<3, 3>& peer_P_pos,
    double uwb_range_meas,
    double packet_delay_s
) {
    // Latency-compensated peer position propagation
    Vec3 peer_p = peer_p_tx + peer_v_tx * clampd(packet_delay_s, 0.0, 0.5);
    Vec3 diff = ekf.p - peer_p;
    double est_range = std::max(1.0, diff.norm());
    Vec3 unit_los = diff / est_range;

    // Weighted Covariance Intersection (Tu et al. 2025): conservative non-overconfident fusion
    double tr_self = ekf.P(0, 0) + ekf.P(1, 1) + ekf.P(2, 2);
    double tr_peer = peer_P_pos.trace();
    // Skip cooperative update if peer is not meaningfully better localized during global blackout
    if (ekf.dr_active && tr_peer > 1.35 * tr_self) return;
    double w_ci = clampd(tr_peer / std::max(1e-3, tr_self + tr_peer), 0.30, 0.85);
    double peer_proj_var = unit_los.dot(mat3_vec(peer_P_pos, unit_los));
    double r_eff = (18.0 + 2.5 * peer_proj_var) / (1.0 - w_ci + 0.15);

    // Apply conservative range innovation along 3D line-of-sight vector without collapsing P below peer floor
    std::array<double, 15> h{};
    h[0] = unit_los.x; h[1] = unit_los.y; h[2] = unit_los.z;
    double innov = clampd(uwb_range_meas - est_range, -35.0, 35.0);
    scalar_joseph_update(ekf, h, innov, r_eff);
    double min_var = 0.28 * (peer_P_pos(0, 0) + peer_P_pos(1, 1));
    ekf.P(0, 0) = std::max(ekf.P(0, 0), min_var);
    ekf.P(1, 1) = std::max(ekf.P(1, 1), min_var);
    ekf.pos_sigma_3 = 3.0 * std::sqrt(ekf.P(0, 0) + ekf.P(1, 1) + ekf.P(2, 2));
    ekf.wci_updates += 1;
}

void eskf_update_diagnostics(EskfState& ekf, const UavTrueState& truth) {
    // Diagnostic NEES over 3D position block (3-DOF chi2 95% upper bound = 7.815)
    Vec3 err = truth.pos_ned - ekf.p;
    Mat<3, 3> Pp{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            Pp(r, c) = ekf.P(r, c);
    Mat<3, 3> Pp_inv{};
    if (mat_inv<3>(Pp, Pp_inv)) {
        ekf.nees = err.dot(mat3_vec(Pp_inv, err));
    }
}

}  // namespace swarm
