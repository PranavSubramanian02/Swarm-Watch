#pragma once

// 15-State Hamilton Quaternion Error-State Kalman Filter (Sola 2017) + NHC + Spoof Gate + WCI

#include "swarm_types.hpp"

namespace swarm {

void init_eskf(EskfState& ekf, const Vec3& p0, const Vec3& v0, const Quat& q0);
void eskf_predict(EskfState& ekf, const SensorPacket& sens, double dt);
bool eskf_update_gnss(EskfState& ekf, const SensorPacket& sens, bool enable_spoof_gate);
void eskf_update_nhc_airdata(EskfState& ekf, const SensorPacket& sens, bool enable_nhc);

void eskf_update_wci_coop(
    EskfState& ekf,
    const Vec3& peer_p_tx,
    const Vec3& peer_v_tx,
    const Mat<3, 3>& peer_P_pos,
    double uwb_range_meas,
    double packet_delay_s
);

void eskf_update_diagnostics(EskfState& ekf, const UavTrueState& truth);

}  // namespace swarm
