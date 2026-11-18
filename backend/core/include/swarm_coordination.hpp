#pragma once

// Time-Based Rotating Master-Slave, Multi-Hop RF/Gilbert-Elliott Comms, Fiedler lambda2, and Event-Driven CBBA

#include <array>
#include <random>
#include <string>
#include <vector>
#include "swarm_types.hpp"

namespace swarm {

struct CommsChannelState {
    double max_range_m = 1650.0;
    double base_loss_prob = 0.14;
    bool burst_jam_state = false;
    std::array<std::array<double, kMaxUavs>, kMaxUavs> packet_delay_s{};
};

void update_comms_topology(
    MasterSlaveState& ms,
    CommsChannelState& ch,
    const std::array<UavTrueState, kMaxUavs>& fleet,
    const RiskFieldConfig& rcfg,
    int n_uavs,
    std::mt19937& rng
);

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
);

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
);

Vec3 compute_formation_offset(FormationType fmt, int slot_idx, int active_count, double master_yaw);

}  // namespace swarm
