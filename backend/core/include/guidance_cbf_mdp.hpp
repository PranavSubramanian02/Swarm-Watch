#pragma once

// Park-Deyst-How L1 guidance, wind-compensated loiter orbit, 3D CBF-QP, Bellman MDP, and deviation predictor

#include <array>
#include "swarm_types.hpp"

namespace swarm {

struct BellmanMdpPolicy {
    std::array<int, 36> pi_star{};
    std::array<double, 36> v_star{};
    int iterations = 0;
};

BellmanMdpPolicy solve_bellman_mdp();
MdpMode query_bellman_mode(const BellmanMdpPolicy& mdp, double pos_sigma_3, bool gps_ok, double soc, bool fault_active);

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
);

DeviationMetrics compute_deviation_and_drift(
    const EskfState& est,
    const Vec3& start_ned,
    const Vec3& goal_ned,
    bool loiter_mode,
    double loiter_radius_m,
    const RiskFieldConfig& rcfg
);

}  // namespace swarm
