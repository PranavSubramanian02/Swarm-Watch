#pragma once

// 6-DOF non-holonomic fixed-wing flight mechanics, Dryden turbulence, wake vortices, and 1-RC ECM

#include <array>
#include <random>
#include "swarm_types.hpp"

namespace swarm {

struct RiskEval {
    double value = 0.0;
    double grad_n = 0.0;
    double grad_e = 0.0;
    double hess_nn = 0.0;
    double hess_ee = 0.0;
};

RiskEval evaluate_risk_field(const RiskFieldConfig& cfg, double north, double east);
double isa_air_density(double alt_m);
double compute_stall_speed(const FixedWingParams& p, double bank_rad, double icing_level);

double compute_wake_vortex_disturbance(
    int self_idx,
    const std::array<UavTrueState, kMaxUavs>& fleet,
    int n_uavs
);

void step_fixed_wing_plant(
    UavTrueState& st,
    const ControlCommand& cmd,
    const FixedWingParams& p,
    const Vec3& ambient_wind_ned,
    double wake_roll_moment,
    bool is_master,
    double dt,
    std::mt19937& rng
);

SensorPacket sample_sensors(
    const UavTrueState& st,
    const FixedWingParams& p,
    const RiskFieldConfig& rcfg,
    const Vec3& ambient_wind_ned,
    double sim_time_s,
    double dt,
    std::mt19937& rng
);

}  // namespace swarm
