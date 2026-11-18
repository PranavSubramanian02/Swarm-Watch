#pragma once

// L1 Adaptive Fault-Tolerant Autopilot with Pseudo-Control Hedging and TECS coupling
// Separation Principle: accepts strictly observable ESKF estimates and sensors (no UavTrueState)

#include "swarm_types.hpp"

namespace swarm {

void init_l1_adaptive(L1AdaptiveState& l1, double v_cruise_ms);

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
);

}  // namespace swarm
