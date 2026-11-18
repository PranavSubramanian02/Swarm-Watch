#pragma once

// Core state, estimator, adaptive FTC, Master-Slave, CBBA, and tactical alert POD structs

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "math_types.hpp"

namespace swarm {

constexpr int kMaxUavs = 5;
constexpr int kMaxTasks = 8;
constexpr int kMaxAlerts = 128;
constexpr int kMaxTrail = 180;

enum class MdpMode : int {
    NOMINAL = 0,
    DEGRADED_COOP = 1,
    CONSERVATIVE_RISK = 2,
    EMERGENCY_RTB = 3
};

enum class FormationType : int {
    V_WEDGE = 0,
    ECHELON_RIGHT = 1,
    DIAMOND = 2,
    WIDE_ISR = 3
};

enum class AlertSeverity : int {
    INFO = 0,
    CAUTION = 1,
    CRITICAL = 2
};

struct HazardPeak {
    double north = 0.0;
    double east = 0.0;
    double amplitude = 5.0;
    double sigma = 260.0;
};

struct RiskFieldConfig {
    double arena_half_m = 1500.0;
    double base_quad_coeff = 2.2e-6;
    std::vector<HazardPeak> peaks;
    double jam_north = 220.0;
    double jam_east = 180.0;
    double jam_radius_m = 460.0;
    bool global_gnss_blackout = false;
    bool gnss_spoof_active = false;
    double spoof_ramp_mps = 4.2;
    double spoof_start_s = -1.0;
};

struct FixedWingParams {
    double mass_kg = 12.0;
    double wing_area_m2 = 0.95;
    double wingspan_m = 2.6;
    double cd0 = 0.028;
    double induced_k = 0.044;
    double cl_max = 1.28;
    double v_min_ms = 14.0;
    double v_cruise_ms = 19.0;
    double v_max_ms = 26.0;
    double max_bank_rad = 38.0 * kPi / 180.0;
    double max_climb_rad = 15.0 * kPi / 180.0;
    double max_thrust_n = 42.0;
    double tau_actuator_s = 0.14;
    double max_aileron_rad = 0.42;
    double max_elevator_rad = 0.35;
    double max_rate_rad_s = 2.6;
    Vec3 inertia_diag{0.42, 0.58, 0.92};
    double capacity_ah = 16.0;
    double r0_ohm = 0.075;
    double r1_ohm = 0.028;
    double c1_f = 620.0;
    double v_cutoff_v = 18.8;
    double soc_reserve = 0.22;
    double hotel_power_w = 24.0;
    double master_rf_power_w = 18.0;
};

// Ground-truth physical UAV state (never passed into L1 FTC or estimator)
struct UavTrueState {
    Vec3 pos_ned{0.0, 0.0, -140.0};
    Vec3 vel_ned{19.0, 0.0, 0.0};
    Vec3 accel_ned_true{0.0, 0.0, 0.0};
    Quat q_nb{1.0, 0.0, 0.0, 0.0};
    Vec3 omega_body{0.0, 0.0, 0.0};
    Vec3 omega_eff_body{0.0, 0.0, 0.0};
    double act_roll = 0.0;
    double act_pitch = 0.0;
    double act_yaw = 0.0;
    double act_throttle = 0.48;
    Vec3 true_ba{0.04, -0.03, 0.05};
    Vec3 true_bg{0.004, -0.003, 0.005};
    double dryden_u = 0.0;
    double dryden_v = 0.0;
    double dryden_w = 0.0;
    double soc = 1.0;
    double vp_v = 0.0;
    double v_term_v = 24.8;
    double power_w = 180.0;
    double eta_roll = 1.0;
    double eta_thrust = 1.0;
    double roll_bias_rad = 0.0;
    double icing_level = 0.0;
    bool alive = true;
    double clock_skew_s = 0.0;
};

// Observable onboard sensor packet produced by plant
struct SensorPacket {
    Vec3 accel_body_meas{0.0, 0.0, -kGravity};
    Vec3 gyro_body_meas{0.0, 0.0, 0.0};
    double airspeed_pitot_ms = 19.0;
    double baro_alt_m = 140.0;
    double mag_yaw_rad = 0.0;
    Vec3 gnss_pos_ned{0.0, 0.0, -140.0};
    Vec3 gnss_vel_ned{19.0, 0.0, 0.0};
    bool gnss_received = true;
    double v_term_meas = 24.8;
    double soc_est = 1.0;
};

// 16D nominal state + 15x15 tangent-space error covariance (Sola 2017)
struct EskfState {
    Vec3 p{0.0, 0.0, -140.0};
    Vec3 v{19.0, 0.0, 0.0};
    Quat q{1.0, 0.0, 0.0, 0.0};
    Vec3 ba{0.0, 0.0, 0.0};
    Vec3 bg{0.0, 0.0, 0.0};
    Mat<15, 15> P = Mat<15, 15>::identity();
    Vec3 wind_est{0.0, 0.0, 0.0};
    double nees = 0.0;
    double nis = 0.0;
    double pos_sigma_3 = 0.0;
    double quat_norm_err = 0.0;
    bool dr_active = false;
    bool spoof_rejected_step = false;
    int spoof_reject_count = 0;
    int wci_updates = 0;
};

// L1 Adaptive FTC state (strictly observable inputs only)
struct L1AdaptiveState {
    double roll_hat = 0.0;
    double p_hat = 0.0;
    double gamma_hat = 0.0;
    double va_hat = 19.0;
    double omega_roll_hat = 1.0;
    double omega_thrust_hat = 1.0;
    double sigma_roll_hat = 0.0;
    double sigma_thrust_hat = 0.0;
    double u_roll_filt = 0.0;
    double u_thrust_filt = 0.48;
    bool fault_flag = false;
    double cusum_stat = 0.0;
    double fault_detect_time_s = -1.0;
};

struct ControlCommand {
    double aileron_cmd = 0.0;
    double elevator_cmd = 0.0;
    double rudder_cmd = 0.0;
    double throttle_cmd = 0.48;
    double bank_cmd_rad = 0.0;
    double climb_cmd_rad = 0.0;
    double va_cmd_ms = 19.0;
};

struct IsrTask {
    int id = 0;
    Vec3 pos_ned{0.0, 0.0, -140.0};
    double loiter_radius_m = 70.0;
    double dwell_required_s = 14.0;
    double dwell_accum_s = 0.0;
    double priority = 1.0;
    int assigned_uav = -1;
    bool completed = false;
};

struct DeviationMetrics {
    Vec3 ideal_pos_ned{0.0, 0.0, -140.0};
    double xte_m = 0.0;
    double ate_m = 0.0;
    double alt_dev_m = 0.0;
    double drift_rate_mps = 0.0;
    double tthi_s = 999.0;
    std::array<Vec3, 5> drift_cone_pts{};
    std::array<double, 5> drift_cone_rad{};
};

struct MasterSlaveState {
    int active_master = 0;
    int next_master = 1;
    int epoch_id = 1;
    double epoch_duration_s = 35.0;
    double epoch_elapsed_s = 0.0;
    bool rotation_enabled = true;
    std::array<double, kMaxUavs> health_scores{};
    std::array<int, kMaxUavs> missed_heartbeats{};
    std::array<bool, kMaxUavs> connected_to_swarm{};
    int total_rotations = 0;
    int total_failovers = 0;
    double last_failover_latency_s = 0.0;
    double fiedler_lambda2 = 0.0;
    double avg_link_latency_ms = 25.0;
    int relay_hops_used = 0;
    int cbba_packets_sent = 0;
    int cbba_packets_saved = 0;
    std::array<std::array<int, kMaxUavs>, kMaxUavs> link_hop_matrix{};
};

struct TacticalAlert {
    uint64_t seq = 0;
    double sim_time_s = 0.0;
    AlertSeverity severity = AlertSeverity::INFO;
    std::string code;
    int uav_id = -1;
    std::string title;
    std::string detail;
};

struct AblationVariantResult {
    std::string label;
    int n_runs = 0;
    double mean_rmse_m = 0.0;
    double peak_trace_p = 0.0;
    double nees_in_bounds_pct = 0.0;
    double mission_completion_pct = 0.0;
    double collision_rate_pct = 0.0;
    double soc_spread_pct = 0.0;
    double min_soc_pct = 0.0;
    double mean_failover_ms = 0.0;
    double cbba_packet_savings_pct = 0.0;
    double mean_xte_m = 0.0;
};

}  // namespace swarm
