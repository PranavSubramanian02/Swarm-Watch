#pragma once

// Unified closed-loop multi-UAV swarm engine, OpenMP ablation runner, and JSON/CSV serializer

#include <array>
#include <deque>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include "guidance_cbf_mdp.hpp"
#include "swarm_coordination.hpp"
#include "swarm_types.hpp"

namespace swarm {

struct SimOptions {
    int n_uavs = 5;
    double dt = 0.1;
    double t_end_s = 220.0;
    bool enable_wci = true;
    bool enable_nhc = true;
    bool enable_spoof_gate = true;
    bool enable_l1_ftc = true;
    bool enable_cbf = true;
    bool enable_master_rotation = true;
    bool enable_ed_cbba = true;
    uint32_t seed = 42;
};

class SwarmEngine {
public:
    SwarmEngine();
    void reset(const SimOptions& opts = SimOptions{});
    void step();

    // Live scenario and fault injection commands (thread-safe)
    void apply_preset_scenario(const std::string& scenario_name);
    void inject_actuator_fault(int uav_id, double eta_roll, double eta_thrust, double roll_bias_rad);
    void inject_icing(int uav_id, double icing_level);
    void set_ew_jamming(bool global_blackout, double jam_n, double jam_e, double jam_r);
    void set_gnss_spoofing(bool active);
    void force_master_rotate();
    void kill_uav(int uav_id);
    void disconnect_uav_comms(int uav_id, bool forced_disconnect);
    void set_wind(double wn, double we, double wd);
    void set_formation(FormationType fmt);
    void add_dynamic_task(double north, double east, double alt_m);
    void set_paused(bool paused);
    bool is_paused() const;

    static std::vector<AblationVariantResult> run_icaav_ablation(int n_runs_per_variant = 100);

    std::string serialize_state_json() const;
    std::string export_csv_log() const;

    void copy_render_snapshot(
        std::array<UavTrueState, kMaxUavs>& fleet_out,
        std::array<EskfState, kMaxUavs>& beliefs_out,
        std::array<DeviationMetrics, kMaxUavs>& devs_out,
        std::array<L1AdaptiveState, kMaxUavs>& l1_out,
        std::array<MdpMode, kMaxUavs>& modes_out,
        std::vector<IsrTask>& tasks_out,
        MasterSlaveState& ms_out,
        RiskFieldConfig& rcfg_out,
        std::array<std::deque<Vec3>, kMaxUavs>& trails_out,
        double& sim_time_out,
        int& n_uavs_out
    ) const;

    double min_inter_uav_sep_m() const { return min_inter_uav_sep_m_; }
    int completed_tasks_count() const;
    double mean_position_rmse_m() const;
    double max_trace_p() const { return max_trace_p_; }
    double soc_spread_pct() const;
    double min_soc() const;
    double nees_in_bounds_pct() const;
    const MasterSlaveState& master_slave_state() const { return ms_; }
    const std::array<EskfState, kMaxUavs>& beliefs() const { return beliefs_; }
    const std::array<UavTrueState, kMaxUavs>& fleet() const { return fleet_; }
    const std::array<L1AdaptiveState, kMaxUavs>& l1_states() const { return l1_; }
    const std::array<MdpMode, kMaxUavs>& modes() const { return modes_; }
    const std::array<DeviationMetrics, kMaxUavs>& deviations() const { return devs_; }
    const std::deque<TacticalAlert>& alerts() const { return alerts_; }
    double sim_time_s() const { return sim_time_s_; }

private:
    void push_alert(AlertSeverity sev, const std::string& code, int uav_id,
                    const std::string& title, const std::string& detail);

    mutable std::mutex mtx_;
    SimOptions opts_{};
    FixedWingParams params_{};
    RiskFieldConfig rcfg_{};
    CommsChannelState comms_{};
    BellmanMdpPolicy mdp_{};
    FormationType formation_ = FormationType::V_WEDGE;
    Vec3 ambient_wind_{3.5, -2.0, 0.0};

    std::array<UavTrueState, kMaxUavs> fleet_{};
    std::array<EskfState, kMaxUavs> beliefs_{};
    std::array<L1AdaptiveState, kMaxUavs> l1_{};
    std::array<MdpMode, kMaxUavs> modes_{};
    std::array<DeviationMetrics, kMaxUavs> devs_{};
    std::array<Vec3, kMaxUavs> leg_starts_{};
    std::array<bool, kMaxUavs> prev_dr_{};
    std::array<bool, kMaxUavs> prev_l1_fault_{};
    std::array<bool, kMaxUavs> prev_connected_{};
    std::array<bool, kMaxUavs> prev_xte_high_{};
    std::array<bool, kMaxUavs> forced_disconnect_{};
    std::array<std::deque<Vec3>, kMaxUavs> trails_{};

    std::vector<IsrTask> tasks_;
    MasterSlaveState ms_{};
    std::deque<TacticalAlert> alerts_;
    std::vector<std::string> csv_rows_;
    bool paused_ = false;

    uint64_t alert_seq_ = 0;
    double sim_time_s_ = 0.0;
    int step_count_ = 0;
    double min_inter_uav_sep_m_ = 9999.0;
    double max_trace_p_ = 0.0;
    double rmse_accum_ = 0.0;
    int rmse_samples_ = 0;
    int nees_total_ = 0;
    int nees_ok_ = 0;
    std::mt19937 rng_{42};
};

}  // namespace swarm
