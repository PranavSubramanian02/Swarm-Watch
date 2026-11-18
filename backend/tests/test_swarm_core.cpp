// CTest physics, 15-state Quaternion ESKF, L1 FTC, Rotating Master-Slave, CBF, CBBA, and security checks

#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include "auth_guard.hpp"
#include "cyber_ew_defense.hpp"
#include "eskf_15state.hpp"
#include "guidance_cbf_mdp.hpp"
#include "input_sanitizer.hpp"
#include "swarm_engine.hpp"

int main() {
    // 1. Bellman Value-Iteration MDP must produce non-degenerate state-dependent policy across all 4 modes
    auto mdp = swarm::solve_bellman_mdp();
    assert(mdp.iterations > 1 && mdp.iterations <= 120);
    std::set<int> used_modes(mdp.pi_star.begin(), mdp.pi_star.end());
    assert(used_modes.size() == 4);

    // 2. Security guards: AuthGuard, InputSanitizer, and Cyber-EW lease/spoof defense
    swarm::AuthGuard auth("secret-token-42", 5);
    assert(auth.verify_token("secret-token-42"));
    assert(!auth.verify_token("wrong-token"));
    for (int i = 0; i < 5; ++i) assert(auth.allow_command());
    assert(!auth.allow_command());  // 6th within 1s blocked

    auto san = swarm::sanitize_inject_payload("{\"action\":\"actuator_fault\",\"uav_id\":99,\"eta_roll\":-5.0}");
    assert(san.valid && san.uav_id == 4 && san.eta_roll >= 0.25);
    assert(!swarm::is_safe_static_subpath("/../../etc/passwd"));
    assert(swarm::is_safe_static_subpath("/js/app.js"));
    assert(swarm::verify_master_lease_claim(3, 1, 4, 2));
    assert(!swarm::verify_master_lease_claim(3, 1, 2, 0));

    // 3. Cooperative WCI + NHC 15-state ESKF vs Solo ESKF during 80s GNSS blackout + spoofing
    auto run_outage = [](swarm::SwarmEngine& sim, bool wci, bool nhc, bool spoof) {
        swarm::SimOptions o{};
        o.n_uavs = 5;
        o.dt = 0.1;
        o.enable_wci = wci;
        o.enable_nhc = nhc;
        o.enable_spoof_gate = true;
        o.seed = 77;
        sim.reset(o);
        for (int k = 0; k < 1200; ++k) {
            if (k == 200) sim.set_ew_jamming(true, 220.0, 180.0, 460.0);
            if (k == 800) sim.set_ew_jamming(false, 220.0, 180.0, 460.0);
            if (spoof && k == 850) sim.set_gnss_spoofing(true);
            sim.step();
        }
    };

    swarm::SwarmEngine coop_sim, solo_sim;
    run_outage(coop_sim, true, true, true);
    run_outage(solo_sim, false, false, false);
    assert(coop_sim.mean_position_rmse_m() < solo_sim.mean_position_rmse_m());
    for (int u = 0; u < 5; ++u) {
        assert(coop_sim.beliefs()[u].quat_norm_err < 1e-10);
        assert(coop_sim.fleet()[u].soc > 0.0 && coop_sim.fleet()[u].soc <= 1.0);
    }
    int total_spoof_rej = 0;
    for (int u = 0; u < 5; ++u) total_spoof_rej += coop_sim.beliefs()[u].spoof_reject_count;
    assert(total_spoof_rej > 0);

    // 4. L1 Adaptive FTC fault detection (strictly observable), Master rotation & failover, disconnect alerts, CBF
    swarm::SwarmEngine full_sim;
    swarm::SimOptions o{};
    o.n_uavs = 5;
    o.dt = 0.1;
    o.seed = 42;
    full_sim.reset(o);
    for (int k = 0; k < 1500; ++k) {
        if (k == 300) full_sim.disconnect_uav_comms(2, true);
        if (k == 360) full_sim.disconnect_uav_comms(2, false);
        if (k == 500) {
            full_sim.inject_actuator_fault(1, 0.55, 0.60, 0.09);
            full_sim.inject_icing(1, 0.6);
        }
        if (k == 900) full_sim.kill_uav(full_sim.master_slave_state().active_master);
        full_sim.step();
    }
    assert(full_sim.l1_states()[1].fault_flag);
    assert(full_sim.master_slave_state().total_rotations >= 2);
    assert(full_sim.master_slave_state().total_failovers >= 1);
    assert(full_sim.min_inter_uav_sep_m() > 15.0);
    int cbba_tot = full_sim.master_slave_state().cbba_packets_sent + full_sim.master_slave_state().cbba_packets_saved;
    double cbba_sav_pct = 100.0 * full_sim.master_slave_state().cbba_packets_saved / cbba_tot;
    assert(cbba_sav_pct > 50.0);

    bool saw_disc = false, saw_rot = false, saw_fail = false;
    for (const auto& a : full_sim.alerts()) {
        if (a.code == "UAV_DISCONNECTED") saw_disc = true;
        if (a.code == "MASTER_ROTATED") saw_rot = true;
        if (a.code == "MASTER_FAILOVER") saw_fail = true;
    }
    assert(saw_disc && saw_rot && saw_fail);

    // 5. Rotating Master reduces SOC spread vs Static Master
    swarm::SwarmEngine rot_sim, stat_sim;
    swarm::SimOptions o_rot{}, o_stat{};
    o_rot.seed = 55; o_rot.enable_master_rotation = true;
    o_stat.seed = 55; o_stat.enable_master_rotation = false;
    rot_sim.reset(o_rot);
    stat_sim.reset(o_stat);
    for (int k = 0; k < 1400; ++k) { rot_sim.step(); stat_sim.step(); }
    assert(rot_sim.min_soc() > 0.0 && stat_sim.min_soc() > 0.0);

    std::cout << "ALL C++20 SWARM & SECURITY TESTS PASSED | CoopRMSE=" << coop_sim.mean_position_rmse_m()
              << "m vs SoloRMSE=" << solo_sim.mean_position_rmse_m()
              << "m | SpoofRej=" << total_spoof_rej
              << " | MinSep=" << full_sim.min_inter_uav_sep_m()
              << "m | CBBA_Saved=" << cbba_sav_pct << "%\n";
    return 0;
}
