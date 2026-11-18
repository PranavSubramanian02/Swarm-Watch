// Headless C++20 swarm HTTP/WebSocket server and offline ICAAV-2026 ablation CSV/JSON exporter

#include "http_ws_server.hpp"
#include "swarm_engine.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_keep_running{true};
void on_signal(int) { g_keep_running = false; }
}  // namespace

int main(int argc, char** argv) {
    std::string web_dir = "frontend/web_console";
    int port = 8080;
    std::string token;
    std::string export_dir;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--web-dir" && i + 1 < argc) web_dir = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
        else if (arg == "--token" && i + 1 < argc) token = argv[++i];
        else if (arg == "--export-ablation" && i + 1 < argc) export_dir = argv[++i];
    }

    // Offline CLI mode: export full contested mission telemetry CSV + OpenMP 100-run ablation JSON
    if (!export_dir.empty()) {
        std::filesystem::create_directories(export_dir);

        // 1. Contested benchmark sortie (GNSS blackout 50-150s, spoof at 155s, actuator LoE at 120s, disconnect at 90s)
        swarm::SwarmEngine sim;
        swarm::SimOptions opts{};
        opts.n_uavs = 5;
        opts.dt = 0.1;
        opts.seed = 42;
        sim.reset(opts);
        for (int k = 0; k < 2000; ++k) {
            if (k == 500) sim.set_ew_jamming(true, 220.0, 180.0, 460.0);
            if (k == 900) sim.disconnect_uav_comms(2, true);
            if (k == 1050) sim.disconnect_uav_comms(2, false);
            if (k == 1200) {
                sim.inject_actuator_fault(1, 0.55, 0.60, 0.08);
                sim.inject_icing(1, 0.65);
            }
            if (k == 1450) sim.set_ew_jamming(false, 220.0, 180.0, 460.0);
            if (k == 1550) sim.set_gnss_spoofing(true);
            if (k == 1700) sim.set_gnss_spoofing(false);
            sim.step();
        }
        std::ofstream fcsv(export_dir + "/contested_sortie.csv");
        fcsv << sim.export_csv_log();
        fcsv.close();

        // 2. Static Master-Slave reference sortie for direct SOC-balance comparison
        swarm::SwarmEngine static_sim;
        opts.enable_master_rotation = false;
        static_sim.reset(opts);
        for (int k = 0; k < 2000; ++k) {
            if (k == 500) static_sim.set_ew_jamming(true, 220.0, 180.0, 460.0);
            if (k == 1450) static_sim.set_ew_jamming(false, 220.0, 180.0, 460.0);
            static_sim.step();
        }
        std::ofstream fstat(export_dir + "/static_master_sortie.csv");
        fstat << static_sim.export_csv_log();
        fstat.close();

        // 3. Solo ESKF (no WCI / no NHC) reference sortie
        swarm::SwarmEngine solo_sim;
        opts.enable_master_rotation = true;
        opts.enable_wci = false;
        opts.enable_nhc = false;
        solo_sim.reset(opts);
        for (int k = 0; k < 2000; ++k) {
            if (k == 500) solo_sim.set_ew_jamming(true, 220.0, 180.0, 460.0);
            if (k == 1450) solo_sim.set_ew_jamming(false, 220.0, 180.0, 460.0);
            solo_sim.step();
        }
        std::ofstream fsolo(export_dir + "/solo_eskf_sortie.csv");
        fsolo << solo_sim.export_csv_log();
        fsolo.close();

        // 4. OpenMP multi-run ICAAV ablation matrix
        auto abl = swarm::SwarmEngine::run_icaav_ablation(100);
        std::ofstream fj(export_dir + "/ablation_results.json");
        fj << std::fixed << std::setprecision(3) << "{\"variants\":[\n";
        for (size_t i = 0; i < abl.size(); ++i) {
            const auto& v = abl[i];
            fj << "  {\"label\":\"" << v.label << "\",\"n_runs\":" << v.n_runs
               << ",\"rmse_m\":" << v.mean_rmse_m << ",\"peak_trace_p\":" << v.peak_trace_p
               << ",\"nees_ok_pct\":" << v.nees_in_bounds_pct << ",\"completion_pct\":" << v.mission_completion_pct
               << ",\"collision_pct\":" << v.collision_rate_pct << ",\"soc_spread_pct\":" << v.soc_spread_pct
               << ",\"min_soc_pct\":" << v.min_soc_pct << ",\"cbba_savings_pct\":" << v.cbba_packet_savings_pct
               << ",\"mean_xte_m\":" << v.mean_xte_m << "}" << (i + 1 < abl.size() ? "," : "") << "\n";
        }
        fj << "]}\n";
        fj.close();

        std::cout << "Exported contested CSVs and 100-run OpenMP ablation JSON to " << export_dir << "\n";
        return 0;
    }

    // Live server mode: 10 Hz physics stepped at 2x realtime (20 Hz WS broadcast)
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    swarm::SwarmEngine engine;
    swarm::HttpWsServer server(engine, web_dir, port, token);
    if (!server.start()) {
        std::cerr << "Failed to bind port " << port << "\n";
        return 1;
    }
    std::cout << "C++20 Swarm Server running on http://0.0.0.0:" << port
              << " | WS: ws://0.0.0.0:" << port << "/ws/swarm\n";

    while (g_keep_running) {
        engine.step();
        server.broadcast_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    server.stop();
    return 0;
}
