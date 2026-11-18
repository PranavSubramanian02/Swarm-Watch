#pragma once

// Strict trust-boundary payload parsing and physical bounds sanitization

#include <string>

namespace swarm {

struct SanitizedInjectCmd {
    std::string action;
    int uav_id = 0;
    double eta_roll = 0.55;
    double eta_thrust = 0.60;
    double roll_bias_rad = 0.08;
    double icing_level = 0.5;
    bool flag = false;
    double val1 = 0.0;
    double val2 = 0.0;
    double val3 = 140.0;
    std::string text_param;
    bool valid = false;
};

// Parse and clamp JSON command fields to safe physical bounds
SanitizedInjectCmd sanitize_inject_payload(const std::string& body, int max_uavs = 5);

// Validate static file path against directory traversal ("..")
bool is_safe_static_subpath(const std::string& req_path);

}  // namespace swarm
