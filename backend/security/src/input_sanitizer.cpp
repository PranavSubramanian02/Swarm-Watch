// Trust-boundary JSON payload extraction and physical range clamping

#include "input_sanitizer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace swarm {

namespace {

std::string extract_json_str(const std::string& body, const std::string& key) {
    std::string pat = "\"" + key + "\"";
    auto pos = body.find(pat);
    if (pos == std::string::npos) return "";
    pos = body.find(':', pos + pat.size());
    if (pos == std::string::npos) return "";
    pos = body.find('"', pos + 1);
    if (pos == std::string::npos) return "";
    auto end = body.find('"', pos + 1);
    if (end == std::string::npos) return "";
    return body.substr(pos + 1, end - pos - 1);
}

double extract_json_num(const std::string& body, const std::string& key, double fallback) {
    std::string pat = "\"" + key + "\"";
    auto pos = body.find(pat);
    if (pos == std::string::npos) return fallback;
    pos = body.find(':', pos + pat.size());
    if (pos == std::string::npos) return fallback;
    ++pos;
    while (pos < body.size() && (body[pos] == ' ' || body[pos] == '\t')) ++pos;
    char* end_ptr = nullptr;
    double val = std::strtod(body.c_str() + pos, &end_ptr);
    if (end_ptr == body.c_str() + pos || !std::isfinite(val)) return fallback;
    return val;
}

bool extract_json_bool(const std::string& body, const std::string& key, bool fallback) {
    std::string pat = "\"" + key + "\"";
    auto pos = body.find(pat);
    if (pos == std::string::npos) return fallback;
    pos = body.find(':', pos + pat.size());
    if (pos == std::string::npos) return fallback;
    auto sub = body.substr(pos + 1, 12);
    if (sub.find("true") != std::string::npos) return true;
    if (sub.find("false") != std::string::npos) return false;
    return fallback;
}

}  // namespace

SanitizedInjectCmd sanitize_inject_payload(const std::string& body, int max_uavs) {
    SanitizedInjectCmd cmd{};
    if (body.size() > 4096) return cmd;

    cmd.action = extract_json_str(body, "action");
    if (cmd.action.empty()) return cmd;

    int uid = static_cast<int>(std::lround(extract_json_num(body, "uav_id", 0.0)));
    cmd.uav_id = std::max(0, std::min(max_uavs - 1, uid));
    cmd.eta_roll = std::clamp(extract_json_num(body, "eta_roll", 0.55), 0.25, 1.0);
    cmd.eta_thrust = std::clamp(extract_json_num(body, "eta_thrust", 0.60), 0.30, 1.0);
    cmd.roll_bias_rad = std::clamp(extract_json_num(body, "roll_bias_rad", 0.08), -0.25, 0.25);
    cmd.icing_level = std::clamp(extract_json_num(body, "icing_level", 0.6), 0.0, 1.0);
    cmd.flag = extract_json_bool(body, "flag", true);
    cmd.val1 = std::clamp(extract_json_num(body, "val1", 0.0), -1450.0, 1450.0);
    cmd.val2 = std::clamp(extract_json_num(body, "val2", 0.0), -1450.0, 1450.0);
    cmd.val3 = std::clamp(extract_json_num(body, "val3", 140.0), 60.0, 500.0);
    cmd.text_param = extract_json_str(body, "text");
    cmd.valid = true;
    return cmd;
}

bool is_safe_static_subpath(const std::string& req_path) {
    if (req_path.empty() || req_path[0] != '/') return false;
    if (req_path.find("..") != std::string::npos) return false;
    if (req_path.find('\\') != std::string::npos) return false;
    if (req_path.find('\0') != std::string::npos) return false;
    return true;
}

}  // namespace swarm
