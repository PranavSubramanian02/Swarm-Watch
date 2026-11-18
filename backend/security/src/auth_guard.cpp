// Constant-time LAN token authentication and sliding-window command rate limiter

#include "auth_guard.hpp"

namespace swarm {

AuthGuard::AuthGuard(std::string expected_token, int max_cmd_per_sec)
    : expected_token_(std::move(expected_token)), max_cmd_per_sec_(max_cmd_per_sec) {}

bool AuthGuard::verify_token(const std::string& provided) const {
    if (expected_token_.empty()) return true;
    if (provided.size() != expected_token_.size()) return false;
    unsigned char diff = 0;
    for (size_t i = 0; i < expected_token_.size(); ++i) {
        diff |= static_cast<unsigned char>(provided[i] ^ expected_token_[i]);
    }
    return diff == 0;
}

bool AuthGuard::allow_command() {
    std::lock_guard<std::mutex> lock(mtx_);
    auto now = std::chrono::steady_clock::now();
    while (!timestamps_.empty() &&
           std::chrono::duration<double>(now - timestamps_.front()).count() > 1.0) {
        timestamps_.pop_front();
    }
    if (static_cast<int>(timestamps_.size()) >= max_cmd_per_sec_) return false;
    timestamps_.push_back(now);
    return true;
}

}  // namespace swarm
