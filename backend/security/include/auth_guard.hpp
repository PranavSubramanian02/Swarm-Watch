#pragma once

// LAN shared-secret token verification and command rate limiting

#include <chrono>
#include <deque>
#include <mutex>
#include <string>

namespace swarm {

class AuthGuard {
public:
    explicit AuthGuard(std::string expected_token = "", int max_cmd_per_sec = 25);

    // Returns true when no token is configured or token matches constant-time comparison
    bool verify_token(const std::string& provided) const;

    // Sliding-window command rate limiter preventing flood attacks
    bool allow_command();

    const std::string& token() const { return expected_token_; }

private:
    std::string expected_token_;
    int max_cmd_per_sec_;
    mutable std::mutex mtx_;
    std::deque<std::chrono::steady_clock::time_point> timestamps_;
};

}  // namespace swarm
