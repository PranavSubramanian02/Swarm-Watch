#pragma once

// Cyber-EW defense: Mahalanobis GNSS spoof gating and monotonic TDMA epoch anti-hijack verification

#include "math_types.hpp"

namespace swarm {

struct SpoofGateResult {
    bool accepted = true;
    double mahalanobis_sq = 0.0;
};

// 3-DOF chi-squared innovation test (chi2_{3, 0.99} = 11.345)
SpoofGateResult check_gnss_spoof_gate(const Vec3& innovation, const Mat<3, 3>& S_cov, double chi2_thresh = 11.345);

// Monotonic epoch-lease verification preventing split-brain or replayed Master claims
bool verify_master_lease_claim(int current_epoch, int current_master, int claimed_epoch, int claimed_master);

}  // namespace swarm
