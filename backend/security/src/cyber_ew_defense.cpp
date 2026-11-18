// Cyber-EW defense: Mahalanobis GNSS spoof rejection and monotonic TDMA lease verification

#include "cyber_ew_defense.hpp"

namespace swarm {

SpoofGateResult check_gnss_spoof_gate(const Vec3& innovation, const Mat<3, 3>& S_cov, double chi2_thresh) {
    SpoofGateResult res{};
    Mat<3, 3> S_inv{};
    if (!mat_inv<3>(S_cov, S_inv)) {
        res.accepted = false;
        res.mahalanobis_sq = 1e6;
        return res;
    }
    Vec3 tmp = mat3_vec(S_inv, innovation);
    res.mahalanobis_sq = innovation.dot(tmp);
    res.accepted = (res.mahalanobis_sq <= chi2_thresh);
    return res;
}

bool verify_master_lease_claim(int current_epoch, int current_master, int claimed_epoch, int claimed_master) {
    if (claimed_epoch > current_epoch) return true;
    if (claimed_epoch == current_epoch && claimed_master < current_master) return true;
    return false;
}

}  // namespace swarm
