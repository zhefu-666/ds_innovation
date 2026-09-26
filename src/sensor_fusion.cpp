#include "rescue/sensor_fusion.hpp"

#include <cmath>

namespace rescue {

SensorFusion::SensorFusion(SensorFusionConfig config) : config_(config) {}

void SensorFusion::update(const SensorState &state) {
    if (state.timestamp_us >= state_.timestamp_us) state_ = state;
}

bool SensorFusion::fresh(uint64_t now_us) const {
    return state_.timestamp_us != 0 && now_us >= state_.timestamp_us &&
           now_us - state_.timestamp_us <= static_cast<uint64_t>(config_.stale_after_ms) * 1000;
}

bool SensorFusion::tiltUnsafe() const {
    const float limit_rad = config_.tilt_limit_deg * (3.14159265358979323846f / 180.0f);
    return !state_.imu_valid || std::abs(state_.pitch_rad) > limit_rad ||
           std::abs(state_.roll_rad) > limit_rad;
}

bool SensorFusion::emergencyStop(uint64_t now_us) const {
    if (!fresh(now_us) || tiltUnsafe()) return true;
    return (state_.tof_valid[0] && state_.tof_fl_m > 0.0f && state_.tof_fl_m <= config_.tof_stop_distance_m) ||
           (state_.tof_valid[1] && state_.tof_fr_m > 0.0f && state_.tof_fr_m <= config_.tof_stop_distance_m) ||
           (state_.tof_valid[2] && state_.tof_rl_m > 0.0f && state_.tof_rl_m <= config_.tof_stop_distance_m) ||
           (state_.tof_valid[3] && state_.tof_rr_m > 0.0f && state_.tof_rr_m <= config_.tof_stop_distance_m);
}

MotionCommand SensorFusion::protect(const MotionCommand &requested, uint64_t now_us) {
    MotionCommand command = requested;


    if (emergencyStop(now_us)) {

        command.vx_mps = 0.0f;
        command.wz_rps = 0.0f;

    }
    return command;
}

} // namespace rescue
