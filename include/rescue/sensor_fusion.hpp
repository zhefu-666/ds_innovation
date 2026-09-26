#pragma once

#include "rescue/types.hpp"

#include <cstdint>

namespace rescue {

struct SensorFusionConfig {
    uint32_t stale_after_ms = 200;
    float tof_stop_distance_m = 0.18f;
    float tilt_limit_deg = 12.0f;
};

class SensorFusion {
public:
    explicit SensorFusion(SensorFusionConfig config = {});
    void update(const SensorState &state);
    const SensorState &state() const { return state_; }
    bool fresh(uint64_t now_us) const;
    bool emergencyStop(uint64_t now_us) const;
    bool tiltUnsafe() const;
    MotionCommand protect(const MotionCommand &requested, uint64_t now_us);

private:
    SensorFusionConfig config_;
    SensorState state_;

};

} // namespace rescue
