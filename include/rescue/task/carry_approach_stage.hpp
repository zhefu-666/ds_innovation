#pragma once

#include <opencv2/core/types.hpp>
#include <cstdint>

namespace rescue {
struct TaskTuning;

// Everything the assumed-drop approach reads from the parent's anchored zone and sensors.
struct CarryApproachContext {
    uint64_t now_us = 0;
    // Body-frame hold centre lifted into the zone frame.
    cv::Point2f hold;
    // Zone-frame lookahead point already turned into the body frame.
    cv::Point2f carrot;
    bool fresh_fix = false;
    uint64_t imu_blocked_us = 0;
    float hold_center = 0;
};

enum class CarryApproachOutcome {
    Continue,
    RaiseRelease
};

struct CarryApproachResult {
    CarryApproachOutcome outcome = CarryApproachOutcome::Continue;
    const char* reason = nullptr;
    // The parent records the release note under this event name before transitioning.
    const char* drop_event = nullptr;
    bool motion = false;
    // True: turn while driving; false: let the parent's steer() decide in-place versus driving.
    bool blend = false;
    float heading = 0;
};

// The assumed-carry visual loop: follow the zone axis toward the front edge until the load
// reaches the release gap, stalls, or the IMU reports the rim. Every permission and transition
// stays parent-owned.
class CarryApproachStage {
  public:
    void reset();
    CarryApproachResult update(const TaskTuning&, const CarryApproachContext&);
    // The capture hand-off either resets or preserves this; tests pin which.
    uint64_t progressUs() const {
        return progress_us_;
    }
    void seedProgressUs(uint64_t us) {
        progress_us_ = us;
    }

  private:
    // Exclusive: no other state reads or writes these.
    uint64_t progress_us_ = 0;
    float best_y_ = 0;
};
} // namespace rescue
