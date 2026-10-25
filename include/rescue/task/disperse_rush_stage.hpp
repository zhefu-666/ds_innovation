#pragma once

#include <cstdint>
#include <opencv2/core/types.hpp>

namespace rescue {
struct TaskTuning;
struct PushObservation;

struct DisperseRushContext {
    uint64_t now_us = 0, sweep_start_us = 0;
    float travel_m = 0, sweep_travel_m = 0, motion_ratio = 1;
    float rush_distance_m = 0, cluster_range_m = 0;
    bool scatter_state = false, burst_swing = false;
    bool far_judge = false, pitch_ready = false, clutter_fresh = false;
    bool has_target = false;
    cv::Point2f target_body{0.f, 0.f};
};

enum class DisperseRushOutcome {
    Continue,
    Complete
};

struct DisperseRushResult {
    DisperseRushOutcome outcome = DisperseRushOutcome::Continue;
    bool motion_requested = false;
    float vx = 0;
    const char* reason = nullptr;
    bool plan_event = false;
    int outside_n = 0;
    float cap_positive = 99.f, cap_negative = 99.f;
    float first_sign = 0;
};

class DisperseRushStage {
  public:
    void beginSweep(bool pending);
    void clear();
    void clearFlags() {
        pending_ = active_ = false;
    }
    bool planActive() const {
        return active_;
    }
    bool planPending() const {
        return pending_;
    }
    float capPositive() const {
        return cap_positive_;
    }
    float capNegative() const {
        return cap_negative_;
    }
    // The parent applies the plan, the swing transition and every motion permission check.
    DisperseRushResult
    update(const TaskTuning&, const PushObservation&, const DisperseRushContext&);

  private:
    bool pending_ = false, active_ = false;
    float cap_positive_ = 99.f, cap_negative_ = 99.f;
};
} // namespace rescue
