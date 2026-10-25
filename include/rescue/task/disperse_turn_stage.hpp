#pragma once

#include <cstdint>

namespace rescue {
struct TaskTuning;
struct PushObservation;

struct DisperseTurnContext {
    uint64_t now_us = 0, sweep_start_us = 0;
    float travel_m = 0, sweep_travel_m = 0, sweep_yaw = 0;
    float motion_ratio = 1, fallback_direction = 1;
    bool camera_ready = false, mid_pitch = false, swing_plan_active = false;
    float cap_positive = 99, cap_negative = 99;
};

enum class DisperseTurnPhase {
    Idle,
    Look,
    Turn,
    Push,
    FrameCheck
};
enum class DisperseTurnOutcome {
    Continue,
    Complete
};
enum class DisperseTurnMotion {
    None,
    Turn,
    Push
};

struct DisperseTurnResult {
    DisperseTurnOutcome outcome = DisperseTurnOutcome::Continue;
    DisperseTurnMotion motion = DisperseTurnMotion::None;
    const char* reason = nullptr;
    float direction = 0;
    bool turn_started = false, push_started = false;
    bool frame_event = false;
    int frame_count = 0, confirmations = 0;
    float turned = 0;
};

class DisperseTurnStage {
  public:
    void begin(bool enabled);
    void cancel() {
        phase_ = DisperseTurnPhase::Idle;
    }
    void clearConfirmation() {
        clear_count_ = 0;
    }
    bool active() const {
        return phase_ != DisperseTurnPhase::Idle;
    }
    bool needsFrameView() const {
        return phase_ == DisperseTurnPhase::FrameCheck;
    }
    DisperseTurnPhase phase() const {
        return phase_;
    }
    // The parent confirms frame readiness and applies progress events before motion.
    DisperseTurnResult
    update(const PushObservation&, const TaskTuning&, const DisperseTurnContext&);

  private:
    DisperseTurnPhase phase_ = DisperseTurnPhase::Idle;
    int clear_count_ = 0;
    float direction_ = 1;
    uint64_t look_ready_us_ = 0, frame_timestamp_us_ = 0;
    float frame_turned0_ = 0;
};
} // namespace rescue
