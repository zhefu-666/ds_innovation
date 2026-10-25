#pragma once

#include <opencv2/core/types.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace rescue {
struct TaskTuning;

struct PushInContext {
    uint64_t now_us = 0, elapsed_us = 0;
    float travel_m = 0;
    uint64_t imu_blocked_us = 0;
    bool anchored_zone = false;
    cv::Point2f hold;
    float zone_yaw = 0, half_x = 0, hold_center = 0;
    bool half_injured = false;
    uint64_t visual_reference_seen_us = 0;
    int repush_count = 0;
};

struct PushInCargoEvidence {
    bool visible = false, fixed = false, clipped = false;
    cv::Point2f rear, body;
    uint64_t seen_us = 0, static_since_us = 0;
};

enum class PushInOutcome {
    Continue,
    Repush,
    Complete
};
enum class PushInEventKind {
    Cargo,
    DropZone,
    Text
};

struct PushInEvent {
    PushInEventKind kind;
    std::string text;
    cv::Point2f position;
    bool visible = false;
};

struct PushInResult {
    PushInOutcome outcome = PushInOutcome::Continue;
    std::string reason;
    bool reset_cargo_reference = false;
    bool drive = false;
    float vx = 0, wz = 0;
    std::vector<PushInEvent> events;
};

class PushInStage {
  public:
    // A new push resets alignment and held heading, retaining probe and confirmation history.
    void prepare(bool align);
    void clearTrip();
    // The parent confirms camera/frame readiness and updates cargo tracking first.
    PushInResult update(const TaskTuning&, const PushInContext&, const PushInCargoEvidence&);

  private:
    bool align_pending_ = false, push_hold_valid_ = false;
    uint64_t align_start_us_ = 0;
    float push_hold_phi_ = 0;
    uint64_t probe_until_us_ = 0, probe_next_us_ = 0;
    float probe_sign_ = 1.f;
    bool enter_unconfirmed_ = false;
};
} // namespace rescue
