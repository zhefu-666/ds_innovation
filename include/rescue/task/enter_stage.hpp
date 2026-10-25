#pragma once

#include <opencv2/core/types.hpp>
#include <cstdint>

namespace rescue {
struct TaskTuning;

struct EnterContext {
    uint64_t elapsed_us = 0;
    // The parent reads the zone and its own missed-frame count before calling the stage.
    bool zone_ok = false;
    bool lost_target = false;
    bool carry = false;
    bool drop_locked = false;
    float drop_centre_y = 0;
    // Body-frame hold centre lifted into the zone frame by the parent's geometry.
    cv::Point2f hold;
    float half_x = 0;
    // Heading error that faces the robot into the zone.
    float zone_yaw = 0;
};

enum class EnterOutcome {
    Continue,
    RaiseRelease,
    AbortDrop
};

struct EnterResult {
    EnterOutcome outcome = EnterOutcome::Continue;
    const char* reason = nullptr;
    // The zone was usable, so the parent's missed-frame count restarts.
    bool reset_misses = false;
    bool drive = false;
    float vx = 0, wz = 0;
};

// Strict ENTER: carry the enclosed load with the frame down until it crosses the deposit line.
// Every motion permission, the frame handshake and the state transitions stay parent-owned.
class EnterStage {
  public:
    EnterResult update(const TaskTuning&, const EnterContext&);
};
} // namespace rescue
