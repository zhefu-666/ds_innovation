#pragma once

#include <opencv2/core/types.hpp>
#include <cstdint>
#include <string>

namespace rescue {
struct TaskTuning;

struct RecoveryContext {
    uint64_t now_us = 0;
    // now - phase_us_ as measured before the parent's frame handshake this tick.
    uint64_t elapsed_us = 0;
    // now - phase_us_ after the parent confirmed the open frame.
    uint64_t since_step_us = 0;
    float travel_m = 0, approach_forward_m = 0, motion_ratio = 0;
    bool corridor_retreat = false, observed_zone = false, retreat_safe = false;
    bool heading_valid = false;
    float heading_rad = 0;
    cv::Point2f zone_origin, zone_forward;
    std::string zone_geometry_id;
    bool lost_hold = false;
};

enum class RecoveryOutcome {
    Continue,
    Reverse,
    Scan,
    Fault
};

struct RecoveryResult {
    RecoveryOutcome outcome = RecoveryOutcome::Continue;
    std::string reason;
    // The blind fallback discards any reversing measured before it took over.
    bool reset_travel = false;
    bool blacklist = false;
    float vx = 0;
};

class RecoveryStage {
  public:
    // Every state change restarts the retreat decision; the phase clock itself stays parent-owned.
    void reset();
    // Called only after the parent has confirmed the open frame is ready.
    RecoveryResult update(const TaskTuning&, const RecoveryContext&);

  private:
    int mode_ = 0; // 0 undecided, 1 zone-measured, 2 blind (dead reckoning + IMU heading)
    uint64_t zone_us_ = 0;
    float heading_ = 0;
    bool origin_valid_ = false;
    cv::Point2f origin_zone_, forward_zone_;
    std::string geometry_id_;
};
} // namespace rescue
