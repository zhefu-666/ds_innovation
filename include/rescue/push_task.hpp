#pragma once
#include "rescue/types.hpp"
#include <string>

namespace rescue {
enum class PushState { WAIT_START, SEARCH_TARGET, APPROACH_TARGET, PUSH_TARGET,
                       ALIGN_ZONE, VERIFY_DELIVERY, BACK_OUT };
// Observations must describe the currently tracked target, in robot coordinates.
// Missing geometry/zone evidence defaults to invalid, never to success.
struct PushObservation {
    uint64_t now_us = 0;
    bool run = false, reset = false, safety_ok = false;
    bool target_valid = false, geometry_valid = false, path_safe = false;
    int target_id = -1, available_count = 1;
    std::string label;
    float distance_m = 0, heading_error = 0;
    bool in_push_region = false;
    bool zone_valid = false, zone_own = false;
    std::string zone_class; // "supply" or "injured"; not red/blue detection class
    bool zone_aligned = false;
    bool fully_inside = false, off_fence = false, stable = false;
    bool separated = false, retreat_safe = false;
    int delivered_count = 0; // independently observed count in the correct subzone
};
struct PushOutput {
    PushState state = PushState::WAIT_START;
    MotionCommand motion;
    int batch_size = 0, delivered_total = 0;
    bool first_ordinary_delivered = false;
};
class PushTask {
public:
    PushOutput update(const PushObservation &in);
    static const char *name(PushState state);
private:
    PushState state_ = PushState::WAIT_START;
    int target_id_ = -1, batch_ = 0, total_ = 0, confirmations_ = 0;
    std::string label_;
    bool first_ = false;
    uint64_t last_us_ = 0, phase_us_ = 0;

    void abandon();
};
} // namespace rescue
