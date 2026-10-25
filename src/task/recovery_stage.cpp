#include "rescue/task/recovery_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
void RecoveryStage::reset() {
    mode_ = 0;
    origin_valid_ = false;
}

RecoveryResult RecoveryStage::update(const TaskTuning& t, const RecoveryContext& context) {
    RecoveryResult result;
    const auto finish = [&](RecoveryOutcome outcome, const char* reason) {
        result.outcome = outcome;
        result.reason = reason;
        return result;
    };
    if (mode_ < 2 && context.observed_zone) {
        mode_ = 1;
        zone_us_ = context.now_us;
    }
    const uint64_t blind_wait = context.corridor_retreat
                                    ? std::min<uint64_t>(t.blind_retreat_wait_us, 200000)
                                    : t.blind_retreat_wait_us;
    if (mode_ == 0   ? context.since_step_us >= blind_wait
        : mode_ == 1 ? context.now_us - zone_us_ >= blind_wait
                     : false) {
        // No zone geometry at FAR: back off blind instead of standing on the objects.
        if (!context.heading_valid || !std::isfinite(context.heading_rad))
            return finish(RecoveryOutcome::Fault, "retreat_unverified_no_heading");
        // travel_ keeps any measured reversing already done before the blind fallback.
        result.reset_travel = mode_ == 0;
        mode_ = 2;
        heading_ = context.heading_rad;
    }
    if (mode_ == 2) {
        const float limit = context.corridor_retreat
                                ? std::min(t.corridor_back_m * context.motion_ratio,
                                           .85f * t.back_speed * float(t.retreat_budget_us) / 1e6f)
                                : std::min(t.abort_back_m, context.approach_forward_m);
        if (context.corridor_retreat && !context.retreat_safe && context.travel_m < limit)
            return finish(RecoveryOutcome::Scan, "corridor_back_off_unsafe");
        if (context.travel_m >= limit) {
            result.blacklist = !context.lost_hold;
            return finish(RecoveryOutcome::Scan, "blind_retreat_complete");
        }
        if (!context.heading_valid || !std::isfinite(context.heading_rad))
            return finish(RecoveryOutcome::Fault, "blind_retreat_heading_missing");
        if (std::abs(wrapAngle(context.heading_rad - heading_)) > t.blind_retreat_yaw_rad)
            return finish(RecoveryOutcome::Fault, "blind_retreat_heading_drift");
        if (context.elapsed_us > t.retreat_budget_us)
            return finish(RecoveryOutcome::Fault, "retreat_unverified");
        result.outcome = RecoveryOutcome::Reverse;
        result.reason = "blind_retreat";
        result.vx = -t.back_speed;
        return result;
    }
    if (context.observed_zone && !origin_valid_) {
        origin_zone_ = context.zone_origin;
        forward_zone_ = context.zone_forward;
        geometry_id_ = context.zone_geometry_id;
        origin_valid_ = true;
    }
    const bool measured =
        origin_valid_ && context.observed_zone && context.zone_geometry_id == geometry_id_;
    const float back = measured ? -(context.zone_origin - origin_zone_).dot(forward_zone_) : 0;
    if (measured && back >= (context.corridor_retreat ? t.corridor_back_m : t.abort_back_m)) {
        result.blacklist = !context.lost_hold;
        return finish(RecoveryOutcome::Scan, "");
    }
    if (context.elapsed_us > t.retreat_budget_us)
        return finish(RecoveryOutcome::Fault, "retreat_unverified");
    if (measured) {
        // The legacy measured reverse keeps whatever reason the state already carried.
        result.outcome = RecoveryOutcome::Reverse;
        result.vx = -t.back_speed;
    }
    return result;
}
} // namespace rescue
