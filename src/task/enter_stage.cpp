#include "rescue/task/enter_stage.hpp"
#include "rescue/task/push_task.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include <cmath>

namespace rescue {
EnterResult EnterStage::update(const TaskTuning& t, const EnterContext& context) {
    EnterResult result;
    const auto finish = [&](EnterOutcome outcome, const char* reason) {
        result.outcome = outcome;
        result.reason = reason;
        return result;
    };
    if (!context.zone_ok) {
        if (!context.lost_target)
            return result;
        return context.carry ? finish(EnterOutcome::RaiseRelease, "assumed_drop_point_zone_lost")
                             : finish(EnterOutcome::AbortDrop, "zone_lost");
    }
    result.reset_misses = true;
    const float lateral = context.hold.x - context.half_x;
    // One leading object crossing the line must not strand the second object: advance the
    // conservative load envelope to the locked drop position. Inventory is credited only after
    // backing out.
    if (context.hold.y >= (context.drop_locked ? context.drop_centre_y : t.deposit_y_m))
        return finish(EnterOutcome::RaiseRelease, "enclosed_load_at_drop");
    if (std::abs(lateral) > t.enter_lateral_m)
        return finish(EnterOutcome::AbortDrop, "enter_lateral");
    if (context.elapsed_us > t.enter_budget_us)
        return finish(EnterOutcome::AbortDrop, "enter_timeout");
    result.drive = true;
    result.vx = t.enter_speed;
    result.wz = t.heading_gain * wrapAngle(context.zone_yaw) + t.lateral_gain * lateral;
    return result;
}
} // namespace rescue
