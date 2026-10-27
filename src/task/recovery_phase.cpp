#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::updateRecoveryPhase(TaskTick& tick) {
    switch (state_) {
        case PushState::CAPTURE_FAIL:
        case PushState::LOST_HOLD:
        case PushState::ABORT_DROP: {
            // Release where the opponent zone is not involved, then reverse off the objects.
            if (actionProgress().step == 0 && corridor_retreat_) {
                commandFrame(1, tick.now);
                actionProgress().step = 1;
                actionProgress().started_us = tick.now;
            } // nothing held: frame stays up
            if (actionProgress().step == 0) {
                if (!tick.in.opponent_zone_clear) {
                    if (tick.elapsed > t_.gripper_timeout_us)
                        fail("drop_blocked_opponent_zone");
                    break;
                }
                commandFrame(1, tick.now);
                const int g = frameWait(tick.in, 1, tick.now);
                if (g < 0) {
                    fail("gripper_open_timeout");
                    break;
                }
                if (g == 0) {
                    reason_ = "gripper_feedback_wait";
                    break;
                }
                actionProgress().step = 1;
                actionProgress().started_us = tick.now;
            }
            RecoveryContext context;
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.since_step_us = tick.now - actionProgress().started_us;
            context.travel_m = actionProgress().travel;
            context.approach_forward_m = trip_context_.approach_forward_;
            context.motion_ratio = motionRatio();
            context.corridor_retreat = corridor_retreat_;
            context.observed_zone = tick.observed_zone;
            context.retreat_safe = tick.in.retreat_safe;
            context.heading_valid = tick.in.heading_valid;
            context.heading_rad = tick.in.heading_rad;
            context.lost_hold = state_ == PushState::LOST_HOLD;
            if (tick.observed_zone) {
                context.zone_origin = tick.in.zone_estimate.bodyToZone({0, 0});
                context.zone_forward =
                    tick.in.zone_estimate.bodyToZone({0, 1}) - context.zone_origin;
                context.zone_geometry_id = tick.in.zone_estimate.geometry_id;
            }
            const auto recovery = recovery_.update(t_, context);
            if (recovery.outcome == RecoveryOutcome::Fault) {
                fail(recovery.reason.c_str());
                break;
            }
            if (recovery.outcome == RecoveryOutcome::Scan) {
                if (recovery.blacklist)
                    blacklist(tick.now);
                scanAfterClear(tick, recovery.reason.c_str());
                break;
            }
            if (recovery.reset_travel)
                actionProgress().travel = 0;
            if (recovery.outcome == RecoveryOutcome::Reverse) {
                if (!recovery.reason.empty())
                    reason_ = recovery.reason;
                drive(tick, recovery.vx, 0);
            }
            break;
        }
        default:
            break;
    }
}
} // namespace rescue
