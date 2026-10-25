#include "rescue/task/carry_approach_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
void CarryApproachStage::reset() {
    progress_us_ = 0;
}

CarryApproachResult CarryApproachStage::update(const TaskTuning& t, const CarryApproachContext& c) {
    CarryApproachResult result;
    if (!progress_us_ || c.hold.y > best_y_ + t.assume_stall_progress_m) {
        best_y_ = c.hold.y;
        progress_us_ = c.now_us;
    }
    // Blocked by the zone rim: no forward progress near the front edge counts as arrived. The IMU
    // decides "pressed against the rim" (dead reckoning keeps advancing while the wheels slip); a
    // vision-only no-progress check needs a fresh fix.
    const bool imu_blocked =
        c.imu_blocked_us >= t.assume_imu_block_us && c.hold.y >= -t.assume_imu_block_near_m;
    if (imu_blocked || (c.fresh_fix && c.hold.y >= -t.assume_stall_near_m &&
                        c.now_us - progress_us_ >= t.assume_stall_us)) {
        result.outcome = CarryApproachOutcome::RaiseRelease;
        result.drop_event = imu_blocked ? "release_stall_imu" : "release_stall";
        result.reason =
            imu_blocked ? "assumed_drop_point_imu_blocked" : "assumed_drop_point_stalled";
        return result;
    }
    if (c.hold.y >= -t.assume_release_gap_m) {
        result.outcome = CarryApproachOutcome::RaiseRelease;
        result.drop_event = "release";
        result.reason = c.fresh_fix ? "assumed_drop_point_visual" : "assumed_drop_point_visual_dr";
        return result;
    }
    result.reason = c.fresh_fix ? "assumed_visual_carry" : "assumed_visual_dead_reckoning";
    const float h = std::atan2(-c.carrot.x, std::max(.02f, c.carrot.y - c.hold_center));
    result.heading = h;
    // Far from the front edge: turn while driving (in-place turns are slow in the field and stall
    // for seconds); square up in place only near the edge.
    result.blend = std::abs(h) > t.rotate_in_place_rad &&
                   c.hold.y < -t.assume_release_gap_m - t.assume_align_zone_m;
    result.motion = true;
    return result;
}
} // namespace rescue
