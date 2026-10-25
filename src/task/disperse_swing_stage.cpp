#include "rescue/task/disperse_swing_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
DisperseSwingResult DisperseSwingStage::update(const TaskTuning& t, const DisperseSwingContext& c) {
    DisperseSwingResult result;
    const float dyaw = wrapAngle(c.heading_rad - c.sweep_yaw);
    const bool centring = swing_count_ >= swing_limit_;
    const float side = ((swing_count_ % 2 == 0) ? 1.f : -1.f) * c.direction;
    float swing_mag = c.small ? t.mid_neighbour_clear_turn_rad : t.disperse_turn_rad;
    if (!centring && c.swing_plan_active)
        swing_mag = std::min(swing_mag, side > 0 ? c.cap_positive : c.cap_negative);
    const float goal = centring ? 0.f : side * swing_mag;
    const float dir = goal > dyaw ? 1.f : -1.f;
    const bool reached =
        std::abs(goal - dyaw) <= (centring ? t.sweep_return_tol_rad : t.disperse_swing_tol_rad) ||
        (dir > 0 ? dyaw >= goal : dyaw <= goal);
    if (reached || c.now_us - c.sweep_start_us > t.disperse_swing_budget_us) {
        if (!centring) {
            ++swing_count_;
            result.restart_clock = true;
            return result;
        }
        result.outcome =
            ++pass_ < pass_limit_ ? DisperseSwingOutcome::NextPass : DisperseSwingOutcome::Complete;
        return result;
    }
    const bool creeping =
        !centring &&
        c.travel_m - c.sweep_travel_m <
            (c.small ? t.mid_neighbour_clear_creep_m : t.disperse_swing_creep_m) * c.motion_ratio;
    result.fast = c.burst || (c.approach_fast && !c.small);
    result.reason = c.burst ? (centring ? "first_burst_centre" : "first_burst_swing")
                            : (centring ? "disperse_centre" : "disperse_swing");
    result.motion_requested = true;
    result.vx = creeping ? t.disperse_swing_vx : 0.f;
    result.direction = dir;
    return result;
}
} // namespace rescue
