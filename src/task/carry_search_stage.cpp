#include "rescue/task/carry_search_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
void CarrySearchStage::reset() {
    *this = CarrySearchStage{};
}

CarrySearchResult CarrySearchStage::search(const TaskTuning& t, const CarrySearchContext& c) {
    CarrySearchResult result;
    const bool heading_ok = c.heading_valid && std::isfinite(c.heading_rad);
    if (!active_) {
        active_ = true;
        phase_ = 0;
        turned_ = 0;
        last_set_ = false;
        t0_ = c.now_us;
        goal_ = t.carry_search_turns * 2.f * float(CV_PI);
    }
    const bool fresh = c.zone_mem && c.now_us - c.zone_mem_us <= t.carry_sight_fresh_us;
    const bool known = c.zone_mem && c.now_us - c.zone_mem_us <= t.carry_sight_mem_us;
    if (phase_ == 0) {
        if (heading_ok) {
            if (last_set_)
                turned_ += std::abs(wrapAngle(c.heading_rad - last_yaw_));
            last_yaw_ = c.heading_rad;
            last_set_ = true;
        }
        const bool spun = turned_ >= goal_ || c.now_us - t0_ > t.carry_spin_cap_us;
        if (!fresh && !spun) {
            result.action = CarrySearchAction::Scan;
            result.reason = "searching_own_zone";
            return result;
        }
        if (!heading_ok || (!known && !c.run_yaw0_set)) {
            result.action = CarrySearchAction::Scan;
            result.reason = "searching_own_zone";
            return result;
        }
        phase_ = 1;
        t0_ = leg_t0_ = c.now_us;
    }
    if (!heading_ok) {
        phase_ = 0;
        t0_ = c.now_us;
        turned_ = 0;
        last_set_ = false;
        goal_ = t.carry_search_respin_turns * 2.f * float(CV_PI);
        return result;
    }
    const float target = known ? c.zone_mem_yaw : wrapAngle(c.run_yaw0 + float(CV_PI));
    const float err = wrapAngle(target - c.heading_rad);
    if (std::abs(err) > t.rotate_in_place_rad)
        leg_t0_ = c.now_us; // the leg clock runs only while driving
    if (c.now_us - leg_t0_ >= t.carry_leg_us || c.now_us - t0_ > t.carry_leg_turn_cap_us) {
        phase_ = 0;
        t0_ = c.now_us;
        turned_ = 0;
        last_set_ = false;
        goal_ = t.carry_search_respin_turns * 2.f * float(CV_PI);
        result.action = CarrySearchAction::Scan;
        result.reason = "searching_own_zone";
        return result;
    }
    result.action = CarrySearchAction::Steer;
    result.reason = fresh   ? "carry_drive_to_sighted_zone"
                    : known ? "carry_drive_to_remembered_zone"
                            : "carry_drive_start_line";
    result.speed = t.carry_speed;
    result.heading = err;
    return result;
}

CarrySearchResult CarrySearchStage::update(const TaskTuning& t, const CarrySearchContext& c) {
    return search(t, c);
}
} // namespace rescue
