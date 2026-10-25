#include "rescue/task/carry_search_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
void CarrySearchStage::reset() {
    *this = CarrySearchStage{};
}

CarrySearchResult CarrySearchStage::enemyEvade(const TaskTuning& t, const CarrySearchContext& c) {
    CarrySearchResult result;
    const bool heading_ok = c.heading_valid && std::isfinite(c.heading_rad);
    float travel = c.travel_m;
    if (evade_phase_ == 0) {
        if (!t.enemy_evade || c.now_us < evade_cool_until_us_ || !c.enemy_zone_seen) {
            seen_frames_ = 0;
            return result;
        }
        if (++seen_frames_ < t.enemy_evade_frames || !heading_ok)
            return result;
        evade_phase_ = 1;
        evade_turned_ = 0;
        evade_last_yaw_ = c.heading_rad;
        evade_start_us_ = c.now_us;
        seen_frames_ = 0;
    }
    if (evade_phase_ == 1) {
        if (heading_ok) {
            evade_turned_ += std::abs(wrapAngle(c.heading_rad - evade_last_yaw_));
            evade_last_yaw_ = c.heading_rad;
        }
        if (!heading_ok || c.now_us - evade_start_us_ > t.enemy_evade_turn_budget_us) {
            evade_phase_ = 0;
            evade_cool_until_us_ = c.now_us + t.enemy_evade_cool_us;
            result.reset_zone_search = true;
            result.zone_search_us = c.now_us;
            return result;
        }
        if (evade_turned_ < t.enemy_evade_turn_rad) {
            result.action = CarrySearchAction::TurnAway;
            result.reason = "enemy_zone_turn_away";
            return result;
        }
        evade_phase_ = 2;
        evade_yaw_ = c.heading_rad;
        evade_start_us_ = c.now_us;
        result.reset_travel = true;
        // The legacy code cleared travel_ here, so this tick's distance check reads zero.
        travel = 0;
    }
    if (travel >= t.enemy_evade_m * c.motion_ratio ||
        c.now_us - evade_start_us_ > t.enemy_evade_drive_budget_us) {
        evade_phase_ = 0;
        evade_cool_until_us_ = c.now_us + t.enemy_evade_cool_us;
        result.reset_zone_search = true;
        result.zone_search_us = c.now_us;
        return result;
    }
    const float err = heading_ok ? wrapAngle(evade_yaw_ - c.heading_rad) : 0.f;
    result.action = CarrySearchAction::DriveAway;
    result.reason = "enemy_zone_drive_away";
    result.speed = t.enemy_evade_speed;
    result.heading = std::clamp(t.heading_gain * err, -t.max_wz, t.max_wz);
    return result;
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

CarrySearchResult CarrySearchStage::evade(const TaskTuning& t, const CarrySearchContext& c) {
    return enemyEvade(t, c);
}

CarrySearchResult CarrySearchStage::update(const TaskTuning& t, const CarrySearchContext& c) {
    const auto evade = enemyEvade(t, c);
    // A live evasion owns the tick; a finished one still lets the search run on the same tick,
    // exactly as the inline pair did.
    if (evade.action != CarrySearchAction::None)
        return evade;
    auto search_result = search(t, c);
    search_result.reset_zone_search = evade.reset_zone_search;
    search_result.zone_search_us = evade.zone_search_us;
    return search_result;
}
} // namespace rescue
