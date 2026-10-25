#pragma once

#include <cstdint>

namespace rescue {
struct TaskTuning;

// Shared reads from the parent: the last zone sighting, the run's start heading, the driven
// distance and the measured motion ratio. Nothing here is written by the stage.
struct CarrySearchContext {
    uint64_t now_us = 0;
    bool heading_valid = false;
    float heading_rad = 0;
    bool zone_mem = false;
    uint64_t zone_mem_us = 0;
    float zone_mem_yaw = 0;
    bool run_yaw0_set = false;
    float run_yaw0 = 0;
    // Enemy-zone evasion inputs.
    bool enemy_zone_seen = false;
    float travel_m = 0;
    float motion_ratio = 1;
};

enum class CarrySearchAction {
    // This tick has no motion of its own; the parent moves on.
    None,
    // Spin in place looking for our own zone.
    Scan,
    // Drive a leg toward the sighted, remembered or start-line zone.
    Steer,
    // Turn away from a seen enemy zone.
    TurnAway,
    // Drive away from the enemy zone.
    DriveAway
};

struct CarrySearchResult {
    CarrySearchAction action = CarrySearchAction::None;
    const char* reason = nullptr;
    // The parent writes this value into zone_search_started_us_ when set.
    bool reset_zone_search = false;
    uint64_t zone_search_us = 0;
    // The parent restarts its travel measurement when a drive-away leg begins.
    bool reset_travel = false;
    float speed = 0, heading = 0;
};

// Carries a load while our own zone is not verified: turn away from a seen enemy zone, else
// spin for our zone and drive a leg once one is sighted, remembered, or the start line helps.
// Every motion permission, reason write, travel reset and zone-search reset stays parent-owned.
class CarrySearchStage {
  public:
    void reset();
    CarrySearchResult update(const TaskTuning&, const CarrySearchContext&);
    // Evasion alone: the strict CARRY path spins once when no evasion owns the tick.
    CarrySearchResult evade(const TaskTuning&, const CarrySearchContext&);
    bool enemyEvadeActive() const {
        return evade_phase_ != 0;
    }

  private:
    CarrySearchResult enemyEvade(const TaskTuning&, const CarrySearchContext&);
    CarrySearchResult search(const TaskTuning&, const CarrySearchContext&);

    // Enemy-zone evasion chain.
    int evade_phase_ = 0;
    int seen_frames_ = 0;
    uint64_t evade_cool_until_us_ = 0, evade_start_us_ = 0;
    float evade_turned_ = 0, evade_last_yaw_ = 0, evade_yaw_ = 0;

    // Zone search chain.
    bool active_ = false;
    int phase_ = 0;
    uint64_t t0_ = 0, leg_t0_ = 0;
    float turned_ = 0, last_yaw_ = 0, goal_ = 0;
    bool last_set_ = false;
};
} // namespace rescue
