#include "rescue/task/carry_search_stage.hpp"
#include "rescue/task/push_task.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

struct Fixture {
    CarrySearchStage stage;
    TaskTuning tuning;
    CarrySearchContext context;

    Fixture() {
        tuning.enemy_evade = true;
        tuning.enemy_evade_frames = 2;
        tuning.enemy_evade_turn_rad = 2.f;
        tuning.enemy_evade_turn_budget_us = 4000000;
        tuning.enemy_evade_m = .5f;
        tuning.enemy_evade_speed = .15f;
        tuning.enemy_evade_drive_budget_us = 3000000;
        tuning.enemy_evade_cool_us = 2000000;
        tuning.heading_gain = 1.5f;
        tuning.max_wz = .8f;
        tuning.carry_search_turns = 1.f;
        tuning.carry_search_respin_turns = 1.f;
        tuning.carry_spin_cap_us = 20000000;
        tuning.carry_sight_fresh_us = 500000;
        tuning.carry_sight_mem_us = 5000000;
        tuning.carry_leg_us = 2000000;
        tuning.carry_leg_turn_cap_us = 6000000;
        tuning.carry_speed = .05f;
        tuning.rotate_in_place_rad = .2f;
        context.now_us = 1000000;
        context.heading_valid = true;
        context.heading_rad = 0;
    }

    CarrySearchResult update() {
        return stage.update(tuning, context);
    }
};

void spinsWhileNothingIsSighted() {
    Fixture f;
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::Scan);
    CHECK(std::string(r.reason) == "searching_own_zone");
}

void drivesToAFreshSighting() {
    Fixture f;
    f.context.zone_mem = true;
    f.context.zone_mem_us = f.context.now_us;
    f.context.zone_mem_yaw = .3f;
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::Steer);
    CHECK(std::string(r.reason) == "carry_drive_to_sighted_zone");
    CHECK(r.speed == f.tuning.carry_speed);
    CHECK(std::abs(r.heading - .3f) < 1e-5f);
}

void drivesToARememberedSighting() {
    Fixture f;
    // Phase 0 spins until the turn goal is met; only then does a remembered sighting steer.
    f.context.zone_mem = true;
    f.context.zone_mem_yaw = .3f;
    CHECK(f.update().action == CarrySearchAction::Scan);
    for (int i = 0; i < 40 && f.update().action == CarrySearchAction::Scan; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
        f.context.zone_mem_us = f.context.now_us - f.tuning.carry_sight_fresh_us - 1;
    }
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::Steer);
    CHECK(std::string(r.reason) == "carry_drive_to_remembered_zone");
}

void drivesTheStartLineWhenNothingIsRemembered() {
    Fixture f;
    f.context.run_yaw0_set = true;
    f.context.run_yaw0 = 0;
    f.context.zone_mem_us = 0;
    // Spin far enough to satisfy the spin goal without a sighting.
    for (int i = 0; i < 40 && f.update().action == CarrySearchAction::Scan; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
    }
    f.context.heading_rad = 3.14f;
    f.context.now_us += 50000;
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::Steer);
    CHECK(std::string(r.reason) == "carry_drive_start_line");
}

void needsAStartHeadingWithoutAMemory() {
    Fixture f;
    f.context.run_yaw0_set = false;
    for (int i = 0; i < 40; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
        const auto r = f.update();
        CHECK(r.action == CarrySearchAction::Scan);
    }
}

void repinsWhenTheHeadingIsLost() {
    Fixture f;
    f.context.zone_mem = true;
    f.context.zone_mem_us = f.context.now_us;
    f.context.zone_mem_yaw = 0;
    CHECK(f.update().action == CarrySearchAction::Steer);
    f.context.heading_valid = false;
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::None);
    CHECK(!f.stage.enemyEvadeActive());
}

void enemySeenFramesGateTheTurn() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    // Below the frame gate the evasion declines, so the search runs instead of turning.
    CHECK(f.update().action == CarrySearchAction::Scan);
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::TurnAway);
    CHECK(std::string(r.reason) == "enemy_zone_turn_away");
}

void turnsUntilTheTurnGoalThenDrivesAway() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    f.update();
    CHECK(f.update().action == CarrySearchAction::TurnAway);
    // Accumulate the measured turn.
    for (int i = 0; i < 10 && f.update().action == CarrySearchAction::TurnAway; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
    }
    const auto away = f.update();
    CHECK(away.action == CarrySearchAction::DriveAway);
    CHECK(std::string(away.reason) == "enemy_zone_drive_away");
    CHECK(away.speed == f.tuning.enemy_evade_speed);
}

void enterDriveAwayResetsTravel() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    f.update();
    f.update();
    bool saw_reset = false;
    for (int i = 0; i < 20; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
        f.context.travel_m = 5.f; // a stale distance must not end the fresh leg at once
        const auto r = f.update();
        if (r.reset_travel) {
            saw_reset = true;
            CHECK(r.action == CarrySearchAction::DriveAway || r.action == CarrySearchAction::None);
            break;
        }
    }
    CHECK(saw_reset);
}

void driveAwayEndsAndCools() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    f.update();
    f.update();
    for (int i = 0; i < 20 && f.update().action == CarrySearchAction::TurnAway; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
    }
    f.context.travel_m = 10.f; // past the evade distance
    const auto r = f.update();
    // The evasion ends here, then the search runs on the same tick.
    CHECK(r.action == CarrySearchAction::Scan);
    CHECK(r.reset_zone_search);
    CHECK(r.zone_search_us == f.context.now_us);
    CHECK(!f.stage.enemyEvadeActive());
}

void coolDownSuppressesANewEvasion() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    f.update();
    f.update();
    for (int i = 0; i < 20 && f.update().action == CarrySearchAction::TurnAway; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
    }
    f.context.travel_m = 10.f;
    f.update(); // ends and cools
    f.context.enemy_zone_seen = true;
    CHECK(f.update().action != CarrySearchAction::TurnAway);
}

void disabledEvadeFallsThroughToTheSearch() {
    Fixture f;
    f.tuning.enemy_evade = false;
    f.context.enemy_zone_seen = true;
    CHECK(f.update().action == CarrySearchAction::Scan);
}

void aFinishedEvasionStillRunsTheSearchThisTick() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    f.update();
    f.update();
    for (int i = 0; i < 20 && f.update().action == CarrySearchAction::TurnAway; ++i) {
        f.context.now_us += 50000;
        f.context.heading_rad += .25f;
    }
    f.context.travel_m = 10.f;
    const auto r = f.update();
    // The evasion ended, so the search runs on the same tick and still reports its action.
    CHECK(r.reset_zone_search);
    CHECK(r.action == CarrySearchAction::Scan);
}

void resetClearsBothChains() {
    Fixture f;
    f.context.enemy_zone_seen = true;
    f.update();
    f.update(); // mid-turn
    CHECK(f.stage.enemyEvadeActive());
    f.stage.reset();
    CHECK(!f.stage.enemyEvadeActive());
    f.context.enemy_zone_seen = true;
    // The frame gate restarts, so the first frame after a reset does not turn away.
    CHECK(f.update().action == CarrySearchAction::Scan);
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"spinsWhileNothingIsSighted", spinsWhileNothingIsSighted},
        {"drivesToAFreshSighting", drivesToAFreshSighting},
        {"drivesToARememberedSighting", drivesToARememberedSighting},
        {"drivesTheStartLineWhenNothingIsRemembered", drivesTheStartLineWhenNothingIsRemembered},
        {"needsAStartHeadingWithoutAMemory", needsAStartHeadingWithoutAMemory},
        {"repinsWhenTheHeadingIsLost", repinsWhenTheHeadingIsLost},
        {"enemySeenFramesGateTheTurn", enemySeenFramesGateTheTurn},
        {"turnsUntilTheTurnGoalThenDrivesAway", turnsUntilTheTurnGoalThenDrivesAway},
        {"enterDriveAwayResetsTravel", enterDriveAwayResetsTravel},
        {"driveAwayEndsAndCools", driveAwayEndsAndCools},
        {"coolDownSuppressesANewEvasion", coolDownSuppressesANewEvasion},
        {"disabledEvadeFallsThroughToTheSearch", disabledEvadeFallsThroughToTheSearch},
        {"aFinishedEvasionStillRunsTheSearchThisTick", aFinishedEvasionStillRunsTheSearchThisTick},
        {"resetClearsBothChains", resetClearsBothChains},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "carry search stage OK\n";
    return 0;
}
