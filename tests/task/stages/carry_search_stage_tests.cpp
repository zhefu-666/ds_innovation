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
}

void freshSightReplacesTheCurrentLegDirection() {
    Fixture f;
    f.context.zone_mem = true;
    f.context.zone_mem_us = f.context.now_us;
    f.context.zone_mem_yaw = .3f;
    CHECK(f.update().action == CarrySearchAction::Steer);
    f.context.now_us += 50000;
    f.context.zone_mem_us = f.context.now_us;
    f.context.zone_mem_yaw = -.2f;
    const auto r = f.update();
    CHECK(r.action == CarrySearchAction::Steer);
    CHECK(std::string(r.reason) == "carry_drive_to_sighted_zone");
    CHECK(std::abs(r.heading + .2f) < 1e-5f);
}

void resetRestartsTheSearchSpin() {
    Fixture f;
    f.context.zone_mem = true;
    f.context.zone_mem_us = f.context.now_us;
    CHECK(f.update().action == CarrySearchAction::Steer);
    f.stage.reset();
    f.context.zone_mem = false;
    f.context.now_us += 50000;
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
        {"freshSightReplacesTheCurrentLegDirection", freshSightReplacesTheCurrentLegDirection},
        {"resetRestartsTheSearchSpin", resetRestartsTheSearchSpin},
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
