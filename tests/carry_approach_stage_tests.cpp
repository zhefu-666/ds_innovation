#include "rescue/task/carry_approach_stage.hpp"
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
    CarryApproachStage stage;
    TaskTuning tuning;
    CarryApproachContext context;

    Fixture() {
        tuning.assume_stall_progress_m = .01f;
        tuning.assume_imu_block_us = 600000;
        tuning.assume_imu_block_near_m = .25f;
        tuning.assume_stall_near_m = .06f;
        tuning.assume_stall_us = 1500000;
        tuning.assume_release_gap_m = -.04f;
        tuning.assume_align_zone_m = .18f;
        tuning.rotate_in_place_rad = .2f;
        context.now_us = 1000000;
        context.hold = {0, -.5f};
        context.carrot = {0, -.1f};
    }

    CarryApproachResult update() {
        return stage.update(tuning, context);
    }
};

void expect(const CarryApproachResult& r, CarryApproachOutcome outcome, const char* reason) {
    CHECK(r.outcome == outcome);
    if (reason == nullptr)
        CHECK(r.reason == nullptr);
    else
        CHECK(r.reason != nullptr && std::string(r.reason) == reason);
}

void drivesWithAFreshFix() {
    Fixture f;
    f.context.fresh_fix = true;
    const auto r = f.update();
    expect(r, CarryApproachOutcome::Continue, "assumed_visual_carry");
    CHECK(r.motion);
    CHECK(r.drop_event == nullptr);
}

void drivesOnDeadReckoningWithoutAFreshFix() {
    Fixture f;
    f.context.fresh_fix = false;
    expect(f.update(), CarryApproachOutcome::Continue, "assumed_visual_dead_reckoning");
}

void releasesAtTheVisualGap() {
    Fixture f;
    f.context.fresh_fix = true;
    f.context.hold = {0, -f.tuning.assume_release_gap_m}; // the check is an inclusive >=
    const auto r = f.update();
    expect(r, CarryApproachOutcome::RaiseRelease, "assumed_drop_point_visual");
    CHECK(std::string(r.drop_event) == "release");
    CHECK(!r.motion);
}

void releaseWithoutAFreshFixIsDeadReckoned() {
    Fixture f;
    f.context.hold = {0, .05f};
    expect(f.update(), CarryApproachOutcome::RaiseRelease, "assumed_drop_point_visual_dr");
}

void imuBlockReleasesNearTheRim() {
    Fixture f;
    f.context.imu_blocked_us = f.tuning.assume_imu_block_us;
    f.context.hold = {0, -f.tuning.assume_imu_block_near_m};
    const auto r = f.update();
    expect(r, CarryApproachOutcome::RaiseRelease, "assumed_drop_point_imu_blocked");
    CHECK(std::string(r.drop_event) == "release_stall_imu");
}

void imuBlockIsIgnoredShortOfItsThreshold() {
    Fixture f;
    f.context.imu_blocked_us = f.tuning.assume_imu_block_us - 1;
    f.context.hold = {0, -.5f};
    expect(f.update(), CarryApproachOutcome::Continue, "assumed_visual_dead_reckoning");
}

void imuBlockIsIgnoredFarFromTheRim() {
    Fixture f;
    f.context.imu_blocked_us = f.tuning.assume_imu_block_us * 2;
    f.context.hold = {0, -f.tuning.assume_imu_block_near_m - .01f};
    expect(f.update(), CarryApproachOutcome::Continue, "assumed_visual_dead_reckoning");
}

void imuBlockWinsOverTheVisionStall() {
    Fixture f;
    f.context.fresh_fix = true;
    f.context.imu_blocked_us = f.tuning.assume_imu_block_us;
    f.context.now_us += f.tuning.assume_stall_us * 2;
    f.context.hold = {0, -f.tuning.assume_imu_block_near_m};
    const auto r = f.update();
    CHECK(std::string(r.reason) == "assumed_drop_point_imu_blocked");
}

void visionStallNeedsAFreshFix() {
    Fixture f;
    f.context.fresh_fix = true;
    f.context.hold = {0, -f.tuning.assume_stall_near_m};
    f.update(); // establish progress
    f.context.now_us += f.tuning.assume_stall_us;
    const auto r = f.update();
    expect(r, CarryApproachOutcome::RaiseRelease, "assumed_drop_point_stalled");
    CHECK(std::string(r.drop_event) == "release_stall");

    Fixture stale;
    stale.context.fresh_fix = false;
    stale.context.hold = {0, -stale.tuning.assume_stall_near_m};
    stale.update();
    stale.context.now_us += stale.tuning.assume_stall_us;
    CHECK(stale.update().outcome == CarryApproachOutcome::Continue);
}

void progressRestartsTheStallClock() {
    Fixture f;
    f.context.hold = {0, -f.tuning.assume_stall_near_m};
    f.update();
    f.context.now_us += f.tuning.assume_stall_us - 1;
    f.context.hold.y += f.tuning.assume_stall_progress_m + .001f; // measurable progress
    expect(f.update(), CarryApproachOutcome::Continue, "assumed_visual_dead_reckoning");
    f.context.now_us += 1;
    CHECK(f.update().outcome == CarryApproachOutcome::Continue);
}

void smoothProgressBelowTheThresholdDoesNotRestart() {
    Fixture f;
    f.context.hold = {0, -f.tuning.assume_stall_near_m};
    f.context.fresh_fix = true;
    f.update();
    f.context.now_us += f.tuning.assume_stall_us;
    f.context.hold.y += f.tuning.assume_stall_progress_m; // exactly the threshold: not progress
    expect(f.update(), CarryApproachOutcome::RaiseRelease, "assumed_drop_point_stalled");
}

void blendsWhileTurnedAndFarFromTheEdge() {
    Fixture f;
    f.context.hold = {0, -.5f};
    f.context.carrot = {.5f, -.1f}; // a large heading error, well short of the edge
    const auto r = f.update();
    expect(r, CarryApproachOutcome::Continue, "assumed_visual_dead_reckoning");
    CHECK(r.blend);
    CHECK(std::abs(r.heading) > f.tuning.rotate_in_place_rad);
}

void steersInPlaceNearTheEdge() {
    Fixture f;
    // Near the edge means at or beyond the blend boundary; deeper back is where it blends.
    f.context.hold = {0, -f.tuning.assume_release_gap_m - f.tuning.assume_align_zone_m};
    f.context.carrot = {.5f, -.1f};
    const auto r = f.update();
    CHECK(!r.blend); // near the edge the parent steers, squaring up in place
}

void noBlendWhenAlreadyAligned() {
    Fixture f;
    f.context.carrot = {0, -.1f};
    CHECK(!f.update().blend);
}

void resetRestartsTheStallClock() {
    Fixture f;
    f.context.hold = {0, -f.tuning.assume_stall_near_m};
    f.context.fresh_fix = true;
    f.update();
    f.context.now_us += f.tuning.assume_stall_us;
    f.stage.reset();
    expect(f.update(), CarryApproachOutcome::Continue, "assumed_visual_carry");
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"drivesWithAFreshFix", drivesWithAFreshFix},
        {"drivesOnDeadReckoningWithoutAFreshFix", drivesOnDeadReckoningWithoutAFreshFix},
        {"releasesAtTheVisualGap", releasesAtTheVisualGap},
        {"releaseWithoutAFreshFixIsDeadReckoned", releaseWithoutAFreshFixIsDeadReckoned},
        {"imuBlockReleasesNearTheRim", imuBlockReleasesNearTheRim},
        {"imuBlockIsIgnoredShortOfItsThreshold", imuBlockIsIgnoredShortOfItsThreshold},
        {"imuBlockIsIgnoredFarFromTheRim", imuBlockIsIgnoredFarFromTheRim},
        {"imuBlockWinsOverTheVisionStall", imuBlockWinsOverTheVisionStall},
        {"visionStallNeedsAFreshFix", visionStallNeedsAFreshFix},
        {"progressRestartsTheStallClock", progressRestartsTheStallClock},
        {"smoothProgressBelowTheThresholdDoesNotRestart",
         smoothProgressBelowTheThresholdDoesNotRestart},
        {"blendsWhileTurnedAndFarFromTheEdge", blendsWhileTurnedAndFarFromTheEdge},
        {"steersInPlaceNearTheEdge", steersInPlaceNearTheEdge},
        {"noBlendWhenAlreadyAligned", noBlendWhenAlreadyAligned},
        {"resetRestartsTheStallClock", resetRestartsTheStallClock},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "carry approach stage OK\n";
    return 0;
}
