#include "rescue/task/transport_rules.hpp"
#include "rescue/core/types.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include "rescue/geometry/zone_dead_reckoning.hpp"
#include "rescue/geometry/planner.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#if __has_include("rescue/task/capture_verification_stage.hpp")
#include "rescue/task/capture_verification_stage.hpp"
#endif
#define private public
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;
constexpr uint64_t kProgress = 777;
constexpr uint64_t kTransaction = 41;
constexpr int kTarget = 17;

Inventory ordinary(int count = 1) {
    Inventory inventory;
    inventory.ordinary = count;
    return inventory;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;

    explicit Rig(TaskTuning tuning = {}) : task(tuning) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
        input.camera_pitch_stable = true;
        input.gripper_done = true;
        input.gripper_feedback_open = 0;
        task.enter(PushState::VERIFY_CAPTURE, kStart, "seed_reason");
        task.target_id_ = kTarget;
        task.label_ = "ordinary_supply";
        task.pending_ = ordinary();
        task.verdict_ = RuleVerdict::DANGEROUS;
        task.last_us_ = kStart;
        task.pitch_cmd_ = tuning.near_pitch_cdeg;
        task.pitch_wait_us_ = kStart;
        task.frame_transaction_ = task.wire_txn_ = kTransaction;
        task.wire_last_ = 0;
        task.carry_us_ = 333;
        task.carry_approach_.seedProgressUs(kProgress);
        task.hold_seen_us_ = 444;
        task.zone_search_started_us_ = 555;
        task.carried_ = 2.f;
    }

    PushOutput tick() {
        input.now_us += kTick;
        output = task.update(input);
        return output;
    }

    PushOutput atElapsed(uint64_t elapsed) {
        input.now_us = kStart + elapsed;
        task.last_us_ = input.now_us - kTick;
        output = task.update(input);
        return output;
    }

    void held(const Inventory& inventory) {
        input.hold_observable = input.captured = input.held_complete = true;
        input.held = inventory;
    }

    void summary(int verdict, const Inventory& inventory) {
        input.multi_view_finished = true;
        input.multi_view_verdict = verdict;
        input.multi_view_inventory = inventory;
    }

    void closed() const {
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
        assert(output.motion.gripper_offset == 20);
        assert(output.motion.frame_transaction == kTransaction);
        assert(task.frame_raised_ == 0);
    }

    void blacklisted(bool expected) const {
        const auto rejected = task.rejectedTargets(input.now_us);
        assert((std::find(rejected.begin(), rejected.end(), kTarget) != rejected.end()) ==
               expected);
        if (expected)
            assert(task.rejected_.at(kTarget) == input.now_us + task.t_.blacklist_us);
    }

    void carry(bool assumed, const char* reason) const {
        assert(output.state == PushState::CARRY && output.reason == reason);
        assert(task.trip_ == ordinary() && output.batch_size == 1);
        assert(task.carry_us_ == input.now_us && task.hold_seen_us_ == input.now_us);
        assert(task.zone_search_started_us_ == 0 && task.carried_ == 0);
        assert(task.phase_us_ == input.now_us);
        assert(task.carry_approach_.progressUs() == (assumed ? 0 : kProgress));
        assert(!task.fault_);
        closed();
        blacklisted(false);
    }

    void report(const char* test) const {
        const auto rejected = task.rejectedTargets(input.now_us);
        std::cout << "case=" << test << " state=" << PushTask::name(output.state)
                  << " reason=" << output.reason << " verdict=" << verdictName(output.verdict)
                  << " frame=" << int(output.motion.gripper_offset)
                  << " transaction=" << output.motion.frame_transaction
                  << " blacklisted=" << rejected.size() << " fault=" << task.fault_
                  << " phase_us=" << task.phase_us_ << " carry_us=" << task.carry_us_
                  << " carry_progress_us=" << task.carry_approach_.progressUs()
                  << " hold_seen_us=" << task.hold_seen_us_
                  << " zone_search_us=" << task.zone_search_started_us_
                  << " carried=" << task.carried_ << '\n';
    }
};

void verifiedTests() {
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        Rig rig(tuning);
        rig.summary(1, ordinary());
        rig.tick();
        rig.carry(false, "multi_view_capture_verified");
        assert(rig.output.verdict == RuleVerdict::OK);
        rig.report("verified_multi");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = false;
        tuning.capture_frames = 2;
        Rig rig(tuning);
        rig.held(ordinary());
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_CAPTURE);
        assert(rig.output.reason == "seed_reason" && rig.output.verdict == RuleVerdict::DANGEROUS);
        assert(rig.task.phase_us_ == kStart && rig.task.carry_approach_.progressUs() == kProgress);
        rig.closed();
        rig.report("continue_single");
        rig.tick();
        rig.carry(false, "capture_verified");
        assert(rig.output.verdict == RuleVerdict::OK);
        rig.report("verified_single");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        tuning.field_rush = true;
        Rig rig(tuning);
        rig.summary(2, {});
        rig.tick();
        rig.carry(true, "assumed_capture");
        assert(rig.output.verdict == RuleVerdict::DANGEROUS);
        rig.report("assumed");
    }
}

void failureTests() {
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        Rig rig(tuning);
        rig.task.rush_yaw_ref_set_ = true;
        rig.task.rush_yaw_ref_ = .4f;
        rig.task.last_target_valid_ = true;
        rig.task.last_target_xy_ = {.1f, .2f};
        rig.summary(1, ordinary(2));
        rig.tick();
        assert(rig.output.state == PushState::ABORT_DROP);
        assert(rig.output.reason == "multi_grab_inventory_mismatch" &&
               rig.output.verdict == RuleVerdict::OK);
        assert(rig.task.phase_us_ == rig.input.now_us && !rig.task.fault_);
        const auto bearings = rig.task.rejectedBearings(rig.input.now_us);
        assert(bearings.size() == 1 && std::abs(bearings.front().world_yaw - .4f) < 1e-6f);
        assert(std::abs(bearings.front().range_m - std::sqrt(.05f)) < 1e-6f);
        assert(!rig.task.rush_yaw_ref_set_);
        rig.closed();
        rig.blacklisted(true);
        rig.report("abort_drop");
        rig.tick();
        assert(rig.output.motion.gripper_offset == 0);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("abort_drop_next_tick");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = false;
        Rig rig(tuning);
        rig.atElapsed(tuning.verify_budget_us + 1);
        assert(rig.output.state == PushState::CAPTURE_FAIL &&
               rig.output.reason == "capture_unverified");
        assert(rig.output.verdict == RuleVerdict::DANGEROUS && !rig.task.fault_);
        assert(rig.task.phase_us_ == rig.input.now_us);
        rig.closed();
        rig.blacklisted(false);
        rig.report("capture_fail_timeout");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        tuning.field_rush = tuning.verify_held_contents = true;
        Rig rig(tuning);
        rig.input.hold_observable = true;
        rig.summary(2, {});
        rig.tick();
        assert(rig.output.state == PushState::CAPTURE_FAIL &&
               rig.output.reason == "capture_target_not_in_frame");
        assert(rig.output.verdict == RuleVerdict::DANGEROUS && !rig.task.fault_);
        assert(rig.task.phase_us_ == rig.input.now_us);
        rig.closed();
        rig.blacklisted(true);
        rig.report("capture_fail_missing_target");
        rig.tick();
        assert(rig.output.motion.gripper_offset == 0);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("capture_fail_next_tick");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        Rig rig(tuning);
        rig.summary(1, ordinary());
        rig.atElapsed(8000001);
        assert(rig.output.state == PushState::SAFE_STOP &&
               rig.output.reason == "multi_view_unconfirmed");
        assert(rig.task.fault_ && rig.task.phase_us_ == kStart);
        assert(rig.output.verdict == RuleVerdict::DANGEROUS);
        rig.closed();
        rig.blacklisted(false);
        rig.report("safe_stop");
    }
}

void lifecycleTests() {
    {
        TaskTuning tuning;
        tuning.require_multi_view = false;
        tuning.capture_frames = 3;
        Rig original(tuning);
        original.held(ordinary());
        original.tick();
        assert(original.output.state == PushState::VERIFY_CAPTURE);
        Rig copied = original;
        Rig moved = std::move(copied);
        original.task.enter(PushState::VERIFY_CAPTURE, original.input.now_us, "reentered_capture");
        moved.tick();
        assert(moved.output.state == PushState::VERIFY_CAPTURE &&
               moved.output.reason == "seed_reason");
        moved.report("moved_continue");
        moved.tick();
        moved.carry(false, "capture_verified");
        moved.report("moved_verified");
        original.tick();
        original.tick();
        assert(original.output.state == PushState::VERIFY_CAPTURE &&
               original.output.reason == "reentered_capture");
        original.closed();
        original.report("reentered_continue");
        original.tick();
        original.carry(false, "capture_verified");
        original.report("reentered_verified");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        Rig rig(tuning);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_CAPTURE);
        assert(rig.output.reason == "seed_reason" && rig.output.verdict == RuleVerdict::DANGEROUS);
        assert(rig.task.phase_us_ == kStart);
        rig.closed();
        rig.report("continue_multi");
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.reason == "not_running");
        assert(rig.task.stopped_from_ == PushState::VERIFY_CAPTURE && !rig.task.fault_);
        rig.closed();
        rig.report("paused");
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::ABORT_DROP && rig.output.reason == "resume");
        assert(rig.task.phase_us_ == rig.input.now_us && !rig.task.fault_);
        rig.closed();
        rig.blacklisted(false);
        rig.report("resumed");
        rig.tick();
        assert(rig.output.motion.gripper_offset == 0);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("resumed_next_tick");
    }
    {
        TaskTuning tuning;
        tuning.require_multi_view = true;
        tuning.field_rush = tuning.verify_held_contents = true;
        Rig rig(tuning);
        auto held = ordinary();
        held.core = 1;
        rig.held(held);
        rig.input.held_objects = {{99, "core_supply", .54f}};
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_CAPTURE &&
               rig.output.reason == "held_illegal_recheck");
        rig.closed();
        rig.report("recheck_continue");
        rig.input.safety_ok = false;
        rig.tick();
        assert(rig.output.state == PushState::SAFE_STOP && rig.output.reason == "safety_veto");
        assert(rig.task.fault_ && rig.task.phase_us_ == kStart);
        rig.closed();
        rig.report("safety_fault");
        rig.input.reset = true;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.reason == "reset");
        assert(!rig.task.fault_ && rig.output.batch_size == 0 &&
               rig.output.verdict == RuleVerdict::OK);
        rig.closed();
        rig.blacklisted(false);
        rig.report("reset");
        rig.input.reset = false;
        rig.input.safety_ok = true;
        rig.input.hold_observable = rig.input.captured = rig.input.held_complete = false;
        rig.input.held = {};
        rig.input.held_objects.clear();
        rig.task.target_id_ = kTarget;
        rig.task.label_ = "ordinary_supply";
        rig.task.pending_ = ordinary();
        rig.task.enter(PushState::VERIFY_CAPTURE, rig.input.now_us, "new_capture");
        rig.summary(2, {});
        rig.tick();
        assert(rig.output.state == PushState::CARRY && rig.output.reason == "assumed_capture");
        assert(rig.task.trip_ == ordinary() && !rig.task.fault_);
        assert(rig.task.carry_us_ == rig.input.now_us &&
               rig.task.hold_seen_us_ == rig.input.now_us);
        assert(rig.task.carry_approach_.progressUs() == 0 && rig.task.carried_ == 0);
        rig.closed();
        rig.blacklisted(false);
        rig.report("new_capture_after_reset");
    }
}
} // namespace

int main() {
    verifiedTests();
    failureTests();
    lifecycleTests();
    std::cout << "Capture verification PushTask boundary checks passed\n";
}
