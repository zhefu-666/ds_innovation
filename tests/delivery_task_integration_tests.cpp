#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_dead_reckoning.hpp"
#include "rescue/planner.hpp"
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
#if __has_include("rescue/task/delivery_verification_stage.hpp")
#include "rescue/task/delivery_verification_stage.hpp"
#endif
#define private public
#include "rescue/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;
constexpr uint64_t kTransaction = 41;

Inventory ordinary(int count = 1) {
    Inventory inventory;
    inventory.ordinary = count;
    return inventory;
}

TaskTuning strictTuning() {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.delivery_frames = 3;
    return tuning;
}

TaskTuning visualTuning(bool assumed = true) {
    auto tuning = strictTuning();
    tuning.assume_all_safe = assumed;
    tuning.field_carry = true;
    return tuning;
}

ZoneEstimate zone(uint64_t timestamp, uint64_t frame) {
    ZoneEstimate estimate;
    estimate.valid = true;
    estimate.source = ZoneEstimate::Source::MULTI_POINT;
    estimate.zone_label = "red_safe_zone";
    estimate.geometry_id = "delivery-test";
    estimate.frame_id = frame;
    estimate.timestamp_us = estimate.observed_us = timestamp;
    estimate.origin_body_m = {0, .5f};
    estimate.inlier_ids = {0, 2, 3};
    estimate.residual_m = .001f;
    estimate.position_sigma_m = .002f;
    estimate.yaw_sigma_rad = .01f;
    return estimate;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;
    bool vref = false, fresh_cargo = true;
    uint64_t frame = 0;

    explicit Rig(TaskTuning tuning = strictTuning(), Inventory trip = ordinary()) : task(tuning) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.zone_valid = input.zone_own = input.zone_identity_verified = true;
        input.zone_inventory_complete = input.zone_counts_valid = true;
        input.zone_supply_count = trip.total();
        input.zone_injured_count = 4;
        input.camera_pitch_cdeg = task.assumeView() ? tuning.assume_view_pitch_cdeg : tuning.far_pitch_cdeg;
        input.camera_pitch_stable = true;
        input.gripper_done = true;
        input.gripper_feedback_open = 1;
        task.trip_ = trip;
        task.baseline_ = 0;
        task.baseline_other_ = 4;
        task.delivery_geometry_id_ = "delivery-test";
        task.total_ = 10;
        task.near_failures_ = 3;
        task.mid_clear_n_ = 4;
        task.attempt_started_ = 777;
        task.bounded_approach_ = .2f;
        task.bounded_retreat_ = .3f;
        task.frame_raised_ = 1;
        task.frame_transaction_ = task.wire_txn_ = kTransaction;
        task.wire_last_ = 1;
        seedVerification("seed_delivery");
    }

    void seedVerification(const char* reason) {
        task.enter(PushState::VERIFY_DELIVERY, input.now_us, reason);
        task.last_us_ = input.now_us;
        task.pitch_cmd_ = task.assumeView() ? task.t_.assume_view_pitch_cdeg : task.t_.far_pitch_cdeg;
        task.pitch_wait_us_ = input.now_us;
        input.camera_pitch_cdeg = task.pitch_cmd_;
    }

    void refresh() {
        input.zone_estimate = zone(input.now_us, ++frame);
        if (!input.cargo_body.empty() && fresh_cargo) input.cargo_ts_us = input.now_us;
        input.vref_valid = vref;
        input.vref_points = vref ? 3 : 0;
        input.vref_zone = zone(input.cargo_body.empty() ? input.now_us : input.cargo_ts_us, frame);
    }

    PushOutput tick() {
        input.now_us += kTick;
        refresh();
        output = task.update(input);
        return output;
    }

    PushOutput atElapsed(uint64_t elapsed) {
        input.now_us = kStart + elapsed;
        task.last_us_ = input.now_us - kTick;
        refresh();
        output = task.update(input);
        return output;
    }

    void visualCargo(float rear_y, bool measured = true) {
        input.zone_counts_valid = false;
        input.cargo_body = {{task.t_.supply_half_x_m, .5f + rear_y}};
        input.cargo_img = {PushObservation::CargoImg{}};
        vref = measured;
        task.cargo_fix_ = measured;
        if (!measured) task.zone_.update(zone(input.now_us, 1), input.now_us);
    }

    void stationaryOpen() const {
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
        assert(output.motion.gripper_offset == 0);
        assert(output.motion.frame_transaction == kTransaction);
    }

    void report(const char* test, std::string events = {}) const {
        std::replace(events.begin(), events.end(), '\n', '|');
        std::cout << "case=" << test << " state=" << PushTask::name(output.state)
                  << " reason=" << output.reason << " total=" << output.delivered_total
                  << " first=" << output.first_ordinary_delivered << " fault=" << task.fault_
                  << " frame=" << int(output.motion.gripper_offset)
                  << " transaction=" << output.motion.frame_transaction
                  << " pitch=" << output.motion.camera_pitch_cdeg << " phase_us=" << task.phase_us_
                  << " repush=" << task.repush_n_ << " visual=" << task.delivery_visual_
                  << " near_failures=" << task.near_failures_ << " mid_clear=" << task.mid_clear_n_
                  << " attempt_us=" << task.attempt_started_
                  << " approach=" << task.bounded_approach_ << " retreat=" << task.bounded_retreat_
                  << " events=" << events << '\n';
    }
};

void scoringTests() {
    {
        Rig rig(strictTuning(), ordinary(2));
        for (int index = 0; index < 6; ++index) {
            rig.tick();
            assert(rig.output.state == PushState::VERIFY_DELIVERY && rig.output.delivered_total == 10);
            rig.stationaryOpen();
        }
        rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "delivered");
        assert(rig.output.delivered_total == 12 && rig.output.first_ordinary_delivered);
        assert(rig.task.near_failures_ == 0 && rig.task.mid_clear_n_ == 0);
        assert(rig.task.attempt_started_ == rig.input.now_us);
        assert(rig.task.bounded_approach_ == 0 && rig.task.bounded_retreat_ == 0);
        assert(rig.task.delivery_visual_ && !rig.task.fault_);
        rig.stationaryOpen();
        rig.report("strict_full");
    }
    {
        Rig rig(strictTuning(), ordinary(2));
        rig.input.zone_supply_count = 1;
        for (int index = 0; index < 7; ++index) rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "partial_delivery");
        assert(rig.output.delivered_total == 11 && !rig.output.first_ordinary_delivered);
        assert(rig.task.near_failures_ == 3 && rig.task.mid_clear_n_ == 4);
        assert(rig.task.attempt_started_ == rig.input.now_us);
        assert(rig.task.bounded_approach_ == 0 && rig.task.bounded_retreat_ == 0);
        rig.report("strict_partial");
    }
    {
        auto trip = ordinary();
        trip.core = 1;
        Rig rig(strictTuning(), trip);
        for (int index = 0; index < 7; ++index) rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.delivered_total == 12);
        assert(!rig.output.first_ordinary_delivered && rig.output.reason == "delivered");
        rig.report("mixed_full_does_not_unlock_first");
    }
    {
        auto tuning = strictTuning();
        Rig rig(tuning);
        rig.input.zone_supply_count = 0;
        rig.atElapsed(tuning.delivery_budget_us + 1);
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "delivery_unverified");
        assert(rig.output.delivered_total == 10 && !rig.output.first_ordinary_delivered);
        assert(rig.task.attempt_started_ == 777 && rig.task.bounded_approach_ == .2f && rig.task.bounded_retreat_ == .3f);
        assert(!rig.task.delivery_visual_);
        rig.report("strict_unverified");
    }
    {
        auto tuning = strictTuning();
        tuning.demo_carry_once = true;
        Rig rig(tuning);
        for (int index = 0; index < 7; ++index) rig.tick();
        assert(rig.output.state == PushState::SAFE_STOP && rig.output.reason == "demo_delivery_complete");
        assert(rig.task.fault_ && rig.task.phase_us_ == kStart);
        assert(rig.output.delivered_total == 11 && rig.output.first_ordinary_delivered);
        rig.stationaryOpen();
        rig.report("strict_demo_complete");
    }
    {
        Rig rig(visualTuning(false));
        rig.visualCargo(.02f);
        for (int index = 0; index < 3; ++index) rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "delivery_unverified");
        assert(rig.output.delivered_total == 10 && !rig.output.first_ordinary_delivered);
        assert(rig.task.delivery_visual_ && rig.task.attempt_started_ == 777);
        rig.report("visual_without_assumption", rig.task.takeDropZoneEvent());
    }
}

void pauseAndViewTests() {
    {
        Rig rig;
        rig.tick();
        rig.tick();
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.reason == "not_running");
        rig.stationaryOpen();
        rig.report("paused");
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY && rig.output.reason == "resume");
        rig.report("resumed");
        rig.tick();
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "delivered");
        assert(rig.input.now_us == kStart + 7 * kTick);
        assert(rig.output.delivered_total == 11);
        rig.report("resumed_keeps_stable_since");
    }
    {
        auto tuning = visualTuning();
        tuning.assume_verify_far_after_us = 100000;
        Rig rig(tuning);
        rig.visualCargo(-.02f, false);
        rig.tick();
        rig.tick();
        assert(rig.output.motion.camera_pitch_cdeg == tuning.assume_view_pitch_cdeg);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        assert(rig.output.motion.camera_pitch_cdeg == tuning.far_pitch_cdeg);
        const auto before_move = rig.task.takeDropZoneEvent();
        assert(!before_move.empty());
        rig.report("far_view_requested", before_move);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY && rig.output.reason == "camera_pitch_wait");
        assert(rig.task.takeDropZoneEvent().empty());
        rig.report("far_view_motion_gates_verification");
        rig.input.camera_pitch_cdeg = tuning.far_pitch_cdeg;
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        const auto after_move = rig.task.takeDropZoneEvent();
        assert(after_move.find("event=verify") != std::string::npos);
        rig.report("far_view_ready", after_move);
    }
}

void cargoHistoryTests() {
    {
        auto tuning = visualTuning();
        tuning.assume_img_verify_obs = 5;
        Rig rig(tuning);
        rig.visualCargo(.02f, false);
        rig.task.cargo_fix_ = true;
        auto& image = rig.input.cargo_img.front();
        image.height_px = 100;
        image.bottom_px = 400;
        image.front_px = 440;
        image.front_valid = true;
        rig.tick();
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "delivered_visual");
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=verify_img") < events.find("event=verify half="));
        assert(events.find("fix=0 repush=0 in=3 out=0") != std::string::npos);
        rig.report("historical_cargo_fix_sets_threshold", events);
    }
    {
        auto tuning = visualTuning();
        tuning.assume_verify_out_obs = 2;
        tuning.assume_align_push = true;
        Rig rig(tuning);
        rig.visualCargo(-.02f);
        rig.task.dropzone_evt_ = "prefix\n";
        rig.task.cargo_ref_us_ = 555;
        rig.task.assume_pushed_ = true;
        rig.task.push_in_.push_hold_valid_ = true;
        rig.task.push_in_.align_start_us_ = 666;
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.reason == "assumed_verify_repush");
        assert(rig.task.repush_n_ == 1 && rig.task.cargo_ref_us_ == 0);
        assert(!rig.task.assume_pushed_ && rig.task.push_in_.align_pending_ && rig.task.push_in_.align_start_us_ == 0 && !rig.task.push_in_.push_hold_valid_);
        rig.stationaryOpen();
        const auto events = rig.task.takeDropZoneEvent();
        const auto before = events.find("fix=1 repush=0 in=0 out=2");
        const auto after = events.find("event=verify_outside_repush");
        assert(events.rfind("prefix\n", 0) == 0 && before < after);
        assert(events.find("fix=1 repush=1 in=0 out=0", after) != std::string::npos);
        rig.report("repush_events_before_transition", events);
        rig.tick();
        assert(rig.output.motion.gripper_offset == 20 && rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("repush_closes_next_tick");
    }
    {
        Rig rig(visualTuning());
        rig.visualCargo(.02f);
        rig.tick();
        assert(rig.task.takeDropZoneEvent().find("in=1 out=0") != std::string::npos);
        rig.fresh_cargo = false;
        rig.input.reset = true;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.reason == "reset");
        rig.input.reset = false;
        rig.task.trip_ = ordinary();
        rig.seedVerification("new_delivery");
        rig.tick();
        rig.tick();
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        assert(rig.task.takeDropZoneEvent().empty());
        rig.report("duplicate_cargo_after_reset");
        rig.fresh_cargo = true;
        rig.tick();
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.tick();
        assert(rig.output.state == PushState::TURN_SCAN && rig.output.reason == "delivered_visual");
        assert(rig.output.delivered_total == 1 && rig.output.first_ordinary_delivered);
        rig.report("fresh_cargo_after_reset", rig.task.takeDropZoneEvent());
    }
}
} // namespace

int main() {
    scoringTests();
    pauseAndViewTests();
    cargoHistoryTests();
    std::cout << "Delivery verification PushTask boundary checks passed\n";
}
