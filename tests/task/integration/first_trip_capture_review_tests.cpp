#include "../../support/push_task_sim.hpp"
#include "rescue/perception/pixel_selector.hpp"
#include "../../../src/task/task_tick.hpp"
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "rescue/app/runtime_config.hpp"
#include "rescue/app/config_loader.hpp"

namespace {
TaskTuning tuning() {
    TaskTuning t;
    t.pixel_grasp_enabled = true;
    t.require_multi_view = false;
    t.startup_advance_us = 0;
    t.near_pitch_cdeg = 2000;
    t.track_pitch_cdeg = t.intermediate_pitch_cdeg = 0;
    t.far_pitch_cdeg = -3000;
    t.rush_budget_us = 15000000;
    t.capture_frames = 1;
    // Sequence-specific legacy fixtures explicitly exercise uninterrupted splits.
    // splitCapturePreemption below covers the production defaults and faster motions.
    t.split_capture_preempt = false;
    t.split_speed_increment_mps = t.split_wz_increment_rps = 0;
    return t;
}
CaptureConfig calibration() {
    CaptureConfig c;
    c.pixel_grasp_enabled = true;
    c.grasp_center_ranges = {
        {"ordinary_supply", 2000, 1280, 720, {0, 226, 1279, 489}, {647.5f, 337}, false},
        {"core_supply", 2000, 1280, 720, {268.5f, 236, 1017, 555}, {647.5f, 337}, true},
        {"injured_person", 2000, 1280, 720, {395, 236, 906, 479}, {647.5f, 337}, true}};
    return c;
}
SegDetection detection(int id, cv::Rect box) {
    SegDetection d;
    d.track_id = id;
    d.label = "ordinary_supply";
    d.confidence = .95f;
    d.box = box;
    return d;
}
struct SceneRig {
    TaskTuning t = tuning();
    PushTask task{t};
    CaptureMonitor monitor{calibration()};
    PushObservation in;
    PushOutput out;
    std::vector<SegDetection> objects;
    PixelSelector selector;
    bool managed_selection = false;
    const CaptureConfig* selector_config = nullptr;
    bool simulate_corner_imu = false;
    float corner_imu_accel_bias = 0;
    uint64_t imu_sequence = 0;
    float imu_sim_velocity = 0, imu_sim_position = 0, imu_sim_peak = 0;
    PixelSelection selection;
    void enableSelector() {
        selector.reset();
        for (int i = 0; i < 3; ++i) {
            in.now_us += 50000;
            for (auto& d : objects)
                d.timestamp_us = in.now_us;
            selection = selector.update(objects, {1280, 720}, 2000, true, in.now_us, false, .25f);
        }
        assert(selection.locked && selection.id == 1);
        managed_selection = true;
    }
    SceneRig(bool single = false, TaskTuning task_tuning = tuning()) : t(task_tuning) {
        in.now_us = 1000000;
        in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.camera_pitch_stable = in.heading_valid = in.sent_velocity_valid = true;
        in.camera_pitch_cdeg = 2000;
        in.target_valid = true;
        in.target_id = 1;
        in.label = "ordinary_supply";
        in.gripper_done = true;
        in.gripper_feedback_open = 1;
        task.last_us_ = in.now_us;
        task.pitch_cmd_ = 2000;
        if (single) {
            task.setSinglePhase("capture");
            task.update(in);
        } else {
            task.enter(PushState::RUSH, in.now_us, "review_fixture");
            task.frame_raised_ = 1;
            task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = 1;
            task.trip_context_.label_ = in.label;
            task.trip_context_.pending_.ordinary = 1;
        }
        objects = {detection(1, {600, 350, 80, 80}), detection(2, {600, 90, 80, 80})};
    }
    PushOutput step(bool fresh = true, bool feedback = true, uint64_t interval_us = 50000) {
        in.now_us += interval_us;
        in.imu_translation_samples.clear();
        if (simulate_corner_imu) {
            for (int j = 1; j <= 5; ++j) {
                const float old = imu_sim_velocity;
                imu_sim_velocity += std::clamp(out.motion.vx_mps - old, -.004f, .004f);
                imu_sim_position += .5f * (old + imu_sim_velocity) * .01f;
                imu_sim_peak = std::max(imu_sim_peak, imu_sim_position);
                ImuTranslationSample sample;
                sample.sequence = ++imu_sequence;
                sample.received_us = in.now_us - 50000 + j * 10000;
                sample.device_time_ms = uint32_t(sample.received_us / 1000);
                sample.acceleration_mps2 = {(imu_sim_velocity - old) / .01f + .01f + corner_imu_accel_bias, 0, 9.8f};
                sample.quaternion_wxyz = {std::cos(in.heading_rad / 2), 0, 0,
                                          std::sin(in.heading_rad / 2)};
                sample.valid = true;
                in.imu_translation_samples.push_back(sample);
            }
        }
        if (fresh)
            for (auto& d : objects)
                d.timestamp_us = in.now_us;
        if (feedback) {
            in.gripper_feedback_open = task.frame_raised_;
            in.gripper_closed_observed = task.frame_raised_ == 0;
        }
        if (managed_selection) {
            const bool preparing = out.phase == TaskPhase::Capture &&
                out.capture_action == CaptureAction::Prepare && !out.recovering;
            Inventory selection_load = out.capture_keep_inventory;
            if (preparing) { selection_load = {}; selection_load.add(task.targetLabel()); }
            selector.setCaptureLoad(selection_load);
            if (out.pixel_selection_paused)
                selector.reset();
            selector.syncSplitSelection(out.split_target_reselecting,
                                        out.split_target_candidate_id);
            if (out.split_return_search)
                selector.syncTaskState(PushState::SCATTER, -1, {});
            else if (!out.split_target_reselecting)
                selector.syncTaskState(
                    task.state_, task.trip_context_.target_id_, task.trip_context_.label_,
                    preparing || (task.phase_ == TaskPhase::Search && !task.recovering_ &&
                        task.search_phase_.action != SearchAction::Prepare &&
                        task.search_phase_.action != SearchAction::TurnAway &&
                        task.search_phase_.action != SearchAction::Recover));
            selection = selector.update(objects,
                                        {1280, 720},
                                        in.camera_pitch_cdeg,
                                        in.camera_pitch_stable && !out.pixel_selection_paused,
                                        in.now_us,
                                        task.first_,
                                        .25f,
                                        task.rejectedTargets(in.now_us), {},
                                        std::numeric_limits<float>::quiet_NaN(), 1.f, selector_config,
                                        !out.recovering && !out.entry_confirmed &&
                                            !out.split_target_reselecting &&
                                            (out.state == PushState::APPROACH ||
                                             out.state == PushState::RUSH) &&
                                            task.frame_raised_ == 1 && in.gripper_done &&
                                            in.gripper_feedback_open == 1 &&
                                            std::abs(in.sent_vx_mps) <= t.pixel_align_stopped_vx &&
                                            std::abs(in.sent_wz_rps) <= t.pixel_align_stopped_wz);
            in.target_valid = selection.locked;
            in.target_id = selection.locked ? selection.id : -1;
            in.label.clear();
            for (const auto& object : objects)
                if (selection.locked && object.track_id == selection.id)
                    in.label = object.label;
            in.target_box = {};
            for (const auto& object : objects)
                if (object.track_id == selection.id)
                    in.target_box = object.box;
            in.pixel_selection_managed = true;
            in.pixel_locked = selection.locked;
            in.pixel_released_id = selection.released_id;
            in.pixel_rebound_from = selection.rebound_from;
            in.pixel_reselected_from = selection.reselected_from;
        }
        monitor.update(in, objects, in.now_us, task.trip_context_.label_);
        if (!fresh)
            in.capture_scene_us = objects.empty() ? 0 : objects.front().timestamp_us;
        out = task.update(in);
        return out;
    }
    PushOutput sentStep(uint64_t interval_us = 50000) {
        // Explicit transport feedback: last control request was actually sent.
        // SceneRig::step itself intentionally keeps the default sent velocity zero.
        in.sent_vx_mps = out.motion.vx_mps;
        in.sent_wz_rps = 0;
        in.sent_velocity_valid = true;
        in.sent_velocity_us = in.now_us;
        return step(true, true, interval_us);
    }
    void finishRetreat() {
        using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
        auto& split = task.search_phase_.first_trip_split_;
        // Simulate actual sending through both visual stops and the finite margin.
        // The original deadline remains authoritative; missing evidence cannot pass.
        while (split.recovery_stage == Stage::Retreat && !task.fault_ &&
               in.now_us >= split.started_us && in.now_us - split.started_us <= t.pile_budget_us)
            sentStep();
    }
    void finishCornerTurn() {
        using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
        auto& split = task.search_phase_.first_trip_split_;
        const auto turning = split.recovery_stage;
        assert(turning == Stage::CornerTurnRight || turning == Stage::CornerTurnBack);
        for (int i = 0; i < 12 && split.pending && split.recovery_stage == turning; ++i) {
            step();
            assert(stopped(out.motion));
        }
        assert(!split.pending || split.recovery_stage != turning);
    }
    void finishObservationResearch() {
        for (int i = 0; i < 80 && task.state_ != PushState::APPROACH; ++i) {
            in.camera_pitch_cdeg = out.motion.camera_pitch_cdeg;
            step();
            assert(stopped(out.motion) && !task.fault_);
        }
        assert(task.state_ == PushState::APPROACH && !task.search_phase_.first_trip_split_.pending);
        assert(task.trip_context_.target_id_ == -1 && task.frame_raised_ == 1);
    }
    void turnCycle() {
        using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
        auto& split = task.search_phase_.first_trip_split_;
        // Simulate reverse until complete unwanted boxes leave the review regions.
        // The post-turn image is independent and can show overlap again.
        auto post_turn_objects = objects;
        if (split.recovery_stage == Stage::Retreat)
            for (size_t i = 0; i < objects.size(); ++i) {
                auto& object = objects[i];
                if (object.track_id != task.trip_context_.target_id_ &&
                    (in.splitInventoryOverlaps(object.box, object.label) ||
                     in.capturePathOverlaps(object.box, object.label))) {
                    post_turn_objects[i].box.y = int(280.f - .5f * object.box.height);
                    object.box.y = 30;
                    object.box.height = std::min(100, object.box.height);
                }
            }
        if (!split.upper_corner && split.recovery_stage == Stage::Retreat)
            for (auto& object : objects)
                if (object.track_id == task.trip_context_.target_id_)
                    object.box.y = int(270.f - .5f * object.box.height);
        finishRetreat();
        assert(split.recovery_stage == Stage::Turn);
        assert(stopped(out.motion) && out.motion.camera_pitch_cdeg == 0);
        objects = post_turn_objects;
        step(); // old UI140 feedback cannot start the UI120 turn
        assert(stopped(out.motion));
        in.camera_pitch_cdeg = 0;
        step();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == split.recovery_direction * .3f);
        const float turn_wz = split.recovery_direction * .3f;
        const auto started = in.now_us;
        while (in.now_us - started < 450000) {
            step();
            assert(out.motion.wz_rps == turn_wz && out.motion.vx_mps == 0);
        }
        step();
        assert(in.now_us - started == 500000);
        assert(stopped(out.motion) && out.immediate_motion_stop);
        assert(split.recovery_stage == Stage::Verify && out.motion.camera_pitch_cdeg == 2000);
        in.camera_pitch_cdeg = 2000;
        for (int i = 0; i < 12 && split.target_reselecting; ++i)
            step();
        assert(!split.target_reselecting);
        assert(out.reason == "first_trip_split_target_accepted");
    }
    void risk() {
        objects[1].box.y = 240; // centre 280 is in the actual jaw/path; target centre is 390.
    }
    void split() {
        risk();
        for (int i = 0; i < 16 && !task.search_phase_.first_trip_split_.pending; ++i) {
            step();
            assert(stopped(out.motion) && out.immediate_motion_stop);
        }
        assert(task.search_phase_.first_trip_split_.pending);
        assert(out.capture_split_runs == 1 && out.split_before_objects.size() == 2);
        objects[1].box.y = 300; // unwanted centre Y340 still requires retreat
        step();
    }
};
void emptySplitReobservesWithoutRetreat() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.objects.resize(1);
    rig.step();
    TaskTick tick(rig.in);
    rig.task.beginPixelSplit(tick);
    const auto& split = rig.task.search_phase_.first_trip_split_;
    assert(split.pending && split.retreat_objects.empty());
    assert(split.recovery_stage == Stage::Turn && !split.upper_corner);
    rig.step();
    assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 0);
    rig.in.camera_pitch_cdeg = 0;
    rig.in.directional_clearance_valid = true;
    rig.in.turn_safe = false;
    rig.step();
    assert(stopped(rig.out.motion));
    rig.in.turn_safe = rig.in.arc_safe = true;
    rig.in.camera_pitch_cdeg = 2000;
    rig.turnCycle();
    std::cout << "PASS empty split reobserves without reverse and retains turn clearance\n";
}
void latestSearchCorridorUsesCaptureRegions() {
    // 2026-10-31 00:03:32: ordinary #9 is selected, core #10 is below every
    // capture region. The old unbounded corridor started an empty retreat.
    for (const cv::Rect core_box : {cv::Rect{955, 572, 133, 147}, cv::Rect{955, 574, 133, 146}}) {
        SceneRig rig;
        auto c = calibration();
        c.grasp_center_ranges[0].area = {342, 226, 1031, 489};
        c.grasp_center_ranges[0].lateral_measured = true;
        rig.monitor = CaptureMonitor{c};
        rig.objects = {detection(9, {854, 286, 274, 244}), detection(10, core_box)};
        rig.objects[1].label = "core_supply";
        rig.in.target_id = 9;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 9;
        rig.task.enter(PushState::PREPARE, rig.in.now_us, "latest_search_corridor_replay");
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.step();
            if (rig.task.search_phase_.first_trip_split_.pending)
                std::cerr << "latest field replay entered empty split: " << rig.out.reason << '\n';
            assert(!rig.task.search_phase_.first_trip_split_.pending);
            assert(rig.out.reason != "pixel_split_unwanted_observation_wait");
        }
        assert(rig.in.corridor.ordinary == 1 && rig.in.corridor.core == 0);
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.out.state == PushState::CARRY);
    }
    std::cout << "PASS latest field replay: outside/clipped core cannot start empty retreat\n";
}
void actualLogOpening(const char* fixture) {
    std::ifstream file(fixture);
    assert(file.good());
    SceneRig rig(true);
    rig.objects.clear();
    int id, x, y, w, h;
    while (file >> id >> x >> y >> w >> h)
        rig.objects.push_back(detection(id, {x, y, w, h}));
    assert(rig.objects.size() == 2);
    for (int i = 0; i < 5; ++i) {
        rig.step();
        assert(rig.out.motion.vx_mps == 0);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
    }
    assert(rig.task.single_phase_initializing_); // risk review owns the stationary window
    assert(rig.in.image_layout_vertical_dominant && rig.in.split_ordinary_count == 2);
    assert(rig.out.capture_scene_status == "risk");
    // The logged X offset must not prevent the confirmed pair from starting a split.
    for (int i = 0; i < 20 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(!rig.task.single_phase_initializing_);
    assert(rig.out.capture_evidence == CaptureEvidence::None);
    std::cout << "PASS actual_log_opening: Y223 neighbor requires confirmed split\n";
}
void ordinaryInventoryReviewBoundary() {
    for (int center_y :
         {139, 140, 163, 173, 179, 180, 193, 203, 204, 205, 226, 489, 490, 529, 530}) {
        SceneRig rig;
        rig.objects[1].box.y = center_y - 40;
        rig.step();
        const bool in_path = center_y >= 205 && center_y < 489;
        // Eighty-pixel box touches the region at centre Y140 / Y529.
        const bool overlaps = center_y >= 140 && center_y <= 529;
        assert(rig.in.capture_path_valid && rig.in.capture_path_px.y == 205.f);
        assert(rig.in.capture_area_px.y == 226.f);
        assert(rig.in.capture_path_px.contains(cv::Point2f(640.f, float(center_y))) == in_path);
        assert(rig.in.ordinaryInventoryOverlaps(rig.objects[1].box) == overlaps);
        assert(rig.in.split_ordinary_count == (overlaps ? 2 : 1));
        assert(rig.out.capture_scene_status == (overlaps ? "risk" : "clear"));
        assert(rig.task.frame_raised_ == 1);
        if (overlaps) {
            assert(stopped(rig.out.motion));
            assert(rig.out.capture_scene_detail == "neighbor_box_overlaps_ordinary_inventory");
        }
    }
    std::cout << "PASS whole-box top/bottom overlap and boundary contact veto closure\n";
}
void actualAcceptanceLogBoxes() {
    const std::vector<std::vector<SegDetection>> scenes = {
        // 22:43:45 accepted two: ordinary#2 centre Y193, ordinary#1 centre Y394.
        {detection(1, {543, 268, 222, 252}), detection(2, {563, 107, 202, 172})},
        // 22:53:10 rejected only after closure: neighbour centre Y203, target Y394.
        {detection(1, {533, 263, 222, 262}), detection(2, {553, 122, 202, 162})},
        // 23:10:30 centre Y173 is outside, but the box bottom Y249 still overlaps.
        {detection(1, {538, 233, 212, 262}), detection(2, {553, 97, 202, 152})},
        // Final accepted image: centre Y163, bottom Y239; still not separated.
        {detection(1, {533, 248, 222, 252}), detection(2, {543, 87, 202, 152})}};
    for (const auto& objects : scenes) {
        SceneRig rig;
        rig.objects = objects;
        rig.task.enter(PushState::LOWER_FRAME, rig.in.now_us, "actual_log_preclose");
        for (int i = 0; i < 24 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();
            assert(rig.task.frame_raised_ == 1);
            assert(stopped(rig.out.motion));
            assert(rig.out.capture_scene_status == "risk");
            assert(rig.out.final_validation == "not_executed");
            assert(rig.out.capture_evidence == CaptureEvidence::None);
        }
        assert(rig.task.search_phase_.first_trip_split_.pending);
        std::cout << "PASS actual log replay neighbour_y=" << objects[1].center().y
                  << " neighbour_bottom=" << objects[1].box.br().y
                  << " preclose_ordinary=" << rig.in.split_ordinary_count
                  << " held_ordinary=" << rig.in.held.ordinary
                  << " scene_detail=" << rig.out.capture_scene_detail
                  << " frame_open=1 final_validation=" << rig.out.final_validation << "\n";
    }
}
void inventorySideNeighborSplitsButUnmeasuredPathWaits() {
    for (int center_y : {163, 173, 193})
        for (int left : {760, 1030}) {
            SceneRig rig;
            rig.objects[1].box = {left, center_y - 76, 80, 152};
            rig.step();
            const bool inventory_overlap = rig.in.ordinaryInventoryOverlaps(rig.objects[1].box);
            if (inventory_overlap) {
                assert(rig.out.capture_scene_status == "risk");
                assert(rig.out.capture_scene_detail == "neighbor_box_overlaps_ordinary_inventory");
                for (int i = 0; i < 16 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
                    rig.step();
                    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                }
                assert(rig.task.search_phase_.first_trip_split_.pending);
                assert(rig.out.final_validation == "not_executed");
            } else {
                for (int i = 0; i < 16; ++i) {
                    rig.step();
                    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                    assert(rig.out.capture_scene_status == "incomplete");
                    assert(rig.out.capture_scene_detail == "neighbor_lateral_range_unmeasured");
                    assert(!rig.task.search_phase_.first_trip_split_.pending);
                }
            }
        }
    std::cout << "PASS inventory side overlap grants splitting only; unmeasured path still waits\n";
}
void laterTripKeepsTwoSupplyRule() {
    SceneRig rig;
    rig.task.first_ = true;
    rig.objects[1].box = {600, 260, 80, 80};
    rig.task.trip_context_.pending_ = inv(2);
    rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "later_trip_fixture");
    rig.task.frame_raised_ = 0;
    for (int i = 0; i < 12 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i)
        rig.step();
    assert(rig.out.capture_evidence == CaptureEvidence::Verified);
    assert(rig.out.final_validation == "passed" && rig.out.batch_size == 2);
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    std::cout << "PASS later trip still verifies two ordinary supplies\n";
}
void heldOrdinaryY180Boundary() {
    for (int center_y : {139, 140, 163, 173, 179, 180, 193, 203, 204, 226, 489, 490, 529, 530}) {
        const CaptureConfig config = calibration();
        CaptureMonitor monitor(config);
        PushObservation in;
        in.camera_pitch_cdeg = 2000;
        in.camera_pitch_stable = in.target_valid = in.gripper_closed_observed = true;
        in.target_id = 1;
        in.label = "ordinary_supply";
        std::vector<SegDetection> objects = {detection(1, {600, 350, 80, 80}),
                                             detection(2, {600, center_y - 40, 80, 80})};
        for (int frame = 0; frame < config.follow_frames; ++frame) {
            in.now_us = 1000000 + frame * 50000;
            for (auto& object : objects)
                object.timestamp_us = in.now_us;
            monitor.update(in, objects, in.now_us);
        }
        const bool overlaps = center_y >= 140 && center_y <= 529;
        assert(in.held.ordinary == (overlaps ? 2 : 1));
        assert(in.split_ordinary_count == in.held.ordinary);
        assert(in.hold_observable && in.captured && in.held_complete);
        assert(in.capture_area_px.y == 226.f && in.capture_path_px.y == 205.f);
        CaptureVerificationStage verification;
        const auto result = verification.update(in, tuning(), {100000, in.label, inv(1), false});
        if (overlaps) {
            assert(result.outcome == CaptureVerificationOutcome::AbortDrop);
            assert(result.inventory.ordinary == 2);
        } else {
            assert(result.outcome == CaptureVerificationOutcome::Verified);
        }
    }
    std::cout << "PASS postclose cannot lose neighbours whose centres passed the boundary\n";
}
void measuredLateralOverlapBoundary() {
    for (int left : {419, 420, 460, 699, 700, 701}) {
        SceneRig rig;
        auto config = calibration();
        config.grasp_center_ranges[0].lateral_measured = true;
        config.grasp_center_ranges[0].area.x1 = 500;
        config.grasp_center_ranges[0].area.x2 = 700;
        rig.monitor = CaptureMonitor(config);
        rig.objects[1].box = {left, 133, 80, 80}; // centre Y173, bottom Y213
        rig.step();
        const bool overlaps = left >= 420 && left <= 700;
        assert(rig.in.split_ordinary_count == (overlaps ? 2 : 1));
        assert(rig.out.capture_scene_status == (overlaps ? "risk" : "clear"));
        assert(rig.task.frame_raised_ == 1);
    }
    std::cout << "PASS measured lateral box overlap/contact follows the same count region\n";
}
void postcloseRejectsLatestLogBoxes() {
    for (int top : {97, 87}) { // actual centres Y173/Y163, box bottoms Y249/Y239
        SceneRig rig;
        rig.objects = {detection(1, {538, 233, 212, 262}), detection(2, {553, top, 202, 152})};
        rig.task.setSinglePhase("capture");
        rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "latest_log_postclose");
        rig.task.frame_raised_ = 0;
        bool held_rejection = false;
        for (int i = 0; i < 32 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();
            assert(stopped(rig.out.motion));
            assert(rig.out.capture_evidence == CaptureEvidence::None);
            assert(rig.out.final_validation != "passed");
            held_rejection |=
                rig.in.held.ordinary == 2 && rig.out.original_reason == "held_too_many_supplies";
        }
        assert(held_rejection && rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.out.run_status != RunStatus::Fault && rig.task.frame_raised_ == 1);
    }
    std::cout << "PASS latest post-close boxes reject the extra cargo then reopen for splitting\n";
}

void overlappingOrdinaryBoxesStillSplit() {
    SceneRig overlapping(true);
    // Actual UI140 boxes from the 22:31:09 run: 12.6% overlap of the smaller box.
    overlapping.objects = {detection(1, {533, 273, 222, 262}), detection(2, {578, 127, 212, 172})};
    overlapping.task.t_.corridor_confirm_frames = 6; // same as the field runtime
    overlapping.step(); // enter the single-phase initialization before evaluating a scene
    for (int i = 0; i < 24 && !overlapping.task.search_phase_.first_trip_split_.pending; ++i) {
        overlapping.step();
        assert(overlapping.out.capture_scene_status == "risk");
        assert(stopped(overlapping.out.motion));
    }
    assert(!overlapping.task.single_phase_initializing_);
    assert(overlapping.task.search_phase_.first_trip_split_.pending);
    assert(overlapping.out.capture_split_runs == 1);
    assert(overlapping.out.capture_evidence == CaptureEvidence::None);
    overlapping.step();
    assert(overlapping.out.motion.vx_mps == -.01f); // centre Y213 is not whole-box clearance
    std::cout << "PASS overlapping ordinary boxes enter confirmed reverse/right split\n";
}
void startupAlreadyInJawStillSplits() {
    SceneRig rig(true);
    rig.risk();
    for (int i = 0; i < 20 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.capture_evidence == CaptureEvidence::None);
    std::cout << "PASS startup_confirmed_pair: stationary review grants split\n";
}
void splitCountsEveryCargoClass() {
    SceneRig rig;
    rig.task.t_.pile_max_runs = 4;
    rig.split();
    rig.turnCycle();
    rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.reason == "first_trip_split_retry_retreat");
    rig.objects[1].box.y = 300; // repeat until the unwanted block clears Y280
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.turnCycle();
    rig.objects[1].box = {600, 90, 80, 80};
    auto core = detection(3, {800, 260, 80, 80});
    core.label = "core_supply";
    rig.objects.push_back(core);
    rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.reason == "first_trip_split_retry_retreat");
    assert(rig.in.split_ordinary_count == 1 && rig.task.frame_raised_ == 1);
    // One ordinary block cannot finish first-trip review while core remains.
    rig.turnCycle();
    rig.objects.pop_back(); // remove the core after the turn, for legal stopped review
    for (int i = 0; i < 12 && rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.reason == "first_trip_split_separation_recheck");
    SceneRig disappeared;
    disappeared.split();
    disappeared.turnCycle();
    disappeared.objects.pop_back();
    for (int i = 0; i < 12 && disappeared.task.search_phase_.first_trip_split_.pending; ++i)
        disappeared.step();
    assert(!disappeared.task.search_phase_.first_trip_split_.pending);
    std::cout << "PASS every cargo class participates in split completion\n";
}

void splitReverseTurnKeepsUi140AndSafety() {
    SceneRig rig;
    rig.split();
    rig.step();
    assert(rig.out.motion.camera_pitch_cdeg == 2000);
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.in.retreat_safe = false;
    rig.step();
    assert(stopped(rig.out.motion));
    rig.in.retreat_safe = true;
    rig.in.camera_pitch_stable = false;
    rig.step();
    assert(stopped(rig.out.motion));
    rig.in.camera_pitch_stable = true;
    rig.objects[1].box.y = 241; // unwanted centre Y281 must keep retreating.
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.objects[1].box.y = 240; // centre Y280 alone still overlaps inventory.
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f);
    rig.objects[1].box.y = 99; // whole box ends at Y179, above inventory/path.
    rig.step();
    assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
    assert(rig.out.reason == "first_trip_split_retreat_brake");
    assert(rig.task.search_phase_.first_trip_split_.recovery_stage ==
           SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat);
    rig.turnCycle();
    rig.task.search_phase_.first_trip_split_.started_us = rig.in.now_us - rig.t.pile_budget_us;
    rig.step();
    assert(!rig.task.fault_ && stopped(rig.out.motion));
    rig.finishObservationResearch();
    std::cout << "PASS staged split UI140 retreat, UI120 0.3rad/s 0.5s, UI140 review\n";
}

void splitUsesUnwantedBlockAndTurnClearance() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.split();
    rig.objects[0].box.y = 230; // selected target is already short of Y280
    rig.objects[1].box.y = 360; // deeper neighbour must still cause straight retreat
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.objects[0].box.y = 240;
    rig.objects[1].box.y = 99; // unwanted whole box has cleared the review regions
    auto& split = rig.task.search_phase_.first_trip_split_;
    rig.finishRetreat();
    assert(split.recovery_stage == Stage::Turn);
    rig.in.camera_pitch_cdeg = 0;
    rig.in.directional_clearance_valid = true;
    rig.in.turn_safe = false;
    for (int i = 0; i < 12; ++i) {
        rig.step();
        assert(stopped(rig.out.motion) && split.recovery_turn_us == 0);
    }
    rig.in.turn_safe = true;
    rig.step();
    assert(rig.out.motion.wz_rps == -.3f && rig.out.motion.vx_mps == 0);
    rig.in.run = false;
    rig.step();
    assert(stopped(rig.out.motion));
    assert(split.recovery_stage == Stage::Retreat && split.recovery_turn_us == 0);
    std::cout << "PASS unwanted block drives retreat; turn clearance and pause respected\n";
}
void approachRiskStopsBeforeClose() {
    SceneRig rig;
    rig.step(); // remember both objects while safely separated
    rig.objects[1].box.y = 120;
    rig.step();
    rig.objects[1].box.y = 160;
    rig.step();
    rig.objects[1].box.y = 200;
    rig.in.sent_vx_mps = .05f;
    rig.step();
    assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
    assert(rig.task.frame_raised_ == 1 && !rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.capture_review_stage == "approach");
    assert(rig.out.final_validation == "not_executed");
    for (int i = 0; i < 6; ++i)
        rig.step();
    assert(!rig.task.search_phase_.first_trip_split_.pending); // residual motion vetoes reverse
    rig.in.sent_vx_mps = 0;
    for (int i = 0; i < 16 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    std::cout << "PASS approach second-block risk brakes before split or closure\n";
}
void separationNeedsOneInsideAndNewStationaryFrames() {
    SceneRig rig;
    rig.split();
    rig.turnCycle();
    rig.objects[1].box.y = 99; // whole box bottom Y179, outside inventory
    rig.in.sent_wz_rps = -.1f;
    for (int i = 0; i < 8; ++i)
        rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    rig.in.sent_wz_rps = 0;
    rig.step();
    for (int i = 0; i < 3; ++i)
        rig.step(false);
    assert(rig.task.search_phase_.first_trip_split_.pending);
    for (int i = 0; i < 12 && rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.reason == "first_trip_split_separation_recheck");
    assert(rig.out.motion.camera_pitch_cdeg == 2000 && stopped(rig.out.motion));
    std::cout << "PASS UI140 verification needs fresh stopped frames\n";
}

void postSplitClearOutsideGraspResumesApproach() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true})
        for (bool jitter : {false, true}) {
            SceneRig rig(single);
            rig.task.single_phase_initializing_ = false;
            rig.task.t_.corridor_confirm_frames = 6;
            auto c = calibration();
            c.pixel_center_membership = true;
            c.grasp_center_ranges[0].area = {209, 226, 1070, 499};
            c.grasp_center_ranges[0].lateral_measured = true;
            rig.monitor = CaptureMonitor(c);
            rig.managed_selection = true;
            // 2026-11-05 05:22:26: target #28 is clear but centred at Y213.5.
            rig.objects = {detection(28, {639, 67, 252, 293}),
                           detection(31, {457, 21, 253, 283})};
            auto& split = rig.task.search_phase_.first_trip_split_;
            split.pending = split.capture_retry = split.vertical_swing = true;
            split.runs = 1;
            split.started_us = split.recovery_view_us = rig.in.now_us;
            split.recovery_stage = Stage::Verify;
            split.target_reselecting = true;
            rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
            rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "logged_clear_outside_grasp");
            rig.in.sent_vx_mps = .02f;
            for (int i = 0; i < 8; ++i) {
                rig.step();
                assert(split.pending && stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            }
            rig.in.sent_vx_mps = 0;
            for (int i = 0; i < 12; ++i) {
                rig.step(false);
                assert(split.pending && stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            }
            for (int i = 0; i < 32 && split.pending; ++i) {
                rig.objects[0].box.y = jitter && i % 2 ? 87 : 67;
                rig.step();
                assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                assert(rig.out.capture_evidence == CaptureEvidence::None);
            }
            if (split.pending)
                std::cerr << "clear target above grasp range remains stopped: " << rig.out.reason
                          << " y=" << rig.in.target_center_px.y << std::endl;
            assert(!split.pending && rig.out.state == PushState::PREPARE);
            assert(rig.out.reason == "first_trip_split_separation_recheck");
            assert(rig.out.target_id == 28 && split.runs == 1);
            rig.objects[0].box.y = 67;
            bool approached = false;
            for (int i = 0; i < 100 && !approached; ++i) {
                rig.in.camera_pitch_cdeg = rig.out.motion.camera_pitch_cdeg;
                rig.step();
                assert(!rig.task.fault_ && rig.task.frame_raised_ == 1);
                assert(rig.out.capture_evidence == CaptureEvidence::None);
                if (rig.in.target_grasp_range_applicable && !rig.in.target_grasp_range_inside)
                    assert(!rig.task.pixelCaptureReady(rig.in));
                approached = rig.out.motion.vx_mps > 0;
                if (rig.out.motion.wz_rps != 0)
                    rig.objects[0].box.x = 521; // simulate ordinary image alignment
            }
            assert(approached);
        }
    std::cout << "PASS logged Y213.5 and boundary jitter resume approach after stopped fresh review; "
                 "closure still requires grasp range\n";
}

void latestLogSplitClosesWithoutForward() {
    for (bool single : {false, true}) {
        SceneRig rig(single);
        // 00:00:01 log: two collinear ordinary boxes before reverse/right.
        rig.objects = {detection(1, {553, 253, 202, 242}), detection(2, {543, 97, 202, 152})};
        for (int i = 0; i < 40 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();
            assert(rig.out.motion.vx_mps == 0 && rig.task.frame_raised_ == 1);
        }
        assert(rig.task.search_phase_.first_trip_split_.pending);
        for (int i = 0; i < 3; ++i)
            rig.step();
        assert(rig.out.motion.vx_mps == -.01f); // centre Y173 still leaves a bottom overlap
        rig.turnCycle();
        // First one-inside frame, followed by the actual stopped target at Y293.5.
        rig.objects = {detection(1, {533, 197, 202, 253}), detection(2, {533, 31, 202, 142})};
        rig.step();
        assert(stopped(rig.out.motion));
        rig.objects = {detection(1, {533, 167, 202, 253}), detection(2, {523, 21, 202, 142})};
        for (int i = 0; i < 16 && rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
        }
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.in.target_grasp_range_inside && !rig.in.pixel_grasp_ready);
        bool aligned = false, preclose = false, split_position = false;
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.step();
            assert(rig.out.motion.vx_mps == 0);
            assert(rig.out.run_status != RunStatus::Fault);
            preclose |= rig.out.capture_review_stage == "preclose";
            split_position |= rig.out.reason == "entry_handoff_enclosure_position";
            if (rig.task.frame_raised_ == 0) {
                assert(rig.in.split_ordinary_count == 1);
                assert(rig.in.target_grasp_range_inside && rig.in.pixel_grasp_aligned);
            }
            if (rig.out.motion.wz_rps != 0) {
                assert(rig.task.frame_raised_ == 1);
                aligned = true;
                // Shift the camera boxes by 10 px after the observed alignment turn.
                for (auto& object : rig.objects)
                    object.box.x += 10;
            }
        }
        assert(aligned && preclose && split_position);
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.out.final_validation == "passed" && rig.out.batch_size == 1);
        assert(!rig.out.first_ordinary_delivered);
        if (single)
            assert(rig.out.single_phase_done);
        std::cout << "PASS 00:00:01 log replay: Y293.5 stays still through alignment/close/verify"
                  << " single=" << single << "\n";
    }
}
void splitPositionDoesNotBypassCaptureChecks() {
    SceneRig rig;
    rig.objects[0].box.y = 240; // Y280 is in range, but below the normal Y337 stop line.
    rig.step();
    assert(!rig.in.pixel_grasp_ready && !rig.task.pixelCaptureReady(rig.in));
    auto& split = rig.task.search_phase_.first_trip_split_;
    split.capture_retry = true;
    split.runs = 1;
    assert(rig.task.pixelCaptureReady(rig.in));
    split.pending = true;
    assert(!rig.task.pixelCaptureReady(rig.in));
    split.pending = false;
    rig.in.capture_scene_us = rig.in.now_us - rig.t.frame_timeout_us - 1;
    assert(!rig.task.pixelCaptureReady(rig.in));
    rig.step();
    rig.objects[0].box.y = 170; // target centre Y210 is outside the calibrated range.
    rig.step();
    assert(!rig.task.pixelCaptureReady(rig.in) && rig.task.frame_raised_ == 1);
    rig.objects[0].box.y = 240;
    rig.risk();
    rig.step();
    assert(rig.in.split_ordinary_count == 2 && !rig.task.pixelCaptureReady(rig.in));
    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
    assert(rig.out.capture_evidence == CaptureEvidence::None);
    std::cout << "PASS recovery position keeps normal Y337, range, count and freshness checks\n";
}

void failureKeepsCauseAndUnexecutedValidation() {
    SceneRig rig;
    rig.task.setSinglePhase("capture");
    rig.task.actionProgress().started_us = rig.in.now_us - rig.t.rush_budget_us;
    rig.step();
    assert(rig.out.reason == "single_phase_recovery_required");
    assert(rig.out.original_reason == "pixel_capture_timeout");
    assert(!rig.out.failure_source.empty() && rig.out.exit_reason == rig.out.reason);
    assert(rig.out.final_validation == "not_executed" &&
           rig.out.capture_evidence == CaptureEvidence::None);
    std::cout << "PASS failure_source=" << rig.out.failure_source
              << " original_reason=" << rig.out.original_reason
              << " exit_reason=" << rig.out.exit_reason
              << " final_validation=" << rig.out.final_validation << "\n";
}
void safeSingleClosesAndVerifies() {
    for (bool visible_outside_neighbour : {false, true}) {
        SceneRig rig(true);
        if (!visible_outside_neighbour)
            rig.objects.pop_back();
        bool preclose = false;
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.step();
            preclose |= rig.out.capture_review_stage == "preclose";
            assert(!rig.task.search_phase_.first_trip_split_.pending);
            if (rig.out.run_status == RunStatus::Fault)
                std::cerr << rig.out.reason << " original=" << rig.out.original_reason << "\n";
            assert(rig.out.run_status != RunStatus::Fault);
        }
        assert(preclose && rig.out.final_validation == "passed");
        assert(rig.out.capture_evidence == CaptureEvidence::Verified && rig.out.single_phase_done);
        assert(!rig.out.first_ordinary_delivered && rig.out.batch_size == 1);
    }
    std::cout << "PASS single capture with no neighbour or a visibly non-overlapping neighbour\n";
}
void outsideClippedNeighborDoesNotBlockSplit() {
    for (bool present_at_start : {false, true}) {
        SceneRig rig;
        rig.objects = {detection(1, {568, 358, 232, 243}),
                       detection(3, {628, 167, 223, 172})};
        rig.objects[1].label = "dangerous_object";
        const auto clipped = detection(4, {0, 0, 51, 161});
        if (present_at_start)
            rig.objects.push_back(clipped);
        rig.step();
        assert(rig.out.capture_scene_status == "risk");
        if (!present_at_start)
            rig.objects.push_back(clipped);
        for (int i = 0; i < 20 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            // An unrelated edge box may change identity or disappear entirely.
            if (i == 1)
                rig.objects.back().track_id = 40;
            if (i == 2)
                rig.objects.pop_back();
            rig.step();
            assert(stopped(rig.out.motion));
            assert(rig.task.frame_raised_ == 1);
            assert(rig.out.capture_scene_status == "risk");
        }
        assert(rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.out.capture_split_runs == 1);
    }
    // A clipped neighbour touching the path must not authorize blind retreat/closure.
    SceneRig blocked;
    blocked.risk();
    blocked.objects.push_back(detection(4, {0, 0, 51, 205}));
    for (int i = 0; i < 12 && !blocked.task.search_phase_.first_trip_split_.pending; ++i) {
        blocked.step();
        assert(stopped(blocked.out.motion));
        assert(blocked.out.capture_scene_status == "incomplete");
        assert(blocked.task.frame_raised_ == 1);
    }
    assert(blocked.task.search_phase_.first_trip_split_.pending);
    assert(blocked.task.search_phase_.first_trip_split_.recovery_stage ==
           SearchPhaseData::FirstTripSplit::RecoveryStage::Turn);
    std::cout << "PASS outside clipped neighbour ignored; path clipping reobserves without retreat\n";
}

void invalidAssociationsWait() {
    SceneRig rig;
    rig.step();
    rig.in.camera_pitch_cdeg = 2050;
    rig.step();
    assert(stopped(rig.out.motion));
    assert(rig.out.reason.find("cross_view_reassociation_required") != std::string::npos);
    SceneRig other;
    other.risk(); // Ambiguity in the action region must still stop review.
    other.step();
    other.objects.push_back(detection(3, other.objects.back().box));
    other.objects.back().box.x += 1;
    other.step();
    assert(stopped(other.out.motion) && other.out.capture_scene_status == "incomplete");
    std::cout << "PASS cross-view and ambiguous re-ID require fresh reassessment\n";
}

void missingNeighborReobservesAndClosesAfterFreshScene() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.objects = {detection(1, {538, 207, 212, 253}), detection(17, {578, 1, 343, 202})};
    rig.objects[1].label = "injured_person";
    rig.step();
    assert(rig.out.capture_scene_status == "risk");
    const auto review_started = rig.task.capture_scene_.started_us;
    rig.objects.pop_back();
    for (int i = 0; i < 16 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
        rig.step();
        assert(rig.task.frame_raised_ == 1 && stopped(rig.out.motion));
        assert(rig.out.capture_scene_blocker_id == 17);
        assert(rig.out.capture_scene_association == "missing");
    }
    auto& split = rig.task.search_phase_.first_trip_split_;
    assert(split.pending && split.recovery_stage == Stage::Turn);
    assert(rig.out.reason == "first_trip_scene_reobserve_turn");
    assert(split.started_us == review_started);
    rig.in.directional_clearance_valid = true;
    rig.in.turn_safe = false;
    rig.in.camera_pitch_cdeg = 0;
    for (int i = 0; i < 8; ++i) {
        rig.step();
        assert(stopped(rig.out.motion) && !split.recovery_turn_us);
    }
    rig.in.turn_safe = true;
    for (int i = 0; i < 20 && split.recovery_stage == Stage::Turn; ++i)
        rig.step();
    assert(split.recovery_stage == Stage::Verify && rig.task.frame_raised_ == 1);
    // An explicitly visible, complete neighbour outside the inventory/path.
    rig.objects.push_back(detection(18, {578, 40, 343, 100}));
    rig.objects.back().label = "injured_person";
    rig.in.camera_pitch_cdeg = 2000;
    for (int i = 0; i < 24 && split.pending; ++i) {
        rig.step();
        assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
    }
    assert(!split.pending);
    for (int i = 0; i < 30 && rig.task.frame_raised_ != 0; ++i) {
        rig.step();
        assert(rig.out.motion.vx_mps == 0);
    }
    assert(rig.task.frame_raised_ == 0);
    std::cout << "PASS latest-log missing injured neighbour reobserves under clearance then closes\n";
}

void uncertainSceneRecoveryRequiresFreshStoppedFramesAndKeepsBudget() {
    for (bool moving : {false, true}) {
        SceneRig rig;
        rig.risk();
        rig.step();
        rig.objects.pop_back();
        if (moving)
            rig.in.sent_wz_rps = .03f;
        for (int i = 0; i < 12; ++i) {
            rig.step(moving); // stationary case repeats stale images
            assert(!rig.task.search_phase_.first_trip_split_.pending);
            assert(rig.task.frame_raised_ == 1 && stopped(rig.out.motion));
        }
    }
    for (bool clipped : {false, true}) {
        SceneRig rig;
        rig.risk();
        rig.step();
        const auto started = rig.task.capture_scene_.started_us;
        if (clipped)
            rig.objects[1].box = {600, 0, 80, 240};
        else {
            rig.objects.push_back(rig.objects[1]);
            rig.objects.back().track_id = 3;
        }
        for (int i = 0; i < 16 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
            rig.step();
        assert(rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.task.frame_raised_ == 1);
        rig.in.camera_pitch_cdeg = 0;
        rig.step(); // apply the recovery view before expiring its original budget
        rig.in.now_us = started + rig.t.pile_budget_us;
        rig.task.last_us_ = rig.in.now_us; // emulate timely ticks up to the deadline
        rig.step();
        assert(rig.out.run_status == RunStatus::Fault && stopped(rig.out.motion));
        assert(rig.out.reason == "first_trip_vertical_split_timeout");
    }
    std::cout << "PASS uncertain recovery rejects stale/moving frames and retains original deadline\n";
}

void precloseNeverBypassesMissingOrMovingEvidence() {
    for (int scenario = 0; scenario < 4; ++scenario) {
        SceneRig rig;
        if (scenario == 2)
            rig.risk(); // Missing evidence concerns a neighbour in the action region.
        rig.task.enter(PushState::LOWER_FRAME, rig.in.now_us, "preclose_fixture");
        rig.step();
        if (scenario == 0) {
            for (int i = 0; i < 8; ++i)
                rig.step(false);
        } else if (scenario == 1) {
            rig.in.sent_wz_rps = .03f;
            for (int i = 0; i < 8; ++i)
                rig.step();
        } else if (scenario == 2) {
            rig.objects.pop_back();
            for (int i = 0; i < 8 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
                rig.step(); // recovery entry stops; it must never close on missing evidence
        } else {
            rig.objects[0].box.y = 310;
            rig.step();                 // centre 350
            rig.objects[0].box.y = 270; // centre 310 is not yet at the close line
            for (int i = 0; i < 8; ++i)
                rig.step();
        }
        assert(rig.task.frame_raised_ == 1 && stopped(rig.out.motion));
        assert(rig.out.final_validation == "not_executed");
    }
    std::cout << "PASS preclose blocks stale/missing/moving/not-ready evidence\n";
}
void clearWindowRestartsWhenRiskReturns() {
    SceneRig rig;
    rig.split();
    rig.turnCycle();
    rig.objects[1].box.y = 99;
    for (int i = 0; i < 4; ++i)
        rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    rig.objects[1].box.y = 100; // whole box touches Y180 again
    rig.step();
    assert(rig.out.capture_scene_status == "risk");
    assert(rig.out.reason == "first_trip_split_retry_retreat");
    rig.turnCycle();
    rig.objects[1].box.y = 99;
    for (int i = 0; i < 12 && rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    std::cout << "PASS renewed inventory contact repeats entire split cycle\n";
}

void postcloseRejectsTwoWithoutSuccessOrMotion() {
    SceneRig rig;
    rig.task.setSinglePhase("capture");
    rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "postclose_fixture");
    rig.task.frame_raised_ = 0;
    rig.risk();
    bool rejected = false;
    for (int i = 0; i < 32 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
        rig.step();
        rejected |= rig.out.failure_source == "postclose_verification" &&
                    rig.out.final_validation == "failed";
        assert(stopped(rig.out.motion));
        assert(rig.out.capture_evidence == CaptureEvidence::None && !rig.out.single_phase_done);
    }
    assert(rejected && rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.task.frame_raised_ == 1 && rig.out.final_validation != "passed");
    std::cout << "PASS post-close rejection credits no success and stops before local splitting\n";
}

void postcloseUsesOnlyHeldCargo() {
    for (bool outside_visible : {false, true}) {
        SceneRig rig;
        auto config = calibration();
        config.pixel_center_membership = true;
        rig.monitor = CaptureMonitor(config);
        rig.objects = {detection(1, {600, 350, 80, 80}), detection(2, {600, 150, 80, 80})};
        rig.objects[1].label = "core_supply";
        rig.step();
        // Remember an unwanted core in the approach path, outside the jaw.
        rig.task.evaluateCaptureScene(rig.in);
        assert(rig.task.capture_scene_.objects.size() == 2);
        rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "held_only_fixture");
        rig.task.frame_raised_ = 0;
        rig.task.trip_context_.pending_ = inv(1);
        if (!outside_visible)
            rig.objects.pop_back();
        bool verified = false;
        for (int i = 0; i < 16 && !verified; ++i) {
            rig.step();
            assert(rig.in.held == inv(1));
            assert(rig.task.frame_raised_ == 0);
            assert(!rig.task.search_phase_.first_trip_split_.pending);
            assert(stopped(rig.out.motion));
            verified = rig.out.capture_evidence == CaptureEvidence::Verified;
        }
        assert(verified && rig.out.state == PushState::CARRY);
        assert(rig.out.phase == TaskPhase::Transport && rig.out.final_validation == "passed");
    }
    std::cout << "PASS postclose transports the wanted held block despite visible or missing "
                 "outside core\n";
}

void postcloseOutsideBoxesDoNotBlockHeldEvidence() {
    SceneRig rig;
    auto config = calibration();
    config.pixel_center_membership = true;
    config.occlusion_iou = .01f;
    rig.monitor = CaptureMonitor(config);
    rig.objects = {detection(1, {600, 240, 80, 200}), detection(2, {600, 100, 80, 180})};
    rig.objects[1].label = "core_supply"; // centre Y190 is outside the core jaw range
    rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "outside_box_overlap_fixture");
    rig.task.frame_raised_ = 0;
    rig.task.trip_context_.pending_ = inv(1);
    for (int i = 0; i < 16 && rig.out.state != PushState::CARRY; ++i) {
        rig.step();
        assert(rig.in.held == inv(1));
        assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 0);
    }
    assert(rig.in.held_complete && rig.out.state == PushState::CARRY);
    std::cout << "PASS outside projected boxes cannot invalidate complete held cargo\n";
}

void postcloseRequiresFreshHeldImages() {
    SceneRig rig;
    rig.objects.resize(1);
    rig.step();
    rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "held_freshness_fixture");
    rig.task.frame_raised_ = 0;
    rig.task.t_.capture_frames = 2;
    rig.task.trip_context_.pending_ = inv(1);
    // Repeated pre-close images cannot establish post-close evidence.
    for (int i = 0; i < 8; ++i) {
        rig.step(false);
        assert(rig.out.capture_evidence == CaptureEvidence::None);
        assert(rig.task.frame_raised_ == 0 && stopped(rig.out.motion));
    }
    for (int i = 0; i < 16 && !rig.in.held_complete; ++i)
        rig.step();
    assert(rig.in.held_complete && rig.out.capture_evidence == CaptureEvidence::None);
    // A complete image counts only once, even across multiple control ticks.
    for (int i = 0; i < 4; ++i) {
        rig.step(false);
        assert(rig.out.capture_evidence == CaptureEvidence::None);
    }
    rig.step();
    assert(rig.out.capture_evidence == CaptureEvidence::Verified);
    assert(rig.out.state == PushState::CARRY);
    std::cout << "PASS postclose requires fresh distinct held images after closure\n";
}

void riskReviewStartsSplitByYOnly() {
    for (bool single : {false, true})
        for (int shift : {-100, 100})
            for (int center_y : {225, 226, 390, 489, 490}) {
                SceneRig rig(single);
                // Isolate the Y-only split-entry rule from initial nearest-target selection,
                // which is covered by the latest diagonal-pair replay below.
                if (single) {
                    rig.step(); // enter the direct capture phase from WAIT_START
                    for (int i = 0; i < 20 && rig.task.single_phase_initializing_; ++i)
                        rig.step();
                    assert(!rig.task.single_phase_initializing_);
                }
                rig.risk();
                rig.task.t_.corridor_confirm_frames = 6; // field runtime
                for (auto& object : rig.objects)
                    object.box.x += shift;
                rig.objects[0].box.y = center_y - rig.objects[0].box.height / 2;
                rig.task.capture_scene_ = {}; // start this fixture at its requested position
                const bool in_y_range = center_y <= 489; // side targets above Y226 use upper split
                for (int i = 0; i < 20 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
                    rig.step();
                    assert(stopped(rig.out.motion));
                    assert(rig.task.frame_raised_ == 1);
                    assert(!rig.in.pixel_grasp_aligned && !rig.task.pixelLateralAccepted(rig.in));
                }
                assert(rig.in.target_grasp_range_valid);
                assert(rig.task.search_phase_.first_trip_split_.pending == in_y_range);
                assert(rig.out.capture_split_runs == (in_y_range ? 1 : 0));
                assert(rig.out.capture_evidence == CaptureEvidence::None);
                if (in_y_range) {
                    assert(rig.out.reason == "first_trip_confirmed_capture_path_split");
                    rig.step();
                    assert(rig.out.motion.vx_mps <= 0 && rig.out.motion.wz_rps == 0);
                } else {
                    assert(rig.out.reason == "first_trip_stop_review_target_not_at_grasp");
                }
            }
    // Even a measured X boundary must not become a split-entry position gate.
    for (int center_x : {540, 740}) {
        SceneRig rig;
        auto config = calibration();
        config.grasp_center_ranges[0].lateral_measured = true;
        config.grasp_center_ranges[0].area.x1 = 600;
        config.grasp_center_ranges[0].area.x2 = 700;
        rig.monitor = CaptureMonitor(config);
        rig.risk();
        rig.objects[0].box.x = center_x - 40;
        for (int i = 0; i < 20 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();
            assert(stopped(rig.out.motion));
        }
        assert(rig.in.target_grasp_range_valid && rig.in.target_grasp_range_lateral_measured);
        assert(!rig.in.target_grasp_range_lateral && !rig.in.target_grasp_range_inside);
        assert(rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.out.capture_evidence == CaptureEvidence::None);
    }
    std::cout << "PASS split starts at Y226..489 without X alignment; outside Y waits\n";
}

void splitSelectorReleaseAndNewIdsRetry() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.risk();
    rig.enableSelector();
    rig.split();
    auto& split = rig.task.search_phase_.first_trip_split_;
    rig.objects[1].box.y = 99; // unwanted whole box clears inventory and path
    rig.finishRetreat();
    assert(split.recovery_stage == Stage::Turn);
    rig.in.camera_pitch_cdeg = 0;
    bool released = false;
    for (int i = 0; i < 16 && split.recovery_stage == Stage::Turn; ++i) {
        rig.step();
        released |= rig.selection.event == "target_lost_timeout";
    }
    assert(released && split.recovery_stage == Stage::Verify);
    assert(rig.out.split_target_reselecting && rig.out.target_id == -1);
    assert(rig.out.split_align_stage == "ui140_verify" && stopped(rig.out.motion));
    // Actual latest-log boxes: old ID 1 is gone; the farther block has the larger area.
    rig.objects = {detection(12, {342, 72, 282, 302}), detection(11, {120, 318, 243, 283})};
    rig.in.camera_pitch_cdeg = 2000;
    rig.step();
    assert(rig.out.split_target_candidate_id == 11 && stopped(rig.out.motion));
    for (int i = 0; i < 2; ++i) {
        rig.step();
        assert(rig.out.split_target_reselecting && !rig.in.target_valid);
        assert(stopped(rig.out.motion));
    }
    rig.step();
    assert(rig.out.reason == "first_trip_split_target_accepted");
    assert(rig.out.target_id == 11 && rig.out.capture_target_id == 11 && split.target_id == 11);
    assert(rig.in.split_ordinary_count == 2 && rig.task.capture_scene_.objects.size() == 2);
    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
    rig.step();
    assert(rig.out.reason == "first_trip_split_retry_retreat");
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f); // #12 centre Y223 still overlaps inventory
    rig.objects[0].box = {342, 30, 100, 100}; // fresh complete box after further reverse
    rig.finishRetreat();
    assert(split.recovery_stage == Stage::Turn);
    rig.in.camera_pitch_cdeg = 0;
    int turn_frames = 0;
    for (int i = 0; i < 16 && split.recovery_stage == Stage::Turn; ++i) {
        rig.step();
        turn_frames += rig.out.motion.wz_rps == -.3f;
    }
    assert(turn_frames >= 10 && split.recovery_stage == Stage::Verify);
    rig.objects[0].box = {342, 1, 282, 170}; // whole far box outside inventory
    rig.objects[1].track_id = 21;
    rig.in.camera_pitch_cdeg = 2000;
    for (int i = 0; i < 24 && split.pending; ++i)
        rig.step();
    assert(!split.pending && rig.out.reason == "first_trip_split_separation_recheck");
    assert(rig.out.target_id == 21 && rig.in.split_ordinary_count == 1);
    assert(rig.out.final_validation == "not_executed");
    assert(rig.task.original_reason_ == "first_trip_confirmed_capture_path_split");
    std::cout << "PASS real selector release -> IDs 11/12 -> three-frame target 11 -> second turn "
                 "-> target 21\n";
}
void splitReselectionMissingEvidenceStops() {
    for (int invalid = 1; invalid < 4; ++invalid) {
        SceneRig rig;
        rig.split();
        rig.turnCycle();
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.target_reselecting = true;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
        rig.in.target_valid = false;
        rig.in.target_id = -1;
        if (invalid == 0)
            rig.objects.clear();
        if (invalid == 1)
            rig.objects[0].box.x = 0;
        if (invalid == 2)
            rig.in.camera_pitch_stable = false;
        for (int i = 0; i < 8; ++i) {
            rig.step(invalid != 3);
            assert(split.pending && split.target_reselecting && stopped(rig.out.motion));
            assert(rig.task.frame_raised_ == 1 && rig.out.final_validation == "not_executed");
        }
        split.started_us = rig.in.now_us - rig.t.pile_budget_us;
        rig.in.camera_pitch_stable = true;
        rig.step();
        assert(!rig.task.fault_ && stopped(rig.out.motion));
        rig.finishObservationResearch();
    }
    std::cout << "PASS clipped/unstable/stale reacquisition restarts search after stopped review expiry\n";
}

void initialCaptureReviewsAllFourCargoClasses() {
    const std::array<std::string, 4> labels{
        "ordinary_supply", "core_supply", "injured_person", "dangerous_object"};
    const std::array<cv::Rect, 4> boxes{cv::Rect{600, 350, 80, 80},
                                        cv::Rect{820, 250, 80, 80},
                                        cv::Rect{500, 280, 100, 80},
                                        cv::Rect{740, 250, 60, 60}};
    for (bool first_delivered : {false, true})
        for (unsigned mask = 1; mask < 16; ++mask) {
            SceneRig rig(true);
            rig.task.first_ = first_delivered;
            rig.task.t_.corridor_confirm_frames = 6;
            rig.managed_selection = true;
            rig.objects.clear();
            Inventory inventory;
            bool selectable = false;
            for (unsigned i = 0; i < labels.size(); ++i)
                if (mask & (1u << i)) {
                    auto object = detection(int(i + 1), boxes[i]);
                    object.label = labels[i];
                    rig.objects.push_back(object);
                    inventory.add(labels[i]);
                    selectable |= targetSelectable(labels[i], first_delivered);
                }
            const auto verdict = checkTrip(inventory, first_delivered);
            const bool needs_split = selectable && verdict != RuleVerdict::OK;
            bool split_started = false, initialized = false;
            for (int i = 0; i < 24; ++i) {
                std::reverse(rig.objects.begin(), rig.objects.end());
                rig.step();
                assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                assert(!rig.out.single_phase_done &&
                       rig.out.capture_evidence == CaptureEvidence::None);
                assert(!rig.out.split_target_reselecting);
                split_started = rig.task.search_phase_.first_trip_split_.pending;
                initialized = !rig.task.single_phase_initializing_;
                if (split_started || initialized)
                    break;
            }
            if (needs_split) {
                assert(split_started && initialized && rig.out.capture_split_runs == 1);
                assert(rig.out.reason == "first_trip_confirmed_capture_path_split");
                assert(rig.out.capture_scene_detail == verdictName(verdict));
                assert(rig.out.split_before_objects.size() == rig.objects.size());
                assert(rig.task.search_phase_.first_trip_split_.retreat_objects.size() + 1 ==
                       rig.objects.size());
                rig.step(); // initialization must hand control to actual split recovery
                assert(rig.out.recovering && rig.out.capture_split_runs == 1);
                assert(rig.out.motion.vx_mps <= 0 && rig.out.motion.wz_rps == 0);
                assert(rig.task.frame_raised_ == 1);
            } else if (selectable) {
                assert(initialized && !split_started && rig.out.capture_split_runs == 0);
                assert(rig.out.reason == "phase_capture_ready");
                assert(rig.task.trip_context_.pending_ == inventory);
            } else {
                // No legal reference: e.g. dangerous-only or pre-first-delivery core/injured.
                assert(!initialized && !split_started && rig.out.capture_split_runs == 0);
                assert(rig.out.target_id == -1);
            }
        }
    std::cout << "PASS all 15 four-class subsets before/after first delivery: split/legal/wait\n";
}

void initialRiskReviewRestartsAfterInvalidEvidence() {
    for (int invalid = 0; invalid < 3; ++invalid) {
        SceneRig rig(true);
        rig.risk();
        rig.task.t_.corridor_confirm_frames = 6;
        for (int i = 0; i < 5; ++i)
            rig.step();
        assert(rig.task.single_phase_initializing_);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        if (invalid == 0)
            rig.in.camera_pitch_stable = false;
        if (invalid == 1)
            rig.in.gripper_done = false;
        if (invalid == 2)
            rig.in.sent_vx_mps = .1f;
        rig.step();
        assert(stopped(rig.out.motion) && rig.task.capture_scene_.confirmations == 0);
        rig.in.camera_pitch_stable = rig.in.gripper_done = true;
        rig.in.sent_vx_mps = 0;
        const auto restarted = rig.in.now_us;
        for (int i = 0; i < 4; ++i) {
            rig.step();
            assert(stopped(rig.out.motion));
            assert(!rig.task.search_phase_.first_trip_split_.pending);
        }
        for (int i = 0; i < 16 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
            rig.step();
        assert(rig.task.search_phase_.first_trip_split_.pending);
        assert(!rig.task.single_phase_initializing_);
        assert(rig.in.now_us - restarted >= 250000 && rig.out.capture_split_runs == 1);
    }
    std::cout << "PASS initial risk review restarts after unstable pitch/gripper/moving feedback\n";
}

void initialMixedPileJitterSplitsBeforeTargetReselection() {
    for (bool single : {false, true})
        for (bool injured : {false, true})
            for (bool tied : {false, true}) {
                SceneRig rig(single);
                rig.task.t_.corridor_confirm_frames = 6; // actual field runtime
                rig.managed_selection = true;
                rig.objects = {detection(1, {495, 139, 203, 153}),
                               detection(2, {707, 134, 183, 143})};
                if (injured) {
                    auto person = detection(3, {495, 291, 426, 274});
                    person.label = "injured_person";
                    rig.objects.push_back(person);
                }
                if (!single)
                    rig.enableSelector(); // full capture inherits the search-confirmed reference
                const uint64_t started = rig.in.now_us;
                auto& split = rig.task.search_phase_.first_trip_split_;
                uint64_t review_started = 0;
                int reference = -1;
                for (int i = 0; i < 28 && !split.pending; ++i) {
                    for (auto& object : rig.objects) {
                        if (object.track_id == 1)
                            object.box.y = tied || i % 2 == 0 ? 139 : 129;
                        if (object.track_id == 2)
                            object.box.y = tied || i % 2 != 0 ? 144 : 134;
                    }
                    // Equal-height detections must not depend on detector output order.
                    std::reverse(rig.objects.begin(), rig.objects.end());
                    rig.step();
                    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                    assert(rig.out.capture_evidence == CaptureEvidence::None);
                    assert(rig.out.final_validation != "passed" && !rig.out.single_phase_done);
                    assert(!rig.out.split_target_reselecting);
                    if (rig.out.capture_scene_status == "risk" && !review_started)
                        review_started = rig.in.now_us;
                    if (review_started && rig.out.target_id >= 0) {
                        if (reference < 0)
                            reference = rig.out.target_id;
                        assert(rig.out.target_id == reference);
                    }
                    if (review_started && rig.in.now_us - review_started < 250000)
                        assert(!split.pending);
                }
                if (!split.pending)
                    std::cerr << "initial mixed review single=" << single << " injured=" << injured
                              << " tied=" << tied << " reason=" << rig.out.reason
                              << " scene=" << rig.out.capture_scene_status << "/"
                              << rig.out.capture_scene_detail << " target=" << rig.out.target_id
                              << " confirmations=" << rig.task.capture_scene_.confirmations << '\n';
                assert(split.pending && split.runs == 1 && !rig.task.single_phase_initializing_);
                assert(review_started && rig.in.now_us - review_started >= 250000);
                assert(rig.in.now_us - started < 2000000);
                assert(rig.out.reason == "first_trip_confirmed_capture_path_split");
                assert(rig.out.split_before_objects.size() == rig.objects.size());
                assert(rig.out.capture_scene_detail ==
                       (injured ? "injured_before_first"
                                : "neighbor_box_overlaps_ordinary_inventory"));
            }
    std::cout
        << "PASS initial two-supply/mixed-injured jitter/ties split before target reselection\n";
}

void latestDiagonalPairStartsSplitBeforeRanking() {
    SceneRig rig(true);
    // Area ranking initially picks far #1. Mixed-scene review now precedes reranking.
    rig.objects = {detection(2, {120, 318, 243, 283}), detection(1, {357, 62, 272, 302})};
    rig.managed_selection = true;
    bool chose_far = false;
    for (int i = 0; i < 48 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
        rig.step();
        chose_far |= rig.in.target_valid && rig.in.target_id == 1;
        assert(!rig.out.split_target_reselecting);
        assert(stopped(rig.out.motion));
        assert(rig.task.frame_raised_ == 1 && rig.out.final_validation == "not_executed");
    }
    assert(chose_far && !rig.task.single_phase_initializing_);
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.target_id == 1 && rig.in.split_ordinary_count == 2);
    rig.step(); // recovery must execute, with near #2 as the unwanted retreat reference
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    assert(rig.out.split_retreat_reference_id == 2 && rig.out.capture_split_runs == 1);
    std::cout << "PASS diagonal pair: review before ranking -> split -> actual recovery step\\n";
}

void nearestFrontBackTarget() {
    PixelSelector selector;
    auto near = detection(1, {600, 350, 60, 60});
    auto far = detection(2, {550, 100, 160, 160}); // larger far box must not win by area
    PixelSelection chosen;
    for (int i = 0; i < 5; ++i) {
        near.timestamp_us = far.timestamp_us = 1000000 + i * 50000;
        chosen =
            selector.update({far, near}, {1280, 720}, 2000, true, near.timestamp_us, false, .25f);
    }
    assert(chosen.id == 1);
    std::cout << "PASS fore/aft first-trip selection prefers near block\n";
}

void splitRetreatUsesUnwantedBlock() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (const std::string label : {"ordinary_supply", "core_supply", "injured_person", "dangerous_object"})
        for (int retained_y : {240, 390}) {
            SceneRig rig;
            rig.objects[0].box.y = retained_y - 40;
            rig.objects[1].box = {900, 300, 80, 80};
            rig.objects[1].label = label;
            auto& split = rig.task.search_phase_.first_trip_split_;
            for (int i = 0; i < 24 && !split.pending; ++i)
                rig.step();
            assert(split.pending);
            rig.objects[1].box.y = 241; // unwanted Y281 controls retreat, regardless of target Y
            rig.step();
            assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
            assert(rig.out.target_id == 1 && !split.target_reselecting);
            rig.objects[1].box.y = 240;
            rig.step();
            if (!split.upper_corner) {
                assert(rig.out.motion.vx_mps == -.01f); // Y280 is insufficient for ordinary split
                rig.objects[1].box.y = 99;
                rig.step();
            }
            assert(stopped(rig.out.motion) && split.recovery_stage == Stage::Retreat);
            rig.finishRetreat();
            assert(split.recovery_stage == Stage::Turn);
            assert(rig.out.target_id == 1 && !split.target_reselecting);
        }
    SceneRig missing;
    missing.split();
    missing.objects.pop_back();
    for (int i = 0; i < 4; ++i) {
        missing.step();
        if (i < 2) assert(missing.out.motion.vx_mps == -.01f);
        else assert(stopped(missing.out.motion));
        assert(missing.task.search_phase_.first_trip_split_.recovery_stage == Stage::Retreat);
    }
    std::cout << "PASS unwanted whole-box clearance (corner Y280); retained target never changes\n";
}

void unwantedReobserveRequiresStoppedFreshFrames() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (int invalid = 0; invalid < 5; ++invalid) {
        SceneRig rig;
        rig.split();
        rig.objects.pop_back();
        auto& split = rig.task.search_phase_.first_trip_split_;
        for (int i = 0; i < 3; ++i)
            rig.step(); // begin, but do not finish, the stopped observation window
        for (int i = 0; i < 16; ++i) {
            rig.in.sent_vx_mps = invalid == 0 ? -.01f : 0;
            rig.in.sent_wz_rps = invalid == 1 ? .01f : 0;
            rig.in.sent_velocity_valid = invalid != 2;
            rig.in.camera_pitch_stable = invalid != 3;
            rig.step(invalid != 4);
            assert(stopped(rig.out.motion) && split.recovery_stage == Stage::Retreat);
            assert(rig.out.capture_evidence == CaptureEvidence::None);
        }
        rig.in.sent_vx_mps = rig.in.sent_wz_rps = 0;
        rig.in.sent_velocity_valid = rig.in.camera_pitch_stable = true;
        for (int i = 0; i < 3; ++i) {
            rig.step();
            assert(split.recovery_stage == Stage::Retreat);
        }
        for (int i = 0; i < 16 && split.recovery_stage == Stage::Retreat; ++i)
            rig.step();
        assert(split.recovery_stage == Stage::Turn);
    }
    // Incomplete/ambiguous references use the same stopped recovery; neither
    // case supplies the evidence required to mark the unwanted block cleared.
    for (bool ambiguous : {false, true}) {
        SceneRig rig;
        rig.split();
        auto& split = rig.task.search_phase_.first_trip_split_;
        if (ambiguous)
            rig.objects.push_back(rig.objects.back());
        else
            rig.objects.back().box.x = 0;
        for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i) {
            rig.step();
            assert(stopped(rig.out.motion));
        }
        assert(split.recovery_stage == Stage::Turn);
        assert(!split.retreat_objects.front().cleared);
        assert(rig.out.capture_evidence == CaptureEvidence::None);
    }
    std::cout << "PASS unwanted reobserve resets on motion/feedback/view/staleness; clipped/ambiguous recover\n";
}

void threeMissingFramesBeforeReobserve() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true})
        for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"}) {
            SceneRig rig(single);
            rig.objects[1].box = {900, 300, 80, 80};
            rig.objects[1].label = label;
            auto& split = rig.task.search_phase_.first_trip_split_;
            for (int i = 0; i < 40 && !split.pending; ++i) rig.step();
            assert(split.pending);
            rig.step();
            assert(rig.out.motion.vx_mps == -.01f);
            auto unwanted = rig.objects.back();
            rig.objects.pop_back();
            rig.step(); // first missing image
            assert(rig.out.motion.vx_mps == -.01f && split.recovery_stage == Stage::Retreat);
            rig.step(false); // duplicate image must not become a second miss
            rig.step(false);
            assert(rig.out.motion.vx_mps == -.01f && split.recovery_stage == Stage::Retreat);
            rig.step(); // second missing image
            assert(rig.out.motion.vx_mps == -.01f && !split.retreat_objects.front().cleared);
            unwanted.track_id = 20; // unique nearby same-class return clears the streak
            rig.objects.push_back(unwanted);
            rig.step();
            assert(rig.out.motion.vx_mps == -.01f);
            rig.objects.pop_back();
            rig.step();
            assert(rig.out.motion.vx_mps == -.01f);
            rig.step();
            assert(rig.out.motion.vx_mps == -.01f);
            rig.step(); // third consecutive missing image
            assert(stopped(rig.out.motion) && rig.out.reason == "pixel_split_unwanted_observation_wait");
            assert(split.recovery_stage == Stage::Retreat && !split.retreat_objects.front().cleared);
            for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i) rig.step();
            assert(split.recovery_stage == Stage::Turn);
        }
    std::cout << "PASS three independent missing frames; duplicates ignored, re-ID resets streak, stopped reobserve retained\n";
}

void latestMissingUnwantedReobserves() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    // 2026-10-31 00:24:00: #4 disappears below Y280 during retreat.
    rig.objects = {detection(1, {485, 190, 203, 254}),
                   detection(2, {702, 185, 194, 243}),
                   detection(3, {705, 432, 137, 153}),
                   detection(4, {515, 420, 163, 158})};
    rig.objects[2].label = rig.objects[3].label = "core_supply";
    auto& split = rig.task.search_phase_.first_trip_split_;
    for (int i = 0; i < 24 && !split.pending; ++i)
        rig.step();
    assert(split.pending && split.retreat_objects.size() == 3);
    const auto budget_start = split.started_us;
    const int runs = split.runs;
    rig.objects = {detection(5, {535, 94, 224, 264}),
                   detection(2, {712, 8, 173, 213}),
                   detection(3, {755, 205, 128, 122})};
    rig.objects[2].label = "core_supply";
    rig.in.target_id = 5;
    rig.step();
    rig.step();
    rig.step(); // third independent missing image starts the existing stop window
    assert(stopped(rig.out.motion));
    assert(rig.out.reason == "pixel_split_unwanted_observation_wait");
    for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i) {
        rig.step();
        assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
        assert(rig.out.capture_evidence == CaptureEvidence::None);
    }
    assert(split.recovery_stage == Stage::Turn);
    assert(!split.upper_corner); // missing evidence must not authorize the blind corner push
    assert(split.started_us == budget_start && split.runs == runs);
    assert(!split.retreat_objects.back().cleared);
    rig.in.camera_pitch_cdeg = 0;
    rig.in.directional_clearance_valid = true;
    rig.in.turn_safe = false;
    rig.step();
    assert(stopped(rig.out.motion) && split.recovery_turn_us == 0);
    rig.in.turn_safe = true;
    rig.step();
    assert(rig.out.motion.vx_mps == 0 && rig.out.motion.wz_rps != 0);
    std::cout << "PASS 00:24 missing unwanted -> stopped fresh frames -> gated reobserve turn\n";
}

void latestRetreatTargetGapDoesNotBlockTurn() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    // 22:38:14 field log: ordinary #1 retained, #2 already above Y280,
    // injured #3 still deep in the frame and requiring reverse.
    rig.objects = {detection(1, {490, 180, 173, 173}),
                   detection(2, {702, 190, 214, 152}),
                   detection(3, {505, 347, 446, 263})};
    rig.objects[2].label = "injured_person";
    auto& split = rig.task.search_phase_.first_trip_split_;
    for (int i = 0; i < 24 && !split.pending; ++i)
        rig.step();
    assert(split.pending && split.upper_corner && split.retreat_objects.size() == 2);
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f);
    // Retained target touches the top, changes ID, then loses the selector lock.
    // A still-uncleared unwanted block disappearing must continue to stop motion.
    rig.objects[0] = detection(7, {566, 0, 183, 216});
    const auto injured = rig.objects[2];
    rig.objects.pop_back();
    rig.in.target_valid = false;
    rig.in.target_id = -1;
    for (int i = 0; i < 3; ++i) {
        rig.step();
        assert(split.recovery_stage == Stage::Retreat);
        if (i < 2) assert(rig.out.motion.vx_mps == -.01f);
        else {
            assert(stopped(rig.out.motion));
            assert(rig.out.reason == "pixel_split_unwanted_observation_wait");
        }
    }
    // Incomplete unwanted detection also cannot establish the retreat endpoint.
    rig.objects.push_back(injured);
    rig.objects.back().box = {0, 180, 436, 285};
    rig.step();
    assert(stopped(rig.out.motion) && rig.out.reason == "pixel_split_unwanted_observation_wait");
    // Unwanted #3 becomes #5: same class, unique nearby location.
    rig.objects.back().track_id = 5;
    rig.objects.back().box = {551, 164, 436, 285}; // centre Y306.5
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && !rig.in.target_valid);
    assert(rig.out.split_retreat_reference_id == 5);
    // Neither an already-cleared ordinary nor the retained ordinary must reappear.
    rig.objects.erase(rig.objects.begin(), rig.objects.begin() + 2);
    rig.objects[0].box = {551, 139, 436, 284}; // centre Y281
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.split_retreat_reference_y == 281);
    rig.objects[0].box.y = 138; // centre Y280
    rig.step();
    assert(stopped(rig.out.motion) && rig.out.reason == "first_trip_split_retreat_brake");
    for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i)
        rig.step();
    assert(split.recovery_stage == Stage::CornerTurnRight);
    rig.in.camera_pitch_cdeg = 0;
    rig.step();
    assert(rig.out.motion.wz_rps < 0 && rig.out.motion.vx_mps == 0);
    assert(!rig.in.target_valid && rig.out.target_id == 1);
    std::cout << "PASS 22:38 field gap: retained clip/re-ID/loss, unwanted re-ID, Y280 -> right90\n";
}

void imuTranslationIntegration() {
    // Tilted stationary chassis: gravity must not create horizontal travel.
    ImuTranslation estimator;
    uint64_t now = 1000000, sequence = 0;
    auto sample = [&](float world_ax) {
        ImuTranslationSample s;
        s.sequence = ++sequence;
        s.received_us = now;
        s.device_time_ms = uint32_t(now / 1000);
        const float pitch = .3f;
        s.acceleration_mps2 = {std::cos(pitch) * world_ax - std::sin(pitch) * 9.8f,
                              0, std::sin(pitch) * world_ax + std::cos(pitch) * 9.8f};
        s.quaternion_wxyz = {std::cos(pitch / 2), 0, std::sin(pitch / 2), 0};
        s.valid = true;
        return s;
    };
    assert(!estimator.calibrate({}, now, 0));
    for (int i = 0; i < 30; ++i) {
        now += 10000;
        estimator.calibrate({sample(.02f)}, now, 0);
    }
    assert(estimator.ready());
    for (int i = 0; i < 100; ++i) {
        now += 10000;
        assert(estimator.advance({sample(.02f)}, now));
    }
    assert(std::abs(estimator.distance()) < .0001f);
    std::vector<ImuTranslationSample> batch;
    for (int i = 0; i < 100; ++i) {
        now += 10000;
        batch.push_back(sample(.22f));
        if (batch.size() == 5) {
            assert(estimator.advance(batch, now));
            batch.clear();
        }
    }
    assert(std::abs(estimator.distance() - .1f) < .002f);
    assert(std::abs(estimator.velocity() - .2f) < .002f);
    const float previous = estimator.distance();
    auto duplicate = sample(.22f);
    duplicate.sequence = sequence - 1;
    assert(estimator.advance({duplicate}, now));
    assert(estimator.distance() == previous);
    now += 210000;
    assert(!estimator.advance({}, now) && !estimator.ready());
    std::cout << "PASS IMU displacement: tilt/gravity, bias, batches, double integration, gaps\n";
}

void splitActionBudgetsStayBounded() {
    for (bool corner : {false, true}) {
        SceneRig rig;
        assert(rig.task.t_.pile_max_runs == 0);
        rig.objects = {detection(1, corner ? cv::Rect{657, 124, 183, 244}
                                         : cv::Rect{525, 170, 224, 274}),
                       detection(45, {687, 53, 203, 264})};
        rig.step();
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.corner_runs = 20;
        split.runs = 40;
        TaskTick tick(rig.in);
        rig.task.beginPixelSplit(tick);
        assert(!rig.task.fault_ && split.pending);
        assert(split.upper_corner == corner && split.runs == 41);
    }

    for (bool corner : {false, true})
      for (bool exhausted : {false, true}) {
        SceneRig rig;
        rig.objects = {detection(1, corner ? cv::Rect{657, 124, 183, 244}
                                         : cv::Rect{525, 170, 224, 274}),
                       detection(45, {687, 53, 203, 264})};
        rig.step();
        auto& split = rig.task.search_phase_.first_trip_split_;
        rig.task.t_.pile_max_runs = 4;
        const int limit = rig.task.t_.pile_max_runs;
        const int own = limit - (exhausted ? 0 : 1);
        split.corner_runs = corner ? own : limit;
        split.runs = own + limit;
        TaskTick tick(rig.in);
        rig.task.beginPixelSplit(tick);
        assert(rig.task.fault_ == exhausted);
        if (exhausted) {
            assert(rig.task.reason_ == "first_trip_split_limit");
            assert(stopped(tick.motion));
            assert(split.runs == own + limit);
        } else {
            assert(split.pending && split.upper_corner == corner);
            assert(split.runsFor(corner) == limit && split.runsFor(!corner) == limit);
            assert(split.runs == 2 * limit);
        }
    }
    std::cout << "PASS corner and ordinary budgets are independent, bounded and reset per trip\n";
}

void splitTurnStartupDrift() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (Stage stage : {Stage::CornerTurnRight, Stage::CornerTurnBack, Stage::InjuredTurn})
    for (int direction : {-1, 1}) {
        if (stage != Stage::InjuredTurn &&
            direction != (stage == Stage::CornerTurnRight ? -1 : 1)) continue;
        // Field 2026-11-06 05:26: +0.034 rad/s at right-turn startup
        // latched a brake for 11.45 s while the goal was still 90 degrees away.
        // Include the observed peak drift (~0.055) and real opposing rotation.
        for (float drift : {.0341366f, .0554773f, .3f}) {
            SceneRig rig;
            auto& split = rig.task.search_phase_.first_trip_split_;
            split.pending = split.capture_retry = true;
            split.upper_corner = stage != Stage::InjuredTurn;
            split.keep_injured = stage == Stage::InjuredTurn;
            split.started_us = rig.in.now_us;
            split.recovery_stage = stage;
            split.recovery_direction = direction;
            split.injured_turn_goal = direction * .15f;
            rig.task.t_.split_wz_increment_rps = .2f;
            rig.task.t_.max_wz = .6f;
            rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "split_startup_drift_fixture");
            rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
            rig.in.heading_rad = stage == Stage::CornerTurnBack ? -1.570796327f : 0;
            rig.step(true, true, 33000);
            for (int i = 0; i < 12; ++i) {
                rig.in.heading_rad -= direction * drift * .033f;
                rig.step(true, true, 33000);
                if (drift == .3f) {
                    assert(split.corner_turn_braking && stopped(rig.out.motion));
                } else {
                    assert(!split.corner_turn_braking);
                    assert(direction * rig.out.motion.wz_rps > 0);
                }
            }
        }
    }
    std::cout << "PASS split startup tolerates logged heading drift and still brakes opposing rotation\n";
}

void cornerTurnBrakesBeforePush() {
    splitTurnStartupDrift();
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    const auto seed = [](SceneRig& rig, Stage stage) {
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.upper_corner = true;
        split.started_us = rig.in.now_us;
        split.recovery_stage = stage;
        rig.task.t_.split_wz_increment_rps = .2f;
        rig.task.t_.max_wz = .6f;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "corner_turn_brake_fixture");
        rig.in.camera_pitch_cdeg = 0;
    };
    for (Stage stage : {Stage::CornerTurnRight, Stage::CornerTurnBack}) {
        const float goal = stage == Stage::CornerTurnRight ? -1.570796327f : -.174532925f;
        {
            SceneRig rig;
            seed(rig, stage);
            rig.in.heading_rad = goal + .08f;
            rig.step();
            // The old fixed 0.5 rad/s request drove through this final approach.
            assert(rig.out.motion.wz_rps < 0 && std::abs(rig.out.motion.wz_rps) < .3f);
        }
        {
            SceneRig rig;
            seed(rig, stage);
            auto& split = rig.task.search_phase_.first_trip_split_;
            rig.in.heading_rad = goal;
            rig.step();
            assert(split.pending && split.recovery_stage == stage && stopped(rig.out.motion));
            // Zero sent commands do not mean a rotating body has stopped.
            for (int i = 0; i < 10; ++i) {
                rig.in.heading_rad = goal + (i % 2 ? .006f : -.006f);
                rig.step();
                assert(split.pending && split.recovery_stage == stage && stopped(rig.out.motion));
            }
            rig.in.heading_rad = goal;
            rig.in.sent_wz_rps = .1f;
            for (int i = 0; i < 8; ++i) {
                rig.step();
                assert(split.pending && split.recovery_stage == stage && stopped(rig.out.motion));
            }
            rig.in.sent_wz_rps = 0;
            for (int i = 0; i < 4; ++i) rig.step();
            rig.in.heading_valid = false;
            rig.step();
            rig.in.heading_valid = true;
            for (int i = 0; i < 5; ++i) {
                rig.step();
                assert(split.pending && split.recovery_stage == stage && stopped(rig.out.motion));
            }
            for (int i = 0; i < 4 && split.pending && split.recovery_stage == stage; ++i) rig.step();
            assert(stopped(rig.out.motion));
            if (stage == Stage::CornerTurnRight)
                assert(split.recovery_stage == Stage::CornerPush);
            else
                assert(!split.pending && rig.out.state == PushState::APPROACH);
        }
        {
            SceneRig rig;
            seed(rig, stage);
            rig.in.heading_rad = goal + .06f;
            rig.step();
            assert(rig.out.motion.wz_rps < 0);
            rig.in.heading_rad = goal - .04f; // crossed the goal while still turning right
            rig.step();
            assert(stopped(rig.out.motion)); // brake before reversing the turn
            for (int i = 0; i < 5; ++i) {
                rig.step();
                assert(stopped(rig.out.motion));
            }
            for (int i = 0; i < 4 && stopped(rig.out.motion); ++i) rig.step();
            assert(rig.out.motion.wz_rps > 0 && rig.out.motion.wz_rps < .3f);
        }
        // Deterministic body lag: exercise the real task with a 100 ms response.
        {
            SceneRig rig;
            seed(rig, stage);
            auto& split = rig.task.search_phase_.first_trip_split_;
            rig.in.heading_rad = stage == Stage::CornerTurnRight ? 0 : -1.570796327f;
            const float direction = stage == Stage::CornerTurnRight ? -1.f : 1.f;
            float rate = 0, overshoot = 0;
            for (int i = 0; i < 1000 && split.pending && split.recovery_stage == stage; ++i) {
                rate += .2f * (rig.out.motion.wz_rps - rate);
                rig.in.heading_rad += rate * .02f;
                rig.in.sent_wz_rps = rig.out.motion.wz_rps;
                overshoot = std::max(overshoot, direction * wrapAngle(rig.in.heading_rad - goal));
                rig.step(true, true, 20000);
                assert(rig.out.motion.vx_mps == 0);
            }
            assert(overshoot <= .025f);
            assert(!split.pending || split.recovery_stage == Stage::CornerPush);
            std::cout << "PASS corner lag simulation stage=" << int(stage)
                      << " overshoot_rad=" << overshoot << '\n';
        }
    }
    std::cout << "PASS corner turns taper, brake before reversal and require stable IMU heading for 250 ms\n";
}

void cornerReturnBalancesCommandsAndResumesSearch() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    // Missing or biased translation samples must not prevent the command cycle.
    for (int scenario = 0; scenario < 4; ++scenario) {
        SceneRig rig;
        rig.simulate_corner_imu = scenario != 0;
        rig.corner_imu_accel_bias = scenario == 2 ? .20f : scenario == 3 ? -.20f : 0;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.upper_corner = true;
        split.started_us = rig.in.now_us;
        split.recovery_stage = Stage::CornerTurnRight;
        split.corner_heading = 0;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "corner_command_fixture");
        rig.in.camera_pitch_cdeg = 0;
        rig.in.heading_rad = -1.570796327f;
        float forward = 0, reverse = 0;
        bool returned = false;
        for (int i = 0; i < 600 && split.recovery_stage != Stage::CornerTurnBack; ++i) {
            rig.in.sent_vx_mps = rig.out.motion.vx_mps;
            rig.in.sent_wz_rps = rig.out.motion.wz_rps;
            forward += std::max(0.f, rig.in.sent_vx_mps) * .05f;
            reverse += std::max(0.f, -rig.in.sent_vx_mps) * .05f;
            returned |= split.recovery_stage == Stage::CornerReturn;
            rig.step();
            assert(!rig.task.fault_);
            assert(reverse <= forward + .005f);
        }
        assert(returned && split.recovery_stage == Stage::CornerTurnBack);
        assert(std::abs(forward - .036f) < .005f);
        assert(std::abs(reverse - forward) < .005f);
        assert(std::abs(rig.out.split_command_forward_m - forward) < .0001f);
        assert(std::abs(rig.out.split_command_reverse_m - reverse) < .0001f);
        rig.in.sent_vx_mps = rig.in.sent_wz_rps = 0;
        rig.in.heading_rad = -.174532925f;
        rig.finishCornerTurn();
        assert(!split.pending && rig.out.state == PushState::APPROACH);
        assert(rig.out.phase == TaskPhase::Search && !rig.task.fault_);
        assert(rig.task.trip_context_.target_id_ == -1);
        assert(rig.task.search_phase_.pixel_reselection_.observing);
        // The normal search flow can select and approach a fresh target.
        rig.objects = {detection(17, {600, 260, 80, 80})};
        rig.in.target_id = 17;
        bool approached = false;
        for (int i = 0; i < 40 && !approached; ++i) {
            rig.step();
            assert(!rig.task.fault_);
            approached = rig.out.motion.vx_mps > 0;
        }
        assert(approached && rig.task.trip_context_.target_id_ == 17);
    }
    for (Stage stage : {Stage::CornerTurnRight, Stage::CornerPush,
                        Stage::CornerReturn, Stage::CornerTurnBack}) {
        SceneRig rig;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.upper_corner = true;
        split.recovery_stage = stage;
        split.started_us = rig.in.now_us;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "corner_timeout_fixture");
        rig.in.camera_pitch_cdeg = 0; // the established recovery view remains valid
        rig.in.now_us += rig.t.pile_budget_us;
        rig.task.last_us_ = rig.in.now_us;
        rig.step();
        assert(!rig.task.fault_ && !split.pending);
        assert(rig.out.state == PushState::APPROACH && rig.out.phase == TaskPhase::Search);
        assert(rig.out.reason == "pixel_split_corner_timeout_research");
        assert(rig.task.trip_context_.target_id_ == -1);
        assert(rig.task.search_phase_.pixel_reselection_.observing);
    }
    std::cout << "PASS corner command balance ignores missing/drifting translation and resumes search; timeout recovers\n";
}

void upperCornerPushAndReturn() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (int side : {-1, 1})
      for (bool above_range : {false, true})
       for (bool single : {false, true})
        for (bool crowded : {false, true}) {
        SceneRig rig(single);
        rig.simulate_corner_imu = true;
        // Exact Y195.5 and left/right target boxes from the 21:41 field log.
        rig.objects[0].box = above_range
            ? (side < 0 ? cv::Rect{404, 124, 183, 143} : cv::Rect{611, 114, 193, 163})
            : cv::Rect{600 + side * 100, 240, 80, 80};
        rig.objects[1].box = {900, 230, 60, 60};
        rig.objects[1].label = "dangerous_object";
        auto& split = rig.task.search_phase_.first_trip_split_;
        for (int i = 0; i < 24 && !split.pending; ++i)
            rig.step();
        assert(split.pending && split.upper_corner);
        rig.objects[1].box.y = 251; // unwanted centre Y281
        rig.step();
        assert(rig.out.motion.vx_mps == -.05f && rig.out.motion.wz_rps == 0);
        rig.objects[1].box.y = 250; // unwanted centre Y280
        // Direction stays latched even if image coordinates subsequently move.
        rig.objects[0].box.x = 600;
        for (int i = 0; i < 30 && split.recovery_stage == Stage::Retreat; ++i)
            rig.step();
        assert(split.recovery_stage == Stage::CornerTurnRight);
        rig.in.camera_pitch_cdeg = 0;
        rig.step();
        assert(rig.out.motion.wz_rps < 0 && rig.out.motion.vx_mps == 0);
        rig.in.directional_clearance_valid = true;
        rig.in.turn_safe = false;
        rig.step();
        assert(stopped(rig.out.motion));
        rig.in.turn_safe = rig.in.arc_safe = true;
        rig.step();
        assert(split.recovery_stage == Stage::CornerTurnRight);
        // IMU, rather than elapsed turn time, completes the quarter turn.
        rig.in.heading_rad = -1.570796327f;
        rig.finishCornerTurn();
        assert(split.recovery_stage == Stage::CornerPush && stopped(rig.out.motion));
        assert(!split.corner_imu.ready()); // translation IMU is not required
        assert(split.recovery_stage == Stage::CornerPush && stopped(rig.out.motion));
        assert(split.corner_distance == .036f);
        rig.imu_sim_position = rig.imu_sim_peak = 0; // linear stage starts after retreat/turn
        rig.in.path_safe = false;
        rig.step();
        assert(stopped(rig.out.motion));
        rig.in.path_safe = true;
        float forward = 0, reverse = 0;
        for (int i = 0; i < 600 && split.recovery_stage != Stage::CornerTurnBack; ++i) {
            rig.in.sent_vx_mps = rig.out.motion.vx_mps;
            rig.in.sent_wz_rps = rig.out.motion.wz_rps;
            forward += std::max(0.f, rig.in.sent_vx_mps) * .05f;
            reverse += std::max(0.f, -rig.in.sent_vx_mps) * .05f;
            rig.step();
            assert(!rig.task.fault_);
        }
        assert(split.recovery_stage == Stage::CornerTurnBack);
        if (std::abs(rig.imu_sim_peak - .036f) >= .006f || std::abs(rig.imu_sim_position) >= .006f)
            std::cerr << "IMU cycle peak=" << rig.imu_sim_peak << " returned=" << rig.imu_sim_position
                      << " estimate=" << split.corner_imu.distance() << '\n';
        assert(std::abs(rig.imu_sim_peak - .036f) < .006f);
        assert(std::abs(rig.imu_sim_position) < .006f);
        assert(std::abs(forward - reverse) < .005f);
        assert(forward > 0 && reverse > 0);
        rig.in.sent_vx_mps = rig.in.sent_wz_rps = 0;
        rig.step();
        assert(rig.out.motion.wz_rps > 0 && rig.out.motion.vx_mps == 0);
        assert(rig.out.reason == "pixel_split_corner_left80");
        rig.in.heading_rad = -.261799388f; // abrupt IMU progress: brake 5 degrees before goal
        rig.step();
        assert(split.recovery_stage == Stage::CornerTurnBack && stopped(rig.out.motion));
        for (int i = 0; i < 12 && stopped(rig.out.motion); ++i) rig.step();
        assert(split.recovery_stage == Stage::CornerTurnBack && rig.out.motion.wz_rps > 0);
        assert(rig.out.motion.wz_rps < .3f);
        rig.in.heading_rad = -.174532925f; // right 90, then left 80
        rig.finishCornerTurn();
        assert(!split.pending && !split.target_reselecting && split.capture_retry);
        assert(rig.out.state == PushState::APPROACH && rig.out.phase == TaskPhase::Search);
        assert(rig.out.reason == "pixel_split_corner_resume_ui120");
        assert(split.runs == 1 && split.corner_runs == 1);
        assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 0);
        assert(rig.task.trip_context_.target_id_ == -1 && rig.task.search_entry_.target_id == -1);
        rig.objects = {detection(17, {600, 260, 80, 80})};
        rig.in.target_id = 17; // normal approach can select a new post-turn identity
        rig.in.camera_pitch_cdeg = 2000;
        rig.step();
        assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 0);
        assert(rig.task.frame_raised_ == 1 && !rig.task.fault_);
        rig.in.camera_pitch_cdeg = 0;
        bool approached = false;
        for (int i = 0; i < 40 && !approached; ++i) {
            rig.step();
            assert(!rig.task.fault_ && rig.task.frame_raised_ == 1);
            assert(rig.out.motion.camera_pitch_cdeg == 0);
            approached = rig.out.motion.vx_mps > 0;
        }
        assert(approached && rig.task.trip_context_.target_id_ == 17);
        // Normal UI120 approach hands off at centre Y540, then reviews at UI140.
        rig.objects[0].box.y = 500;
        for (int i = 0; i < 40 && rig.out.motion.camera_pitch_cdeg != 2000; ++i)
            rig.step();
        assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 2000);
        rig.in.camera_pitch_cdeg = 2000;
        rig.objects[0].box = {608, 310, 80, 80};
        if (crowded) {
            // 12:43:15: after the upper-corner cycle, #43/#45 still share inventory.
            // The ordinary split must retain both attempts after that preprocessing.
            rig.objects = {detection(17, {525, 170, 224, 274}),
                           detection(45, {687, 53, 203, 264})};
            for (int i = 0; i < 60 && !split.pending; ++i)
                rig.step();
            assert(split.pending && !split.upper_corner && split.runs == 2);
            rig.turnCycle();
            rig.step(); // neighbour remains: the ordinary split needs its second attempt
            assert(!rig.task.fault_);
            assert(rig.out.reason == "first_trip_split_retry_retreat" && split.runs == 3);
            rig.turnCycle();
            rig.objects.resize(1); // second ordinary split has separated the neighbour
            rig.objects[0].box = {608, 310, 80, 80};
            for (int i = 0; i < 20 && split.pending; ++i)
                rig.step();
            assert(!split.pending && !rig.task.fault_);
        }
        bool preclose = false;
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.step();
            preclose |= rig.out.capture_review_stage == "preclose";
            assert(!rig.task.fault_ && !split.pending);
        }
        assert(preclose && rig.out.final_validation == "passed");
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(split.corner_runs == 1 && split.runs == (crowded ? 3 : 1));
        assert(single ? rig.out.single_phase_done : rig.out.state == PushState::CARRY);
        rig.task.clearTrip();
        assert(split.runs == 0 && split.corner_runs == 0);
    }
    std::cout << "PASS upper corners: retreat, IMU right90, push .036m by sent-command integration, balanced return, left80, UI120 approach, UI140 handoff, capture\n";
}

void mixedCargoDirectionAndLifecycle() {
    for (bool first_delivered : {false, true})
        for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"})
            for (int side : {-1, 1}) {
                if (!targetSelectable(label, first_delivered))
                    continue;
                SceneRig rig;
                rig.task.first_ = first_delivered;
                rig.in.label = rig.task.trip_context_.label_ = label;
                rig.task.trip_context_.pending_ = {};
                rig.task.trip_context_.pending_.add(label);
                rig.objects.resize(1);
                rig.objects[0].label = label;
                // Keep this legacy direction test in the lower half of every class range.
                rig.objects[0].box.y = 400;
                rig.objects[0].box.x += side * 100;
                auto danger = detection(2, {900, 290, 60, 60}); // unwanted centre Y320
                danger.label = "dangerous_object";
                rig.objects.push_back(danger);
                if (!first_delivered) {
                    auto core = detection(3, {780, 230, 60, 60});
                    core.label = "core_supply";
                    auto injured = detection(4, {420, 230, 60, 60});
                    injured.label = "injured_person";
                    rig.objects.push_back(core);
                    rig.objects.push_back(injured);
                } else {
                    auto extra = detection(3, {780, 230, 60, 60});
                    extra.label = label == "injured_person" ? "ordinary_supply" : "core_supply";
                    rig.objects.push_back(extra);
                }
                for (int i = 0; i < 24 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
                    rig.step();
                    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                }
                auto& split = rig.task.search_phase_.first_trip_split_;
                assert(split.pending && split.recovery_direction == side);
                assert(rig.out.split_before_objects.size() == rig.objects.size());
                rig.step();
                rig.step();
                assert(rig.out.motion.vx_mps == -.05f && rig.out.motion.wz_rps == 0);
                // X movement during retreat cannot flip this cycle's latched direction.
                rig.objects[0].box.x = 600 - side * 100;
                rig.turnCycle();
                assert(split.recovery_direction == side);
                // A previously observed danger must be visibly outside before it
                // disappears. Deletion alone cannot clear the new edge review.
                rig.objects[0].box.x = 608;
                for (auto& object : rig.objects)
                    if (object.label == "dangerous_object" ||
                        ((!first_delivered || label == "injured_person") && object.track_id != 1))
                        object.box.y = 30; // whole unwanted boxes leave inventory and path vertically
                for (int i = 0; i < 8; ++i)
                    rig.step();
                // Upper outside authorization needs a current visible detection.
                // Retain the cleared danger while removing other unwanted cargo.
                rig.objects.erase(std::remove_if(rig.objects.begin(), rig.objects.end(),
                    [&](const auto& object) {
                        return object.track_id != 1 && object.label != "dangerous_object" &&
                               (!first_delivered || label == "injured_person");
                    }), rig.objects.end());
                rig.objects[0].box.x = 608; // centre 648, inside ordinary alignment tolerance
                for (int i = 0; i < 24 && split.pending; ++i) {
                    rig.step();
                    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                }
                if (split.pending)
                    std::cerr << "mixed fixture still pending first=" << first_delivered << ",label=" << label
                              << ",side=" << side << ",reason=" << rig.out.reason
                              << ",scene=" << rig.out.capture_scene_detail << "\n";
                assert(!split.pending);
                const auto inventory_count = std::count_if(rig.objects.begin(), rig.objects.end(),
                    [&](const auto& object) {
                        return rig.in.captureInventoryOverlaps(object.box, object.label);
                    });
                assert(rig.task.trip_context_.pending_.total() == inventory_count);
                for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified;
                     ++i) {
                    rig.step();
                    if (rig.out.motion.vx_mps != 0 || rig.out.run_status == RunStatus::Fault)
                        std::cerr << "mixed lifecycle first=" << first_delivered << " label=" << label
                                  << " side=" << side << " state=" << PushTask::name(rig.out.state)
                                  << " reason=" << rig.out.reason << " scene=" << rig.out.capture_scene_detail
                                  << " first_id=" << rig.out.entry_first_id << '\n';
                    assert(rig.out.motion.vx_mps == 0 && rig.out.run_status != RunStatus::Fault);
                }
                assert(rig.out.capture_evidence == CaptureEvidence::Verified);
                assert(rig.out.final_validation == "passed" && rig.out.state == PushState::CARRY);
                assert(checkTrip(rig.task.trip_context_.trip_, first_delivered) == RuleVerdict::OK);
                assert(rig.task.trip_context_.trip_.dangerous == 0);
                assert(rig.out.first_ordinary_delivered == first_delivered);
            }
    std::cout << "PASS mixed cargo: right -> left, left -> right, legal batch -> close -> carry\n";
}

void laterTripMixedRulesAndUnknown() {
    for (const std::string forbidden : {"dangerous_object", "injured_person", "core_supply"}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.risk();
        rig.objects[1].label = forbidden;
        if (forbidden == "core_supply")
            rig.objects.push_back(detection(3, {800, 230, 60, 60}));
        rig.enableSelector();
        for (int i = 0; i < 24 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
            rig.step();
        assert(rig.task.search_phase_.first_trip_split_.pending && rig.task.frame_raised_ == 1);
    }
    for (const std::string label : {"unknown", "dangerous_object"}) {
        SceneRig rig;
        rig.objects.resize(1);
        auto neighbor = detection(2, {800, 140, 60, 40}); // bottom touches Y180
        neighbor.label = label;
        rig.objects.push_back(neighbor);
        for (int i = 0; i < 24 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
            rig.step();
        assert(rig.task.frame_raised_ == 1 && rig.out.capture_evidence == CaptureEvidence::None);
        if (label == "unknown")
            assert(!rig.task.search_phase_.first_trip_split_.pending && stopped(rig.out.motion));
        else
            assert(rig.task.search_phase_.first_trip_split_.pending);
    }
    std::cout << "PASS later danger, injury mixing, >2 supplies and unknown/touching boxes\n";
}

void mixedPostcloseReturnsToReview() {
    for (bool single : {false, true}) {
        SceneRig rig(single);
        rig.task.first_ = true;
        rig.task.single_phase_initializing_ = false;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 1;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.trip_context_.pending_ = inv(1);
        rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "mixed_postclose_fixture");
        rig.task.frame_raised_ = 0;
        rig.objects[1].box = {800, 230, 60, 60};
        rig.objects[1].label = "dangerous_object";
        for (int i = 0; i < 40 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.out.capture_evidence == CaptureEvidence::None);
            assert(rig.out.run_status != RunStatus::Fault);
        }
        assert(rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.task.rejected_.count(1) == 0);
        assert(rig.task.frame_raised_ == 1);
    }
    std::cout
        << "PASS post-close mixed inventory reopens and splits, including standalone capture\n";
}

void splitSelectorAcceptsLaterCargo() {
    for (const std::string label : {"core_supply", "injured_person"}) {
        PixelSelector selector;
        selector.syncSplitSelection(true, 7);
        auto cargo = detection(7, {600, 230, 80, 80});
        cargo.label = label;
        auto danger = detection(8, {900, 230, 80, 80});
        danger.label = "dangerous_object";
        PixelSelection selected;
        for (int i = 0; i < 3; ++i) {
            cargo.timestamp_us = danger.timestamp_us = 1000000 + i * 50000;
            selected = selector.update(
                {cargo, danger}, {1280, 720}, 2000, true, cargo.timestamp_us, true, .25f);
        }
        assert(selected.locked && selected.id == 7);
        PixelSelector first;
        first.syncSplitSelection(true, 7);
        for (int i = 0; i < 4; ++i) {
            cargo.timestamp_us += 50000;
            assert(!first.update({cargo}, {1280, 720}, 2000, true, cargo.timestamp_us, false, .25f)
                        .locked);
        }
    }
    std::cout
        << "PASS split selector supports later core/injured and keeps first-trip restriction\n";
}

void rushIdentityContinuityAndGap() {
    // Real selector + monitor: repeated tracker IDs must not restart alignment or emit zero.
    SceneRig rig;
    rig.objects.resize(1);
    rig.objects[0].box = {608, 80, 80, 80};
    rig.enableSelector();
    for (int i = 0; i < 40 && rig.out.motion.vx_mps == 0; ++i) rig.step();
    assert(rig.out.motion.vx_mps > 0);
    const auto started = rig.task.actionProgress().started_us;
    for (int id = 10; id < 15; ++id) {
        rig.objects[0].track_id = id;
        rig.step();
        assert(rig.selection.locked && rig.selection.rebound_from >= 0);
        assert(rig.out.target_id == id && rig.out.motion.vx_mps > 0);
        assert(rig.task.actionProgress().started_us == started);
        assert(rig.task.pixel_alignment_.stage == PushTask::PixelAlignment::Stage::Advance);
    }
    // Gap observations never fabricate a target. Continuation needs fresh full corridor evidence.
    for (int scenario = 0; scenario < 7; ++scenario) {
        SceneRig gap;
        gap.objects.resize(1);
        gap.objects[0].box = {608, 80, 80, 80};
        gap.enableSelector();
        for (int i = 0; i < 40 && gap.out.motion.vx_mps == 0; ++i) gap.step();
        assert(gap.out.motion.vx_mps > 0);
        auto missing = gap.in;
        const auto last_seen = gap.task.capture_phase_.pixel_forward_us_;
        missing.now_us += scenario == 1 ? 200001 : 50000;
        missing.capture_scene_us = missing.now_us;
        missing.pixel_grasp_sample_us = missing.now_us;
        missing.target_valid = false;
        missing.target_id = -1;
        missing.label.clear();
        missing.capture_image_objects.clear();
        missing.corridor = {};
        missing.corridor.ordinary = 1;
        missing.corridor_complete = missing.corridor_occlusion_free = true;
        if (scenario == 2) missing.corridor_complete = false;
        if (scenario == 3) missing.path_safe = false;
        if (scenario == 4) missing.camera_pitch_stable = false;
        if (scenario == 5) {
            auto danger = gap.in.capture_image_objects.front();
            danger.id = 99;
            danger.label = "dangerous_object";
            danger.box = {600, 250, 80, 80};
            missing.capture_image_objects.push_back(danger);
        }
        if (scenario == 6) missing.pixel_released_id = gap.out.target_id;
        auto out = gap.task.update(missing);
        if (scenario == 0) {
            assert(out.reason == "pixel_rush_gap_hold" && out.motion.vx_mps > 0);
            assert(out.target_id == 1 && gap.task.frame_raised_ == 1);
            assert(gap.task.capture_phase_.pixel_forward_us_ == last_seen);
            missing.now_us = last_seen + 200001;
            missing.capture_scene_us = missing.now_us;
            out = gap.task.update(missing);
            assert(stopped(out.motion)); // gap ticks cannot extend the 200 ms deadline
        } else {
            assert(stopped(out.motion));
            assert(gap.task.frame_raised_ == 1);
            if (scenario == 6) assert(out.reason == "pixel_target_released");
        }
    }
    std::cout << "PASS RUSH re-ID continuity, bounded gap and clearance vetoes\n";
}
void approachOutsideInventoryAndNextTrip() {
    SceneRig rig;
    rig.objects.resize(1);
    rig.objects[0].box = {608, 80, 80, 80}; // entirely above Y180
    for (int i = 0; i < 30 && rig.out.motion.vx_mps == 0; ++i)
        rig.step();
    assert(rig.out.motion.vx_mps > 0 && rig.task.frame_raised_ == 1);
    // A completed trip must not leave scene identities or split limits in the next trip.
    auto& split = rig.task.search_phase_.first_trip_split_;
    split.runs = 20;
    rig.task.trip_context_.trip_ = inv(1);
    rig.task.clearTrip();
    assert(split.runs == 0 && rig.task.capture_scene_.objects.empty());
    rig.task.first_ = true;
    rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 1;
    rig.task.trip_context_.label_ = rig.in.label = "ordinary_supply";
    rig.task.enter(PushState::RUSH, rig.in.now_us, "next_trip_fixture");
    rig.objects[0].box = {608, 350, 80, 80};
    auto danger = detection(2, {850, 230, 60, 60});
    danger.label = "dangerous_object";
    rig.objects.push_back(danger);
    for (int i = 0; i < 24 && !split.pending; ++i)
        rig.step();
    assert(split.pending && split.runs == 1);
    std::cout << "PASS normal approach outside inventory and fresh next-trip split budget\n";
}

void laterSplitReacquiresNewCargoClass() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (const std::string label : {"core_supply", "injured_person"}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.objects.resize(1);
        rig.objects[0].box = {608, 350, 80, 80};
        auto danger = detection(2, {900, 230, 60, 60});
        danger.label = "dangerous_object";
        rig.objects.push_back(danger);
        rig.enableSelector();
        auto& split = rig.task.search_phase_.first_trip_split_;
        for (int i = 0; i < 32 && !split.pending; ++i)
            rig.step();
        assert(split.pending);
        rig.objects[0].box.y = 240;
        rig.objects[1].box.y = 100; // unwanted dangerous box fully clears both regions
        rig.finishRetreat();
        assert(split.recovery_stage == Stage::Turn);
        rig.in.camera_pitch_cdeg = 0;
        for (int i = 0; i < 16 && split.recovery_stage == Stage::Turn; ++i)
            rig.step();
        assert(split.recovery_stage == Stage::Verify && split.target_reselecting);
        // After the turn the old ordinary target has left; nominate a new legal class and ID.
        rig.objects = {detection(11, {608, 230, 80, 80})};
        rig.objects[0].label = label;
        rig.in.camera_pitch_cdeg = 2000;
        bool nominated = false;
        for (int i = 0; i < 40 && split.pending; ++i) {
            rig.step();
            nominated |= rig.out.split_target_candidate_id == 11;
            assert(stopped(rig.out.motion));
        }
        assert(nominated && !split.pending);
        assert(rig.out.target_id == 11 && rig.out.capture_target_id == 11);
        assert(rig.task.trip_context_.label_ == label);
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i)
            rig.step();
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.out.state == PushState::CARRY);
        assert(rig.task.trip_context_.trip_.count(targetKind(label)) == 1);
    }
    std::cout << "PASS real selector + monitor: new core/injured class and ID -> close -> carry\n";
}
// 00:43:39: ordinary #18 alternates complete/top-clipped outside Y180.
// Exercise the real monitor and selector; leaving UI140 is not capture success.
void topClippedPostTurnTargetReacquires() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true}) for (bool jitter : {false, true})
    for (int gate = 0; gate < 5; ++gate) {
        SceneRig rig(single);
        rig.task.single_phase_initializing_ = false;
        auto config = calibration();
        config.pixel_center_membership = true; // actual field profile uses box centres
        rig.monitor = CaptureMonitor{config};
        rig.managed_selection = true;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.vertical_swing = true;
        split.runs = 2;
        split.started_us = split.recovery_view_us = rig.in.now_us;
        split.recovery_stage = Stage::Verify;
        split.target_reselecting = true;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "top_clipped_postturn_replay");
        rig.objects = {detection(18, {498, 0, 193, 209})};
        if (gate == 1) rig.in.sent_vx_mps = .02f;
        if (gate == 3) rig.in.camera_pitch_stable = false;
        // A clipped box whose centre remains in the split region must still wait.
        if (gate == 4) rig.objects[0].box = {498, 0, 193, 430};
        bool resumed = false;
        for (int i = 0; i < 16; ++i) {
            if (jitter && gate != 4)
                rig.objects[0].box = i % 2 ? cv::Rect{498, 7, 193, 192}
                                          : cv::Rect{498, 0, 193, 209};
            rig.step(gate != 2);
            assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
            if (gate != 0) assert(split.pending);
            if (!split.pending) {
                resumed = rig.out.state == PushState::APPROACH &&
                          rig.out.reason == "pixel_split_resume_ui120";
                break;
            }
        }
        if (gate != 0) {
            rig.in.sent_vx_mps = 0;
            rig.in.camera_pitch_stable = true;
            for (int i = 0; i < 16 && split.pending; ++i) {
                rig.objects[0].box = jitter && i % 2 ? cv::Rect{498, 7, 193, 192}
                                                    : cv::Rect{498, 0, 193, 209};
                rig.step();
                assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                resumed |= rig.out.state == PushState::APPROACH &&
                           rig.out.reason == "pixel_split_resume_ui120";
            }
        }
        if (!resumed || split.pending || split.runs != 2 || !split.capture_retry)
            std::cerr << "top-clipped replay single=" << single << " jitter=" << jitter
                      << " gate=" << gate << " reason=" << rig.out.reason
                      << " pending=" << split.pending << " confirmations="
                      << rig.task.capture_scene_.confirmations << " stopped_us="
                      << rig.task.capture_scene_.stopped_us << " now=" << rig.in.now_us
                      << " capture_retry=" << split.capture_retry << std::endl;
        assert(resumed && !split.pending && split.runs == 2 && split.capture_retry);
        assert(rig.out.motion.camera_pitch_cdeg == 0);
        assert(rig.out.final_validation == "not_executed");
    }
    std::cout << "PASS top-clipped/jitter post-turn target resumes UI120 only after stopped fresh outside review\n";
}

void latestOutsideInventoryTargetReacquires() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true}) {
        SceneRig rig(single);
        rig.task.single_phase_initializing_ = false;
        auto c = calibration();
        c.grasp_center_ranges[0].area = {342, 226, 1031, 489};
        c.grasp_center_ranges[0].lateral_measured = true;
        rig.monitor = CaptureMonitor{c};
        rig.managed_selection = true;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.vertical_swing = true;
        split.runs = 1;
        split.started_us = split.recovery_view_us = rig.in.now_us;
        split.recovery_stage = Stage::Verify;
        split.target_reselecting = true;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "latest_postturn_replay");
        // 2026-10-31 00:35:53: after the first reverse/turn, ordinary #12 is
        // complete at the top left, outside X342..1031 / Y180..489 inventory.
        rig.objects = {detection(12, {5, 3, 253, 264}),
                       detection(13, {207, 281, 335, 294}),
                       detection(14, {338, 240, 92, 52})};
        rig.objects[1].label = "dangerous_object";
        rig.objects[2].label = "core_supply";
        // Fresh observations while coasting cannot start the recovery transition.
        rig.in.sent_vx_mps = .02f;
        for (int i = 0; i < 12; ++i) {
            rig.step();
            assert(split.pending && stopped(rig.out.motion));
        }
        rig.in.sent_vx_mps = 0;
        for (int i = 0; i < 12; ++i) {
            rig.step(false);
            assert(split.pending && stopped(rig.out.motion));
        }
        bool resumed = false;
        for (int i = 0; i < 12; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            resumed |= rig.out.state == PushState::APPROACH &&
                       rig.out.reason == "pixel_split_resume_ui120";
            if (resumed)
                break;
        }
        if (!resumed)
            std::cerr << "latest post-turn target #12 cannot resume approach: " << rig.out.reason
                      << std::endl;
        assert(resumed);
        assert(rig.out.motion.camera_pitch_cdeg == 0 && !split.pending && split.capture_retry);
        assert(split.runs == 1 && rig.out.final_validation == "not_executed");
        // Continue through the real selector and monitor at both pitches. Only a
        // newly observed legal inventory can finish this capture attempt.
        rig.objects = {detection(17, {600, 260, 80, 80})};
        rig.in.camera_pitch_cdeg = 0;
        rig.in.heading_valid = !single; // normal UI120 approach also supports image-only input
        bool approached = false;
        for (int i = 0; i < 40 && !approached; ++i) {
            rig.step();
            assert(!rig.task.fault_ && rig.task.frame_raised_ == 1);
            approached = rig.out.motion.vx_mps > 0;
        }
        assert(approached && rig.out.target_id == 17);
        rig.objects[0].box.y = 500;
        for (int i = 0; i < 40 && rig.out.motion.camera_pitch_cdeg != 2000; ++i)
            rig.step();
        assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 2000);
        rig.in.camera_pitch_cdeg = 2000;
        rig.objects[0].box = {608, 310, 80, 80};
        bool preclose = false;
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.step();
            preclose |= rig.out.capture_review_stage == "preclose";
            assert(!rig.task.fault_ && !split.pending);
        }
        assert(preclose && rig.out.final_validation == "passed");
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(single ? rig.out.single_phase_done : rig.out.state == PushState::CARRY);
    }
    std::cout << "PASS latest outside-inventory target: stopped fresh review -> UI120 approach -> "
                 "UI140 capture\n";
}

// 00:44:51: #28 is acquired before #31 enters view, then short gaps keep
// the tiny top-edge target alive until the 15s approach deadline.
void latestCornerReturnWaitsForScene() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true}) {
        SceneRig rig(single);
        rig.task.single_phase_initializing_ = false;
        rig.managed_selection = true;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.upper_corner = true;
        split.runs = 1;
        split.started_us = rig.in.now_us;
        split.recovery_stage = Stage::CornerTurnBack;
        split.corner_heading = 0;
        rig.in.heading_rad = -.174532925f; // final heading after the left-80 return
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "latest_corner_return");
        rig.in.camera_pitch_cdeg = 0;
        rig.objects = {detection(28, {1142, 15, 123, 107})};
        for (int i = 0; i < 12 && rig.out.reason != "pixel_split_corner_resume_ui120"; ++i)
            rig.step();
        assert(rig.out.reason == "pixel_split_corner_resume_ui120");
        const auto resumed = rig.in.now_us;
        // Coasting and repeated old images must not consume the observation window.
        rig.in.sent_wz_rps = .1f;
        for (int i = 0; i < 12; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.out.target_id == -1);
        }
        rig.in.sent_wz_rps = 0;
        for (int i = 0; i < 12; ++i) {
            rig.step(false);
            assert(stopped(rig.out.motion) && rig.out.target_id == -1);
        }
        for (int i = 0; i < 7; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.out.target_id == -1);
        }
        rig.in.camera_pitch_stable = false;
        rig.step();
        assert(stopped(rig.out.motion) && rig.out.pixel_selection_paused);
        rig.in.camera_pitch_stable = true;
        for (int i = 0; i < 3; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.out.pixel_selection_paused);
        }
        rig.objects = {detection(28, {609, 8, 56, 31}),
                       detection(31, {829, 114, 163, 142})};
        for (int i = 0; i < 16 && rig.out.target_id < 0; ++i)
            rig.step();
        assert(rig.out.target_id == 31 && rig.out.capture_target_id == 31);
        assert(rig.in.now_us - resumed > 500000 && !rig.task.fault_);
    }
    std::cout << "PASS latest corner return: stopped fresh observation selects later #31\n";
}
void latestIntermittentTargetReselects() {
    for (bool single : {false, true}) {
        SceneRig rig(single);
        rig.task.single_phase_initializing_ = false;
        rig.managed_selection = true;
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "latest_stuck_target");
        rig.task.search_phase_.approach_pitch_ = 0;
        rig.task.search_phase_.pixel_approach_started_us_ = rig.in.now_us;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 28;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.search_phase_.first_trip_split_.capture_retry = true;
        rig.task.search_phase_.first_trip_split_.runs = 1;
        rig.task.frame_raised_ = 1;
        rig.in.camera_pitch_cdeg = 0;
        // Start with an actually acquired selector lock, as in the field log.
        for (int i = 0; i < 3; ++i) {
            rig.in.now_us += 50000;
            auto old = detection(28, {609, 8, 56, 31});
            old.timestamp_us = rig.in.now_us;
            rig.selection = rig.selector.update({old}, {1280, 720}, 0, true,
                                                 rig.in.now_us, false, .25f);
        }
        assert(rig.selection.id == 28 && rig.selection.locked);
        const auto started = rig.in.now_us;
        rig.task.search_phase_.pixel_approach_started_us_ = started;
        bool released = false, selected = false;
        for (int i = 0; i < 120 && !selected; ++i) {
            rig.objects = {detection(31, {829, 114, 163, 142})};
            if (i % 3 == 0)
                rig.objects.push_back(detection(28, {609, 8, 56, 31}));
            rig.step();
            released |= rig.out.reason == "pixel_approach_reselect";
            selected = rig.out.target_id == 31;
            assert(!rig.task.fault_ && rig.out.state != PushState::CAPTURE_FAIL);
            if (released && !selected)
                assert(stopped(rig.out.motion));
        }
        if (!selected) std::cerr << "stuck #28 never selected #31: " << rig.out.reason << '\n';
        assert(released && selected && rig.in.now_us - started < 6000000);
        // Reselection preserves the original approach deadline and all capture checks.
        assert(rig.task.search_phase_.pixel_approach_started_us_ == started);
        assert(rig.task.rejected_.count(28) && rig.task.frame_raised_ == 1);
        rig.objects = {detection(31, {600, 260, 80, 80})};
        bool approached = false;
        for (int i = 0; i < 40 && !approached; ++i) {
            rig.step();
            approached = rig.out.motion.vx_mps > 0;
        }
        assert(approached);
        rig.objects[0].box.y = 500;
        for (int i = 0; i < 40 && rig.out.motion.camera_pitch_cdeg != 2000; ++i)
            rig.step();
        assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 2000);
        rig.in.camera_pitch_cdeg = 2000;
        rig.objects[0].box = {608, 310, 80, 80};
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i)
            rig.step();
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(single ? rig.out.single_phase_done : rig.out.state == PushState::CARRY);
    }
    std::cout << "PASS latest intermittent #28: stopped reselect #31 -> normal verified capture\n";
}

void latestUi120HandoffJitterCompletesCapture() {
    for (bool single : {false, true}) {
        SceneRig rig(single);
        rig.task.single_phase_initializing_ = false;
        rig.managed_selection = true;
        rig.task.t_.pixel_align_brake_motion_px = 30; // actual field runtime
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "latest_handoff_jitter");
        rig.task.search_phase_.approach_pitch_ = 0;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 18;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.search_phase_.first_trip_split_.capture_retry = true;
        rig.task.search_phase_.first_trip_split_.runs = 1;
        rig.task.frame_raised_ = 1;
        rig.in.camera_pitch_cdeg = 0;
        rig.objects = {detection(18, {561, 412, 213, 254})};
        for (int i = 0; i < 4; ++i) {
            rig.step();
            assert(rig.out.motion.camera_pitch_cdeg == 0);
        }
        // Same #18 boxes as 12:20:33: Y539/549 straddle the Y540 handoff.
        rig.objects[0].box.y = 422;
        rig.in.sent_wz_rps = .1f;
        for (int i = 0; i < 8; ++i) {
            rig.step();
            assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 0);
        }
        rig.in.sent_wz_rps = 0;
        for (int i = 0; i < 6; ++i) {
            rig.step(false); // repeated images cannot confirm a stopped handoff
            assert(stopped(rig.out.motion) && rig.out.motion.camera_pitch_cdeg == 0);
        }
        const auto started = rig.in.now_us;
        bool switched = false;
        for (int i = 0; i < 40 && !switched; ++i) {
            rig.objects[0].box.y = i % 2 ? 412 : 422;
            rig.step();
            assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            switched = rig.out.motion.camera_pitch_cdeg == 2000;
        }
        if (!switched) std::cerr << "Y539/549 handoff stuck: " << rig.out.reason << '\n';
        assert(switched && rig.in.now_us - started < 2000000);
        assert(rig.out.capture_evidence != CaptureEvidence::Verified);
        // A view handoff is not capture evidence: finish through UI140 entry,
        // original preclose scene checks, close feedback and held-object review.
        rig.in.camera_pitch_cdeg = 2000;
        rig.objects[0].box = {608, 310, 80, 80};
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i)
            rig.step();
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(single ? rig.out.single_phase_done : rig.out.state == PushState::CARRY);
    }
    std::cout << "PASS latest #18 Y539/549 handoff jitter -> UI140 -> verified capture\n";
}

void ui120HandoffInvalidEvidenceClearsIntent() {
    for (int interruption = 0; interruption < 3; ++interruption) {
        SceneRig rig;
        rig.task.t_.pixel_align_brake_motion_px = 30;
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "handoff_interrupted");
        rig.task.search_phase_.approach_pitch_ = 0;
        rig.task.frame_raised_ = 1;
        rig.in.camera_pitch_cdeg = 0;
        rig.objects = {detection(1, {561, 422, 213, 254})};
        rig.step();
        assert(rig.task.pixel_alignment_.holding_position);
        if (interruption == 0)
            rig.objects.clear();
        else if (interruption == 1)
            rig.in.camera_pitch_stable = false;
        else {
            rig.in.target_id = 19;
            rig.objects = {detection(19, {561, 412, 213, 254})};
        }
        rig.step();
        assert(!rig.task.pixel_alignment_.holding_position);
        rig.in.target_id = 1;
        rig.in.camera_pitch_stable = true;
        rig.objects = {detection(1, {561, 412, 213, 254})}; // Y539 alone cannot hand off
        for (int i = 0; i < 12; ++i) {
            rig.step();
            assert(rig.out.motion.camera_pitch_cdeg == 0);
            assert(!rig.task.pixel_alignment_.holding_position);
        }
    }
    std::cout << "PASS UI120 handoff intent resets on missing target, unstable camera or changed ID\n";
}

void ui120ProgressAndUnusableAlternativesKeepLock() {
    for (int scenario = 0; scenario < 3; ++scenario) {
        SceneRig rig;
        rig.managed_selection = true;
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "reselect_retention");
        rig.task.search_phase_.approach_pitch_ = 0;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 28;
        rig.in.camera_pitch_cdeg = 0;
        for (int i = 0; i < 3; ++i) {
            rig.in.now_us += 50000;
            auto old = detection(28, {610, 180, 56, 31});
            old.timestamp_us = rig.in.now_us;
            rig.selection = rig.selector.update({old}, {1280, 720}, 0, true,
                                                 rig.in.now_us, false, .25f);
        }
        assert(rig.selection.id == 28);
        bool advanced = false;
        for (int i = 0; i < 120; ++i) {
            rig.objects.clear();
            if (scenario == 0 || i % 3 == 0)
                rig.objects.push_back(detection(28, {610, 180, 56, 31}));
            if (scenario < 2) {
                auto other = detection(31, scenario == 0 ? cv::Rect{829, 114, 163, 142}
                                                         : cv::Rect{0, 114, 163, 142});
                rig.objects.push_back(other);
            }
            rig.in.sent_vx_mps = rig.out.motion.vx_mps;
            rig.in.sent_wz_rps = rig.out.motion.wz_rps;
            rig.step();
            advanced |= rig.out.motion.vx_mps > 0;
            assert(rig.out.target_id == 28 && !rig.out.pixel_selection_paused);
            assert(rig.out.reason != "pixel_approach_reselect" && !rig.task.fault_);
        }
        if (scenario == 0) assert(advanced);
    }
    std::cout << "PASS progressing lock, clipped alternate, and no alternate retain #28\n";
}
void splitRetreatClearanceUsesWholeReviewRegions() {
    SceneRig rig;
    auto& split = rig.task.search_phase_.first_trip_split_;
    rig.step();
    PushObservation::CaptureImageObject object{2, "ordinary_supply", {600, 100, 80, 80}, true};
    assert(!rig.task.splitRetreatCleared(rig.in, object)); // bottom touches Y180
    object.box.y = 99;
    assert(rig.task.splitRetreatCleared(rig.in, object));
    rig.in.capture_path_px = {500, 90, 300, 200};
    assert(!rig.task.splitRetreatCleared(rig.in, object)); // outside inventory, still in path
    rig.in.capture_path_px = {500, 205, 300, 284};
    object.label = "core_supply";
    rig.in.capture_class_regions_px[object.label] = {500, 100, 300, 100};
    assert(!rig.task.splitRetreatCleared(rig.in, object)); // class-specific inventory
    object.box.y = 19;
    assert(rig.task.splitRetreatCleared(rig.in, object));
    object.box.y = 240;
    object.label = "ordinary_supply";
    assert(!rig.task.splitRetreatCleared(rig.in, object));
    split.upper_corner = true;
    assert(rig.task.splitRetreatCleared(rig.in, object)); // corner preprocessing still stops at Y280
    object.box.y = 241;
    assert(!rig.task.splitRetreatCleared(rig.in, object));
    std::cout << "PASS split clearance: contact, class regions, path, corner Y280\n";
}
void latestSplitRetreatDoesNotSpendRetriesOnOverlappingBoxes() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    // 2026-10-31 13:23:26: centres already crossed Y280, but the full
    // neighbours still overlap inventory. Both ordinary attempts did no reverse.
    for (const auto& boxes : {
             std::pair{cv::Rect{485, 235, 284, 305}, cv::Rect{647, 99, 264, 294}},
             std::pair{cv::Rect{202, 271, 284, 294}, cv::Rect{353, 73, 244, 285}},
             std::pair{cv::Rect{116, 296, 314, 325}, cv::Rect{237, 139, 315, 274}}}) {
        SceneRig rig;
        auto c = calibration();
        c.grasp_center_ranges[0].area = {342, 226, 1031, 489};
        c.grasp_center_ranges[0].lateral_measured = true;
        rig.monitor = CaptureMonitor{c};
        rig.objects = {detection(1, boxes.first), detection(2, boxes.second)};
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.runs = split.corner_runs = 1; // the corner preprocessing has finished
        for (int i = 0; i < 20 && !split.pending; ++i)
            rig.step();
        assert(split.pending && !split.upper_corner && split.runs == 2);
        assert(rig.in.ordinaryInventoryOverlaps(boxes.second));
        assert(boxes.second.y + .5f * boxes.second.height <= 280.f);
        for (int i = 0; i < 20; ++i) {
            rig.step();
            assert(split.recovery_stage == Stage::Retreat);
            assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
            assert(split.runs == 2 && !rig.task.fault_);
            assert(rig.out.capture_evidence == CaptureEvidence::None);
        }
        // Fresh, complete evidence of actual clearance permits the existing turn.
        rig.objects[1].box = {700, 50, 80, 80};
        rig.turnCycle();
        // The split returns to normal capture only after the retained block is
        // inside its calibrated range and the whole neighbour has left.
        rig.objects[0].box = {608, 310, 80, 80};
        for (int i = 0; i < 120 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.step();
            assert(!rig.task.fault_ && split.runs == 2);
        }
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.out.final_validation == "passed");
    }
    std::cout << "PASS 13:23:26 overlap replay: reverse until whole-box clearance, capture without exhausting retries\n";
}

void setupDangerousSideRig(SceneRig& rig, bool mirror = false, float scale = 1.f) {
    auto c = calibration();
    c.grasp_center_ranges[0].area = {342, 226, 1031, 489};
    c.grasp_center_ranges[0].lateral_measured = true;
    for (auto& r : c.grasp_center_ranges) {
        if (mirror) {
            const auto old = r.area;
            r.area.x1 = 1280 - old.x2;
            r.area.x2 = 1280 - old.x1;
            r.target_center_px.x = 1280 - r.target_center_px.x;
        }
        r.area.x1 *= scale; r.area.x2 *= scale;
        r.target_center_px.x *= scale;
        r.image_width_px = int(1280 * scale);
        r.image_height_px = 720;
    }
    rig.monitor = CaptureMonitor{c};
    rig.objects = {detection(1, {500, 235, 213, 264}), detection(29, {1061, 94, 219, 264})};
    for (auto& object : rig.objects) {
        if (mirror)
            object.box.x = 1280 - object.box.br().x;
        object.box = cv::Rect{int(object.box.x * scale), object.box.y,
                              int(object.box.width * scale), object.box.height};
    }
    rig.objects[1].label = "dangerous_object";
}
cv::Rect sideBox(cv::Rect box, bool mirror, float scale) {
    if (mirror)
        box.x = 1280 - box.br().x;
    return {int(box.x * scale), box.y, int(box.width * scale), box.height};
}
void waitForSideClosure(SceneRig& rig) {
    for (int i = 0; i < 100 && rig.task.frame_raised_ != 0; ++i)
        rig.step();
    assert(rig.task.frame_raised_ == 0);
}
void latestDangerousSideJitterKeepsFirstClosure() {
    for (const bool mirror : {false, true})
        for (const float scale : {.5f, 1.f, 2.f}) {
            SceneRig rig;
            setupDangerousSideRig(rig, mirror, scale);
            waitForSideClosure(rig);
            const std::vector<cv::Rect> jitter = {{1021, 94, 254, 264}, {1041, 94, 239, 264},
                                                {1021, 84, 254, 264}, {1036, 94, 244, 264}};
            bool pending_seen = false;
            for (int i = 0; i < 80 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
                rig.objects[1].box = sideBox(jitter[std::min(i / 3, 3)], mirror, scale);
                rig.step();
                pending_seen |= rig.in.capture_dangerous_side_pending;
                if (rig.task.frame_raised_ != 0 || rig.task.search_phase_.first_trip_split_.pending)
                    std::cerr << "14:12 side-jitter reopened/retried: " << rig.out.reason << '\n';
                assert(rig.task.frame_raised_ == 0);
                assert(!rig.task.search_phase_.first_trip_split_.pending);
                assert(stopped(rig.out.motion));
            }
            if (!pending_seen)
                std::cerr << "pending absent mirror=" << mirror << ",scale=" << scale << ",scene=" << rig.out.capture_scene_detail << "\n";
            assert(pending_seen);
            assert(rig.out.capture_evidence == CaptureEvidence::Verified);
            assert(rig.out.final_validation == "passed");
        }
    std::cout << "PASS 14:12 dangerous side-jitter verifies first closure, mirrored and laterally scaled\n";
}
void dangerousSidePendingDoesNotGrantClosureOrSuccess() {
    SceneRig pre;
    setupDangerousSideRig(pre);
    pre.objects[1].box = {1021, 94, 254, 264};
    for (int i = 0; i < 14; ++i) {
        pre.step();
        assert(pre.task.frame_raised_ == 1 && stopped(pre.out.motion));
        assert(!pre.task.search_phase_.first_trip_split_.pending);
    }
    pre.objects[1].box = {1036, 94, 244, 264};
    for (int i = 0; i < 100 && pre.out.capture_evidence != CaptureEvidence::Verified; ++i)
        pre.step();
    assert(pre.out.capture_evidence == CaptureEvidence::Verified);

    SceneRig post;
    setupDangerousSideRig(post);
    waitForSideClosure(post);
    post.objects[1].box = {1021, 94, 254, 264};
    bool failed = false;
    for (int i = 0; i < 50; ++i) {
        post.step();
        assert(stopped(post.out.motion));
        assert(post.out.capture_evidence != CaptureEvidence::Verified);
        if (post.out.final_validation == "failed") {
            failed = true;
            break;
        }
        assert(post.task.frame_raised_ == 0);
        assert(!post.task.search_phase_.first_trip_split_.pending);
        assert(!post.in.held_complete);
    }
    assert(failed && post.out.reason.find("dangerous_side_boundary_pending") != std::string::npos);
    std::cout << "PASS shallow dangerous overlap waits before close and expires verification budget\n";
}
void dangerousSideClearanceRejectsInvalidEvidence() {
    for (int fault = 0; fault < 6; ++fault) {
        SceneRig rig;
        setupDangerousSideRig(rig);
        waitForSideClosure(rig);
        rig.objects[1].box = {1021, 94, 254, 264};
        rig.step(); rig.step();
        assert(rig.in.capture_dangerous_side_pending);
        rig.objects[1].box = {1036, 94, 244, 264};
        if (fault == 0) rig.in.sent_vx_mps = .02f;
        if (fault == 1) rig.in.camera_pitch_stable = false;
        if (fault == 2) rig.objects[1].track_id = 30;
        if (fault == 3) rig.objects.resize(1);
        if (fault == 4) rig.in.sent_velocity_valid = false;
        for (int i = 0; i < 6; ++i) {
            rig.step(fault != 5);
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
            assert(rig.task.frame_raised_ == 0 && stopped(rig.out.motion));
        }
        // No invalid interval may contribute to the later clear window.
        if (fault == 0) rig.in.sent_vx_mps = 0;
        if (fault == 1) rig.in.camera_pitch_stable = true;
        if (fault == 4) rig.in.sent_velocity_valid = true;
        if (fault != 2 && fault != 3) {
            for (int i = 0; i < 4; ++i) {
                rig.step();
                assert(rig.out.capture_evidence != CaptureEvidence::Verified);
            }
            for (int i = 0; i < 15 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i)
                rig.step();
            assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        }
    }
    std::cout << "PASS dangerous clearance requires stable identity, fresh frames and stopped camera/chassis\n";
}
void dangerousDefiniteRiskIsNotErasedByShallowOverlap() {
    for (const cv::Rect box : {cv::Rect{980, 94, 254, 264}, cv::Rect{900, 94, 254, 264}}) {
        SceneRig rig;
        setupDangerousSideRig(rig);
        waitForSideClosure(rig);
        rig.objects[1].box = box;
        rig.step();
        assert(rig.in.held.dangerous == 1);
        rig.objects[1].box = {1021, 94, 254, 264};
        bool rejected = false;
        for (int i = 0; i < 12; ++i) {
            rig.step();
            assert(rig.in.held.dangerous == 1);
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
            if (rig.task.frame_raised_ != 0 || rig.out.final_validation == "failed") {
                rejected = true;
                break;
            }
        }
        assert(rejected);
    }
    // Ordinary, core, injured and unknown preserve the established whole-box rule.
    for (const std::string label : {"ordinary_supply", "core_supply", "injured_person", "unknown"}) {
        SceneRig rig;
        setupDangerousSideRig(rig);
        rig.objects[1].label = label;
        rig.objects[1].box = {1021, 94, 254, 264};
        rig.step();
        assert(rig.in.captureInventoryOverlaps(rig.objects[1].box, label));
        assert(!rig.in.capture_dangerous_side_pending);
    }
    std::cout << "PASS definite dangerous intrusion remains rejected and other cargo retains whole-box checks\n";
}

void dangerousSideHistoryEndsAfterConfirmedClearance() {
    for (const bool resolve_first : {false, true}) {
        SceneRig rig;
        setupDangerousSideRig(rig);
        rig.objects[1].box = {1021, 94, 254, 264};
        rig.step();
        assert(rig.in.capture_dangerous_side_pending);
        if (resolve_first) {
            rig.objects[1].box = {1036, 94, 244, 264};
            for (int i = 0; i < 8; ++i)
                rig.step();
            assert(!rig.in.capture_dangerous_side_pending);
        }
        rig.objects.resize(1); // confirmed outside may leave; unresolved may not disappear safely
        rig.objects[0].track_id = 42;
        rig.in.target_id = 42;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 42;
        rig.task.capture_scene_ = {};
        rig.step();
        assert(rig.in.capture_dangerous_side_pending == !resolve_first);
        if (resolve_first) {
            for (int i = 0; i < 80 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i)
                rig.step();
            assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        } else {
            for (int i = 0; i < 8; ++i) {
                rig.step();
                assert(rig.out.capture_evidence != CaptureEvidence::Verified);
                assert(stopped(rig.out.motion));
            }
        }
    }
    std::cout << "PASS confirmed outside history expires on target change; unresolved missing danger stays pending\n";
}

void dangerousWholeBoxCanClearVertically() {
    SceneRig rig;
    setupDangerousSideRig(rig);
    rig.objects[1].box = {980, 290, 254, 60};
    rig.step();
    assert(rig.in.corridor.dangerous == 1);
    rig.objects[1].box = {980, 30, 254, 60};
    for (int i = 0; i < 9; ++i)
        rig.step();
    assert(!rig.in.capture_dangerous_side_pending);
    assert(!rig.in.captureInventoryOverlaps(rig.objects[1].box, "dangerous_object"));
    assert(!rig.in.capturePathOverlaps(rig.objects[1].box, "dangerous_object"));
    std::cout << "PASS complete dangerous bbox can confirm clearance above the capture regions\n";
}

// Field replay 2026-10-31 15:29:54: the dangerous block's entering bottom
// alternates Y176/Y181; image-top clipping at Y0 does not hide that bottom.
// Keep the production four held frames and 1.5 s budget at the 25 ms image cadence.
TaskTuning upperBoundaryTuning() {
    auto t = tuning();
    t.capture_frames = 4;
    t.verify_budget_us = 1500000;
    return t;
}
void setupDangerousUpperRig(SceneRig& rig) {
    setupDangerousSideRig(rig);
    rig.objects[0].box = {541, 235, 213, 264};
    rig.objects[1] = detection(2, {889, 13, 244, 163});
    rig.objects[1].label = "dangerous_object";
    assert(rig.t.capture_frames == 4 && rig.t.verify_budget_us == 1500000);
}
void upperStep(SceneRig& rig, bool fresh = true) {
    rig.step(fresh, true, 25000);
}
bool upperDangerClear(const SceneRig& rig, int id = 2) {
    for (const auto& object : rig.in.capture_image_objects)
        if (object.id == id && object.label == "dangerous_object")
            return object.dangerous_side_clear;
    return false;
}
void waitForUpperClosure(SceneRig& rig) {
    for (int i = 0; i < 160 && rig.task.frame_raised_ != 0; ++i) {
        upperStep(rig);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
    }
    assert(rig.task.frame_raised_ == 0);
}
void latestDangerousUpperJitterKeepsFirstClosure() {
    SceneRig rig(false, upperBoundaryTuning());
    setupDangerousUpperRig(rig);
    waitForUpperClosure(rig);
    const auto closed_us = rig.in.now_us;
    const std::vector<cv::Rect> jitter = {
        {879, 8, 244, 173}, {889, 13, 244, 163}, {889, 0, 244, 176},
        {889, 8, 244, 173}, {879, 0, 244, 176}, {879, 13, 244, 163}};
    int held_confirmation_frames = 0;
    for (int i = 0; i < 60 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
        rig.objects[1].box = jitter[size_t(i) % jitter.size()];
        upperStep(rig);
        if (rig.in.held_complete && rig.in.captured && rig.in.held.ordinary == 1)
            ++held_confirmation_frames;
        if (rig.task.frame_raised_ != 0 || rig.task.search_phase_.first_trip_split_.pending)
            std::cerr << "15:29 upper-jitter reopened/retried: " << rig.out.reason << '\n';
        assert(rig.task.frame_raised_ == 0);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.in.held.dangerous == 0);
        assert(stopped(rig.out.motion));
    }
    assert(rig.out.capture_evidence == CaptureEvidence::Verified);
    assert(rig.out.final_validation == "passed");
    assert(held_confirmation_frames >= 4);
    assert(rig.in.now_us - closed_us <= 1500000);
    std::cout << "PASS 15:29 Y176/Y181/Y0H176 replay keeps first closure with 4 frames in 1.5 s\n";
}
void dangerousUpperClippedOutsideResumesPrecloseReview() {
    SceneRig rig(false, upperBoundaryTuning());
    setupDangerousUpperRig(rig);
    // Reproduce the unresolved last-open review, then show the same visible bottom
    // in independent complete and image-top-clipped frames for the clear window.
    rig.objects[1].box = {879, 8, 244, 173};
    upperStep(rig);
    assert(rig.task.frame_raised_ == 1);
    bool clipped_seen = false;
    int closes = 0, reopens = 0;
    int previous_frame = rig.task.frame_raised_;
    for (int i = 0; i < 160 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
        rig.objects[1].box = i % 2 ? cv::Rect{889, 13, 244, 163}
                                  : cv::Rect{889, 0, 244, 176};
        upperStep(rig);
        closes += previous_frame == 1 && rig.task.frame_raised_ == 0;
        reopens += previous_frame == 0 && rig.task.frame_raised_ == 1;
        previous_frame = rig.task.frame_raised_;
        for (const auto& object : rig.in.capture_image_objects)
            if (object.id == 2 && object.box.y == 0) {
                clipped_seen = true;
                assert(!object.complete); // Only the entering edge is usable.
            }
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(stopped(rig.out.motion));
    }
    assert(clipped_seen && rig.task.frame_raised_ == 0);
    assert(closes == 1 && reopens == 0);
    assert(rig.out.capture_evidence == CaptureEvidence::Verified);
    std::cout << "PASS visible Y176 bottom clears last-open review despite unrelated top clipping\n";
}
void dangerousUpperInitialShallowDoesNotGrantClosure() {
    for (const int bottom : {180, 181, 182}) {
        SceneRig rig(false, upperBoundaryTuning());
        setupDangerousUpperRig(rig);
        rig.objects[1].box = {879, 8, 244, bottom - 8};
        for (int i = 0; i < 24; ++i) {
            upperStep(rig);
            assert(rig.in.capture_dangerous_side_pending);
            assert(!upperDangerClear(rig));
            assert(rig.task.frame_raised_ == 1 && stopped(rig.out.motion));
            assert(!rig.task.search_phase_.first_trip_split_.pending);
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
        }
    }
    std::cout << "PASS initial Y180/Y181/Y182 uncertainty cannot grant closure or success\n";
}
void dangerousUpperDefiniteRiskCannotBeWashedByShallow() {
    for (const cv::Rect deep : {cv::Rect{879, 8, 244, 175}, cv::Rect{879, 8, 244, 183},
                                cv::Rect{879, 0, 244, 191}}) {
        SceneRig rig(false, upperBoundaryTuning());
        setupDangerousUpperRig(rig);
        waitForUpperClosure(rig);
        rig.objects[1].box = deep;
        upperStep(rig);
        assert(rig.in.held.dangerous == 1);
        rig.objects[1].box = {879, 8, 244, 173};
        bool rejected = false;
        for (int i = 0; i < 20; ++i) {
            upperStep(rig);
            assert(rig.in.held.dangerous == 1);
            assert(!upperDangerClear(rig));
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
            if (rig.task.frame_raised_ != 0 || rig.out.final_validation == "failed") {
                rejected = true;
                break;
            }
        }
        assert(rejected || rig.in.capture_dangerous_side_detail.find("risk") != std::string::npos);
    }
    std::cout << "PASS Y183/Y191 definite intrusion, including Y0 clipping, is not cleared by one-pixel shallowness\n";
}
void dangerousUpperOutsideCannotSurviveInvalidEvidence() {
    for (int fault = 0; fault < 15; ++fault) {
        SceneRig rig(false, upperBoundaryTuning());
        setupDangerousUpperRig(rig);
        // Hold the target above its ready depth while warming the boundary history,
        // so invalid evidence is never injected after an already verified capture.
        rig.objects[0].box.y = 168;
        for (int i = 0; i < 12; ++i)
            upperStep(rig);
        assert(upperDangerClear(rig));
        rig.objects[1].box = {879, 8, 244, 173};
        if (fault == 0) rig.in.sent_vx_mps = .02f;
        if (fault == 1) rig.in.camera_pitch_stable = false;
        if (fault == 2) rig.objects[1].track_id = 3;
        if (fault == 3) rig.objects.resize(1);
        if (fault == 4) rig.in.sent_velocity_valid = false;
        if (fault == 6) rig.objects[1].timestamp_us -= 25000;
        if (fault == 7) rig.objects[1].timestamp_us -= 300000;
        if (fault == 8) rig.objects[1].box = {879, 0, 244, 720}; // entering bottom invisible
        if (fault == 9) rig.objects.push_back(rig.objects[1]); // nonunique identity
        if (fault == 10) rig.in.camera_pitch_cdeg = 2100; // calibrated, different view
        if (fault == 11) rig.objects[1].box = {10, 8, 244, 173}; // discontinuous position
        if (fault == 12) {
            rig.objects[0].track_id = rig.in.target_id = 42;
            rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 42;
        }
        if (fault == 13) rig.in.target_valid = false;
        if (fault == 14) {
            const auto returned = rig.objects[1];
            rig.objects.resize(1);
            upperStep(rig);
            assert(rig.in.capture_dangerous_side_pending);
            rig.objects.push_back(returned);
        }
        const bool fresh = fault != 5 && fault != 6 && fault != 7;
        for (int i = 0; i < 4; ++i) {
            upperStep(rig, fresh);
            const int current_danger_id = fault == 2 ? 3 : 2;
            assert(!upperDangerClear(rig, current_danger_id));
            if (fault == 2) {
                bool current_pending = false;
                for (const auto& object : rig.in.capture_image_objects)
                    if (object.id == 3 && object.label == "dangerous_object")
                        current_pending = object.dangerous_side_pending && !object.dangerous_side_clear;
                assert(current_pending);
            }
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
        }
    }
    std::cout << "PASS upper outside authorization requires current identity, frame, edge, stopped and same-view evidence\n";
}
void dangerousUpperReviewKeepsOtherRisksAndWholeBoxes() {
    SceneRig rig(false, upperBoundaryTuning());
    setupDangerousUpperRig(rig);
    for (int i = 0; i < 12; ++i)
        upperStep(rig);
    rig.objects[1].box = {879, 8, 244, 173};
    auto other = detection(3, {610, 210, 80, 80});
    other.label = "dangerous_object";
    rig.objects.push_back(other);
    upperStep(rig);
    assert(!rig.in.captureInventoryOverlaps(rig.objects[1].box, "dangerous_object"));
    assert(rig.in.captureInventoryOverlaps(other.box, "dangerous_object"));
    assert(rig.in.capturePathOverlaps(other.box, "dangerous_object"));
    assert(rig.in.corridor.dangerous == 1);
    assert(rig.out.capture_evidence != CaptureEvidence::Verified);
    for (const std::string label : {"ordinary_supply", "core_supply", "injured_person", "unknown"}) {
        SceneRig cargo(false, upperBoundaryTuning());
        setupDangerousUpperRig(cargo);
        cargo.objects[1].label = label;
        cargo.objects[1].box = {879, 8, 244, 173};
        upperStep(cargo);
        assert(cargo.in.captureInventoryOverlaps(cargo.objects[1].box, label));
        assert(!cargo.in.capture_dangerous_side_pending);
    }
    // A class-specific review region is suppressed by the same shared helper.
    // Outside the ordinary Y180 region is insufficient when that same box is
    // already inside another calibrated dangerous-object region starting at Y150.
    SceneRig class_region(false, upperBoundaryTuning());
    setupDangerousUpperRig(class_region);
    auto c = calibration();
    c.grasp_center_ranges[0].area = {342, 226, 1031, 489};
    c.grasp_center_ranges[0].lateral_measured = true;
    c.grasp_center_ranges.push_back(
        {"dangerous_object", 2000, 1280, 720, {342, 150, 1031, 489}, {647.5f, 337}, true});
    class_region.monitor = CaptureMonitor{c};
    for (int i = 0; i < 16; ++i) {
        upperStep(class_region);
        assert(class_region.in.captureInventoryOverlaps(class_region.objects[1].box, "dangerous_object"));
        assert(class_region.in.corridor.dangerous == 1);
        assert(!upperDangerClear(class_region));
        assert(class_region.out.capture_evidence != CaptureEvidence::Verified);
    }
    std::cout << "PASS upper jitter never hides another inventory/path hazard or changes other cargo whole-box rules\n";
}

// The 2026-11-04 01:11 captures saw two ordinary blocks and a dangerous
// box centred at Y175.5. The old Y180 split gate omitted that dangerous box.
void loggedDangerousFrontRisk(const char* fixture) {
    const auto root = std::filesystem::path(fixture).parent_path().parent_path().parent_path();
    std::vector<std::string> values{"danger-front-check", "--config", (root / "config/project.json").string(),
        "--phase", "capture", "--capture-after-first", "--hardware", "--imu",
        "--known-frame-angle", "0", "--known-frame-id", "22"};
    std::vector<char*> args;
    for (auto& value : values) args.push_back(value.data());
    std::ostringstream diagnostics;
    const auto cfg = resolveLegacyRuntimeConfig(
        rescue::app::loadAppConfig(int(args.size()), args.data()).config, diagnostics);
    for (bool first_delivered : {false, true})
        for (auto state : {PushState::LOWER_FRAME, PushState::VERIFY_CAPTURE}) {
            SceneRig rig;
            rig.monitor = CaptureMonitor{cfg.capture};
            rig.task.setSinglePhase("capture", first_delivered);
            rig.task.enter(state, rig.in.now_us, "logged_danger_front_risk");
            rig.task.frame_raised_ = state == PushState::VERIFY_CAPTURE ? 0 : 1;
            rig.objects = {detection(1, {743,210,305,335}),
                           detection(2, {389,276,315,305}),
                           detection(3, {394,38,264,275})};
            rig.objects.back().label = "dangerous_object";
            bool risk = false;
            for (int i = 0; i < 16; ++i) {
                rig.step();
                const auto& hazard = rig.objects.back();
                assert(rig.in.splitInventoryOverlaps(hazard.box, hazard.label));
                assert(rig.in.capturePathOverlaps(hazard.box, hazard.label));
                assert(rig.in.captureInventoryOverlaps(hazard.box, hazard.label));
                assert(rig.in.corridor.dangerous == 1);
                if (state == PushState::VERIFY_CAPTURE && i == 0)
                    assert(rig.in.held.dangerous == 1);
                assert(rig.out.capture_evidence != CaptureEvidence::Verified);
                assert(!rig.out.single_phase_done && rig.out.state != PushState::CARRY);
                risk |= rig.out.capture_scene_status == "risk";
                if (rig.task.search_phase_.first_trip_split_.pending) break;
            }
            assert(risk);
            if (state == PushState::LOWER_FRAME)
                assert(rig.task.frame_raised_ == 1 && rig.task.search_phase_.first_trip_split_.pending);
        }
    // Inclusive band edges, outside neighbours and the UI120 reset use the same monitor.
    for (const auto& probe : std::vector<std::pair<cv::Rect, bool>>{
             {{208,159,2,2}, true}, {{208,159,2,1}, false},
             {{208,175,2,1}, true}, {{207,175,3,1}, false},
             {{1070,175,2,1}, true}, {{1071,175,1,1}, false}}) {
        CaptureMonitor monitor(cfg.capture);
        PushObservation in;
        in.camera_pitch_cdeg = 2000;
        in.camera_pitch_stable = in.target_valid = in.gripper_closed_observed = true;
        in.target_id = 1; in.label = "ordinary_supply"; in.now_us = 1000000;
        auto target = detection(1, {541,210,213,254});
        auto hazard = detection(2, probe.first); hazard.label = "dangerous_object";
        target.timestamp_us = hazard.timestamp_us = in.now_us;
        monitor.update(in, {target, hazard}, in.now_us);
        assert(in.captureInventoryOverlaps(hazard.box, hazard.label) == probe.second);
        assert(in.capturePathOverlaps(hazard.box, hazard.label) == probe.second);
        assert(in.held.dangerous == int(probe.second));
        for (const auto& label : {"ordinary_supply", "core_supply", "injured_person"})
            assert(!in.captureInventoryOverlaps(hazard.box, label));
        in.camera_pitch_cdeg = 0;
        monitor.update(in, {target, hazard}, in.now_us);
        assert(in.capture_dangerous_front_region_px.empty());
    }
    // The after-first rule still accepts the same two ordinary blocks without danger.
    SceneRig legal(false, upperBoundaryTuning());
    legal.monitor = CaptureMonitor{cfg.capture};
    legal.task.setSinglePhase("capture", true);
    legal.task.enter(PushState::LOWER_FRAME, legal.in.now_us, "legal_two_ordinary");
    legal.objects = {detection(1, {743,210,305,335}), detection(2, {389,276,315,305})};
    waitForUpperClosure(legal);
    for (int i = 0; i < 80 && legal.out.capture_evidence != CaptureEvidence::Verified; ++i)
        upperStep(legal);
    if (legal.out.capture_evidence != CaptureEvidence::Verified)
        std::cerr << "legal pair state=" << int(legal.out.state) << " reason=" << legal.out.reason
                  << " held=" << legal.in.held.ordinary << " complete=" << legal.in.held_complete
                  << " captured=" << legal.in.captured << " scene=" << legal.out.capture_scene_status
                  << " detail=" << legal.out.capture_scene_detail << '\n';
    assert(legal.out.capture_evidence == CaptureEvidence::Verified);
    assert(legal.out.single_phase_done && legal.out.final_validation == "passed");
    std::cout << "PASS logged dangerous Y175.5 blocks pre/post-close before/after first; legal pair passes\n";
}

void suppliedGraspableAndOutsideSamples(const char* fixture) {
    const auto root = std::filesystem::path(fixture).parent_path().parent_path().parent_path();
    const auto project = (root / "config/project.json").string();
    std::vector<std::string> values{"sample-check", "--config", project, "--phase", "capture",
                                    "--hardware", "--imu", "--known-frame-angle", "0",
                                    "--known-frame-id", "22"};
    std::vector<char*> args;
    for (auto& value : values) args.push_back(value.data());
    std::ostringstream diagnostics;
    auto cfg = resolveLegacyRuntimeConfig(rescue::app::loadAppConfig(int(args.size()), args.data()).config, diagnostics);
    std::ifstream file(root / "config/gripper-image-reference.json");
    nlohmann::json reference;
    file >> reference;
    assert(reference.at("non_graspable_samples").size() == 8);
    // User-confirmed 18:59:02 in-frame ordinary target, mirrored about X640.
    // Exercise real config -> monitor membership -> split nomination -> selector acceptance.
    const std::vector<cv::Rect> retained_boxes{{925,280,292,311}, {935,340,272,291},
                                              {935,325,272,301}, {925,335,272,301}};
    for (bool legacy : {true, false})
        for (bool mirror : {false, true})
            for (const auto& retained : retained_boxes) {
                auto capture = cfg.capture;
                for (auto& range : capture.grasp_center_ranges)
                    if (range.label == "ordinary_supply") {
                        assert(range.area.x1 == 209 && range.area.x2 == 1071);
                        assert(range.area.y1 == 226 && range.area.y2 == 500);
                        assert(range.area.x1 + range.area.x2 == 1280);
                        assert(range.target_center_px == cv::Point2f(647.5f, 337));
                        if (legacy) {
                            range.area.x1 = 342;
                            range.area.x2 = 1031;
                        }
                    }
                SceneRig rig;
                rig.monitor = CaptureMonitor{capture};
                rig.managed_selection = true;
                auto& split = rig.task.search_phase_.first_trip_split_;
                split.pending = split.capture_retry = split.vertical_swing = true;
                split.runs = 1;
                split.started_us = split.recovery_view_us = rig.in.now_us;
                split.recovery_stage = SearchPhaseData::FirstTripSplit::RecoveryStage::Verify;
                split.target_reselecting = true;
                split.target_candidate_id = -1;
                rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
                rig.task.trip_context_.label_.clear();
                rig.in.target_valid = false;
                rig.in.target_id = -1;
                rig.in.label.clear();
                rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "symmetric_postturn_replay");
                rig.out.split_target_reselecting = true;
                rig.out.split_target_candidate_id = -1;
                rig.objects = {detection(16, retained), detection(17, {853,295,126,201}),
                               detection(15, {1151,100,129,251})};
                rig.objects[0].confidence = .896146f;
                rig.objects[1].label = "core_supply";
                rig.objects[2].label = "dangerous_object";
                if (mirror)
                    for (auto& object : rig.objects)
                        object.box.x = 1280 - object.box.br().x;
                bool nominated = false, accepted = false, outside = false;
                for (int i = 0; i < 32 && !accepted && !outside; ++i) {
                    rig.step();
                    nominated |= rig.out.split_target_candidate_id == 16;
                    accepted = rig.out.reason == "first_trip_split_target_accepted" &&
                               rig.out.target_id == 16 && rig.selection.locked;
                    outside |= rig.out.reason == "pixel_split_target_outside_inventory";
                    assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
                    assert(rig.out.capture_evidence == CaptureEvidence::None);
                    assert(rig.out.motion.camera_pitch_cdeg == 2000);
                }
                if (legacy) {
                    assert(outside && !nominated && !accepted);
                } else {
                    if (!accepted)
                        std::cerr << "symmetric split target not accepted: " << rig.out.reason << '\n';
                    assert(nominated && accepted && !outside);
                    assert(rig.in.captureInventoryOverlaps(rig.objects[0].box, "ordinary_supply"));
                    assert(split.runs == 1 && split.target_id == 16);
                }
            }
    std::cout << "PASS symmetric logged split + mirror: old region rejects; new region nominates and accepts\n";
    for (const auto& sample : reference.at("non_graspable_samples")) {
        const auto box = sample.at("bbox_xywh_px").get<std::vector<int>>();
        for (const auto& label : sample.at("applies_to")) {
            CaptureMonitor monitor(cfg.capture);
            PushObservation in;
            in.camera_pitch_cdeg = 2000;
            in.camera_pitch_stable = in.target_valid = in.sent_velocity_valid = true;
            in.target_id = 1; in.label = "ordinary_supply";
            in.gripper_closed_observed = true;
            auto target = detection(1, {541, 210, 213, 254});
            auto outside = detection(2, {box[0], box[1], box[2], box[3]});
            outside.label = label.get<std::string>();
            for (int i = 0; i < 12; ++i) {
                in.now_us = 1000000 + i * 25000;
                target.timestamp_us = outside.timestamp_us = in.now_us;
                monitor.update(in, {target, outside}, in.now_us);
            }
            if (in.captureInventoryOverlaps(outside.box, outside.label))
                std::cerr << "outside sample counted: " << sample.at("role")
                          << " label=" << outside.label << "\n";
            assert(!in.captureInventoryOverlaps(outside.box, outside.label));
            assert(!in.capturePathOverlaps(outside.box, outside.label));
            assert(in.held.ordinary == 1 && in.held.total() == 1);
            assert(in.corridor.ordinary == 1 && in.corridor.total() == 1);
            assert(in.held_complete && !in.capture_dangerous_side_pending);
            SceneRig rig(false, upperBoundaryTuning());
            rig.monitor = CaptureMonitor{cfg.capture};
            rig.objects = {detection(1, {541,235,213,264}), outside};
            waitForUpperClosure(rig);
            for (int i = 0; i < 60 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
                upperStep(rig);
                assert(rig.task.frame_raised_ == 0);
                assert(!rig.task.search_phase_.first_trip_split_.pending);
            }
            assert(rig.out.capture_evidence == CaptureEvidence::Verified);
            assert(rig.out.final_validation == "passed");
        }
    }
    // Every previously measured positive boundary still belongs to its own class.
    for (const auto& entry : reference.at("classes").items())
        for (const auto& sample : entry.value().at("samples")) {
            CaptureMonitor monitor(cfg.capture);
            const auto box = sample.at("bbox_xywh_px").get<std::vector<int>>();
            auto target = detection(1, {box[0], box[1], box[2], box[3]});
            target.label = entry.key(); target.timestamp_us = 1000000;
            PushObservation in;
            in.camera_pitch_cdeg = 2000;
            in.camera_pitch_stable = in.target_valid = true;
            in.target_id = 1; in.label = target.label; in.now_us = target.timestamp_us;
            monitor.update(in, {target}, in.now_us);
            assert(in.target_grasp_range_valid && in.target_grasp_range_inside);
            assert(in.captureInventoryOverlaps(target.box, target.label));
        }
    // Reproduce the latest stopped dangerous-box jitter, including deep bbox-corner
    // overlap with the old Y180 rectangle. The target is already aligned here;
    // this is a closure scenario, not a claim that the raw off-centre trace closed.
    {
        SceneRig rig(false, upperBoundaryTuning());
        rig.monitor = CaptureMonitor{cfg.capture};
        rig.objects = {detection(1, {541,235,213,264}), detection(13, {176,8,254,193})};
        rig.objects[1].label = "dangerous_object";
        const std::vector<cv::Rect> boxes{{176,8,254,193},{156,8,254,173},
            {161,0,244,191},{146,0,254,176},{171,8,244,193}};
        for (int i = 0; i < 220 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.objects[1].box = boxes[size_t(i) % boxes.size()];
            upperStep(rig);
            assert(!rig.task.search_phase_.first_trip_split_.pending);
            assert(rig.out.reason != "first_trip_split_limit");
        }
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.out.final_validation == "passed");
    }
    // Real in-region danger and unknowns remain blockers, not selectable cargo.
    for (const auto& label : {"dangerous_object", "unknown"}) {
        CaptureMonitor monitor(cfg.capture);
        auto target = detection(1, {541,210,213,254});
        auto hazard = detection(2, {750,300,80,80}); hazard.label = label;
        target.timestamp_us = hazard.timestamp_us = 1000000;
        PushObservation in;
        in.camera_pitch_cdeg = 2000; in.camera_pitch_stable = in.target_valid = true;
        in.target_id = 1; in.label = "ordinary_supply"; in.now_us = 1000000;
        monitor.update(in, {target, hazard}, in.now_us);
        assert(in.captureInventoryOverlaps(hazard.box, label));
        assert(in.capturePathOverlaps(hazard.box, label));
    }
    std::cout << "PASS 8 outside positions x 4 kinds, all measured positive boundaries and real hazards\n";
}

void runUpperBoundaryRegression(const std::string& name) {
    if (name == "upper-jitter") latestDangerousUpperJitterKeepsFirstClosure();
    else if (name == "upper-clipped") dangerousUpperClippedOutsideResumesPrecloseReview();
    else if (name == "upper-initial") dangerousUpperInitialShallowDoesNotGrantClosure();
    else if (name == "upper-risk") dangerousUpperDefiniteRiskCannotBeWashedByShallow();
    else if (name == "upper-invalid") dangerousUpperOutsideCannotSurviveInvalidEvidence();
    else if (name == "upper-regions") dangerousUpperReviewKeepsOtherRisksAndWholeBoxes();
    else assert(false && "unknown upper-boundary regression");
}

} // namespace
// Real monitor + selector: after a completed split, an empty view must search
// without translation, then lock a newly seen block before normal approach.
void postSplitMissingTargetSpinsBeforeApproach() {
    for (int scenario = 0; scenario < 3; ++scenario) {
        SceneRig rig;
        rig.enableSelector();
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "post_split_search_fixture");
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = true;
        split.runs = 1;
        split.started_us = split.recovery_view_us = rig.in.now_us;
        using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
        split.recovery_stage = scenario == 0 ? Stage::CornerTurnBack : Stage::Verify;
        split.upper_corner = scenario == 0;
        split.corner_heading = 0;
        if (scenario == 0) rig.in.camera_pitch_cdeg = 0;
        rig.in.heading_rad = scenario == 0 ? -.174532925f : 0;
        rig.objects.clear();
        if (scenario == 2) {
            auto danger = detection(12, {1100, 40, 60, 60});
            danger.label = "dangerous_object";
            rig.objects.push_back(danger);
        }
        bool resumed = false;
        for (int i = 0; i < 25 && !resumed; ++i) {
            rig.step();
            resumed = rig.out.state == PushState::APPROACH;
            assert(stopped(rig.out.motion));
        }
        assert(resumed);
        rig.in.camera_pitch_cdeg = 0;
        bool spun = false;
        // Search is independent of the 15-second approach deadline.
        for (int i = 0; i < 330; ++i) {
            rig.step();
            assert(!rig.task.fault_ && rig.out.motion.vx_mps == 0);
            assert(rig.out.motion.gripper_offset == 0);
            spun |= rig.out.motion.wz_rps != 0;
        }
        assert(spun);
        assert(rig.task.search_phase_.pixel_approach_started_us_ == 0);
        rig.in.camera_pitch_stable = false;
        rig.step();
        assert(stopped(rig.out.motion));
        rig.in.camera_pitch_stable = true;
        for (auto& object : rig.objects)
            object.timestamp_us = rig.in.now_us - rig.t.frame_timeout_us - 1;
        rig.step(false);
        assert(stopped(rig.out.motion));
        rig.in.directional_clearance_valid = true;
        rig.in.turn_safe = false;
        rig.step();
        assert(stopped(rig.out.motion));
        rig.in.turn_safe = rig.in.arc_safe = true;
        const auto discovery_started = split.search_started_us;
        // Logged 22:05:00: edge target #30 is briefly locked, then disappears.
        // The selector's grace must stop; after release the same split must search again.
        rig.objects = {detection(30, {40, 5, 111, 50})};
        bool flash_locked = false;
        for (int i = 0; i < 12 && !flash_locked; ++i) {
            rig.step();
            assert(rig.out.motion.vx_mps == 0 && rig.task.frame_raised_ == 1);
            flash_locked = rig.out.reason == "pixel_split_search_target_found";
        }
        assert(flash_locked && rig.selection.locked && rig.selection.id == 30);
        // Corner case: the lock disappears on the very next frame, before normal
        // approach can adopt it. The other entries reproduce the logged extra valid frame.
        if (scenario != 0)
            rig.step();
        rig.objects.clear();
        bool released = false, searched_again = false;
        for (int i = 0; i < 30 && !searched_again; ++i) {
            rig.step();
            released |= rig.selection.released_id == 30;
            assert(rig.out.motion.vx_mps == 0 && rig.task.frame_raised_ == 1);
            if (!released) assert(stopped(rig.out.motion));
            searched_again |= released && rig.out.reason == "pixel_split_search_spin" &&
                              rig.out.motion.wz_rps != 0;
        }
        assert(released && searched_again);
        assert(rig.task.trip_context_.target_id_ == -1);
        assert(rig.task.search_phase_.pixel_approach_started_us_ == 0);
        assert(split.search_started_us == discovery_started);
        // Longer than the real log's 10.8-second stall, with no manual target insertion.
        for (int i = 0; i < 225; ++i) {
            rig.step();
            assert(rig.out.reason == "pixel_split_search_spin");
            assert(rig.out.motion.vx_mps == 0 && rig.out.motion.wz_rps != 0);
            assert(!rig.task.fault_ && rig.task.frame_raised_ == 1);
        }
        rig.in.camera_pitch_stable = false;
        rig.step();
        assert(stopped(rig.out.motion));
        rig.in.camera_pitch_stable = true;
        rig.step(false);
        assert(stopped(rig.out.motion));
        rig.in.directional_clearance_valid = true;
        rig.in.turn_safe = false;
        rig.step();
        assert(stopped(rig.out.motion));
        rig.in.turn_safe = rig.in.arc_safe = true;
        rig.objects = {detection(26, {608, 200, 80, 80})};
        bool approached = false;
        for (int i = 0; i < 40 && !approached; ++i) {
            rig.step();
            if (rig.out.reason == "pixel_split_search_target_found")
                assert(stopped(rig.out.motion));
            if (rig.out.motion.vx_mps > 0) {
                assert(rig.in.target_valid && rig.in.pixel_locked && rig.out.target_id == 26);
                approached = true;
            }
        }
        assert(approached);
        assert(rig.task.search_phase_.pixel_approach_started_us_ != 0);
        assert(split.runs == 1);
        // A second loss after forward approach must also recover, without renewing the budget.
        rig.objects.clear();
        released = searched_again = false;
        for (int i = 0; i < 30 && !searched_again; ++i) {
            rig.step();
            released |= rig.selection.released_id == 26;
            assert(rig.out.motion.vx_mps == 0 && rig.task.frame_raised_ == 1);
            if (!released) assert(stopped(rig.out.motion));
            searched_again |= released && rig.out.reason == "pixel_split_search_spin" &&
                              rig.out.motion.wz_rps != 0;
        }
        assert(released && searched_again && split.runs == 1);
        assert(split.search_started_us == discovery_started);
        while (rig.in.now_us <= discovery_started + rig.t.pile_budget_us) {
            rig.step(); // keep fresh periodic control; a large gap is a separate stale-frame stop
            assert(rig.out.motion.vx_mps == 0 && rig.task.frame_raised_ == 1);
        }
        assert(!rig.task.fault_ && rig.out.state == PushState::APPROACH);
        assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
        assert(rig.out.reason == "pixel_split_search_restart");
        assert(split.search_started_us > discovery_started);
    }
    std::cout << "PASS post-split flash loss and approach loss resume bounded safe search\n";
}


// UI120 -> UI140 chooses wanted in-frame cargo, independently of old ID,
// class, image location, ground mapping and IMU heading.
void recognizedEdgeTargetReachesCapture() {
    SceneRig rig;
    auto cfg=calibration(); cfg.pixel_center_membership=true;
    rig.monitor=CaptureMonitor(cfg);
    rig.selector_config=&cfg;
    rig.managed_selection=true;
    rig.task.first_=true;
    rig.in.label=rig.task.trip_context_.label_="injured_person";
    rig.task.trip_context_.target_id_=rig.task.trip_context_.capture_target_id_=312;
    rig.task.enter(PushState::PREPARE,rig.in.now_us,"recognized_top_edge");
    auto& handoff=rig.task.capture_phase_.view_handoff_;
    handoff.near=true; handoff.started_us=rig.in.now_us;
    rig.objects={detection(312,{440,0,400,600})};
    rig.objects[0].label="injured_person";
    bool confirmed=false;
    for(int i=0;i<100 && rig.out.state!=PushState::CARRY;++i) {
        rig.step();
        confirmed |= rig.out.reason=="pixel_capture_handoff_ready";
        assert(rig.out.reason!="capture_view_retry_120");
        assert(rig.out.reason!="first_trip_scene_reobserve_turn");
        assert(rig.out.phase==TaskPhase::Capture || rig.out.phase==TaskPhase::Transport);
    }
    if(rig.out.state!=PushState::CARRY)
        std::cerr<<"edge target stalled: "<<rig.out.reason<<" scene="<<rig.task.capture_scene_.detail<<'\n';
    assert(confirmed && rig.out.state==PushState::CARRY);
    assert(rig.out.capture_evidence==CaptureEvidence::Verified);
    std::cout<<"PASS recognized top-clipped injury passes selection, near confirmation, positioning and verified capture\n";
}

void postSplitNearViewSelectsCurrentCargo() {
    for (int scenario = 0; scenario < 5; ++scenario) {
        SceneRig rig;
        auto cfg = calibration();
        cfg.pixel_center_membership = true;
        rig.selector_config = &cfg;
        rig.managed_selection = true;
        rig.task.first_ = (scenario == 2 || scenario == 4);
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "view_selection_fixture");
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
        rig.task.trip_context_.label_.clear();
        rig.task.capture_scene_ = {};
        rig.task.search_phase_.approach_pitch_ = rig.t.track_pitch_cdeg;
        rig.task.search_phase_.pixel_approach_started_us_ = 0;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.runs = 1;
        split.search_started_us = rig.in.now_us;
        const auto deadline_origin = split.search_started_us;
        rig.in.camera_pitch_cdeg = rig.t.track_pitch_cdeg;
        rig.objects = {detection(167, {493, 505, 242, 215})};
        if (scenario == 2 || scenario == 4) rig.objects[0].label = "core_supply";
        bool switched = false;
        for (int i = 0; i < 60 && !switched; ++i) {
            rig.step();
            switched = rig.out.phase == TaskPhase::Capture;
        }
        assert(switched && rig.out.target_id == 167);
        rig.in.camera_pitch_stable = false;
        rig.in.camera_pitch_cdeg = rig.t.near_pitch_cdeg;
        rig.step();
        rig.in.camera_pitch_stable = true;
        const int selected_id = scenario == 0 ? 167 : 175;
        auto wanted = detection(selected_id, {478, 177, 272, 313});
        if ((scenario == 2 || scenario == 4)) wanted.label = "core_supply";
        if (scenario >= 3) wanted.box = {478, 40, 272, 160};
        auto clipped = detection(176, {0, 132, 227, 243});
        auto danger = detection(177, {493, 1, 242, 223});
        danger.label = "dangerous_object";
        rig.objects = {wanted, clipped, danger};
        bool selected = false;
        for (int i = 0; i < 35 && !selected; ++i) {
            rig.step();
            assert(rig.out.motion.camera_pitch_cdeg == rig.t.near_pitch_cdeg);
            assert(rig.out.reason != "pixel_target_released");
            selected = rig.out.reason == "pixel_capture_handoff_ready";
        }
        assert(selected && rig.out.target_id == selected_id);
        assert(rig.selection.rebound_from == -1);
        assert(rig.task.trip_context_.label_ == wanted.label);
        assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
        assert(split.search_started_us == deadline_origin && split.runs == 1);
        assert(rig.task.capture_scene_.objects.empty());
        // The task acknowledges the new selection on the following observation.
        rig.step();
        assert(rig.selection.reselected_from == -1);
        assert(rig.out.motion.camera_pitch_cdeg == rig.t.near_pitch_cdeg);
    }
    std::cout << "PASS view change selects wanted in-frame cargo without ground identity matching\n";
}

// 2026-11-04 20:10:47: post-split target #173 disappears on UI120 -> UI140;
// only left-clipped #175 remains. Release must restore the bounded UI120 search.
void postSplitNearViewLossResumesSearch() {
    for (bool clipped : {false, true}) {
        SceneRig rig;
        auto cfg = calibration();
        cfg.pixel_center_membership = true;
        rig.selector_config = &cfg;
        rig.managed_selection = true;
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "near_view_loss_fixture");
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = -1;
        rig.task.trip_context_.label_.clear();
        rig.task.capture_scene_ = {};
        rig.task.search_phase_.approach_pitch_ = rig.t.track_pitch_cdeg;
        rig.task.search_phase_.pixel_approach_started_us_ = 0;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.runs = 2;
        split.search_started_us = rig.in.now_us;
        auto deadline_origin = split.search_started_us;
        rig.in.camera_pitch_cdeg = rig.t.track_pitch_cdeg;
        rig.objects = {detection(173, {600, 500, 80, 100})};
        bool switched = false;
        for (int i = 0; i < 60 && !switched; ++i) {
            rig.step();
            assert(!rig.task.fault_ && rig.out.motion.vx_mps == 0);
            switched = rig.out.phase == TaskPhase::Capture;
        }
        assert(switched && rig.out.target_id == 173);
        assert(rig.out.motion.camera_pitch_cdeg == rig.t.near_pitch_cdeg);
        rig.in.camera_pitch_cdeg = rig.t.near_pitch_cdeg;
        rig.objects.clear();
        if (clipped) {
            rig.objects = {detection(176, {312, 1, 222, 203})};
            rig.objects[0].label = "dangerous_object";
        }
        const auto approach_started = rig.task.search_phase_.pixel_approach_started_us_;
        bool retried = false;
        for (int i=0;i<40 && !retried;++i) {
            rig.step();
            assert(rig.out.phase == TaskPhase::Capture && stopped(rig.out.motion));
            retried = rig.out.reason == "capture_view_retry_120";
        }
        assert(retried && rig.out.target_id == 173);
        assert(rig.task.targetLabel() == "ordinary_supply");
        assert(rig.task.search_phase_.pixel_approach_started_us_ == approach_started);
        assert(split.search_started_us == deadline_origin && split.runs == 2);
        rig.in.camera_pitch_cdeg = rig.t.track_pitch_cdeg;
        rig.in.camera_pitch_stable = true;
        rig.objects.clear();
        for (int i=0;i<20;++i) {
            rig.step();
            assert(rig.out.phase == TaskPhase::Capture && stopped(rig.out.motion));
            assert(rig.out.motion.camera_pitch_cdeg == rig.t.track_pitch_cdeg);
        }
        // Re-associate the retained cargo in UI120. No unrelated scan or turn.
        rig.objects = {detection(179,{600,500,80,100})};
        bool approached=false;
        for(int i=0;i<35 && !approached;++i) {
            rig.step();
            assert(rig.out.phase==TaskPhase::Capture);
            assert(rig.out.motion.camera_pitch_cdeg==rig.t.track_pitch_cdeg);
            approached=rig.out.motion.vx_mps>0;
        }
        assert(approached && rig.out.target_id==179);
        assert(rig.task.search_phase_.pixel_approach_started_us_==approach_started);
        rig.objects[0].box.y=550; // Y600: new progress permits the next stopped view change
        for(int i=0;i<24 && rig.out.motion.camera_pitch_cdeg!=rig.t.near_pitch_cdeg;++i)
            rig.step();
        assert(rig.out.phase==TaskPhase::Capture);
        assert(rig.out.motion.camera_pitch_cdeg==rig.t.near_pitch_cdeg);
    }
    std::cout << "PASS clipped/missing UI140 target retries in Capture with retained class and deadline\n";
}

void independentSplitRegionGatesCapture(const char* fixture) {
    const auto root = std::filesystem::path(fixture).parent_path().parent_path().parent_path();
    std::vector<std::string> values{"split-gate-check", "--config", (root / "config/project.json").string(),
        "--phase", "capture", "--hardware", "--imu", "--known-frame-angle", "0", "--known-frame-id", "22"};
    std::vector<char*> args;
    for (auto& value : values) args.push_back(value.data());
    std::ostringstream diagnostics;
    const auto cfg = resolveLegacyRuntimeConfig(
        rescue::app::loadAppConfig(int(args.size()), args.data()).config, diagnostics);
    assert(cfg.capture.pixel_center_membership);
    // 22:26:47 log: #65 centre Y185.5 is in split Y180..489, outside jaw Y226..489.
    // Run post-close first: the old code accepts one held block while this neighbour remains.
    for (const auto state : {PushState::VERIFY_CAPTURE, PushState::RUSH, PushState::LOWER_FRAME}) {
        SceneRig rig;
        rig.monitor = CaptureMonitor{cfg.capture};
        rig.objects = {detection(62, {380, 285, 251, 261}), detection(65, {480, 100, 231, 171})};
        rig.in.target_id = 62;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 62;
        rig.task.enter(state, rig.in.now_us, "logged_independent_split_gate");
        rig.task.frame_raised_ = state == PushState::VERIFY_CAPTURE ? 0 : 1;
        bool reviewed = false;
        for (int i = 0; i < 16; ++i) {
            rig.step();
            // Inspect actual holding before failure recovery opens the frame again.
            if (state == PushState::VERIFY_CAPTURE && i == 0) assert(rig.in.held.ordinary == 1);
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
            assert(!rig.out.single_phase_done && rig.out.state != PushState::CARRY);
            reviewed |= rig.out.capture_scene_status == "risk";
            if (rig.task.search_phase_.first_trip_split_.pending) break;
        }
        assert(reviewed);
        assert(rig.in.split_ordinary_count == 2);
        assert(rig.in.ordinaryInventoryOverlaps(rig.objects[0].box));
        assert(!rig.in.ordinaryInventoryOverlaps(rig.objects[1].box));
        if (state != PushState::VERIFY_CAPTURE)
            assert(rig.task.frame_raised_ == 1 && rig.task.search_phase_.first_trip_split_.pending);
    }
    // Split and jaw boundaries remain independent, including exact front/rear edges.
    for (float y : {179.5f, 180.f, 185.5f, 204.f, 205.f, 225.5f, 226.f, 489.f, 489.5f}) {
        SceneRig edge;
        edge.monitor = CaptureMonitor{cfg.capture};
        edge.objects = {detection(1, {607,350,80,80}), detection(65, {600,int(2*y)-1,80,1})};
        edge.objects[1].box.y = int(y - .5f);
        edge.objects[1].box.height = int(2*(y-edge.objects[1].box.y));
        edge.step();
        const bool split_member = y >= 180 && y <= 489;
        const bool jaw_member = y >= 226 && y <= 489;
        assert(edge.in.split_ordinary_count == (split_member ? 2 : 1));
        assert(edge.in.captureInventoryOverlaps(edge.objects[1].box, "ordinary_supply") == jaw_member);
        assert(edge.out.capture_scene_status == (split_member ? "risk" : "clear"));
    }
    // In-progress separation must keep #65 latched until it leaves the split region.
    // A completed turn may not release this gate into PREPARE while the pair remains.
    for (bool cleared : {false, true}) {
        SceneRig after;
        after.monitor = CaptureMonitor{cfg.capture};
        after.objects = {detection(1, {607,350,80,80}), detection(65, {480,100,231,171})};
        after.task.enter(PushState::ABORT_DROP, after.in.now_us, "split_verify_gate");
        auto& split = after.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.vertical_swing = true;
        split.runs = 2;
        split.started_us = split.recovery_view_us = after.in.now_us;
        split.recovery_stage = SearchPhaseData::FirstTripSplit::RecoveryStage::Verify;
        after.step();
        assert(after.task.frame_raised_ == 1 && !after.out.single_phase_done);
        assert(after.out.capture_scene_status == "risk");
        assert(split.recovery_stage == SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat);
        assert(split.runs == 3 && split.retreat_objects.size() == 1);
        assert(split.retreat_objects[0].observation.id == 65 && !split.retreat_objects[0].cleared);
        if (!cleared) continue;
        after.objects[1].box.y = 70;
        split.recovery_stage = SearchPhaseData::FirstTripSplit::RecoveryStage::Verify;
        split.recovery_view_us = after.in.now_us;
        after.task.capture_scene_.objects.clear();
        after.in.sent_wz_rps = .1f;
        for (int i=0;i<8;++i) after.step();
        assert(split.pending && after.task.frame_raised_ == 1);
        after.in.sent_wz_rps = 0;
        after.step();
        for (int i=0;i<8;++i) after.step(false);
        assert(split.pending && after.task.frame_raised_ == 1);
        for (int i=0;i<20 && split.pending;++i) after.step();
        assert(!split.pending && after.out.state == PushState::PREPARE);
        assert(after.out.reason == "first_trip_split_separation_recheck");
        assert(stopped(after.out.motion) && after.task.frame_raised_ == 1);
        for (int i=0;i<100 && after.out.capture_evidence != CaptureEvidence::Verified;++i) after.step();
        assert(after.out.capture_evidence == CaptureEvidence::Verified);
    }
    // A neighbour outside the independent split range still allows the existing full flow.
    SceneRig clear(true);
    clear.monitor = CaptureMonitor{cfg.capture};
    clear.objects = {detection(1, {607, 350, 80, 80}), detection(65, {480, 70, 231, 171})};
    for (int i = 0; i < 100 && !clear.out.single_phase_done; ++i) clear.step();
    assert(clear.in.split_ordinary_count == 1);
    assert(clear.out.capture_evidence == CaptureEvidence::Verified && clear.out.single_phase_done);
    std::cout << "PASS independent split region blocks closure/success; held region stays separate\n";
}

void ordinarySplitTurnsWithoutExtraRetreat() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool timestamped : {false, true}) {
        SceneRig rig;
        assert(rig.task.t_.split_margin_budget_us == 0);
        rig.split();
        auto& split = rig.task.search_phase_.first_trip_split_;
        assert(!split.upper_corner && rig.out.motion.vx_mps < 0);
        rig.objects[1].box.y = 70;
        const auto cleared_us = rig.in.now_us;
        for (int i = 0; i < 16 && split.recovery_stage == Stage::Retreat; ++i) {
            rig.in.sent_velocity_us = timestamped ? rig.in.now_us : 0;
            rig.step();
            assert(stopped(rig.out.motion)); // no reverse request after visual clearance
            assert(split.margin_used_us == 0 && rig.out.split_margin_budget_us == 0);
            assert(rig.out.split_margin_stage != "advance");
            if (rig.in.now_us - cleared_us < 250000)
                assert(split.recovery_stage == Stage::Retreat);
        }
        assert(split.recovery_stage == Stage::Turn);
        assert(rig.out.reason == "first_trip_split_switch_ui120");
        assert(rig.in.now_us - cleared_us <= 400000);
        assert(rig.task.frame_raised_ == 1);
    }
    std::cout << "PASS default ordinary split: visual clearance + original stop -> turn, no extra reverse\n";
}

void ordinaryRetreatMarginWaitsForActualSend() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.task.t_.split_margin_budget_us = 500000; // explicitly test the optional mechanism
    rig.split();
    auto& split = rig.task.search_phase_.first_trip_split_;
    assert(!split.upper_corner);
    rig.objects[1].box.y = 70; // clear independent split/path regions, fully visible
    bool margin_requested = false;
    for (int i = 0; i < 20 && !margin_requested; ++i) {
        rig.in.sent_velocity_us = rig.in.now_us;
        rig.step();
        // Even after the original stopped fresh-frame window, ordinary splitting
        // must request its extra retreat before it can switch to the turn.
        assert(split.recovery_stage != Stage::Turn);
        margin_requested = rig.out.motion.vx_mps == -.01f;
    }
    assert(margin_requested);
    for (int i = 0; i < 15; ++i) {
        rig.in.sent_velocity_us = rig.in.now_us;
        rig.step(); // requested reverse but actual sent feedback remains zero
        assert(split.recovery_stage != Stage::Turn);
        assert(rig.out.motion.vx_mps == -.01f && rig.task.frame_raised_ == 1);
    }
    std::cout << "PASS ordinary retreat requires a bounded actually-sent margin before turn\n";
}

void startOrdinaryMargin(SceneRig& rig, const std::string& label = "ordinary_supply") {
    using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
    rig.task.t_.split_margin_budget_us = 500000; // production default has no extra retreat
    rig.objects[1].label = label;
    rig.split();
    auto& split = rig.task.search_phase_.first_trip_split_;
    assert(!split.upper_corner);
    rig.objects[1].box = {600, 70, 80, 80};
    for (int i = 0; i < 20 && split.margin_stage != Margin::Advancing; ++i) rig.sentStep();
    assert(split.margin_stage == Margin::Advancing && split.margin_used_us == 0);
    rig.sentStep(); // request first reverse, previous actual feedback is zero
    assert(rig.out.motion.vx_mps == -.01f);
    rig.sentStep(); // first actual reverse sample seeds continuity, no invented prior travel
    assert(split.margin_used_us == 0);
}
void ordinaryMarginBoundariesAndInterruptions() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
    for (int side : {-1, 1})
      for (const std::string label : {"ordinary_supply", "core_supply", "injured_person", "dangerous_object"}) {
        SceneRig rig;
        rig.objects[0].box.x = side < 0 ? 500 : 700;
        startOrdinaryMargin(rig, label);
        auto& split = rig.task.search_phase_.first_trip_split_;
        const auto begun = split.started_us; const int runs = split.runs;
        for (int i = 0; i < 9; ++i) rig.sentStep();
        rig.sentStep(49000);
        assert(split.margin_used_us == 499000 && split.margin_stage == Margin::Advancing);
        assert(rig.out.motion.vx_mps == -.01f && rig.task.frame_raised_ == 1);
        rig.sentStep(1000);
        assert(split.margin_used_us == 500000 && split.margin_stage == Margin::Brake);
        assert(stopped(rig.out.motion) && split.recovery_stage == Stage::Retreat);
        // Completion is not a turn permit: a second stopped new-frame window is required.
        for (int i = 0; i < 4; ++i) {rig.sentStep(); assert(split.recovery_stage == Stage::Retreat);}
        rig.sentStep();
        assert(split.recovery_stage == Stage::Turn && split.margin_stage == Margin::Complete);
        assert(split.started_us == begun && split.runs == runs);
        assert(rig.out.split_margin_used_us == 500000 && rig.out.split_margin_complete);
        rig.in.sent_vx_mps = 0; rig.turnCycle();
        rig.objects[1].box.y = 240; // turning can return the same danger, never close on it
        rig.step();
        assert(rig.task.frame_raised_ == 1 && rig.out.capture_evidence == CaptureEvidence::None);
    }
    for (int fault = 0; fault < 9; ++fault) {
        SceneRig rig; startOrdinaryMargin(rig);
        auto& split = rig.task.search_phase_.first_trip_split_;
        rig.sentStep(); rig.sentStep();
        const auto used = split.margin_used_us;
        if (fault == 7) rig.objects[0].timestamp_us = rig.objects[1].timestamp_us = split.margin_scene_us;
        for (int i = 0; i < 6; ++i) {
            rig.in.sent_vx_mps = -.01f; rig.in.sent_wz_rps = 0;
            rig.in.sent_velocity_us = rig.in.now_us;
            if (fault == 0) rig.in.sent_vx_mps = 0;
            if (fault == 1) rig.in.sent_velocity_valid = false;
            if (fault == 2) rig.in.retreat_safe = false;
            if (fault == 3) rig.in.camera_pitch_stable = false;
            if (fault == 4) rig.in.sent_velocity_us = rig.in.now_us - 150000;
            if (fault == 5) rig.in.sent_wz_rps = .02f;
            if (fault == 6) rig.in.sent_vx_mps = std::numeric_limits<float>::quiet_NaN();
            if (fault == 8) rig.in.heading_valid = false;
            rig.step(fault != 7);
            assert(split.margin_used_us == used && split.recovery_stage != Stage::Turn);
            if (fault != 0) assert(stopped(rig.out.motion));
        }
        rig.in.sent_velocity_valid = rig.in.retreat_safe = rig.in.camera_pitch_stable = rig.in.heading_valid = true;
        rig.in.sent_vx_mps = rig.in.sent_wz_rps = 0;
        rig.in.sent_velocity_us = rig.in.now_us;
        rig.step(); // explicitly establish stopped actual feedback, even after sent-zero fault
        // An unstable view can require the existing pitch/view preparation path
        // before reverse is requested again. That waiting time must stay excluded.
        for (int i = 0; i < 12 && rig.out.motion.vx_mps != -.01f; ++i) {
            rig.in.sent_velocity_us = rig.in.now_us;
            rig.step();
            assert(split.margin_used_us == used);
        }
        assert(rig.out.motion.vx_mps == -.01f);
        rig.sentStep(); // first fresh actual reverse sample starts continuity again
        assert(split.margin_used_us == used);
        rig.sentStep();
        assert(split.margin_used_us == used + 50000);
    }
    SceneRig gap; startOrdinaryMargin(gap);
    auto& split = gap.task.search_phase_.first_trip_split_;
    gap.sentStep(); const auto used = split.margin_used_us;
    gap.sentStep(150000);
    assert(stopped(gap.out.motion) && split.margin_used_us == used);
    gap.sentStep(); gap.sentStep();
    assert(split.margin_used_us == used);
    std::cout << "PASS margin 499/500ms, second brake, both directions/four classes, zero/invalid/stale/blocked/unstable/old/heading/gap\n";
}
void ordinaryMarginReturnKeepsConsumedBudget() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
    for (int return_kind = 0; return_kind < 5; ++return_kind) {
        SceneRig rig; startOrdinaryMargin(rig); rig.sentStep(); rig.sentStep();
        auto& split = rig.task.search_phase_.first_trip_split_;
        const auto used = split.margin_used_us;
        if (return_kind == 0) rig.objects[1].box.y = 220; // complete known return
        if (return_kind == 1) rig.objects[1].box = {0, 190, 650, 200}; // clipped known return
        if (return_kind == 2) {rig.objects[1].box.y = 220; rig.objects[1].track_id = 42; rig.objects[1].label = "dangerous_object";}
        if (return_kind == 3 || return_kind == 4) {
            auto object = detection(43, {600, 220, 80, 80});
            object.label = "dangerous_object";
            if (return_kind == 4) object.label = "unrecognized_obstacle";
            rig.objects.push_back(object);
        }
        rig.sentStep();
        assert(split.recovery_stage == Stage::Retreat && split.margin_stage == Margin::Idle);
        assert(split.margin_used_us == used && rig.task.frame_raised_ == 1);
        if (return_kind == 1 || return_kind == 4) {assert(stopped(rig.out.motion)); continue;}
        for (auto& object : rig.objects) if (object.track_id != 1) object.box = {600,70,80,80};
        for (int i = 0; i < 20 && split.margin_stage != Margin::Advancing; ++i) rig.sentStep();
        assert(split.margin_stage == Margin::Advancing && split.margin_used_us == used);
        rig.finishRetreat();
        assert(split.recovery_stage == Stage::Turn && split.margin_used_us == 500000);
    }
    for (bool locked : {false, true}) {
        SceneRig retained; startOrdinaryMargin(retained);
        retained.objects[0].track_id = 3;
        retained.in.target_id = locked ? 3 : -1; retained.in.target_valid = locked;
        retained.sentStep(); retained.sentStep();
        assert(retained.out.motion.vx_mps == -.01f);
        assert(retained.task.search_phase_.first_trip_split_.retreat_objects.size() == 1);
    }
    // Only a previously observed clear top exit may retain its existing clearance fact.
    SceneRig top; startOrdinaryMargin(top);
    top.objects[1].box = {600,0,80,100}; top.sentStep(); top.sentStep();
    assert(top.task.search_phase_.first_trip_split_.margin_used_us > 0);
    top.objects[1].box = {0,200,700,200}; top.sentStep();
    assert(stopped(top.out.motion));
    std::cout << "PASS margin return/re-ID/new/unknown/clipped evidence revokes turn without refilling budget; top exit retained\n";
}
void ordinaryMarginKeepsOnlyVisibleRawClearDanger() {
    using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
    for (int fault = 0; fault < 3; ++fault) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "core_supply";
        rig.objects[0].label = "core_supply";
        rig.objects[0].box = {608,400,80,80};
        startOrdinaryMargin(rig, "dangerous_object");
        auto& split = rig.task.search_phase_.first_trip_split_;
        // CaptureMonitor intentionally revokes its legacy upper outside evidence
        // during reverse. A visible raw-clear latched danger must not restart margin.
        assert(rig.in.capture_dangerous_side_pending);
        assert(!rig.in.capture_dangerous_pending_ids.empty());
        assert(rig.out.motion.vx_mps == -.01f && split.margin_stage == Margin::Advancing);
        const auto used = split.margin_used_us;
        if (fault == 0) {
            rig.sentStep(); assert(split.margin_used_us == used + 50000);
            rig.finishRetreat();
            assert(split.margin_used_us == 500000 && split.margin_stage == Margin::Complete);
        } else if (fault == 1) {
            rig.objects.pop_back(); rig.sentStep();
            assert(rig.in.capture_dangerous_side_pending && !rig.in.capture_dangerous_pending_ids.empty());
            assert(stopped(rig.out.motion) && split.margin_used_us == used);
        } else {
            rig.objects[1].box = {600,101,80,80}; // raw bottom181 touches split despite pending helper
            rig.sentStep();
            assert(rig.in.capture_dangerous_side_pending);
            assert(stopped(rig.out.motion) && split.margin_used_us == used);
        }
    }
    std::cout << "PASS actual core/danger reverse retains raw-clear identity only; missing and raw overlap block\n";
}

void completedOrdinaryMarginRequiresFreshSecondStop() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
    for (bool keep_pending : {false, true}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "core_supply";
        rig.objects[0].label = "core_supply"; rig.objects[0].box = {608,400,80,80};
        startOrdinaryMargin(rig, "dangerous_object");
        auto& split = rig.task.search_phase_.first_trip_split_;
        for (int i = 0; i < 10; ++i) rig.sentStep();
        assert(split.margin_used_us == 500000 && split.margin_stage == Margin::Brake);
        // A final actual reverse packet invalidates legacy dangerous outside evidence
        // and returns the ordinary recovery to Idle with the quota already consumed.
        rig.in.sent_vx_mps = -.01f; rig.in.sent_wz_rps = 0;
        rig.in.sent_velocity_us = rig.in.now_us; rig.step();
        assert(split.margin_stage == Margin::Idle && split.margin_used_us == 500000);
        assert(stopped(rig.out.motion) && rig.in.capture_dangerous_side_pending);
        if (keep_pending) rig.objects.pop_back(); // also cover reobserve-turn exit
        rig.in.sent_vx_mps = 0; rig.in.sent_velocity_us = rig.in.now_us;
        rig.step(); // one genuine zero write, then keep its UART timestamp frozen
        for (int i = 0; i < 20; ++i) {
            rig.step();
            assert(split.recovery_stage == Stage::Retreat && stopped(rig.out.motion));
            assert(split.margin_used_us == 500000 && rig.task.frame_raised_ == 1);
        }
        if (keep_pending) assert(rig.in.capture_dangerous_side_pending);
        for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i) {
            rig.in.sent_velocity_us = rig.in.now_us; rig.step();
        }
        assert(split.recovery_stage == Stage::Turn && split.margin_used_us == 500000);
    }
    std::cout << "PASS completed margin requires fresh second-stop UART on direct and pending reobserve exits\n";
}

void loggedRetreatDoesNotPulseBeforeClearance() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.objects[0].box.x = 430;
    rig.split();
    auto& split = rig.task.search_phase_.first_trip_split_;
    float previous_speed = .05f;
    bool slowed = false;
    // 2026-11-06 01:10:57: the old predictor stopped at Y424.5, then
    // restarted six short reverse bursts. Predictions must reduce speed,
    // never manufacture clearance or interrupt a clear reverse corridor.
    for (int y : {465, 465, 455, 445, 425, 425, 425, 425,
                  405, 385, 365, 345, 318, 318, 318, 298, 288, 278, 271}) {
        rig.objects[0].box.y = y - 40;
        rig.sentStep(33000);
        const float speed = -rig.out.motion.vx_mps;
        assert(split.recovery_stage == Stage::Retreat);
        assert(speed >= .01f && speed <= previous_speed + .000001f);
        assert(rig.out.motion.wz_rps == 0 && !rig.out.immediate_motion_stop);
        slowed |= speed < .049f;
        previous_speed = speed;
    }
    assert(slowed);
    rig.objects[0].box.y = 230; // Actual Y270, not a predicted crossing.
    rig.sentStep(33000);
    assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
    assert(split.recovery_stage == Stage::Retreat);
    for (int i = 0; i < 3; ++i) {
        rig.sentStep(33000);
        assert(stopped(rig.out.motion) && split.recovery_stage == Stage::Retreat);
    }
    rig.finishRetreat();
    assert(split.recovery_stage == Stage::Turn);
    std::cout << "PASS logged retreat: continuous deceleration, actual clearance and stopped confirmation\n";
}

void loggedLowerRetreatStopsWithoutOvershootCommands() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.objects[0].box.x = 430;
    rig.split();
    auto& split = rig.task.search_phase_.first_trip_split_;
    assert(!split.upper_corner && rig.task.t_.split_margin_budget_us == 0);
    assert(rig.out.motion.vx_mps == -.05f);
    float previous_speed = .05f;
    // Actual 21:37 centres: taper before clearance without inserting zero commands.
    for (int y : {465, 465, 475, 475, 474, 454, 414, 383, 368, 358, 328}) {
        rig.objects[0].box.y = y - 40;
        rig.sentStep(33000);
        const float speed = -rig.out.motion.vx_mps;
        assert(rig.out.motion.wz_rps == 0 && split.recovery_stage == Stage::Retreat);
        assert(speed >= .01f && speed <= previous_speed + .000001f);
        assert(!rig.out.immediate_motion_stop);
        previous_speed = speed;
    }
    assert(previous_speed < .05f);
    for (int y : {267, 250, 245}) {
        rig.objects[0].box.y = y - 40;
        rig.sentStep(33000);
        assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
        assert(split.recovery_stage == Stage::Retreat);
    }
    for (int i = 0; i < 24 && split.recovery_stage == Stage::Retreat; ++i) {
        rig.sentStep(33000);
        assert(stopped(rig.out.motion));
    }
    assert(split.recovery_stage == Stage::Turn && split.margin_used_us == 0);
    std::cout << "PASS logged retreat: continuous speed taper, stopped actual clearance required\n";
}

void retreatLagSimulationMatchesSlowStop() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    struct Result { float y, seconds; };
    for (float response_s : {.08f, .12f}) {
        const auto simulate = [&](bool corrected) {
            SceneRig rig;
            rig.objects[0].box.x = 430;
            rig.split();
            auto& split = rig.task.search_phase_.first_trip_split_;
            float y = 470, velocity = 0, command = corrected ? .05f : .01f;
            std::vector<float> commands(3, 0), images(2, y);
            constexpr float dt = .033f, pixel_gain = 14000.f;
            bool saw_full_speed = false;
            int stopped_frames = 0;
            for (int i = 0; i < 300; ++i) {
                const float delayed_command = commands.front();
                commands.erase(commands.begin());
                commands.push_back(command);
                velocity += (delayed_command - velocity) * dt / response_s;
                y -= velocity * dt * pixel_gain;
                const float observed_y = images.front();
                images.erase(images.begin());
                images.push_back(y);
                if (corrected) {
                    rig.objects[0].box.y = int(std::lround(observed_y)) - 40;
                    for (auto& object : rig.objects)
                        object.timestamp_us = rig.in.now_us + 33000 - 66000;
                    rig.in.sent_vx_mps = -command;
                    rig.in.sent_wz_rps = 0;
                    rig.in.sent_velocity_us = rig.in.now_us;
                    rig.step(false, true, 33000);
                    command = -rig.out.motion.vx_mps;
                    assert(command == 0 || (command >= .01f && command <= .05f));
                    // A clear corridor cannot alternate zero and reverse above Y270.
                    if (observed_y >= 270.5f)
                        assert(command > 0);
                    saw_full_speed |= command == .05f;
                    if (split.recovery_stage != Stage::Retreat) {
                        assert(split.recovery_stage == Stage::Turn && saw_full_speed);
                        return Result{y, (i + 1) * dt};
                    }
                } else {
                    command = observed_y > 270 ? .01f : 0;
                    stopped_frames = command == 0 && velocity < .0001f ? stopped_frames + 1 : 0;
                    if (stopped_frames >= 8)
                        return Result{y, (i + 1) * dt};
                }
            }
            assert(false && "retreat simulation did not finish");
            return Result{};
        };
        const auto slow = simulate(false), fast = simulate(true);
        std::cout << "RETREAT_SIM response_s=" << response_s
                  << " baseline_001_y=" << slow.y << " corrected_005_y=" << fast.y
                  << " baseline_s=" << slow.seconds << " corrected_s=" << fast.seconds << "\n";
        assert(fast.y <= 270 && fast.y >= slow.y - 12);
        assert(fast.seconds < slow.seconds);
    }
    std::cout << "PASS 0.05 m/s delayed camera/chassis simulations retain 0.01 stop precision\n";
}

void lowerCornerRetreatUsesWantedCentre() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (int x : {430, 780}) {
        SceneRig rig;
        rig.objects[0].box.x = x;
        rig.split();
        auto& split = rig.task.search_phase_.first_trip_split_;
        assert(!split.upper_corner);
        assert(split.retreat_objects.size() == 1);
        assert(split.retreat_objects[0].observation.id == 1);
        // Clearing the neighbour must not stop reversal while wanted Y > 270.
        rig.objects[1].box.y = 30;
        rig.step();
        assert(rig.out.motion.vx_mps < 0);
        rig.objects[0].box.y = 240; // centre 280 must still reverse
        rig.step();
        assert(rig.out.motion.vx_mps < 0);
        rig.objects[0].box.y = 231; // centre 271
        rig.step();
        assert(rig.out.motion.vx_mps < 0);
        // Neighbour still overlapping must not extend reversal at wanted Y == 270.
        rig.objects[1].box.y = 220;
        rig.objects[0].box.y = 230;
        rig.step();
        assert(stopped(rig.out.motion));
        for (int i = 0; i < 12 && split.recovery_stage == Stage::Retreat; ++i) {
            rig.sentStep();
            assert(stopped(rig.out.motion));
        }
        assert(split.recovery_stage == Stage::Turn);
        assert(split.margin_used_us == 0);
    }
    std::cout << "PASS lower left/right wanted Y280/Y271 reverse, Y270 stop despite neighbour overlap\n";
}

void postSplitRiskOutsideGraspRetries() {
    for (bool single : {false, true}) {
        SceneRig rig(single);
        rig.task.single_phase_initializing_ = false;
        rig.task.frame_raised_ = 1;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 1;
        rig.task.trip_context_.label_ = rig.in.label;
        rig.objects = {detection(1, {600, 180, 80, 80}),
                       detection(2, {650, 210, 80, 80})};
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "post_split_risk_fixture");
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = true;
        split.runs = 1;
        split.target_id = 1;
        split.started_us = split.recovery_view_us = rig.in.now_us;
        split.recovery_stage = SearchPhaseData::FirstTripSplit::RecoveryStage::Verify;
        for (int i = 0; i < 20 && split.runs == 1; ++i) {
            rig.step();
            assert(rig.out.capture_evidence != CaptureEvidence::Verified);
        }
        assert(!rig.in.target_grasp_range_inside);
        assert(split.runs == 2);
        assert(split.recovery_stage == SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat);
    }
    std::cout << "PASS post-split risk outside grasp Y retries separation\n";
}

void splitConfirmationIgnoresCaptureBoundaryJitter() {
    SceneRig r(true);
    auto c = calibration();
    c.pixel_center_membership = true;
    r.monitor = CaptureMonitor(c);
    auto danger = detection(3, {700, 380, 80, 80});
    danger.label = "dangerous_object";
    r.objects = {detection(1, {600, 350, 80, 80}),
                 detection(2, {480, 184, 80, 80}), danger};
    for (int i = 0; i < 9 && !r.task.search_phase_.first_trip_split_.pending; ++i) {
        r.objects[1].box.y = i % 2 ? 194 : 184; // centre Y224/234, split region unchanged
        r.step();
    }
    assert(r.task.search_phase_.first_trip_split_.pending);
    assert(r.out.capture_split_runs == 1);
}

void confirmationUsesInventoryForItsVerdict() {
    SceneRig r;
    r.task.first_ = true; // Later trips permit two ordinary supplies.
    auto c = calibration();
    c.pixel_center_membership = true;
    r.monitor = CaptureMonitor(c);
    r.objects = {detection(1, {600, 350, 80, 80}), detection(2, {480, 184, 80, 80})};
    const auto evaluate = [&]() {
        r.in.now_us += 50000;
        for (auto& d : r.objects) d.timestamp_us = r.in.now_us;
        r.monitor.update(r.in, r.objects, r.in.now_us, "ordinary_supply");
        return r.task.evaluateCaptureScene(r.in);
    };
    assert(evaluate() == PushTask::CaptureSceneVerdict::Clear);
    r.task.capture_scene_.confirmations = 7;
    r.objects[1].box.y = 194;
    assert(evaluate() == PushTask::CaptureSceneVerdict::Clear);
    assert(r.task.capture_scene_.confirmations == 0); // capture inventory changed
    auto danger = detection(3, {700, 380, 80, 80});
    danger.label = "dangerous_object";
    r.objects.push_back(danger);
    assert(evaluate() == PushTask::CaptureSceneVerdict::Risk);
    r.task.capture_scene_.confirmations = 7;
    r.objects[1].box.y = 184;
    assert(evaluate() == PushTask::CaptureSceneVerdict::Risk);
    assert(r.task.capture_scene_.confirmations == 7); // only capture membership changed
    r.objects[1].box.y = 154; // Keep movement within the same-object association gate.
    assert(evaluate() == PushTask::CaptureSceneVerdict::Risk);
    assert(r.task.capture_scene_.confirmations == 7);
    r.objects[1].box.y = 139; // centre Y179 leaves split inventory
    assert(evaluate() == PushTask::CaptureSceneVerdict::Risk);
    assert(r.task.capture_scene_.confirmations == 0);
}

void scoreLegalSubsets() {
    for (bool first : {false, true})
        for (int ordinary = 0; ordinary <= 4; ++ordinary)
            for (int core = 0; core <= 4; ++core)
                for (int injured = 0; injured <= 3; ++injured) {
                    const auto choice = chooseTrip(inv(ordinary, core, injured, 1, 1), first);
                    int best = 0;
                    bool tie_injured = false;
                    for (int o = 0; o <= ordinary; ++o)
                        for (int c = 0; c <= core; ++c)
                            for (int i = 0; i <= injured; ++i)
                                if (checkTrip(inv(o, c, i), first) == RuleVerdict::OK) {
                                    const int score = 5 * o + 10 * c + 15 * i;
                                    if (score > best) { best = score; tie_injured = i > 0; }
                                    else if (score == best && i) tie_injured = true;
                                }
                    assert(choice.score() == best);
                    assert((choice.inventory.injured > 0) == tie_injured);
                    assert(choice.inventory.dangerous == 0 && choice.inventory.unknown == 0);
                    assert(!choice.inventory.total() || checkTrip(choice.inventory, first) == RuleVerdict::OK);
                }
    assert(chooseTrip(inv(1, 1, 1), true).inventory == inv(0, 0, 1));
    assert(chooseTrip(inv(2, 1, 1), true).inventory == inv(0, 0, 1));
    assert(chooseTrip(inv(1, 2, 1), true).inventory == inv(0, 2));
    std::cout << "PASS exhaustive legal subsets, capacity two, tie favours injured\n";
}
void scoreScene(SceneRig& rig, bool two_core) {
    rig.task.first_ = true;
    rig.objects = {detection(1, {608, 400, 80, 80}),
                   detection(2, {760, 340, 80, 80}),
                   detection(3, {608, 330, 80, 80})};
    rig.objects[1].label = "core_supply";
    rig.objects[2].label = "injured_person";
    if (two_core) {
        auto core = detection(4, {820, 300, 80, 80});
        core.label = "core_supply";
        rig.objects.push_back(core);
    }
    rig.enableSelector();
}
void scoreSelectsAndCaptures() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool two_core : {false, true}) {
        SceneRig rig;
        scoreScene(rig, two_core);
        for (int i = 0; i < 100 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
            rig.step();

        }
        auto& plan = rig.task.capture_load_;
        auto& split = rig.task.search_phase_.first_trip_split_;
        if (!split.pending || !plan.active || rig.task.fault_)
            std::cerr << "score split stalled: two_core=" << two_core << " reason=" << rig.out.reason
                      << " scene=" << rig.out.capture_scene_detail << " active=" << plan.active
                      << " target=" << rig.out.target_id << " input_target=" << rig.in.target_id
                      << " nominate=" << split.target_candidate_id << " reselect=" << split.target_reselecting
                      << " state=" << PushTask::name(rig.out.state) << '\n';
        assert(split.pending && plan.active && !rig.task.fault_);
        assert(plan.choice.inventory == (two_core ? inv(0, 2) : inv(0, 0, 1)));
        assert(plan.choice.supply_score == (two_core ? 20 : 15));
        assert(plan.choice.injured_score == 15);
        assert(rig.task.trip_context_.label_ == (two_core ? "core_supply" : "injured_person"));
        // A completed physical split is supplied as fresh UI140 observations.
        split.recovery_stage = Stage::Verify;
        split.recovery_view_us = rig.in.now_us;
        split.target_reselecting = true;
        split.target_candidate_id = -1;
        for (auto& d : rig.objects)
            if (!rig.task.captureLoadAllows(d.label)) d.box.y = 30;
        for (auto& d : rig.objects)
            if (rig.task.captureLoadAllows(d.label)) {
                d.box.x = two_core && d.track_id == 4 ? 740 : 608;
                d.box.y = 310;
            }
        for (int i = 0; i < 180 && rig.out.capture_evidence != CaptureEvidence::Verified && !rig.task.fault_; ++i)
            rig.step();
        if (rig.out.capture_evidence != CaptureEvidence::Verified)
            std::cerr << "score capture stalled: " << rig.out.reason << " state=" << PushTask::name(rig.out.state) << '\n';
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.task.trip_context_.trip_ == (two_core ? inv(0, 2) : inv(0, 0, 1)));
        assert(!plan.active);
        rig.task.clearTrip();
        assert(!plan.active && !plan.choice.inventory.total());
    }
    std::cout << "PASS score choice survives real selector, split review, closure and capture\n";
}
void scorePostcloseReopens() {
    SceneRig rig;
    scoreScene(rig, false);
    rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "score_postclose");
    rig.task.frame_raised_ = 0;
    bool rejected = false;
    for (int i = 0; i < 100 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
        rig.step();
        rejected |= rig.out.state == PushState::ABORT_DROP;
        assert(rig.out.capture_evidence == CaptureEvidence::None && !rig.task.fault_);
    }
    assert(rejected && rig.task.frame_raised_ == 1);
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.task.capture_load_.choice.inventory == inv(0, 0, 1));
    std::cout << "PASS postclose mixed load scores, reopens, retargets injured and splits\n";
}
void scoreEvidenceAndReplan() {
    SceneRig rig;
    scoreScene(rig, true);
    // Directly exercise the stopped scene-review boundary with the real monitor.
    const auto review = [&](bool fresh = true) {
        rig.in.now_us += 50000;
        if (fresh) for (auto& d : rig.objects) d.timestamp_us = rig.in.now_us;
        rig.monitor.update(rig.in, rig.objects, rig.in.now_us, rig.task.trip_context_.label_);
        if (!fresh) rig.in.capture_scene_us = rig.objects.front().timestamp_us;
        TaskTick tick(rig.in);
        return rig.task.reviewCaptureLoad(tick);
    };
    for (int i = 0; i < 4; ++i) assert(!review());
    assert(!rig.task.capture_load_.active);
    for (int i = 0; i < 3; ++i) assert(!review(false));
    for (int i = 0; i < 12 && !rig.task.capture_load_.active; ++i) review();
    auto& plan = rig.task.capture_load_;
    assert(plan.active && plan.choice.inventory == inv(0, 2));
    const auto budget_start = plan.started_us;
    auto removed = rig.objects.back();
    rig.objects.pop_back();
    for (int i = 0; i < 10; ++i) {
        assert(!review());
        assert(plan.choice.inventory == inv(0, 2));
        assert(rig.task.reason_ == "capture_score_member_missing");
    }
    removed.track_id = 44; // Unique positional re-ID restores the same physical core.
    rig.objects.push_back(removed);
    for (int i = 0; i < 10; ++i) review();
    assert(plan.choice.inventory == inv(0, 2));
    rig.objects.back().box.y = 30; // Visibly exited, not merely missing.
    assert(!review());
    assert(plan.choice.inventory == inv(0, 2));
    for (int i = 0; i < 12; ++i) review();
    assert(plan.choice.inventory == inv(0, 0, 1));
    assert(plan.started_us == budget_start);
    rig.in.run = false;
    rig.task.update(rig.in);
    assert(plan.confirmations == 0);
    std::cout << "PASS stale frames, occlusion, positional re-ID, visible exit and fixed replan budget\n";
}
void scoreScopeAndSelector() {
    SceneRig rig;
    scoreScene(rig, true);
    rig.objects.back().box.y = 30; // A distant visible core cannot add ten points.
    for (int i = 0; i < 100 && !rig.task.search_phase_.first_trip_split_.pending; ++i) rig.step();
    assert(rig.task.capture_load_.choice.inventory == inv(0, 0, 1));
    assert(rig.task.capture_load_.choice.supply_score == 15);
    PixelSelector selector;
    selector.setCaptureLoad(inv(0, 0, 1));
    PixelSelection selected;
    for (int i = 0; i < 8; ++i) {
        const uint64_t now = 1000000 + i * 50000;
        for (auto& d : rig.objects) d.timestamp_us = now;
        selected = selector.update(rig.objects, {1280, 720}, 0, true, now, true, .25f);
    }
    assert(selected.locked && selected.id == 3); // UI120 also obeys the load group.
    selector.setCaptureLoad({});
    selector.reset();
    rig.objects[2].confidence = .1f; // below the capture selection threshold
    rig.task.capture_load_ = {};
    rig.in.now_us += 50000;
    for (auto& d : rig.objects) d.timestamp_us = rig.in.now_us;
    rig.monitor.update(rig.in, rig.objects, rig.in.now_us, "ordinary_supply");
    TaskTick tick(rig.in);
    assert(rig.task.reviewCaptureLoad(tick) && !rig.task.capture_load_.active);
    std::cout << "PASS scoring excludes outside/low-confidence cargo; UI120 respects chosen group\n";
}
void injuredFrontUsesUnwantedSplit() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool retry : {false, true}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "injured_person";
        auto c = calibration();
        c.pixel_center_membership = true;
        rig.monitor = CaptureMonitor(c);
        // Board log 20261104-034255: injured #9 centre (664.5,243),
        // inside its grasp range and above the longitudinal midpoint.
        rig.objects = {detection(1, {498, 26, 333, 434}),
                       detection(2, {825, 157, 323, 313})};
        rig.objects[0].label = "injured_person";
        rig.step();
        assert(rig.in.target_grasp_range_valid && rig.in.target_grasp_range_inside);
        assert(!rig.in.pixel_grasp_ready); // Entry into the frame is enough for this split policy.
        auto& split = rig.task.search_phase_.first_trip_split_;
        if (retry) {
            rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "injured_inside_retry_fixture");
            split.pending = split.capture_retry = true;
            split.runs = 1;
            split.target_id = 1;
            split.upper_corner = true; // Previous round's location must not decide the next round.
            split.started_us = split.recovery_view_us = rig.in.now_us;
            split.recovery_stage = Stage::Verify;
            rig.task.capture_load_.active = rig.task.capture_load_.keep_injured = true;
            rig.task.capture_load_.choice.inventory.injured = 1;
            rig.task.capture_load_.started_us = rig.in.now_us;
            for (int i = 0; i < 30 && split.runs == 1; ++i) rig.step();
            assert(split.runs == 2);
        } else {
            TaskTick tick(rig.in);
            rig.task.beginPixelSplit(tick);
        }
        assert(split.keep_injured && split.upper_corner);
        assert(split.recovery_stage == Stage::Retreat);
        assert(split.retreat_objects.size() == 1 &&
               split.retreat_objects.front().observation.id == 2);
        assert(split.corner_runs == 1);
        rig.step();
        assert(rig.out.motion.vx_mps < 0 && rig.out.motion.wz_rps == 0);
        assert(rig.out.capture_evidence != CaptureEvidence::Verified);
    }
    std::cout << "PASS logged front-half injured clears unwanted blocks on initial split and retry\n";
}

void injuredRemovalTurnOrder() {
    for (int centre_y : {358, 400})
    for (int centre_x : {450, 647, 830}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "injured_person";
        rig.objects[0].label = "injured_person";
        rig.objects[0].box = {centre_x - 40, centre_y - 40, 80, 80};
        rig.objects[1].box = {600, 300, 80, 80};
        rig.step();
        TaskTick tick(rig.in);
        rig.task.beginPixelSplit(tick);
        auto& split = rig.task.search_phase_.first_trip_split_;
        assert(!split.upper_corner);
        const int direction = centre_x > 659.5f ? 1 : -1;
        assert(split.recovery_direction == direction);
        rig.step();
        assert(rig.out.motion.vx_mps == 0 && rig.out.motion.camera_pitch_cdeg == 0);
        rig.in.camera_pitch_cdeg = 0;
        for (int i = 0; i < 12 && rig.out.motion.wz_rps == 0; ++i) rig.step();
        assert(rig.out.motion.vx_mps == 0 && rig.out.motion.wz_rps * direction > 0);
        // The turn must follow measured heading, not expire after 0.5 seconds.
        for (int i = 0; i < 12; ++i) {
            rig.step();
            assert(rig.out.motion.vx_mps == 0 && rig.out.motion.wz_rps * direction > 0);
        }
        rig.in.heading_rad = direction * .15f;
        rig.in.sent_wz_rps = direction * .1f;
        for (int i = 0; i < 8; ++i) { rig.step(); assert(stopped(rig.out.motion)); }
        rig.in.sent_wz_rps = 0;
        for (int i = 0; i < 12 && split.recovery_stage != SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat; ++i)
            rig.step();
        assert(split.recovery_stage == SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat);
        rig.in.camera_pitch_cdeg = 2000;
        rig.objects[0].track_id = 19; // Tracker restarts across the turn/view switch.
        for (int i = 0; i < 12 && rig.out.motion.vx_mps == 0; ++i) rig.step();
        assert(rig.out.motion.vx_mps < 0 && rig.out.motion.wz_rps == 0);
        rig.objects[0].box.y = 230; // Wanted centre Y270 ends this retreat segment.
        for (int i = 0; i < 16 && split.recovery_stage == SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat; ++i)
            rig.step();
        assert(split.recovery_stage == SearchPhaseData::FirstTripSplit::RecoveryStage::Verify);
        assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
        assert(rig.out.capture_evidence != CaptureEvidence::Verified);
    }
    for (int centre_x : {450, 647, 830}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "injured_person";
        rig.objects[0].label = "injured_person";
        rig.objects[0].box = {centre_x - 40, 180, 80, 80}; // Centre Y220 is outside the frame.
        rig.objects[1].box.y = 300;
        rig.step();
        TaskTick tick(rig.in);
        rig.task.beginPixelSplit(tick);
        assert(rig.task.search_phase_.first_trip_split_.upper_corner);
    }
    std::cout << "PASS lower-half injured turns then retreats; outside front keeps corner\n";
}
void scoredSplitRebuildsView() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    scoreScene(rig, false);
    for (int i = 0; i < 100 && !rig.task.search_phase_.first_trip_split_.pending; ++i) rig.step();
    auto& split = rig.task.search_phase_.first_trip_split_;
    auto& plan = rig.task.capture_load_;
    assert(plan.active && plan.choice.inventory == inv(0, 0, 1));
    const auto started = plan.started_us;
    split.recovery_stage = Stage::Turn;
    split.recovery_turn_us = rig.in.now_us - 500000;
    rig.in.camera_pitch_cdeg = 0;
    for (int i = 0; i < 12 && split.recovery_stage != Stage::Verify; ++i) rig.step();
    assert(split.recovery_stage == Stage::Verify);
    rig.in.camera_pitch_cdeg = 2000;
    // All IDs and image positions changed, just as in the failing board trace.
    rig.objects = {detection(18, {350, 260, 80, 80}), detection(19, {760, 340, 80, 80})};
    rig.objects[1].label = "injured_person";
    assert(plan.reobserve_after_us != 0);
    for (int i = 0; i < 8; ++i) {
        rig.step(false);
        assert(plan.reobserve_after_us != 0 && stopped(rig.out.motion));
        assert(rig.task.frame_raised_ == 1);
    }
    rig.in.camera_pitch_stable = false;
    for (int i = 0; i < 8; ++i) {
        rig.step();
        assert(plan.reobserve_after_us != 0 && stopped(rig.out.motion));
    }
    rig.in.camera_pitch_stable = true;
    bool retargeted = false;
    for (int i = 0; i < 100 && split.runs == 1 && !rig.task.fault_; ++i) {
        rig.step();
        assert(rig.out.reason != "capture_score_member_missing");
        assert(rig.out.capture_evidence != CaptureEvidence::Verified);
        retargeted |= rig.out.reason == "first_trip_split_target_accepted";
    }
    assert(!rig.task.fault_ && retargeted && split.runs == 2);
    assert(plan.choice.inventory == inv(0, 0, 1) && plan.started_us == started);
    // A visible exit may change current counts, never the committed injured group.
    rig.objects[1].box.y = 30;
    for (int i = 0; i < 12; ++i) {
        rig.in.now_us += 50000;
        for (auto& d : rig.objects) d.timestamp_us = rig.in.now_us;
        rig.monitor.update(rig.in, rig.objects, rig.in.now_us, "injured_person");
        TaskTick review(rig.in);
        rig.task.reviewCaptureLoad(review);
    }
    assert(plan.choice.inventory == inv(0, 0, 1) && !rig.task.captureLoadAllows("ordinary_supply"));
    rig.in.now_us = started + rig.t.pile_budget_us + 1;
    TaskTick expired(rig.in);
    assert(!rig.task.reviewCaptureLoad(expired));
    assert(!rig.task.fault_ && rig.task.reason_ == "capture_score_observation_wait");
    std::cout << "PASS scored post-turn new IDs rebuild view and retry unwanted cargo without extending budget\n";
}
void injuredSideCaptureLifecycle() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true}) {
        SceneRig rig(single);
        scoreScene(rig, false);
        auto& split = rig.task.search_phase_.first_trip_split_;
        for (int i = 0; i < 100 && !split.pending; ++i) rig.step();
        assert(split.keep_injured && split.recovery_stage == Stage::InjuredTurn);
        rig.in.camera_pitch_cdeg = 0;
        for (int i = 0; i < 12 && rig.out.motion.wz_rps == 0; ++i) rig.step();
        assert(rig.out.motion.wz_rps < 0 && rig.out.motion.vx_mps == 0);
        rig.in.heading_rad = -.15f;
        for (int i = 0; i < 16 && split.recovery_stage == Stage::InjuredTurn; ++i) rig.step();
        assert(split.recovery_stage == Stage::Retreat);
        rig.in.camera_pitch_cdeg = 2000;
        rig.objects[2].track_id = 13;
        for (int i = 0; i < 12 && rig.out.motion.vx_mps == 0; ++i) rig.step();
        assert(rig.out.motion.vx_mps < 0);
        // Simulated physical removal: unwanted cargo remains visible outside,
        // the retained injured person remains in its calibrated closing range.
        rig.objects[0].box.y = rig.objects[1].box.y = 30;
        rig.objects[2].box.y = 230;
        for (int i = 0; i < 180 && rig.out.capture_evidence != CaptureEvidence::Verified && !rig.task.fault_; ++i)
            rig.step();
        if (rig.out.capture_evidence != CaptureEvidence::Verified)
            std::cerr << "injured lifecycle stalled single=" << single << " reason=" << rig.out.reason << '\n';
        assert(!rig.task.fault_ && rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.task.trip_context_.trip_ == inv(0, 0, 1));
        assert(!rig.task.capture_load_.active);
    }
    std::cout << "PASS real selector + injured turn/retreat/new-ID review + close verifies injured alone in capture and full task\n";
}

void frontSplitLogAndBoundary() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    {
        SceneRig rig;
        auto c = calibration(); c.pixel_center_membership = true;
        rig.monitor = CaptureMonitor(c);
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "injured_person";
        // Exact 20:25:40 board scene: injured centre (634,253.5), two supplies below it.
        rig.objects = {detection(1, {432,157,404,193}),
                       detection(2, {412,328,223,233}), detection(3, {639,308,232,253})};
        rig.objects[0].label = "injured_person";
        rig.step();
        assert(rig.in.target_grasp_range_inside && rig.in.target_center_px.y == 253.5f);
        TaskTick tick(rig.in); rig.task.beginPixelSplit(tick);
        const auto& split = rig.task.search_phase_.first_trip_split_;
        assert(split.upper_corner && split.recovery_stage == Stage::Retreat);
        assert(split.retreat_objects.size() == 2);
        for (const auto& object : split.retreat_objects)
            assert(object.observation.id != 1 && object.observation.label == "ordinary_supply");
    }
    for (bool retry : {false, true})
    for (int y : {220, 253, 300, 357, 358, 400})
    for (int x : {450, 634, 830}) {
        SceneRig rig;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "injured_person";
        auto c = calibration(); c.pixel_center_membership = true;
        rig.monitor = CaptureMonitor(c);
        rig.objects = {detection(1, {x - 40, y - 40, 80, 80}),
                       detection(2, {600, 360, 80, 80})};
        rig.objects[0].label = "injured_person";
        rig.step();
        auto& split = rig.task.search_phase_.first_trip_split_;
        if (retry) {
            rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "front_retry_fixture");
            split.pending = split.capture_retry = true;
            split.runs = 1;
            split.target_id = 1;
            split.started_us = split.recovery_view_us = rig.in.now_us;
            split.recovery_stage = Stage::Verify;
            rig.task.capture_load_.active = rig.task.capture_load_.keep_injured = true;
            rig.task.capture_load_.choice.inventory.injured = 1;
            rig.task.capture_load_.started_us = rig.in.now_us;
            // The target itself must be in the review inventory on retry.
            if (y == 220) continue;
            for (int i = 0; i < 30 && split.runs == 1; ++i) rig.step();
            assert(split.runs == 2);
        } else {
            TaskTick tick(rig.in); rig.task.beginPixelSplit(tick);
        }
        assert(split.upper_corner == (y < 357.5f));
        assert(split.recovery_stage == (y < 357.5f ? Stage::Retreat : Stage::InjuredTurn));
        assert(split.retreat_objects.size() == 1);
        assert(split.retreat_objects.front().observation.id == (y < 357.5f ? 2 : 1));
    }
    std::cout << "PASS front-half injured uses unwanted-block split on entry and retry\n";
}

void injuredTurnLagRecovery() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (int direction : {-1, 1})
    for (float response : {.1f, .2f, .4f}) {
        SceneRig rig;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = split.keep_injured = true;
        split.started_us = rig.in.now_us;
        split.recovery_stage = Stage::InjuredTurn;
        split.recovery_direction = direction;
        split.injured_turn_goal = direction * .15f;
        rig.task.t_.split_wz_increment_rps = .2f;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "injured_lag_fixture");
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        float rate = 0, previous_nonzero = 0;
        uint64_t zero_since = 0;
        for (int i = 0; i < 500 && split.recovery_stage == Stage::InjuredTurn; ++i) {
            rate += response * (rig.out.motion.wz_rps - rate);
            rig.in.heading_rad += rate * .02f;
            rig.in.sent_wz_rps = rig.out.motion.wz_rps;
            rig.step(true, true, 20000);
            const float cmd = rig.out.motion.wz_rps;
            assert(std::abs(cmd) <= .15001f);
            assert(rig.out.motion.vx_mps == 0);
            if (cmd == 0) { if (!zero_since) zero_since = rig.in.now_us; }
            else {
                if (previous_nonzero * cmd < 0)
                    assert(zero_since && rig.in.now_us - zero_since >= 250000);
                previous_nonzero = cmd; zero_since = 0;
            }
        }
        assert(!rig.task.fault_ && split.recovery_stage == Stage::Retreat);
        assert(std::abs(wrapAngle(split.injured_turn_goal - rig.in.heading_rad)) <= .02f);
    }
    std::cout << "PASS injured turn tapers and finishes under body lag without immediate reversals\n";
}

void injuredTurnStopsBeforeCorrectionAndAfterPause() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    auto& split = rig.task.search_phase_.first_trip_split_;
    split.pending = split.capture_retry = split.keep_injured = true;
    split.started_us = rig.in.now_us;
    split.recovery_stage = Stage::InjuredTurn;
    split.injured_turn_goal = -.15f;
    rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "injured_brake_fixture");
    rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
    rig.step(); assert(rig.out.motion.wz_rps < 0);
    rig.in.heading_rad = -.20f; // actual body crossed the target while still turning
    rig.step(); assert(stopped(rig.out.motion) && split.corner_turn_braking);
    for (int i = 0; i < 10; ++i) {
        rig.in.heading_rad = -.20f + (i % 2 ? .006f : -.006f);
        rig.step(); assert(stopped(rig.out.motion) && split.recovery_stage == Stage::InjuredTurn);
    }
    rig.in.run = false; rig.step();
    assert(stopped(rig.out.motion));
    rig.in.run = true; rig.in.heading_rad = -.20f; rig.step();
    for (int i = 0; i < 4; ++i) { rig.step(); assert(stopped(rig.out.motion)); }
    rig.in.heading_valid = false; rig.step(); assert(stopped(rig.out.motion));
    rig.in.heading_valid = true;
    for (int i = 0; i < 5; ++i) { rig.step(); assert(stopped(rig.out.motion)); }
    for (int i = 0; i < 6 && stopped(rig.out.motion); ++i) rig.step();
    assert(rig.out.motion.wz_rps > 0 && rig.out.motion.wz_rps <= .15f);
    rig.in.heading_rad = -.15f; rig.in.sent_wz_rps = .1f;
    for (int i = 0; i < 8; ++i) { rig.step(); assert(stopped(rig.out.motion)); }
    assert(split.recovery_stage == Stage::InjuredTurn);
    rig.in.sent_wz_rps = 0;
    for (int i = 0; i < 12 && split.recovery_stage == Stage::InjuredTurn; ++i) rig.step();
    assert(split.recovery_stage == Stage::Retreat && !rig.task.fault_);
    std::cout << "PASS injured turn brakes on crossing and rebuilds stop evidence after pause/IMU loss\n";
}

void splitObservationReturnsToSearch() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (bool single : {false, true})
    for (bool initial : {false, true})
    for (bool near_deadline : {false, true}) {
        SceneRig rig(single);
        rig.task.frame_raised_ = 1; // The real initial capture entry confirms an open jaw.
        auto& split = rig.task.search_phase_.first_trip_split_;
        rig.task.first_ = true;
        rig.in.label = rig.task.trip_context_.label_ = "injured_person";
        rig.objects[0].label = "injured_person";
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "missing_reference_fixture");
        split.pending = split.capture_retry = split.upper_corner = split.keep_injured = true;
        if (initial) {
            split.pending = split.capture_retry = false;
            rig.task.enter(PushState::RUSH, rig.in.now_us, "initial_observation_fixture");
        }
        if (!initial) rig.task.single_phase_initializing_ = false;
        split.runs = split.corner_runs = 1;
        split.started_us = split.recovery_view_us = rig.in.now_us;
        split.recovery_stage = Stage::Retreat;
        rig.task.capture_load_.active = rig.task.capture_load_.keep_injured = true;
        rig.task.capture_load_.choice.inventory.injured = 1;
        rig.task.capture_load_.started_us = rig.in.now_us;
        if (near_deadline) {
            rig.in.now_us += rig.t.pile_budget_us - 250000;
            rig.task.last_us_ = rig.in.now_us;
            rig.task.actionProgress().started_us = rig.in.now_us;
        }
        // No valid near scene can establish clearance. Recovery must remain live.
        rig.objects.clear(); rig.in.target_valid = false;
        for (int i = 0; i < 120 && rig.task.state_ != PushState::APPROACH; ++i) {
            rig.in.camera_pitch_cdeg = rig.out.motion.camera_pitch_cdeg == kCameraPitchInvalid
                ? 2000 : rig.out.motion.camera_pitch_cdeg;
            rig.step();
            assert(rig.out.motion.vx_mps == 0);
        }
        assert(!rig.task.fault_ && !split.pending && rig.out.state == PushState::APPROACH);
        assert(rig.task.trip_context_.target_id_ == -1 && split.retreat_objects.empty());
        assert(rig.task.capture_phase_.view_handoff_.started_us == 0);
        assert(rig.task.capture_load_.choice.inventory == inv(0, 0, 1));
        assert(rig.task.capture_load_.objects.empty());
        rig.in.camera_pitch_cdeg = 0;
        rig.objects = {detection(19, {600, 350, 80, 80})};
        rig.objects[0].label = "injured_person";
        rig.in.target_id = 19; rig.in.target_valid = true;
        rig.in.label = "injured_person";
        for (int i = 0; i < 35 && !rig.task.search_phase_.pixel_approach_started_us_; ++i) rig.step();
        assert(!rig.task.fault_ && rig.task.search_phase_.pixel_approach_started_us_);
    }
    std::cout << "PASS missing split evidence resumes UI120 and reacquires injured in full/capture modes\n";
}

void injuredRemovalTests() {
    injuredFrontUsesUnwantedSplit();
    injuredRemovalTurnOrder();
    scoredSplitRebuildsView();
    injuredSideCaptureLifecycle();
}

void captureScoreTests() {
    scoreLegalSubsets();
    scoreSelectsAndCaptures();
    scorePostcloseReopens();
    scoreEvidenceAndReplan();
    scoreScopeAndSelector();
}

// Field 2026-11-05: NEXT briefly enters CAPTURE then rejects an unchanged #126.
// Exercise actual near-view search, the inspection pause, and capture handoff.
void searchCaptureCheckpointKeepsEntryOrder() {
    for (int interruption : {0, 1, 2, 3}) {
        SceneRig rig;
        auto config = calibration();
        config.grasp_center_ranges[0].lateral_measured = true;
        config.grasp_center_ranges[0].area.x1 = 209;
        config.grasp_center_ranges[0].area.x2 = 1071;
        rig.monitor = CaptureMonitor(config);
        rig.objects = {detection(1, {211, 288, 283, 293})};
        rig.task.setPhaseStepping(true);
        rig.task.enter(PushState::APPROACH, rig.in.now_us, "checkpoint_search");
        rig.task.search_phase_.approach_pitch_=rig.t.near_pitch_cdeg;
        for (int i = 0; i < 40 && !rig.out.phase_waiting; ++i)
            rig.step();
        std::cout << "checkpoint search: phase=" << int(rig.out.phase)
                  << " reason=" << rig.out.reason << std::endl;
        assert(rig.out.phase_waiting && rig.out.phase == TaskPhase::Search);
        assert(rig.out.next_phase == TaskPhase::Capture && !rig.out.entry_confirmed);
        const auto transaction = rig.out.motion.frame_transaction;
        rig.in.run = rig.in.safety_ok = false;
        // Fresh, stationary observations continue while the operator inspects the scene.
        for (int i = 0; i < 3; ++i) {
            rig.in.camera_pitch_stable = !(interruption == 1 && i == 1);
            if (interruption == 2 && i == 1)
                rig.objects.clear();
            else
                rig.objects = {detection(1, {211, 288, 283, 293})};
            rig.step(!(interruption == 3 && i == 1), true, 100000000);
            assert(rig.out.phase_waiting && stopped(rig.out.motion));
            assert(rig.out.motion.frame_transaction == transaction);
        }
        rig.in.run = rig.in.safety_ok = true;
        rig.step();
        assert(rig.out.phase == TaskPhase::Capture && !rig.out.phase_waiting);
        assert(stopped(rig.out.motion));
        rig.step();
        std::cout << "checkpoint NEXT: phase=" << int(rig.out.phase)
                  << " reason=" << rig.out.reason
                  << " first_id=" << rig.task.search_entry_.first_id << std::endl;
        assert(rig.out.phase == TaskPhase::Capture);
        assert(rig.out.reason != "entry_handoff_position_invalid");
        // Entry and inventory are established after NEXT with fresh observations.
        for (int i = 0; i < 40 && rig.task.frame_raised_ != 0; ++i)
            rig.step();
        assert(rig.out.phase == TaskPhase::Capture && rig.task.frame_raised_ == 0);
        assert(rig.task.search_entry_.first_id == 1 && rig.out.entry_confirmed);
    }
    std::cout << "PASS SEARCH checkpoint precedes Capture entry/inventory; NEXT rechecks fresh evidence after inspection\n";
}

void splitCapturePreemption() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    for (const auto stage : {Stage::CornerReturn, Stage::CornerTurnBack}) {
        SceneRig rig;
        rig.task.t_.split_capture_preempt = true;
        rig.task.t_.split_speed_increment_mps = rig.task.t_.split_wz_increment_rps = .2f;
        rig.objects.resize(1);
        rig.objects[0].box = {608, 350, 80, 80};
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = true;
        split.started_us = rig.in.now_us;
        split.recovery_stage = stage;
        split.runs = 2;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "preempt_fixture");
        // A confirmed UI120 candidate brakes before another return command.
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        rig.in.pixel_selection_managed = rig.in.pixel_locked = true;
        TaskTick tick(rig.in);
        tick.now = rig.in.now_us += 50000;
        for (auto& object : rig.objects) object.timestamp_us = rig.in.now_us;
        rig.monitor.update(rig.in, rig.objects, rig.in.now_us, rig.in.label);
        tick.forward_ok = tick.scan_ok = true;
        rig.task.updateRecoveryPhase(tick);
        assert(!split.pending && rig.task.state_ == PushState::APPROACH);
        assert(tick.immediate_motion_stop && stopped(tick.motion));
        assert(split.runs == 2 && rig.task.frame_raised_ == 1);
    }
    const auto seed = [](SceneRig& rig, Stage stage) {
        const TaskTuning defaults;
        assert(defaults.split_capture_preempt);
        assert(defaults.split_speed_increment_mps == .2f && defaults.split_wz_increment_rps == .2f);
        rig.task.t_.split_capture_preempt = defaults.split_capture_preempt;
        rig.task.t_.split_speed_increment_mps = defaults.split_speed_increment_mps;
        rig.task.t_.split_wz_increment_rps = defaults.split_wz_increment_rps;
        rig.task.t_.max_wz = .6f;
        rig.task.t_.approach_speed = .07f;
        rig.objects = {detection(1, {608, 350, 80, 80})};
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = true;
        split.started_us = rig.in.now_us;
        split.recovery_stage = stage;
        split.runs = 2;
        rig.task.enter(PushState::ABORT_DROP, rig.in.now_us, "production_split_fixture");
    };
    for (int invalid = 0; invalid < 8; ++invalid) {
        SceneRig rig;
        seed(rig, Stage::CornerReturn);
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        rig.in.pixel_selection_managed = rig.in.pixel_locked = true;
        rig.in.now_us += 50000;
        for (auto& object : rig.objects) object.timestamp_us = rig.in.now_us;
        if (invalid == 3)
            rig.objects[0].label = rig.in.label = "dangerous_object";
        if (invalid == 4) rig.objects[0].box.x = 0;
        rig.monitor.update(rig.in, rig.objects, rig.in.now_us, rig.in.label);
        if (invalid == 0) rig.in.capture_scene_us = rig.in.now_us - rig.t.frame_timeout_us - 1;
        if (invalid == 1) rig.in.camera_pitch_stable = false;
        if (invalid == 2) rig.in.target_valid = false;
        if (invalid == 5) rig.in.target_in_zone = true;
        if (invalid == 6) rig.task.rejected_[1] = rig.in.now_us + 1000000;
        if (invalid == 7) { rig.in.pixel_selection_managed = true; rig.in.pixel_locked = false; }
        TaskTick tick(rig.in);
        tick.forward_ok = tick.scan_ok = true;
        rig.task.updateRecoveryPhase(tick);
        assert(rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.task.frame_raised_ == 1);
    }
    // Exercise the full update/selector/monitor chain during UI120 turning and
    // push/return, then prove the normal near-view checks still lead to capture.
    for (const auto stage : {Stage::CornerReturn, Stage::CornerTurnBack}) {
        SceneRig rig;
        seed(rig, stage);
        rig.enableSelector();
        rig.objects = {detection(11, {608, 350, 80, 80})};
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        rig.in.sent_wz_rps = -.3f;
        for (int i = 0; i < 8 && rig.task.search_phase_.first_trip_split_.pending; ++i)
            rig.step();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(rig.out.reason == "pixel_split_target_interrupt");
        assert(rig.out.state == PushState::APPROACH && rig.out.target_id == 11);
        assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
        assert(rig.task.frame_raised_ == 1 && rig.out.capture_evidence == CaptureEvidence::None);
    }
    {
        SceneRig rig;
        seed(rig, Stage::CornerReturn);
        rig.enableSelector();
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        rig.in.sent_vx_mps = -.21f;
        for (int i = 0; i < 8 && rig.task.search_phase_.first_trip_split_.pending; ++i)
            rig.step();
        assert(rig.out.state == PushState::APPROACH && rig.out.immediate_motion_stop);
        assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
        rig.in.sent_vx_mps = 0;
        rig.objects[0].box = {600, 510, 80, 80}; // approach reaches the UI120 handoff
        for (int i = 0; i < 100 && rig.out.capture_evidence != CaptureEvidence::Verified; ++i) {
            rig.in.camera_pitch_cdeg = rig.out.motion.camera_pitch_cdeg;
            if (rig.in.camera_pitch_cdeg == 2000)
                rig.objects[0].box = {608, 350, 80, 80};
            rig.step();
        }
        assert(rig.out.capture_evidence == CaptureEvidence::Verified);
        assert(rig.out.state == PushState::CARRY);
    }
    // Forward split stages keep running even with the same confirmed UI120 cargo.
    for (const auto stage : {Stage::Retreat, Stage::InjuredTurn, Stage::Turn,
                            Stage::CornerTurnRight, Stage::CornerPush}) {
        SceneRig rig;
        seed(rig, stage);
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        rig.in.pixel_selection_managed = rig.in.pixel_locked = true;
        rig.in.now_us += 50000;
        for (auto& object : rig.objects) object.timestamp_us = rig.in.now_us;
        rig.monitor.update(rig.in, rig.objects, rig.in.now_us, rig.in.label);
        TaskTick tick(rig.in);
        tick.forward_ok = tick.scan_ok = true;
        rig.task.updateRecoveryPhase(tick);
        assert(rig.task.search_phase_.first_trip_split_.pending);
    }
    // Motion-only cases have no selected cargo: verify the exact requested gains.
    for (const auto stage : {Stage::Turn, Stage::InjuredTurn, Stage::CornerTurnRight,
                            Stage::CornerPush, Stage::CornerReturn, Stage::CornerTurnBack}) {
        SceneRig rig;
        seed(rig, stage);
        rig.in.target_valid = false;
        rig.in.camera_pitch_cdeg = rig.task.pitch_cmd_ = 0;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.corner_heading = 0;
        split.corner_command_origin_m = stage == Stage::CornerReturn ? -.036f : 0;
        split.injured_turn_goal = -.3f;
        rig.in.heading_rad = stage == Stage::CornerPush || stage == Stage::CornerReturn ? -float(CV_PI)/2 : 0;
        rig.step();
        if (stage == Stage::CornerPush || stage == Stage::CornerReturn)
            assert(std::abs(std::abs(rig.out.motion.vx_mps) - .27f) < 1e-6f);
        else {
            const float expected = stage == Stage::InjuredTurn ? .15f :
                stage == Stage::CornerTurnBack ? 2.f * .174532925f : .5f;
            assert(std::abs(std::abs(rig.out.motion.wz_rps) - expected) < 1e-6f);
        }
    }
    {
        SceneRig rig;
        seed(rig, Stage::Retreat);
        rig.task.t_.split_capture_preempt = false;
        rig.objects[0].timestamp_us = rig.in.now_us;
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.retreat_objects.push_back({{1, "ordinary_supply", rig.objects[0].box, true}, false});
        rig.step();
        assert(std::abs(rig.out.motion.vx_mps + .05f) < 1e-6f);
    }
    std::cout << "PASS return-only preemption, invalid evidence, capture lifecycle and +0.2 motion gains\n";
}


void startupFirstGrabFromLoggedPair() {
    for (int scenario = 0; scenario < 7; ++scenario) {
        auto t = tuning();
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        if (scenario == 5) t.verify_budget_us = 1000000;
        SceneRig rig(false, t);
        auto c = calibration();
        c.pixel_center_membership = true;
        c.grasp_center_ranges[0].area = {209, 226, 1071, 500};
        c.grasp_center_ranges[0].lateral_measured = true;
        rig.monitor = CaptureMonitor(c);
        rig.task.enter(PushState::START_ADVANCE, rig.in.now_us, "logged_startup_first_grab");
        rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
        rig.task.startup_pitch_ = t.near_pitch_cdeg;
        rig.task.startup_handoff_.pending = true;
        rig.task.startup_handoff_.started_us = rig.in.now_us;
        rig.task.startup_handoff_.candidate_id = 178; // the logged pending selection
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 178;
        rig.in.target_id = 178;
        rig.objects = {detection(178, {465, 69, 184, 254}),
                       detection(179, {643, 89, 183, 234})};
        for (int i = 0; i < 3; ++i) rig.step();
        assert(rig.out.phase == TaskPhase::Startup);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        // Replay the two observation-region objects; no split before approaching.
        bool corrected = false;
        for (int i = 0; i < 8; ++i) {
            rig.step();
            corrected |= rig.out.motion.wz_rps != 0;
            assert(rig.out.phase == TaskPhase::Startup && rig.task.frame_raised_ == 1);
            assert(!rig.task.search_phase_.first_trip_split_.pending);
        }
        assert(corrected);
        rig.objects[0].box.x = 556; // selected centre X648, aligned
        bool advanced = false;
        for (int i = 0; i < 24 && !advanced; ++i) {
            rig.step();
            advanced |= rig.out.motion.vx_mps > 0;
            assert(rig.task.frame_raised_ == 1 && rig.out.phase == TaskPhase::Startup);
        }
        assert(advanced);
        if (scenario == 2) { // A revoked path still stops startup motion.
            rig.in.path_safe = false;
            rig.step();
            assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            continue;
        }
        if (scenario == 3) { // An old target frame cannot grant closure.
            for (int i = 0; i < 8; ++i) rig.step(false);
            assert(stopped(rig.out.motion) && rig.task.frame_raised_ == 1);
            continue;
        }
        if (scenario == 6) {
            rig.in.run = false;
            rig.step();
            assert(stopped(rig.out.motion));
            rig.in.run = true;
            rig.step();
            assert(rig.out.phase == TaskPhase::Startup && rig.task.startup_first_capture_);
        }
        // Preserve the frame-to-frame motion of a physical approach, rather than
        // teleporting the selected centre across the association gate.
        for (int y : {109, 149, 189}) {
            rig.objects[0].box.y = y;
            rig.step();
        }
        rig.objects[0].box.y = 210; // centre Y337, calibrated close line
        if (scenario == 1) rig.objects[1].box = {800, 210, 80, 80}; // actually held second ordinary
        if (scenario == 4) {
            auto danger = detection(180, {800, 210, 80, 80});
            danger.label = "dangerous_object";
            rig.objects.push_back(danger);
        }
        bool closed = false, rejected = false;
        for (int i = 0; i < 100 && !rejected && rig.out.state != PushState::CARRY; ++i) {
            rig.step();
            closed |= rig.task.frame_raised_ == 0;
            if (scenario == 5 && rig.out.state == PushState::VERIFY_CAPTURE) {
                rig.objects.clear();
                rig.in.target_valid = false;
            }
            rejected |= rig.out.state == PushState::ABORT_DROP || rig.out.state == PushState::CAPTURE_FAIL;
            if (scenario == 4 && rig.task.search_phase_.first_trip_split_.pending) break;
        }
        if (scenario == 4) {
            assert(!closed && rig.task.search_phase_.first_trip_split_.pending);
        } else if (scenario == 1 || scenario == 5) {
            if (scenario == 1) assert(rig.in.held.ordinary == 2);
            assert(closed && rejected && rig.out.final_validation == "failed");
            assert(rig.out.capture_evidence == CaptureEvidence::None);
            for (int i = 0; i < 8 && rig.task.frame_raised_ == 0; ++i) rig.step();
            assert(rig.task.frame_raised_ == 1);
            assert(!rig.task.startup_first_capture_);
            if (scenario == 1) {
                for (int i = 0; i < 25 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
                    rig.step();
                assert(rig.task.search_phase_.first_trip_split_.pending);
            }
        } else {
            assert(closed && !rejected && rig.out.state == PushState::CARRY);
            assert(rig.out.capture_evidence == CaptureEvidence::Verified);
            assert(rig.out.final_validation == "passed" && rig.task.trip_context_.trip_.ordinary == 1);
            assert(rig.in.held.ordinary == 1 && rig.in.split_ordinary_count == 2);
        }
    }
    std::cout << "PASS startup logged pair approaches/closes before actual held-count verification; path, freshness and danger retained\n";
}

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[2]) == "postclose-held-only") {
        postcloseUsesOnlyHeldCargo();
        postcloseOutsideBoxesDoNotBlockHeldEvidence();
        postcloseRequiresFreshHeldImages();
        postcloseRejectsTwoWithoutSuccessOrMotion();
        postcloseRejectsLatestLogBoxes();
        mixedPostcloseReturnsToReview();
        return 0;
    }
    if (argc == 3 && std::string(argv[2]) == "startup-first-grab") {
        startupFirstGrabFromLoggedPair(); return 0;
    }
    assert(argc == 2 || argc == 3);
    if (argc == 3) {
        if (std::string(argv[2]) == "front-split-recovery") { frontSplitLogAndBoundary(); injuredTurnLagRecovery(); injuredTurnStopsBeforeCorrectionAndAfterPause(); splitObservationReturnsToSearch(); return 0; }
        if (std::string(argv[2]) == "split-preempt") { splitCapturePreemption(); return 0; }
        if (std::string(argv[2]) == "postsplit-approach") { postSplitClearOutsideGraspResumesApproach(); return 0; }
        if (std::string(argv[2]) == "capture-checkpoint") { searchCaptureCheckpointKeepsEntryOrder(); return 0; }
        if (std::string(argv[2]) == "injured-removal") { injuredRemovalTests(); return 0; }
        if (std::string(argv[2]) == "search-view-selection") { recognizedEdgeTargetReachesCapture(); postSplitNearViewSelectsCurrentCargo(); return 0; }
        if (std::string(argv[2]) == "postsplit-search") { postSplitNearViewSelectsCurrentCargo(); postSplitMissingTargetSpinsBeforeApproach(); postSplitNearViewLossResumesSearch(); return 0; }
        if (std::string(argv[2]) == "capture-score") { captureScoreTests(); return 0; }
        if (std::string(argv[2]) == "independent-confirmation") { splitConfirmationIgnoresCaptureBoundaryJitter(); confirmationUsesInventoryForItsVerdict(); return 0; }
        if (std::string(argv[2]) == "split-attempts") { splitActionBudgetsStayBounded(); postSplitRiskOutsideGraspRetries(); approachOutsideInventoryAndNextTrip(); return 0; }
        if (std::string(argv[2]) == "postsplit-risk-outside") { postSplitRiskOutsideGraspRetries(); return 0; }
        if (std::string(argv[2]) == "corner-turn-brake") { cornerTurnBrakesBeforePush(); latestCornerReturnWaitsForScene(); return 0; }
        if (std::string(argv[2]) == "corner-imu-drift") { cornerReturnBalancesCommandsAndResumesSearch(); return 0; }
        if (std::string(argv[2]) == "upper-corner-motion") { upperCornerPushAndReturn(); return 0; }
        if (std::string(argv[2]) == "retreat-continuity") { loggedRetreatDoesNotPulseBeforeClearance(); return 0; }
        if (std::string(argv[2]) == "retreat-overshoot") { loggedLowerRetreatStopsWithoutOvershootCommands(); retreatLagSimulationMatchesSlowStop(); return 0; }
        if (std::string(argv[2]) == "lower-target-y270") { lowerCornerRetreatUsesWantedCentre(); return 0; }
        if (std::string(argv[2]) == "three-missing-frames") { threeMissingFramesBeforeReobserve(); return 0; }
        if (std::string(argv[2]) == "no-extra-retreat") { ordinarySplitTurnsWithoutExtraRetreat(); return 0; }
        if (std::string(argv[2]) == "top-clipped-postturn") { topClippedPostTurnTargetReacquires(); latestOutsideInventoryTargetReacquires(); return 0; }
        if (std::string(argv[2]) == "ordinary-retreat-margin") { ordinaryRetreatMarginWaitsForActualSend(); ordinaryMarginBoundariesAndInterruptions(); ordinaryMarginReturnKeepsConsumedBudget(); ordinaryMarginKeepsOnlyVisibleRawClearDanger(); completedOrdinaryMarginRequiresFreshSecondStop(); return 0; }
        if (std::string(argv[2]) == "independent-split-region") {
            independentSplitRegionGatesCapture(argv[1]);
            return 0;
        }
        if (std::string(argv[2]) == "dangerous-front-risk") {
            loggedDangerousFrontRisk(argv[1]);
            return 0;
        }
        if (std::string(argv[2]) == "supplied-grasp-ranges") {
            suppliedGraspableAndOutsideSamples(argv[1]);
            return 0;
        }
        runUpperBoundaryRegression(argv[2]);
        return 0;
    }
    startupFirstGrabFromLoggedPair();
    loggedRetreatDoesNotPulseBeforeClearance();
    loggedLowerRetreatStopsWithoutOvershootCommands();
    retreatLagSimulationMatchesSlowStop();
    splitCapturePreemption();
    searchCaptureCheckpointKeepsEntryOrder();
    injuredRemovalTests();
    captureScoreTests();
    splitConfirmationIgnoresCaptureBoundaryJitter();
    confirmationUsesInventoryForItsVerdict();
    postSplitRiskOutsideGraspRetries();
    threeMissingFramesBeforeReobserve();
    ordinarySplitTurnsWithoutExtraRetreat();
    ordinaryRetreatMarginWaitsForActualSend();
    ordinaryMarginBoundariesAndInterruptions();
    ordinaryMarginReturnKeepsConsumedBudget();
    ordinaryMarginKeepsOnlyVisibleRawClearDanger();
    completedOrdinaryMarginRequiresFreshSecondStop();
    independentSplitRegionGatesCapture(argv[1]);
    postSplitMissingTargetSpinsBeforeApproach();
    recognizedEdgeTargetReachesCapture();
    postSplitNearViewSelectsCurrentCargo();
    postSplitNearViewLossResumesSearch();
    suppliedGraspableAndOutsideSamples(argv[1]);
    latestDangerousUpperJitterKeepsFirstClosure();
    dangerousUpperClippedOutsideResumesPrecloseReview();
    dangerousUpperInitialShallowDoesNotGrantClosure();
    dangerousUpperDefiniteRiskCannotBeWashedByShallow();
    dangerousUpperOutsideCannotSurviveInvalidEvidence();
    dangerousUpperReviewKeepsOtherRisksAndWholeBoxes();
    latestDangerousSideJitterKeepsFirstClosure();
    dangerousSidePendingDoesNotGrantClosureOrSuccess();
    dangerousSideClearanceRejectsInvalidEvidence();
    dangerousDefiniteRiskIsNotErasedByShallowOverlap();
    dangerousSideHistoryEndsAfterConfirmedClearance();
    dangerousWholeBoxCanClearVertically();
    splitRetreatClearanceUsesWholeReviewRegions();
    latestSplitRetreatDoesNotSpendRetriesOnOverlappingBoxes();
    latestUi120HandoffJitterCompletesCapture();
    ui120HandoffInvalidEvidenceClearsIntent();
    ui120ProgressAndUnusableAlternativesKeepLock();
    latestCornerReturnWaitsForScene();
    latestIntermittentTargetReselects();
    topClippedPostTurnTargetReacquires();
    latestOutsideInventoryTargetReacquires();
    unwantedReobserveRequiresStoppedFreshFrames();
    latestMissingUnwantedReobserves();
    emptySplitReobservesWithoutRetreat();
    latestSearchCorridorUsesCaptureRegions();
    initialCaptureReviewsAllFourCargoClasses();
    initialRiskReviewRestartsAfterInvalidEvidence();
    initialMixedPileJitterSplitsBeforeTargetReselection();
    latestRetreatTargetGapDoesNotBlockTurn();
    splitRetreatUsesUnwantedBlock();
    imuTranslationIntegration();
    splitActionBudgetsStayBounded();
    cornerTurnBrakesBeforePush();
    cornerReturnBalancesCommandsAndResumesSearch();
    upperCornerPushAndReturn();
    rushIdentityContinuityAndGap();
    laterSplitReacquiresNewCargoClass();
    approachOutsideInventoryAndNextTrip();
    mixedCargoDirectionAndLifecycle();
    laterTripMixedRulesAndUnknown();
    mixedPostcloseReturnsToReview();
    splitSelectorAcceptsLaterCargo();
    latestDiagonalPairStartsSplitBeforeRanking();
    actualLogOpening(argv[1]);
    ordinaryInventoryReviewBoundary();
    actualAcceptanceLogBoxes();
    inventorySideNeighborSplitsButUnmeasuredPathWaits();
    laterTripKeepsTwoSupplyRule();
    heldOrdinaryY180Boundary();
    measuredLateralOverlapBoundary();
    postcloseRejectsLatestLogBoxes();
    overlappingOrdinaryBoxesStillSplit();
    startupAlreadyInJawStillSplits();
    splitCountsEveryCargoClass();
    splitReverseTurnKeepsUi140AndSafety();
    splitUsesUnwantedBlockAndTurnClearance();
    approachRiskStopsBeforeClose();
    separationNeedsOneInsideAndNewStationaryFrames();
    postSplitClearOutsideGraspResumesApproach();
    latestLogSplitClosesWithoutForward();
    splitPositionDoesNotBypassCaptureChecks();
    failureKeepsCauseAndUnexecutedValidation();
    safeSingleClosesAndVerifies();
    outsideClippedNeighborDoesNotBlockSplit();
    invalidAssociationsWait();
    missingNeighborReobservesAndClosesAfterFreshScene();
    uncertainSceneRecoveryRequiresFreshStoppedFramesAndKeepsBudget();
    nearestFrontBackTarget();
    splitSelectorReleaseAndNewIdsRetry();
    splitReselectionMissingEvidenceStops();
    riskReviewStartsSplitByYOnly();
    postcloseUsesOnlyHeldCargo();
    postcloseOutsideBoxesDoNotBlockHeldEvidence();
    postcloseRequiresFreshHeldImages();
    precloseNeverBypassesMissingOrMovingEvidence();
    clearWindowRestartsWhenRiskReturns();
    postcloseRejectsTwoWithoutSuccessOrMotion();
}
