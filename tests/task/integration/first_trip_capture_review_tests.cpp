#include "../../support/push_task_sim.hpp"
#include "rescue/perception/pixel_selector.hpp"
#include <fstream>

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
    return t;
}
CaptureConfig calibration() {
    CaptureConfig c;
    c.pixel_grasp_enabled = true;
    c.grasp_center_ranges = {
        {"ordinary_supply", 2000, 1280, 720, {0, 226, 1279, 489}, {647.5f, 337}, false},
        {"core_supply", 2000, 1280, 720, {268.5f, 236, 1017, 555}, {647.5f, 337}, true}};
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
    SceneRig(bool single = false) {
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
    PushOutput step(bool fresh = true, bool feedback = true) {
        in.now_us += 50000;
        if (fresh)
            for (auto& d : objects)
                d.timestamp_us = in.now_us;
        if (feedback) {
            in.gripper_feedback_open = task.frame_raised_;
            in.gripper_closed_observed = task.frame_raised_ == 0;
        }
        if (managed_selection) {
            selector.syncSplitSelection(out.split_target_reselecting,
                                        out.split_target_candidate_id);
            if (!out.split_target_reselecting)
                selector.syncTaskState(
                    task.state_, task.trip_context_.target_id_, task.trip_context_.label_);
            selection = selector.update(objects,
                                        {1280, 720},
                                        in.camera_pitch_cdeg,
                                        in.camera_pitch_stable,
                                        in.now_us,
                                        false,
                                        .25f);
            in.target_valid = selection.locked;
            in.target_id = selection.locked ? selection.id : -1;
            in.label = selection.locked ? "ordinary_supply" : "";
            in.pixel_selection_managed = true;
            in.pixel_locked = selection.locked;
            in.pixel_released_id = selection.released_id;
            in.pixel_rebound_from = selection.rebound_from;
        }
        monitor.update(in, objects, in.now_us, task.trip_context_.label_);
        if (!fresh)
            in.capture_scene_us = objects.empty() ? 0 : objects.front().timestamp_us;
        out = task.update(in);
        return out;
    }
    void turnCycle() {
        using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
        auto& split = task.search_phase_.first_trip_split_;
        // Move the deepest target to the split retreat stop line Y280.
        objects[0].box.y = int(280.f - .5f * objects[0].box.height);
        for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i)
            step();
        assert(split.recovery_stage == Stage::Turn);
        assert(stopped(out.motion) && out.motion.camera_pitch_cdeg == 0);
        step(); // old UI140 feedback cannot start the UI120 turn
        assert(stopped(out.motion));
        in.camera_pitch_cdeg = 0;
        step();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == -.3f);
        const auto started = in.now_us;
        while (in.now_us - started < 450000) {
            step();
            assert(out.motion.wz_rps == -.3f && out.motion.vx_mps == 0);
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
        step();
    }
};
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
    assert(!rig.task.single_phase_initializing_);
    assert(rig.in.image_layout_vertical_dominant && rig.in.split_ordinary_count == 2);
    assert(rig.out.capture_scene_status == "risk");
    // The logged X offset must not prevent the confirmed pair from starting a split.
    for (int i = 0; i < 20 && !rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
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
    for (int top : {97, 87}) { // actual latest logs: centres Y173/Y163, bottoms Y249/Y239
        SceneRig rig;
        rig.objects = {detection(1, {538, 233, 212, 262}), detection(2, {553, top, 202, 152})};
        rig.task.setSinglePhase("capture");
        rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "latest_log_postclose");
        rig.task.frame_raised_ = 0;
        for (int i = 0; i < 12 && rig.out.run_status != RunStatus::Fault; ++i) {
            rig.step();
            assert(stopped(rig.out.motion));
            assert(rig.out.capture_evidence == CaptureEvidence::None);
            assert(rig.out.final_validation != "passed");
        }
        assert(rig.in.held.ordinary == 2);
        assert(rig.out.run_status == RunStatus::Fault);
        assert(rig.out.final_validation == "failed");
        assert(rig.out.original_reason == "held_too_many_supplies");
    }
    std::cout << "PASS latest log boxes rejected after closure even with centres Y163/Y173\n";
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
    assert(overlapping.out.motion.vx_mps < 0 && overlapping.out.motion.wz_rps == 0);
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
void splitUsesOrdinaryInventoryCount() {
    SceneRig rig;
    rig.split();
    rig.turnCycle();
    rig.step();
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.reason == "first_trip_split_retry_retreat");
    rig.objects[0].box.y = 350;
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.turnCycle();
    rig.objects[1].box = {600, 90, 80, 80};
    auto core = detection(3, {800, 260, 80, 80});
    core.label = "core_supply";
    rig.objects.push_back(core);
    for (int i = 0; i < 12 && rig.task.search_phase_.first_trip_split_.pending; ++i)
        rig.step();
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.reason == "first_trip_split_separation_confirmed");
    assert(rig.in.split_ordinary_visible_count == 2 && rig.in.split_ordinary_count == 1);
    SceneRig disappeared;
    disappeared.split();
    disappeared.turnCycle();
    disappeared.objects.pop_back();
    for (int i = 0; i < 12 && disappeared.task.search_phase_.first_trip_split_.pending; ++i)
        disappeared.step();
    assert(!disappeared.task.search_phase_.first_trip_split_.pending);
    std::cout << "PASS staged split retries multiple and verifies ordinary inventory only\n";
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
    rig.objects[0].box.y = 241; // centre Y281 must keep retreating, even below Y337.
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.objects[0].box.y = 240; // centre Y280 stops before the stopped-frame review.
    rig.step();
    assert(stopped(rig.out.motion) && rig.out.immediate_motion_stop);
    assert(rig.out.reason == "first_trip_split_retreat_brake");
    assert(rig.task.search_phase_.first_trip_split_.recovery_stage ==
           SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat);
    rig.turnCycle();
    rig.task.search_phase_.first_trip_split_.started_us = rig.in.now_us - rig.t.pile_budget_us;
    rig.step();
    assert(rig.out.run_status == RunStatus::Fault && stopped(rig.out.motion));
    assert(rig.out.reason == "first_trip_vertical_split_timeout");
    std::cout << "PASS staged split UI140 retreat, UI120 0.3rad/s 0.5s, UI140 review\n";
}

void splitUsesDeepestBlockAndTurnClearance() {
    using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
    SceneRig rig;
    rig.split();
    rig.objects[0].box.y = 230; // selected target is already short of Y280
    rig.objects[1].box.y = 360; // deeper neighbour must still cause straight retreat
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.objects[0].box.y = 240;
    rig.objects[1].box.y = 200;
    auto& split = rig.task.search_phase_.first_trip_split_;
    for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i)
        rig.step();
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
    std::cout << "PASS deepest block drives retreat; turn clearance and pause respected\n";
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
    assert(rig.out.reason == "first_trip_split_separation_confirmed");
    assert(rig.out.motion.camera_pitch_cdeg == 2000 && stopped(rig.out.motion));
    std::cout << "PASS UI140 verification needs fresh stopped frames\n";
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
        assert(rig.out.motion.vx_mps < 0 && rig.out.motion.wz_rps == 0);
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
            split_position |= rig.out.reason == "first_trip_split_enclosure_position";
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
void invalidAssociationsWait() {
    SceneRig rig;
    rig.step();
    rig.in.camera_pitch_cdeg = 2050;
    rig.step();
    assert(stopped(rig.out.motion));
    assert(rig.out.reason.find("cross_view_reassociation_required") != std::string::npos);
    SceneRig other;
    other.step();
    other.objects.push_back(detection(3, other.objects.back().box));
    other.objects.back().box.x += 1;
    other.step();
    assert(stopped(other.out.motion) && other.out.capture_scene_status == "incomplete");
    std::cout << "PASS cross-view and ambiguous re-ID require fresh reassessment\n";
}

void precloseNeverBypassesMissingOrMovingEvidence() {
    for (int scenario = 0; scenario < 4; ++scenario) {
        SceneRig rig;
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
            for (int i = 0; i < 8; ++i)
                rig.step();
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
    for (int i = 0; i < 10 && rig.out.run_status != RunStatus::Fault; ++i)
        rig.step();
    assert(rig.out.run_status == RunStatus::Fault);
    assert(rig.out.final_validation == "failed");
    assert(rig.out.failure_source == "postclose_verification");
    assert(rig.out.capture_evidence == CaptureEvidence::None && !rig.out.single_phase_done);
    assert(stopped(rig.out.motion));
    std::cout << "PASS postclose rejects two actual held objects and credits no success\n";
}

void postcloseCannotForgetOriginalNeighbor() {
    SceneRig rig;
    rig.step();
    rig.task.setSinglePhase("capture");
    rig.task.enter(PushState::VERIFY_CAPTURE, rig.in.now_us, "postclose_missing_fixture");
    rig.task.frame_raised_ = 0;
    rig.objects.pop_back();
    for (int i = 0; i < 300 && rig.out.run_status != RunStatus::Fault; ++i) {
        rig.step();
        assert(rig.out.capture_evidence == CaptureEvidence::None);
        assert(stopped(rig.out.motion));
    }
    assert(rig.out.run_status == RunStatus::Fault && rig.out.final_validation == "failed");
    assert(rig.out.failure_source == "postclose_verification");
    assert(rig.out.original_reason.find("association_ambiguous") != std::string::npos);
    std::cout << "PASS postclose retains original neighbor and rejects missing evidence\n";
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
                const bool in_y_range = center_y >= 226 && center_y <= 489;
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
    rig.objects[0].box.y = 240;
    for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i)
        rig.step();
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
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.objects[1].box.y = int(280.f - .5f * rig.objects[1].box.height);
    for (int i = 0; i < 20 && split.recovery_stage == Stage::Retreat; ++i)
        rig.step();
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
    assert(!split.pending && rig.out.reason == "first_trip_split_separation_confirmed");
    assert(rig.out.target_id == 21 && rig.in.split_ordinary_count == 1);
    assert(rig.out.final_validation == "not_executed");
    assert(rig.task.original_reason_ == "first_trip_confirmed_capture_path_split");
    std::cout << "PASS real selector release -> IDs 11/12 -> three-frame target 11 -> second turn "
                 "-> target 21\n";
}
void splitReselectionMissingEvidenceStops() {
    for (int invalid = 0; invalid < 4; ++invalid) {
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
        assert(rig.out.reason == "first_trip_vertical_split_timeout");
        assert(rig.out.run_status == RunStatus::Fault && stopped(rig.out.motion));
    }
    std::cout << "PASS missing/clipped/unstable/stale reacquisition stops within original budget\n";
}

void latestDiagonalPairStartsAndRepeatsSplit() {
    SceneRig rig(true);
    // 00:56:36: area ranking initially picks far #1; the near #2 overlaps inventory.
    rig.objects = {detection(2, {120, 318, 243, 283}), detection(1, {357, 62, 272, 302})};
    rig.managed_selection = true;
    bool chose_far = false, requested_near = false;
    for (int i = 0; i < 48 && !rig.task.search_phase_.first_trip_split_.pending; ++i) {
        rig.step();
        chose_far |= rig.in.target_valid && rig.in.target_id == 1;
        requested_near |=
            rig.out.split_target_reselecting && rig.out.split_target_candidate_id == 2;
        assert(stopped(rig.out.motion));
        assert(rig.task.frame_raised_ == 1 && rig.out.final_validation == "not_executed");
    }
    assert(chose_far && requested_near);
    assert(!rig.task.single_phase_initializing_);
    assert(rig.task.search_phase_.first_trip_split_.pending);
    assert(rig.out.target_id == 2 && rig.in.split_ordinary_count == 2);
    rig.step();
    assert(rig.out.motion.vx_mps == -.01f && rig.out.motion.wz_rps == 0);
    rig.turnCycle();
    rig.step();
    assert(rig.out.reason == "first_trip_split_retry_retreat");
    assert(rig.task.frame_raised_ == 1);
    std::cout
        << "PASS 00:56 diagonal pair: far #1 -> three-frame near #2 -> initial split -> retry\n";
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
} // namespace
int main(int argc, char** argv) {
    assert(argc == 2);
    latestDiagonalPairStartsAndRepeatsSplit();
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
    splitUsesOrdinaryInventoryCount();
    splitReverseTurnKeepsUi140AndSafety();
    splitUsesDeepestBlockAndTurnClearance();
    approachRiskStopsBeforeClose();
    separationNeedsOneInsideAndNewStationaryFrames();
    latestLogSplitClosesWithoutForward();
    splitPositionDoesNotBypassCaptureChecks();
    failureKeepsCauseAndUnexecutedValidation();
    safeSingleClosesAndVerifies();
    invalidAssociationsWait();
    nearestFrontBackTarget();
    splitSelectorReleaseAndNewIdsRetry();
    splitReselectionMissingEvidenceStops();
    riskReviewStartsSplitByYOnly();
    postcloseCannotForgetOriginalNeighbor();
    precloseNeverBypassesMissingOrMovingEvidence();
    clearWindowRestartsWhenRiskReturns();
    postcloseRejectsTwoWithoutSuccessOrMotion();
}
