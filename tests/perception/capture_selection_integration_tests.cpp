#include "rescue/perception/pixel_selector.hpp"
#include "rescue/perception/multi_view_capture.hpp"
#include "rescue/config/task_calibration.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace rescue;
SegDetection object(int id, cv::Rect box, std::string label = "ordinary_supply") {
    SegDetection d;
    d.track_id = id;
    d.box = box;
    d.label = label;
    d.confidence = .9;
    d.timestamp_us = 100000;
    d.ground_position_valid = d.ground_contact_valid = true;
    d.body_xy_m = {0, .13f};
    return d;
}
void startupTrackerRebind() {
    PushObservation in;
    in.camera_pitch_stable = true;
    in.camera_pitch_cdeg = 0;
    in.pixel_grasp_view = true;
    const uint64_t now = 1000000;
    StartupSightReference ref{7, "injured_person", {600,200,100,100}, 0, now - 30000};
    auto same = object(8, {604,202,100,100}, "injured_person");
    auto other = object(9, {850,450,100,100}, "injured_person");
    same.timestamp_us = other.timestamp_us = now;
    applyStartupSight(in, {same, other}, {1280,720}, now, .5f, 7, false, -1, ref);
    assert(in.target_valid && in.target_id == 8 && in.startup_rebound_from == 7);
    auto ambiguous = same;
    ambiguous.track_id = 10;
    applyStartupSight(in, {same, ambiguous}, {1280,720}, now, .5f, 7, false, -1, ref);
    assert(!in.target_valid && in.startup_rebound_from == -1);
    same.label = "ordinary_supply";
    applyStartupSight(in, {same}, {1280,720}, now, .5f, 7, false, -1, ref);
    assert(!in.target_valid);
    same.label = ref.label;
    in.camera_pitch_cdeg = 2000;
    applyStartupSight(in, {same}, {1280,720}, now, .5f, 7, false, -1, ref);
    assert(!in.target_valid);
    in.camera_pitch_cdeg = 0;
    ref.sample_us = 1;
    const uint64_t expired_now = now + kStartupTargetLossHoldUs;
    same.timestamp_us = expired_now;
    applyStartupSight(in, {same}, {1280,720}, expired_now, .5f, 7, false, -1, ref);
    assert(!in.target_valid);
    same.timestamp_us = now;
    ref.sample_us = now - 30000;
    // Logged UI140 loss: ID120 changes to ID124 with a resized box. The lower
    // IoU caused a target release although this is the unique nearby match.
    StartupSightReference logged{120, "ordinary_supply", {976,13,203,224}, 2000, now-30000};
    auto resized = object(124, {1012,3,254,244});
    auto neighbour = object(125, {865,8,244,193});
    resized.timestamp_us = neighbour.timestamp_us = now;
    in.camera_pitch_cdeg = 2000;
    applyStartupSight(in, {resized,neighbour}, {1280,720}, now, .5f, 120, true, -1, logged);
    assert(in.target_valid && in.target_id == 124 && in.startup_rebound_from == 120);
    in.camera_pitch_cdeg = 0;
    auto original = same;
    original.track_id = 7;
    applyStartupSight(in, {original, same}, {1280,720}, now, .5f, 7, false, -1, ref);
    assert(in.target_id == 7 && in.startup_rebound_from == -1);
}

void startupSightEvidence() {
    const uint64_t now = 1000000;
    PushObservation in;
    in.camera_pitch_stable = true;
    in.pixel_grasp_view = true;
    auto ordinary = object(1, {600, 510, 80, 80});
    auto danger = object(2, {100, 100, 80, 80}, "dangerous_object");
    ordinary.timestamp_us = danger.timestamp_us = now;
    ordinary.ground_position_valid = danger.ground_position_valid = false;
    applyStartupSight(in, {ordinary, ordinary}, {1280, 720}, now, .5f, -1);
    assert(in.startup_visible_classes == 1);
    assert(in.startup_visible_count == 1);
    applyStartupSight(in, {ordinary, danger}, {1280, 720}, now, .5f, -1);
    assert(in.startup_visible_classes == 9 && in.startup_scene_us == now);
    assert(in.startup_visible_count == 2);
    assert(in.target_id == 1 && in.pixel_view_handoff_ready && in.pixel_grasp_error_px == 0);
    assert(!in.geometry_valid);
    applyStartupSight(in, {ordinary, danger}, {1280, 720}, now, .5f, 2);
    assert(in.target_id == 2 && in.label == "dangerous_object");
    for (const char* label : {"injured_person", "core_supply", "dangerous_object"}) {
        danger.label = label;
        applyStartupSight(in, {danger}, {1280, 720}, now, .5f, 2);
        assert(in.startup_visible_classes != 0 && in.target_valid && in.label == label);
    }
    // Unlocked selection compares all classes; locked selection ignores neighbour jitter.
    danger.label = "dangerous_object";
    danger.box = {600, 620, 80, 80};
    auto lower_ordinary = ordinary;
    lower_ordinary.track_id = 3;
    lower_ordinary.box.y = 580;
    applyStartupSight(in, {ordinary, lower_ordinary, danger}, {1280, 720}, now, .5f, 2);
    assert(in.target_id == 2 && in.label == "dangerous_object");
    applyStartupSight(in, {ordinary, lower_ordinary, danger}, {1280, 720}, now, .5f, 1);
    assert(in.target_id == 1);
    applyStartupSight(in, {lower_ordinary, danger}, {1280, 720}, now, .5f, 1);
    assert(!in.target_valid && in.startup_visible_count == 2);
    applyStartupSight(in, {danger}, {1280, 720}, now, .5f, 3);
    assert(!in.target_valid);
    applyStartupSight(in, {lower_ordinary, danger}, {1280, 720}, now, .5f, -1);
    assert(in.target_id == 2);
    // UI140 must select an ordinary even when a non-ordinary is lower/nearer.
    applyStartupSight(in, {lower_ordinary, danger}, {1280, 720}, now, .5f, -1, true);
    assert(in.target_id == 3 && in.label == "ordinary_supply");
    applyStartupSight(in, {ordinary, lower_ordinary, danger}, {1280, 720}, now, .5f, -1, true, 3);
    assert(in.target_id == 1 && in.startup_visible_count == 3);
    applyStartupSight(in, {danger}, {1280, 720}, now, .5f, -1, true);
    assert(!in.target_valid && in.startup_visible_count == 1);
    for (bool near : {false, true}) {
        in.pixel_grasp_near_view = near;
        for (int jitter = -15; jitter <= 15; ++jitter) {
            danger.box.y = ordinary.box.y + jitter;
            applyStartupSight(in, {ordinary, danger}, {1280, 720}, now, .5f, 1);
            assert(in.target_id == 1 && in.target_valid);
        }
    }
    for (int bad = 0; bad < 5; ++bad) {
        auto d = ordinary;
        if (bad == 0) d.timestamp_us = now - 200001;
        if (bad == 1) d.timestamp_us = now + 1;
        if (bad == 2) d.confidence = .49f;
        if (bad == 3) d.label = "blue_safe_zone";
        if (bad == 4) d.box = {1400, 0, 80, 80};
        applyStartupSight(in, {d}, {1280, 720}, now, .5f, -1);
        assert(in.startup_visible_classes == 0 && !in.target_valid);
    }
    in.camera_pitch_stable = false;
    applyStartupSight(in, {ordinary, danger}, {1280, 720}, now, .5f, -1);
    assert(!in.startup_visible_classes && !in.target_valid);
}
// Exercise the real detector adapter and UI90 startup state machine together.
void startupTwoObjectsReachTask() {
    for (int scenario = 0; scenario < 14; ++scenario) {
        TaskTuning t;
        t.startup_advance_us = 1;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.far_pitch_cdeg = -3000;
        PushTask task(t);
        task.setSinglePhase("startup");
        PushObservation in;
        in.now_us = 1000000;
        in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.camera_pitch_stable = in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 0;
        in.camera_pitch_cdeg = t.far_pitch_cdeg;
        bool approached = false, scanned = false;
        for (int frame = 0; frame < 12; ++frame) {
            in.now_us += 50000;
            const char* labels[] = {"ordinary_supply", "core_supply", "injured_person", "dangerous_object"};
            auto first = object(11, {580, 300, 80, 80}, labels[scenario % 4]);
            auto second = object(12, {800, 200, 80, 80}, first.label);
            first.timestamp_us = second.timestamp_us = in.now_us;
            std::vector<SegDetection> detections{first, second};
            if (scenario == 4) detections[1].label = "core_supply"; // mixed pair
            if (scenario == 5) detections.push_back(object(13, {300, 200, 80, 80}));
            if (scenario == 5) detections.back().timestamp_us = in.now_us;
            if (scenario == 6) detections.resize(1);
            if (scenario == 7) detections[1] = first; // one detection repeated
            if (scenario == 8) detections[1].timestamp_us = in.now_us - 200001;
            if (scenario == 9) detections[1].timestamp_us = in.now_us + 1;
            if (scenario == 10) detections[1].confidence = .49f;
            if (scenario == 11) detections[1].label = "blue_safe_zone";
            if (scenario == 12) detections.clear();
            if (scenario == 13) in.camera_pitch_stable = false;
            applyStartupSight(in, detections, {1280, 720}, in.now_us, .5f,
                              task.startupSightTargetId());
            const auto out = task.update(in);
            scanned |= out.reason == "startup_find_two_objects";
            if (out.reason == "startup_approach_far") {
                approached = true;
                assert(out.motion.vx_mps > 0 && out.motion.camera_pitch_cdeg == t.far_pitch_cdeg);
                break;
            }
            in.sent_vx_mps = out.motion.vx_mps;
            in.sent_wz_rps = out.motion.wz_rps;
        }
        std::cout << "UI90 object-count scenario=" << scenario << " approached=" << approached << std::endl;
        assert(approached == (scenario <= 5));
        if (scenario >= 6 && scenario != 13) assert(scanned);
    }
    std::cout << "PASS UI90 two-or-more physical objects, including same-class pairs, start approach\n";
}

void startupLowerCornerEvidence() {
    constexpr uint64_t now = 1000000;
    PushObservation in;
    in.camera_pitch_stable = true;
    auto left = object(1, {20, 480, 100, 100});
    auto right = object(2, {1160, 480, 100, 100}, "core_supply");
    left.timestamp_us = right.timestamp_us = now;
    const auto observe = [&](std::vector<SegDetection> detections) {
        // Corner evidence must remain available even if the locked guide is missing.
        applyStartupSight(in, detections, {1280, 720}, now, .5f, 77);
        return in.startup_lower_corners_occupied;
    };
    assert(observe({left, right}) && !in.target_valid);
    assert(!observe({left}) && !observe({right}));
    auto high = right;
    high.box.y = 100;
    assert(!observe({left, high})); // left/right alone is insufficient
    auto middle = right;
    middle.box = {590, 550, 100, 100};
    assert(!observe({left, middle}));
    auto wide = left;
    wide.box = {0, 550, 1280, 100};
    assert(!observe({wide}) && !observe({wide, wide}));
    for (int bad = 0; bad < 4; ++bad) {
        auto invalid = right;
        if (bad == 0) invalid.timestamp_us = now - 200001;
        if (bad == 1) invalid.timestamp_us = now + 1;
        if (bad == 2) invalid.confidence = .49f;
        if (bad == 3) invalid.label = "blue_safe_zone";
        assert(!observe({left, invalid}));
    }
    // Scale the regions with the actual frame dimensions, not fixed pixels.
    left.box = {10, 240, 50, 50};
    right.box = {580, 240, 50, 50};
    applyStartupSight(in, {left, right}, {640, 360}, now, .5f, -1);
    assert(in.startup_lower_corners_occupied);
    left.box = {20, 480, 100, 100};
    right.box = {1160, 480, 100, 100};
    in.camera_pitch_stable = false;
    assert(!observe({left, right}));
    std::cout << "PASS Startup lower corners use distinct fresh cargo boxes, independent of target lock\n";
}

void startupLowerCornersReachTask() {
    TaskTuning t;
    t.startup_advance_us = 1;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.far_pitch_cdeg = -3000;
    t.track_pitch_cdeg = 0;
    PushTask task(t);
    task.setSinglePhase("startup");
    PushObservation in;
    in.now_us = 1000000;
    in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
    in.camera_pitch_stable = in.gripper_done = in.heading_valid = in.sent_velocity_valid = true;
    in.gripper_feedback_open = 0;
    in.camera_pitch_cdeg = t.far_pitch_cdeg;
    for (int frame = 0; frame < 120; ++frame) {
        in.now_us += 50000;
        const bool track = in.camera_pitch_cdeg == t.track_pitch_cdeg;
        in.pixel_grasp_view = track;
        auto left = object(11, {20, track ? 480 : 550, 100, 100});
        auto right = object(12, {1160, track ? 480 : 550, 100, 100}, "core_supply");
        left.timestamp_us = right.timestamp_us = in.now_us;
        applyStartupSight(in, {left, right}, {1280, 720}, in.now_us, .5f,
                          task.startupSightTargetId(), task.startupOrdinaryOnly(), -1,
                          task.startupSightReference(), task.startupPileApproach());
        const auto out = task.update(in);
        if (out.reason == "startup_pitch_turn_ready") {
            assert(track && in.startup_lower_corners_occupied);
            assert(in.target_center_px.y == 530 && !in.pixel_view_handoff_ready);
            assert(out.immediate_motion_stop && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            std::cout << "PASS UI90 -> UI120 -> corner maneuver through real perception and task\n";
            return;
        }
        in.camera_pitch_cdeg = out.motion.camera_pitch_cdeg;
        in.sent_vx_mps = out.motion.vx_mps;
        in.sent_wz_rps = out.motion.wz_rps;
    }
    assert(false && "UI120 corner evidence did not reach the maneuver");
}

void splitRetreatBoxEvidence() {
    constexpr uint64_t now = 1000000;
    auto original = object(7, {300, 80, 60, 60});
    auto centre = object(8, {610, 0, 60, 60});
    original.timestamp_us = centre.timestamp_us = now;
    original.ground_position_valid = original.ground_contact_valid = false;
    centre.ground_position_valid = centre.ground_contact_valid = false;
    PushObservation in;
    applySplitRetreatBox(in, {original, centre}, {1280, 720}, now, .25f, 7);
    assert(in.split_ordinary_box == original.box && in.split_ordinary_box_us == now);
    assert(in.split_ordinary_box_id == 7 && in.split_image_width_px == 1280);
    assert(!in.target_valid && !in.geometry_valid);
    applySplitRetreatBox(in, {centre}, {1280, 720}, now, .25f, 7);
    assert(in.split_ordinary_box.y == 0 && in.split_ordinary_box_us == now);
    for (int invalid = 0; invalid < 6; ++invalid) {
        auto bad = centre;
        if (invalid == 0)
            bad.label = "core_supply";
        if (invalid == 1)
            bad.timestamp_us = now - 200001;
        if (invalid == 2)
            bad.timestamp_us = now + 1;
        if (invalid == 3)
            bad.confidence = .24f;
        if (invalid == 4)
            bad.box.width = 0;
        if (invalid == 5)
            bad.box.y = -100; // entirely outside, not a top-edge observation
        applySplitRetreatBox(in, {bad}, {1280, 720}, now, .25f, 7);
        assert(in.split_ordinary_box.empty() && in.split_ordinary_box_us == 0);
    }
    centre.box.y = -1; // detector coordinates crossing the image top are clipped to Y=0
    applySplitRetreatBox(in, {centre}, {1280, 720}, now, .25f, 7);
    assert(in.split_ordinary_box.y == 0 && in.split_ordinary_box.height == 59);
}
void nearOwnZoneExclusion() {
    const uint64_t now = 1000000;
    for (const std::string team : {"blue", "red"}) {
        auto zone = object(-1, {200, 200, 300, 300}, team + "_safe_zone");
        zone.timestamp_us = now;
        zone.ground_position_valid = zone.ground_contact_valid = false;
        for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"}) {
            auto cargo = object(11, {140, 250, 30, 30}, label);
            cargo.timestamp_us = now;
            const auto excluded = [&](cv::Rect box) {
                cargo.box = box;
                return targetsNearOwnZone({cargo, zone}, team, {1000, 1000}, now, .25f);
            };
            assert(excluded({140, 250, 30, 30}) == std::vector<int>{11}); // 30 px boundary
            assert(excluded({139, 250, 30, 30}).empty());                 // 31 px away
            assert(excluded({250, 140, 30, 30}) == std::vector<int>{11});
            assert(excluded({250, 531, 30, 30}).empty());
            assert(excluded({150, 150, 30, 30}) == std::vector<int>{11}); // diagonal < 30
            assert(excluded({140, 140, 30, 30}).empty());                 // diagonal > 30
            assert(excluded({180, 250, 30, 30}) == std::vector<int>{11}); // overlap
            assert(excluded({250, 250, 30, 30}) == std::vector<int>{11}); // inside
            cargo.box = {280, 500, 60, 60};
            auto scaled = zone;
            scaled.box = {400, 400, 600, 600};
            assert(targetsNearOwnZone({cargo, scaled}, team, {2000, 2000}, now, .25f) ==
                   std::vector<int>{11});
            cargo.box = {140, 250, 30, 30};
            auto enemy = zone;
            enemy.label = team == "blue" ? "red_safe_zone" : "blue_safe_zone";
            assert(targetsNearOwnZone({cargo, enemy}, team, {1000, 1000}, now, .25f).empty());
            assert(targetsNearOwnZone({cargo, enemy, zone}, team, {1000, 1000}, now, .25f) ==
                   std::vector<int>{11});
            auto bad = zone;
            bad.timestamp_us = now - 200001;
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            bad.timestamp_us = now + 1;
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            bad.timestamp_us = now;
            bad.confidence = .24f;
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            bad = zone;
            bad.box = {1100, 200, 100, 100};
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            assert(targetsNearOwnZone({cargo, zone}, team, {0, 0}, now, .25f).empty());
            assert(targetsNearOwnZone({cargo, zone}, "unknown", {1000, 1000}, now, .25f).empty());
        }
    }
    // A late zone veto immediately releases a lock, including relaxed selection.
    for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"}) {
        PixelSelector selector;
        selector.setRelax(2);
        auto near = object(21, {140, 250, 30, 30}, label);
        auto far = object(22, {800, 250, 10, 10});
        auto zone = object(-1, {200, 200, 300, 300}, "blue_safe_zone");
        PixelSelection selected;
        for (int i = 0; i < 3; ++i) {
            near.timestamp_us = far.timestamp_us = now + i * 10000;
            selected = selector.update(
                {near, far}, {1000, 1000}, 500, true, near.timestamp_us, true, .25f);
        }
        assert(selected.locked && selected.id == 21);
        for (int i = 3; i < 7; ++i) {
            near.timestamp_us = far.timestamp_us = zone.timestamp_us = now + i * 10000;
            const auto rejected = targetsNearOwnZone(
                {near, far, zone}, "blue", {1000, 1000}, near.timestamp_us, .25f);
            assert(rejected == std::vector<int>{near.track_id});
            selected = selector.update(
                {near, far}, {1000, 1000}, 500, true, near.timestamp_us, true, .25f, rejected);
            assert(selected.id != near.track_id);
            const auto old = makePushObservation(
                {near, far}, near.timestamp_us, true, .25f, near.track_id, rejected);
            assert(!old.target_valid);
            near.track_id = 23; // tracker re-ID cannot evade the spatial veto
        }
        assert(selected.locked && selected.id == far.track_id);
        near.timestamp_us = zone.timestamp_us = now + 80000;
        const auto rejected =
            targetsNearOwnZone({near, zone}, "blue", {1000, 1000}, near.timestamp_us, .25f);
        assert(
            !makePushObservation({near}, near.timestamp_us, true, .25f, -1, rejected).target_valid);
    }
}
void persistentTargetTests() {
    for (int n : {2, 3, 5}) {
        PixelSelector sel;
        uint64_t now = 1000000;
        std::vector<SegDetection> ds;
        for (int i = 0; i < n; ++i) {
            auto d = object(i + 1, {80 + i * 160, 300, i ? 40 : 70, i ? 40 : 70});
            d.body_xy_m = {-.5f + .25f * i, .8f};
            ds.push_back(d);
        }
        auto step = [&] {
            now += 20000;
            for (auto& d : ds)
                d.timestamp_us = now;
            return sel.update(ds, {1000, 800}, 500, true, now, false, .25f);
        };
        step();
        step();
        auto out = step();
        assert(out.locked && out.id == 1);
        sel.syncTaskTarget(1, "ordinary_supply");
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < n; ++j)
                ds[j].box.width = ds[j].box.height = j == 0 ? 35 : (i / 5 % n == j ? 110 : 60);
            out = step();
            assert(out.locked && out.id == 1 && out.event == "target_held");
        }
        ds[0].ground_contact_valid = ds[0].ground_position_valid = false;
        assert(step().id == 1);
        sel.syncTaskTarget(-1, "");
        assert(!step().locked);
    }
    {
        PixelSelector sel;
        uint64_t now = 2000000;
        auto a = object(1, {100, 300, 60, 60});
        auto b = object(2, {700, 300, 50, 50});
        auto step = [&](std::vector<SegDetection> ds, uint64_t dt = 20000) {
            now += dt;
            for (auto& d : ds)
                d.timestamp_us = now;
            return sel.update(ds, {1000, 800}, 500, true, now, false, .25f);
        };
        step({a, b});
        step({a, b});
        assert(step({a, b}).id == 1);
        sel.syncTaskTarget(1, a.label);
        for (int i = 0; i < 14; ++i) {
            auto out = step({b});
            assert(!out.locked && out.held_id == 1 && out.released_id == -1);
        }
        assert(step({a, b}).id == 1);
        auto re = a;
        re.track_id = 9;
        auto other = a;
        other.track_id = 10;
        other.box.x += 20;
        for (int i = 0; i < 3; ++i) {
            const auto out = step({re, other, b});
            assert(!out.locked && out.event == "rebind_ambiguous");
        }
        step({re, b});
        step({re, b});
        auto out = step({re, b});
        assert(out.id == 9 && out.rebound_from == 1);
        sel.syncTaskTarget(1, a.label); // A task guard may delay accepting the rebind.
        out = step({re, b});
        assert(out.id == 9 && out.rebound_from == 1 && out.event == "await_task_rebind");
        sel.syncTaskTarget(9, a.label);
        for (int i = 0; i < 15; ++i)
            assert(!step({b}).locked);
        out = step({b});
        assert(out.released_id == 9 && out.event == "target_lost_timeout");
        sel.syncTaskTarget(9, a.label);
        out = step({b});
        assert(!out.locked && out.released_id == 9 && out.event == "await_task_release");
        sel.syncTaskTarget(-1, "");
        step({b});
        step({b});
        out = step({b});
        assert(out.id == 2 && out.rebound_from == -1);
        b.timestamp_us = now + 20000;
        out = sel.update({b}, {1000, 800}, 500, true, now + 20000, false, .25f, {2});
        assert(!out.locked && out.released_id == 2 && out.event == "target_rejected");
    }
    {
        PixelSelector sel;
        uint64_t now = 4000000;
        auto a = object(1, {100, 300, 60, 60});
        a.body_xy_m = {-.2f, .8f};
        auto b = object(2, {700, 300, 50, 50});
        b.body_xy_m = {.4f, .8f};
        auto step = [&](std::vector<SegDetection> ds, int16_t pitch, bool stable, float yaw) {
            now += 50000;
            for (auto& d : ds)
                d.timestamp_us = now;
            return sel.update(ds, {1000, 800}, pitch, stable, now, false, .25f, {}, "", yaw, 1.f, nullptr, true);
        };
        step({a, b}, 500, true, 0);
        step({a, b}, 500, true, 0);
        assert(step({a, b}, 500, true, 0).id == 1);
        sel.syncTaskTarget(1, a.label);
        for (int i = 0; i < 8; ++i)
            assert(!step({}, 2000, false, 0).locked);
        a.track_id = 9;
        a.box = {500, 100, 70, 70};
        // The new view chooses current cargo even without any ground evidence.
        a.ground_position_valid = a.ground_contact_valid = false;
        b.ground_position_valid = b.ground_contact_valid = false;
        step({a, b}, 2000, true, .1f);
        step({a, b}, 2000, true, .1f);
        const auto out = step({a, b}, 2000, true, .1f);
        assert(out.id == 9 && out.reselected_from == 1 && out.rebound_from == -1 &&
               out.event == "view_target_reselected");
    }
    {
        PixelSelector sel;
        uint64_t now = 6000000;
        auto a = object(1, {100, 300, 60, 60});
        auto step = [&](int16_t pitch, bool stable) {
            now += 50000;
            a.timestamp_us = now;
            return sel.update({a}, {1000, 800}, pitch, stable, now, false, .25f);
        };
        step(500, true);
        step(500, true);
        assert(step(500, true).id == 1);
        step(2000, false);
        PixelSelection out;
        for (int i = 0; i < 7; ++i) {
            out = step(2000, true);
            assert(!out.locked);
        }
        assert(out.released_id == 1 && out.event == "target_lost_timeout");
        step(2000, true);
        step(2000, true);
        assert(step(2000, true).id == 1);
    }
}
void scatterSelectionLifecycle() {
    PixelSelector selector;
    auto d = object(13, {600, 280, 26, 26});
    d.body_xy_m = {0.f, 1.73f};
    PixelSelection selected;
    uint64_t now = 2000000;
    const auto step = [&](PushState state, int target_id) {
        selector.syncTaskState(state, target_id, "ordinary_supply");
        d.timestamp_us = now;
        selected = selector.update({d}, {1280, 720}, -3000, true, now, false, .5f);
        now += 30000;
    };
    step(PushState::SCAN, -1);
    step(PushState::SCAN, -1);
    step(PushState::SCAN, -1);
    assert(selected.locked && selected.id == 13);
    // Entry clears the old episode once; ongoing scatter reaches three fresh frames.
    for (int i = 0; i < 10; ++i) {
        step(PushState::SCATTER, -1);
        assert(selected.locked == (i >= 2));
    }
    step(PushState::SCAN, -1);
    assert(selected.locked && selected.id == 13);
    step(PushState::APPROACH, 13);
    assert(selected.locked && selected.id == 13);
    step(PushState::PILE, -1);
    assert(!selected.locked);
    step(PushState::PILE, -1);
    assert(!selected.locked);
    step(PushState::PILE, -1);
    assert(selected.locked);
    step(PushState::SCATTER, -1);
    assert(!selected.locked);
}

void rushContinuousIdentity() {
    for (int scenario = 0; scenario < 5; ++scenario) {
        PixelSelector selector;
        uint64_t now = 8000000;
        auto target = object(1, {300, 300, 100, 100});
        auto neighbour = object(2, {700, 300, 60, 60});
        auto step = [&](std::vector<SegDetection> ds, bool stable = true) {
            now += 30000;
            for (auto& d : ds) d.timestamp_us = now;
            return selector.update(ds, {1000, 800}, 2000, stable, now, false, .25f);
        };
        step({target, neighbour});
        step({target, neighbour});
        assert(step({target, neighbour}).id == 1);
        selector.syncTaskState(PushState::RUSH, 1, target.label);
        auto rebound = target;
        rebound.track_id = 9;
        rebound.box.x += 2;
        if (scenario == 0) {
            neighbour.box = {650, 250, 250, 250};
            assert(step({target, neighbour}).id == 1); // no area-based target switch
            auto out = step({rebound, neighbour});
            assert(out.locked && out.id == 9 && out.rebound_from == 1);
        } else if (scenario == 1) {
            auto duplicate = rebound;
            duplicate.track_id = 10;
            auto out = step({rebound, duplicate, neighbour});
            assert(!out.locked && out.event == "rebind_ambiguous");
        } else if (scenario == 2) {
            auto out = step({neighbour});
            assert(!out.locked && out.held_id == 1 && out.id != 2);
            for (int i = 0; i < 10; ++i) out = step({neighbour});
            assert(out.released_id == 1 && out.event == "target_lost_timeout");
        } else if (scenario == 3) {
            step({target, neighbour}, false);
            assert(!step({rebound, neighbour}).locked); // view change still needs confirmation
        } else {
            rebound.box.width = rebound.box.height = 180; // weak overlap keeps three frames
            assert(!step({rebound, neighbour}).locked);
        }
    }
}

// Real adapter -> task loop: two visible boxes alternate their centre-Y order.
// Both remain stationary within the braking tolerance; rank jitter must not
// restart the UI90 handoff's stopped-image confirmation.
void startupHandoffKeepsVisibleGuide() {
    TaskTuning t;
    t.startup_advance_us = 1;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.far_pitch_cdeg = -3000;
    t.track_pitch_cdeg = 0;
    PushTask task(t);
    task.setSinglePhase("startup");
    PushObservation in;
    in.now_us = 1000000;
    in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
    in.camera_pitch_stable = in.gripper_done = in.heading_valid = in.sent_velocity_valid = true;
    in.gripper_feedback_open = 0;
    in.camera_pitch_cdeg = t.far_pitch_cdeg;
    int handoff_frames = 0;
    for (int frame = 0; frame < 60; ++frame) {
        in.now_us += 50000;
        auto guide = object(48, {714, 507 + frame % 2 * 2, 112, 147}, "core_supply");
        auto neighbour = object(46, {548, 519 - frame % 2 * 2, 112, 127}, "dangerous_object");
        guide.timestamp_us = neighbour.timestamp_us = in.now_us;
        applyStartupSight(in, {guide, neighbour}, {1280,720}, in.now_us, .5f,
                          task.startupSightTargetId(), false, -1, task.startupSightReference());
        const auto out = task.update(in);
        if (out.reason == "pixel_align_brake" || out.reason == "pixel_align_settle")
            ++handoff_frames;
        if (out.reason == "startup_switch_to_120") {
            assert(handoff_frames > 0 && handoff_frames <= 20);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            std::cout << "PASS Startup rank jitter retains guide and completes handoff within 1 second\n";
            return;
        }
        in.sent_vx_mps = out.motion.vx_mps;
        in.sent_wz_rps = out.motion.wz_rps;
    }
    assert(false && "visible guide rank jitter stalled UI90 handoff");
}


void startupLoggedVerticalRebind() {
    // 17:34: tracked injured_person #63 jumps vertically and becomes #75;
    // the neighbouring dangerous object must not influence identity association.
    constexpr uint64_t now = 1000000;
    PushObservation in;
    in.camera_pitch_stable = in.pixel_grasp_view = true;
    in.camera_pitch_cdeg = 0;
    StartupSightReference ref{63, "injured_person", {538,238,152,273}, 0, now-30000};
    auto same = object(75, {538,97,172,253}, "injured_person");
    auto neighbour = object(74, {110,188,263,252}, "dangerous_object");
    same.timestamp_us = neighbour.timestamp_us = now;
    applyStartupSight(in, {same,neighbour}, {1280,720}, now, .5f, 63, false, -1, ref);
    assert(in.target_valid && in.target_id == 75 && in.startup_rebound_from == 63);
    auto ambiguous = same;
    ambiguous.track_id = 76;
    applyStartupSight(in, {same,ambiguous}, {1280,720}, now, .5f, 63, false, -1, ref);
    assert(!in.target_valid);
    auto lateral = same;
    lateral.box.x += 140;
    applyStartupSight(in, {lateral}, {1280,720}, now, .5f, 63, false, -1, ref);
    assert(!in.target_valid);
    std::cout << "PASS Logged vertical re-ID resumes immediately; ambiguous/lateral matches rejected\n";
}


// The 18:12 UI120 frame: the rightmost lower box must not define the heading.
void startupPileCenterRegression() {
    PushObservation in;
    in.camera_pitch_stable = in.pixel_grasp_view = true;
    in.camera_pitch_cdeg = 0;
    constexpr uint64_t now = 1000000;
    std::vector<SegDetection> pile{
        object(63, {694,152,182,203}, "dangerous_object"),
        object(62, {568,137,172,192}, "injured_person"),
        object(61, {885,188,162,212}, "injured_person"),
        object(65, {920,72,112,122}, "injured_person")};
    for (auto& d : pile) d.timestamp_us = now;
    applyStartupSight(in, pile, {1280,720}, now, .5f, 61, false, -1, {}, true);
    std::cout << "UI120 aim_x=" << in.target_center_px.x
              << " pile_bounds_center_x=807.5" << std::endl;
    assert(in.target_valid && std::abs(in.target_center_px.x - 807.5f) < .01f);
    assert(std::abs(in.pixel_grasp_error_px + 167.5f) < .01f);
    assert(in.target_id == kStartupPileTargetId && in.label == "startup_pile");
    const StartupSightReference ref{in.target_id, in.label, in.target_box, 0, now};
    // Re-ID and detection order do not turn a pile guide back into a single box.
    for (auto& d : pile) d.track_id += 100;
    std::reverse(pile.begin(), pile.end());
    applyStartupSight(in, pile, {1280,720}, now, .5f, ref.id, false, -1, ref, true);
    assert(in.target_id == ref.id && in.target_center_px.x == 807.5f);
    // Even loss of the former individual guide leaves the remaining pile usable.
    pile.erase(std::remove_if(pile.begin(), pile.end(),
                             [](const auto& d) { return d.track_id == 161; }), pile.end());
    applyStartupSight(in, pile, {1280,720}, now, .5f, ref.id, false, -1, ref, true);
    assert(in.target_valid && in.target_id == ref.id && in.target_center_px.x == 800.f);
    auto invalid = object(999, {0,0,1280,720});
    invalid.timestamp_us = now - 200001;
    pile.push_back(invalid);
    invalid.timestamp_us = now;
    invalid.label = "blue_safe_zone";
    pile.push_back(invalid);
    applyStartupSight(in, pile, {1280,720}, now, .5f, ref.id, false, -1, ref, true);
    assert(in.target_center_px.x == 800.f);
    applyStartupSight(in, {}, {1280,720}, now, .5f, ref.id, false, -1, ref, true);
    assert(!in.target_valid && !in.target_center_px_valid && !in.pixel_grasp_sample_us);
    // Only the closed UI120 pile approach changes; far/near/ordinary keep real IDs.
    auto ordinary = object(3, {1000,500,100,100});
    ordinary.timestamp_us = now;
    pile.push_back(ordinary);
    in.pixel_grasp_view = false;
    applyStartupSight(in, pile, {1280,720}, now, .5f, -1, false, -1, {}, true);
    assert(in.target_id == ordinary.track_id && in.target_center_px.x == 1050.f);
    in.pixel_grasp_view = in.pixel_grasp_near_view = true;
    applyStartupSight(in, pile, {1280,720}, now, .5f, -1, false, -1, {}, true);
    assert(in.target_id == ordinary.track_id);
    in.pixel_grasp_near_view = false;
    applyStartupSight(in, pile, {1280,720}, now, .5f, -1, true, -1, {}, true);
    assert(in.target_id == ordinary.track_id && in.label == "ordinary_supply");
    std::cout << "PASS UI120 pile centre survives re-ID/loss; other views retain real targets\n";
}

void startupPileHeadingReachesTask() {
    TaskTuning t;
    t.startup_advance_us = 1;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.far_pitch_cdeg = -3000;
    t.track_pitch_cdeg = 0;
    PushTask task(t);
    task.setSinglePhase("startup");
    PushObservation in;
    in.now_us = 1000000;
    in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
    in.camera_pitch_stable = in.gripper_done = in.sent_velocity_valid = true;
    in.camera_pitch_cdeg = t.far_pitch_cdeg;
    in.gripper_feedback_open = 0;
    for (int frame = 0; frame < 120; ++frame) {
        in.now_us += 50000;
        const bool track = in.camera_pitch_cdeg == t.track_pitch_cdeg;
        in.pixel_grasp_view = track;
        auto left = object(track ? 100 + frame : 100, track ? cv::Rect(280,100,100,100)
                                              : cv::Rect(580,550,100,100));
        auto right = object(track ? 300 + frame : 300, track ? cv::Rect(900,300,100,100)
                                               : cv::Rect(800,530,100,100), "injured_person");
        left.timestamp_us = right.timestamp_us = in.now_us;
        applyStartupSight(in, {left,right}, {1280,720}, in.now_us, .5f,
                          task.startupSightTargetId(), task.startupOrdinaryOnly(), -1,
                          task.startupSightReference(), task.startupPileApproach());
        if (track && task.startupPileApproach()) {
            assert(in.target_id == kStartupPileTargetId);
            assert(in.target_center_px.x == 640.f && in.pixel_grasp_error_px == 0);
        }
        const auto out = task.update(in);
        if (out.reason == "startup_approach_120") {
            assert(track && out.motion.vx_mps > 0 &&
                   std::abs(out.motion.wz_rps + .05f) < 1e-6f);
            std::cout << "PASS UI90 -> UI120 task advances with zone-3 right bias despite per-frame re-ID\n";
            return;
        }
        in.camera_pitch_cdeg = out.motion.camera_pitch_cdeg;
        in.sent_vx_mps = out.motion.vx_mps;
        in.sent_wz_rps = out.motion.wz_rps;
    }
    assert(false && "pile heading did not reach UI120 advance");
}

// Logged 18:23 dropout: the left member disappears while the right box stays still.
// The raw union error flips from +36 to -85; an already aligned pile must keep advancing.
void startupPileDropoutDoesNotRestartAlignment() {
    TaskTuning tuning;
    tuning.startup_advance_us = 1;
    tuning.start_zone = 3;
    tuning.pixel_grasp_enabled = true;
    tuning.pixel_align_brake_motion_px = 30;
    tuning.far_pitch_cdeg = -3000;
    tuning.track_pitch_cdeg = 0;
    tuning.near_pitch_cdeg = 2000;
    PushTask task(tuning);
    task.setSinglePhase("startup");
    StartupPileGuide guide;
    PushObservation in;
    in.now_us = 1000000;
    in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
    in.camera_pitch_stable = in.gripper_done = in.sent_velocity_valid = true;
    in.camera_pitch_cdeg = tuning.far_pitch_cdeg;
    in.gripper_feedback_open = 0;
    bool advanced = false;
    int dropout_advances = 0;
    for (int frame = 0; frame < 160; ++frame) {
        in.now_us += 27000;
        const bool track = in.camera_pitch_cdeg == tuning.track_pitch_cdeg;
        in.pixel_grasp_view = track;
        auto left = object(44, track ? cv::Rect(427,72,152,192) : cv::Rect(580,550,100,100),
                           "injured_person");
        auto right = object(46, track ? cv::Rect(649,87,152,192) : cv::Rect(800,530,100,100),
                            "injured_person");
        left.timestamp_us = right.timestamp_us = in.now_us;
        // Begin inside the acceptance band, then replay the logged +36/-85
        // union jump. A member dropout must not reverse the accepted heading.
        if (track && advanced) left.box.x = 407;
        std::vector<SegDetection> scene{left, right};
        const bool dropout = track && advanced && frame % 3 == 0;
        if (dropout) scene.erase(scene.begin());
        applyStartupSight(in, scene, {1280,720}, in.now_us, .5f,
                          task.startupSightTargetId(), task.startupOrdinaryOnly(), -1,
                          task.startupSightReference(), task.startupPileApproach(), &guide);
        const auto out = task.update(in);
        if (track && out.reason == "startup_approach_120") advanced = true;
        if (dropout) {
            assert(out.reason == "startup_approach_120" && out.motion.vx_mps > 0);
            if (++dropout_advances == 12) {
                // Coherent real displacement must still stop forward motion after
                // the three-image median catches up (at most two new images).
                left.box.x += 200;
                right.box.x += 200;
                PushOutput displaced;
                for (int i = 0; i < 2; ++i) {
                    in.now_us += 27000;
                    left.timestamp_us = right.timestamp_us = in.now_us;
                    applyStartupSight(in, {left,right}, {1280,720}, in.now_us, .5f,
                                      task.startupSightTargetId(), false, -1,
                                      task.startupSightReference(), true, &guide);
                    displaced = task.update(in);
                    in.sent_vx_mps = displaced.motion.vx_mps;
                    in.sent_wz_rps = displaced.motion.wz_rps;
                }
                assert(displaced.reason == "pixel_align_brake" &&
                       displaced.motion.vx_mps == 0 && displaced.immediate_motion_stop);
                std::cout << "PASS UI120 dropouts keep advancing; real displacement still brakes\n";
                return;
            }
        }
        in.camera_pitch_cdeg = out.motion.camera_pitch_cdeg;
        in.sent_vx_mps = out.motion.vx_mps;
        in.sent_wz_rps = out.motion.wz_rps;
    }
    assert(false && "UI120 dropout regression never reached continuous advance");
}

void startupPileGuideUsesOnlyFreshSteeringHistory() {
    StartupPileGuide guide;
    PushObservation in;
    in.camera_pitch_stable = in.pixel_grasp_view = true;
    in.camera_pitch_cdeg = 0;
    uint64_t now = 1000000;
    auto left = object(44, {407,72,152,192}, "injured_person");
    auto right = object(46, {649,87,152,192}, "injured_person");
    const auto step = [&](std::vector<SegDetection> scene, bool advance = true) {
        if (advance) now += 27000;
        for (auto& d : scene) d.timestamp_us = now;
        applyStartupSight(in, scene, {1280,720}, now, .5f, -1, false, -1, {}, true, &guide);
    };
    step({left,right});
    assert(in.target_center_px.x == 604.f);
    step({right});
    assert(in.target_center_px.x == 604.f && in.startup_visible_count == 1);
    assert(in.target_box == right.box); // raw box/counts must not contain retained cargo
    // A tracker re-ID must update the same member without an admission delay.
    left.track_id = 144;
    step({left,right});
    assert(in.target_center_px.x == 604.f);
    // A one-frame box resize is rejected by the centre median.
    auto resized = left;
    resized.box.x -= 180;
    step({resized,right});
    assert(in.target_center_px.x == 604.f);
    step({left,right});
    assert(in.target_center_px.x == 604.f);
    // Three distinct images admit a new extremity; repeated images count once.
    auto added = object(90, {100,100,100,100});
    step({left,right,added});
    step({left,right,added}, false);
    step({left,right,added}, false);
    assert(in.target_center_px.x == 604.f);
    step({left,right,added});
    assert(in.target_center_px.x == 604.f);
    step({left,right,added});
    step({left,right,added});
    assert(in.target_center_px.x >= 574.f && in.target_center_px.x < 604.f);
    // Admission changes the cohort gradually, while raw scene evidence was
    // already available from its very first fresh image.
    for (int i = 0; i < 20; ++i) step({left,right,added});
    assert(in.target_center_px.x == 450.5f);
    // Sustained disappearance expires, rather than preserving the old heading.
    for (int i = 0; i < 8; ++i) step({right});
    assert(in.target_center_px.x == 725.f);
    // New lower-corner evidence is immediate even during member admission.
    left.box = {0,550,120,150};
    added.box = {1100,550,120,150};
    step({left,added});
    assert(in.startup_lower_corners_occupied && in.startup_visible_count == 2);
    step({right});
    assert(!in.startup_lower_corners_occupied && in.startup_visible_count == 1);
    step({});
    assert(!in.target_valid && !in.pixel_grasp_sample_us);
    step({right});
    assert(in.target_center_px.x == 725.f);
    // A view change discards history even when UI120 is revisited immediately.
    step({left,right});
    in.pixel_grasp_near_view = true;
    in.camera_pitch_cdeg = 2000;
    step({right});
    assert(in.target_id == right.track_id);
    in.pixel_grasp_near_view = false;
    in.camera_pitch_cdeg = 0;
    step({right});
    assert(in.target_center_px.x == 725.f);
    // Repeated images cannot refresh a missing member's lifetime.
    guide.reset();
    step({left,right});
    step({right});
    right.timestamp_us = now;
    now += 160000;
    applyStartupSight(in, {right}, {1280,720}, now, .5f, -1, false, -1, {}, true, &guide);
    assert(in.target_center_px.x == 725.f);
    std::cout << "PASS UI120 steering expiry/admission/re-ID; scene evidence stays current\n";
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--startup-pile-dropout") {
        startupPileDropoutDoesNotRestartAlignment();
        startupPileGuideUsesOnlyFreshSteeringHistory(); return 0;
    }
    startupPileDropoutDoesNotRestartAlignment();
    startupPileGuideUsesOnlyFreshSteeringHistory();
    if (argc == 2 && std::string(argv[1]) == "--startup-pile") {
        startupPileCenterRegression(); startupPileHeadingReachesTask(); return 0;
    }

    if (argc == 2 && std::string(argv[1]) == "--startup-object-count") {
        startupTwoObjectsReachTask(); return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--startup-rebind") {
        startupLoggedVerticalRebind(); return 0;
    }
    startupPileCenterRegression();
    startupPileHeadingReachesTask();
    startupLoggedVerticalRebind();
    if (argc == 2 && std::string(argv[1]) == "--startup-stability") {
        startupHandoffKeepsVisibleGuide(); return 0;
    }
    startupHandoffKeepsVisibleGuide();
    rushContinuousIdentity();
    startupSightEvidence();
    startupTwoObjectsReachTask();
    startupLowerCornerEvidence();
    startupLowerCornersReachTask();
    startupTrackerRebind();
    splitRetreatBoxEvidence();
    scatterSelectionLifecycle();
    persistentTargetTests();
    nearOwnZoneExclusion();
    assert(frameAngle(FrameAction::Open) == 0 && frameAngle(FrameAction::Close) == 20);
    PixelSelector selector;
    auto small = object(1, {10, 10, 10, 10}), large = object(2, {30, 10, 30, 30}),
         illegal = object(3, {0, 0, 90, 90}, "dangerous_object");
    PixelSelection s;
    for (int i = 0; i < 3; ++i) {
        small.timestamp_us = large.timestamp_us = 100000 + i * 10000;
        s = selector.update(
            {small, large, illegal}, {100, 100}, 500, true, 100000 + i * 10000, false, .25);
    }
    assert(s.locked && s.id == 2 && s.mode == "box_area" && std::abs(s.raw - .09) < 1e-6);
    s = selector.update({small, large}, {100, 100}, 600, true, 140000, false, .25);
    assert(!s.locked);
    selector.reset();
    large.mask = cv::Mat::zeros(100, 100, CV_8UC1);
    large.mask(cv::Rect(30, 10, 2, 2)).setTo(255);
    small.mask = cv::Mat::zeros(100, 100, CV_8UC1);
    small.mask(small.box).setTo(255);
    for (int i = 0; i < 3; ++i) {
        small.timestamp_us = large.timestamp_us = 150000 + i * 10000;
        s = selector.update({small, large}, {100, 100}, 500, true, 150000 + i * 10000, false, .25);
    }
    assert(s.id == 1 && s.mode == "mask_pixels");
    s = selector.update({small, illegal}, {0, 0}, 500, true, 190000, false, .25);
    assert(!s.locked);
    { // The same physical block re-IDed on every frame still confirms after three frames.
        PixelSelector rid;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            auto b = object(20 + i, {40, 40, 30, 30});
            b.timestamp_us = 900000 + i * 10000;
            p = rid.update({b}, {1000, 1000}, 500, true, 900000 + i * 10000, false, .25);
        }
        assert(p.locked && p.id == 22);
    }
    { // A long disappearance releases the episode; no unbounded nearest-pixel inheritance.
        const auto run = [&](uint64_t window) {
            PixelSelector sel;
            sel.setRelock(window, .10);
            PixelSelection p;
            uint64_t t = 2000000;
            for (int i = 0; i < 4; ++i) {
                auto a = object(40, {100, 300, 40, 40});
                a.timestamp_us = t;
                p = sel.update({a}, {1000, 1000}, 500, true, t, false, .25);
                t += 100000;
            }
            assert(p.locked && p.id == 40);
            for (int i = 0; i < 30; ++i) {
                p = sel.update({}, {1000, 1000}, 500, true, t, false, .25);
                t += 100000;
            }
            for (int i = 0; i < 4; ++i) {
                auto near_ = object(41, {110, 310, 40, 40});
                auto far_ = object(42, {700, 300, 50, 50});
                near_.timestamp_us = far_.timestamp_us = t;
                p = sel.update({near_, far_}, {1000, 1000}, 500, true, t, false, .25);
                t += 100000;
            }
            return p.id;
        };
        assert(run(4000000) == 42 && run(1500000) == 42);
    }
    { // Pile edge: an injured piece on the robot->target line in front of the larger block ranks it
      // behind a clean one; relax levels lift that.
        const auto run = [&](int relax) {
            PixelSelector lane;
            lane.setRelax(relax);
            PixelSelection p;
            for (int i = 0; i < 3; ++i) {
                const uint64_t t = 700000 + i * 10000;
                auto a = object(30, {10, 10, 60, 60});
                a.body_xy_m = {0, .60f};
                auto in_front = object(31, {80, 10, 5, 5}, "injured_person");
                in_front.body_xy_m = {.02f, .40f};
                auto clean = object(32, {10, 80, 40, 40});
                clean.body_xy_m = {.40f, .50f};
                a.timestamp_us = in_front.timestamp_us = clean.timestamp_us = t;
                // Before first delivery the injured block is an obstacle;
                // after first delivery its 15-point load now correctly wins.
                p = lane.update({a, in_front, clean}, {200, 200}, 500, true, t, false, .25);
            }
            return p.id;
        };
        assert(run(0) == 32 && run(1) == 30 && run(2) == 30);
    }
    { // prefer_label restricts the lock to the preferred class when it is visible, else falls back.
        PixelSelector pref;
        auto big = object(5, {10, 10, 60, 60}), inj = object(6, {80, 10, 10, 10}, "injured_person");
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            big.timestamp_us = inj.timestamp_us = 500000 + i * 10000;
            p = pref.update({big, inj},
                            {100, 100},
                            500,
                            true,
                            500000 + i * 10000,
                            true,
                            .25,
                            {},
                            "injured_person");
        }
        assert(p.locked && p.id == 6);
        PixelSelector fallback;
        for (int i = 0; i < 3; ++i) {
            big.timestamp_us = 600000 + i * 10000;
            p = fallback.update(
                {big}, {100, 100}, 500, true, 600000 + i * 10000, true, .25, {}, "injured_person");
        }
        assert(p.locked && p.id == 5);
    }
    { // A tall injured box can acquire and retain a lock, regardless of aspect ratio.
        PixelSelector selector;
        auto tall = object(8, {10, 10, 40, 58}, "injured_person"),
             wide = object(9, {70, 10, 20, 10}, "injured_person");
        PixelSelection p;
        for (int i = 0; i < 8; ++i) {
            tall.timestamp_us = wide.timestamp_us = 700000 + i * 10000;
            p = selector.update({tall, wide}, {100, 100}, 500, true,
                                tall.timestamp_us, true, .25f, {}, "injured_person");
            if (i >= 2) assert(p.locked && p.id == 8);
        }
    }
    { // A larger box without ground contact (image-edge clip) never takes the lock from a
      // measurable one.
        PixelSelector geo;
        auto edge = object(12, {10, 10, 60, 60}, "injured_person"),
             ok = object(15, {10, 80, 30, 15}, "injured_person");
        edge.ground_contact_valid = false;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            edge.timestamp_us = ok.timestamp_us = 900000 + i * 10000;
            p = geo.update({edge, ok},
                           {100, 100},
                           2000,
                           true,
                           900000 + i * 10000,
                           true,
                           .25,
                           {},
                           "injured_person");
        }
        assert(p.locked && p.id == 15);
        // No measurable candidate at all: old behaviour (largest box) is kept.
        PixelSelector none;
        ok.ground_contact_valid = false;
        for (int i = 0; i < 3; ++i) {
            edge.timestamp_us = ok.timestamp_us = 950000 + i * 10000;
            p = none.update({edge, ok},
                            {100, 100},
                            2000,
                            true,
                            950000 + i * 10000,
                            true,
                            .25,
                            {},
                            "injured_person");
        }
        assert(p.locked && p.id == 12);
        // The geometry filter never overrides the preferred class.
        PixelSelector cls;
        auto ord = object(20, {10, 10, 30, 30});
        edge.timestamp_us = ord.timestamp_us = 980000;
        for (int i = 0; i < 3; ++i) {
            edge.timestamp_us = ord.timestamp_us = 980000 + i * 10000;
            p = cls.update({edge, ord},
                           {100, 100},
                           2000,
                           true,
                           980000 + i * 10000,
                           true,
                           .25,
                           {},
                           "injured_person");
        }
        assert(p.locked && p.id == 12);
    }
    { // A fresh acquisition prefers the uncrowded block over a larger one with an injured piece
      // inside its footprint.
        PixelSelector iso;
        auto pile = object(30, {10, 10, 50, 50}),
             inj = object(31, {70, 10, 10, 10}, "injured_person"),
             lone = object(32, {10, 70, 30, 30});
        pile.body_xy_m = {0, .8f};
        inj.body_xy_m = {.05f, .82f};
        lone.body_xy_m = {.5f, .8f};
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            pile.timestamp_us = inj.timestamp_us = lone.timestamp_us = 1000000 + i * 10000;
            p = iso.update(
                {pile, inj, lone}, {100, 100}, 500, true, 1000000 + i * 10000, false, .25);
        }
        assert(p.locked && p.id == 32);
        // A tiny but clean block (below 30% of the best value) is not preferred.
        PixelSelector tiny;
        lone.box = {10, 70, 10, 10};
        for (int i = 0; i < 3; ++i) {
            pile.timestamp_us = inj.timestamp_us = lone.timestamp_us = 1100000 + i * 10000;
            p = tiny.update(
                {pile, inj, lone}, {100, 100}, 500, true, 1100000 + i * 10000, false, .25);
        }
        assert(p.locked && p.id == 30);
    }
    { // A fresh acquisition prefers a legal pair (one ordinary neighbour) over a lone block, but
      // not a trio.
        auto a1 = object(40, {10, 10, 30, 30}), a2 = object(41, {50, 10, 30, 30}),
             lone = object(42, {10, 60, 30, 30}), a3 = object(43, {50, 60, 30, 30});
        a1.body_xy_m = {0, .8f};
        a2.body_xy_m = {.05f, .8f};
        lone.body_xy_m = {.5f, .8f};
        a3.body_xy_m = {.025f, .8f};
        PixelSelection p;
        PixelSelector pr;
        for (int i = 0; i < 3; ++i) {
            a1.timestamp_us = a2.timestamp_us = lone.timestamp_us = 1200000 + i * 10000;
            p = pr.update({lone, a1, a2}, {100, 100}, 500, true, 1200000 + i * 10000, true, .25);
        }
        assert(p.locked && (p.id == 40 || p.id == 41));
        PixelSelector trio;
        for (int i = 0; i < 3; ++i) {
            a1.timestamp_us = a2.timestamp_us = a3.timestamp_us = lone.timestamp_us =
                1300000 + i * 10000;
            p = trio.update(
                {a1, a2, a3, lone}, {100, 100}, 500, true, 1300000 + i * 10000, true, .25);
        }
        assert(p.locked && p.id == 42);
    }
    // A larger challenger must never preempt a valid lock.
    PixelSelector hysteresis;
    small.mask.release();
    large.mask.release();
    small.box = {10, 10, 20, 20};
    large.box = {40, 10, 10, 10};
    auto rank = [&](uint64_t t) {
        small.timestamp_us = large.timestamp_us = t;
        return hysteresis.update({small, large}, {100, 100}, 500, true, t, false, .25);
    };
    rank(300000);
    rank(320000);
    assert(rank(340000).id == 1);
    large.box = {40, 10, 30, 30};
    assert(rank(360000).id == 1);
    assert(rank(380000).id == 1);
    assert(rank(400000).id == 1);
    assert(!hysteresis.update({small, large}, {100, 100}, 500, true, 410000, false, .25)
                .locked);         // repeated captured frame
    assert(!rank(800000).locked); // gap breaks continuity
    { // A short tracker dropout keeps the lock; only a re-IDed object at the old place takes over,
      // not a neighbour.
        PixelSelector g;
        auto big = object(1, {10, 10, 24, 24}), nb = object(2, {60, 10, 20, 20});
        auto step = [&](uint64_t t, std::vector<SegDetection> v) {
            for (auto& x : v)
                x.timestamp_us = t;
            return g.update(v, {100, 100}, 500, true, t, false, .25);
        };
        step(1000000, {big, nb});
        step(1020000, {big, nb});
        assert(step(1040000, {big, nb}).id == 1);
        auto r = step(1060000, {nb});
        assert(!r.locked && r.id == -1 && r.reason == "lock_grace_waiting");
        r = step(1080000, {nb});
        r = step(1100000, {nb});
        assert(!r.locked && r.reason == "lock_grace_waiting"); // neighbour never inherits the lock
        assert(step(1120000, {big, nb}).id ==
               1); // original returns within grace: still locked, no re-confirmation
        auto re = big;
        re.track_id = 9;
        step(1140000, {re, nb});
        step(1160000, {re, nb});
        assert(step(1180000, {re, nb}).id == 9); // re-IDed cube relocked by position
    }
    { // Alternating tracker IDs must keep the pending target by image position.
        PixelSelector alternating;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            const uint64_t t = 1200000 + i * 10000;
            auto target = object(10 + i, i == 0 ? cv::Rect{20, 30, 30, 30} : cv::Rect{29, 39, 12, 12});
            auto neighbour = object(20, {70, 30, i == 1 ? 32 : 10, i == 1 ? 32 : 10});
            target.timestamp_us = neighbour.timestamp_us = t;
            p = alternating.update(
                {target, neighbour}, {100, 100}, 500, true, t, false, .25);
        }
        assert(p.locked && p.id == 12);
    }
    FrameView a{500, {{5, 5}, {95, 5}, {95, 95}, {5, 95}}, true, true}, b = a;
    b.pitch_cdeg = 2000;
    MultiViewCapture multi({a, b}, {100, 100});
    uint64_t now = 1000000;
    auto d = object(1, {25, 25, 10, 10});
    auto sample = [&](bool has = true) {
        now += 20000;
        d.timestamp_us = now;
        auto blue_zone = object(100, {0, 0, 100, 100}, "blue_safe_zone");
        auto red_zone = object(101, {20, 20, 30, 30}, "red_safe_zone");
        blue_zone.timestamp_us = red_zone.timestamp_us = now;
        multi.update(now,
                     now,
                     7,
                     true,
                     true,
                     multi.desiredPitch(),
                     true,
                     has ? std::vector<SegDetection>{d, blue_zone, red_zone}
                         : std::vector<SegDetection>{blue_zone, red_zone});
    };
    multi.begin(now, 7, 1);
    for (int i = 0; i < 4; ++i)
        sample();
    assert(!multi.result().finished);
    sample();
    assert(multi.result().view == 1);
    d.track_id = 99; // ids need not survive view change
    for (int i = 0; i < 5; ++i) {
        sample();
    }
    assert(multi.result().verdict == CaptureVerdict::Enclosed &&
           multi.result().inventory.total() == 1);
    multi.begin(now, 7, 2);
    for (int i = 0; i < 5; ++i)
        sample();
    d.label = "core_supply";
    for (int i = 0; i < 5; ++i) {
        sample();
    }
    assert(multi.result().finished && multi.result().verdict == CaptureVerdict::Uncertain);
    multi.begin(now, 7, 3);
    for (int i = 0; i < 10; ++i)
        sample(false);
    assert(multi.result().verdict == CaptureVerdict::Empty);
    multi.begin(now, 7, 4);
    multi.update(now + 1, now + 1, 8, true, true, 500, true, {});
    assert(multi.result().finished && multi.result().verdict == CaptureVerdict::Uncertain);
    multi.begin(now, 7, 5);
    d = object(4, {90, 30, 10, 10});
    for (int i = 0; i < 10; ++i)
        sample();
    assert(multi.result().verdict == CaptureVerdict::Uncertain);
    multi.update(now + 8000001, now + 8000001, 7, true, true, 500, true, {});
    assert(multi.result().finished);
    multi.begin(now, 7, 6);
    multi.update(now + 1, now + 1, 7, true, false, 500, true, {});
    assert(multi.result().finished);
    // A stale/duplicate frame cannot satisfy the minimum five independent frames.
    multi.begin(now, 7, 7);
    for (int i = 0; i < 10; ++i)
        multi.update(now + 1, now + 1, 7, true, true, 500, true, {});
    assert(multi.result().frames == 1);
    { // image_polygon: no ground ranging; per-view pixel continuity, cross-view class counts only.
        MultiViewCapture img({a, b}, {100, 100}, true);
        uint64_t t = 2000000;
        auto g = object(1, {25, 25, 10, 10});
        g.ground_position_valid = g.ground_contact_valid = false;
        g.body_xy_m = {NAN, NAN};
        auto feed = [&](std::vector<SegDetection> ds) {
            t += 20000;
            for (auto& x : ds)
                x.timestamp_us = t;
            img.update(t, t, 9, true, true, img.desiredPitch(), true, ds);
        };
        img.begin(t, 9, 1);
        for (int i = 0; i < 5; ++i)
            feed({g});
        assert(img.result().view == 1);
        g.box = {50, 60, 12, 12}; // different pixel position at the second pitch is expected
        for (int i = 0; i < 5; ++i)
            feed({g});
        assert(img.result().verdict == CaptureVerdict::Enclosed &&
               img.result().inventory.total() == 1);
        img.begin(t, 9, 2);
        for (int i = 0; i < 5; ++i)
            feed({g});
        auto h = g;
        h.label = "core_supply";
        for (int i = 0; i < 5; ++i)
            feed({h});
        assert(img.result().finished && img.result().verdict == CaptureVerdict::Uncertain);
        img.begin(t, 9, 3);
        for (int i = 0; i < 10; ++i)
            feed({});
        assert(img.result().verdict == CaptureVerdict::Empty);
        // Jumping pixel position within one view restarts the five-frame count.
        img.begin(t, 9, 4);
        auto j = g;
        for (int i = 0; i < 4; ++i) {
            j.box.x = i % 2 ? 20 : 60;
            feed({j});
        }
        assert(img.result().view == 0 && img.result().frames == 1);
        // Boundary-crossing box stays uncertain even without ground requirements.
        img.begin(t, 9, 5);
        auto e = object(2, {90, 30, 10, 10});
        for (int i = 0; i < 10; ++i)
            feed({e});
        assert(img.result().verdict == CaptureVerdict::Uncertain);
        // Body-ground mode still rejects detections lacking ground validity.
        MultiViewCapture body({a, b}, {100, 100});
        body.begin(t, 9, 6);
        for (int i = 0; i < 10; ++i) {
            t += 20000;
            g.timestamp_us = t;
            body.update(t, t, 9, true, true, body.desiredPitch(), true, {g});
        }
        assert(body.result().verdict != CaptureVerdict::Enclosed);
    }
    std::cout << "mechanical integration checks passed\n";
}
