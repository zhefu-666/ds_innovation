#include <array>
#include <tuple>
#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_dead_reckoning.hpp"
#include "rescue/planner.hpp"
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <set>
#include <vector>
#include "rescue/task/capture_verification_stage.hpp"
#include "rescue/task/delivery_verification_stage.hpp"
#include "rescue/task/release_retreat_stage.hpp"
#define private public // test-only: seed first_ so an injured target is selectable
#if __has_include("rescue/task/disperse_rush_stage.hpp")
#define RESCUE_HAS_RUSH_STAGE 1
#endif
#include "rescue/push_task.hpp"
#undef private
#include "rescue/capture_monitor.hpp"
#include "rescue/config.hpp"
#include "rescue/motion_readiness.hpp"
#include "rescue/motion_watch.hpp"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <iostream>
using namespace rescue;
bool stopped(const MotionCommand &m) { return m.vx_mps == 0 && m.wz_rps == 0; }
Inventory inv(int ordinary, int core = 0, int injured = 0, int dangerous = 0, int unknown = 0) {
    Inventory i; i.ordinary = ordinary; i.core = core; i.injured = injured; i.dangerous = dangerous; i.unknown = unknown;
    return i;
}
// Kinematic world: robot pose in our zone frame (x right facing the entry, y into the zone).
// The previous command is integrated before every update; gripper feedback lags one frame.
// The camera servo follows the commanded pitch at pitch_rate per frame and reads back stable
// after three still frames; the closed frame is in view only at NEAR (like CaptureMonitor).
static bool g_non_supply_sweep = false; // only the dedicated tests exercise the sweep of non-supply targets
struct Sim {
    TaskTuning tune;
    PushTask task{tune};
    PushObservation in;
    PushOutput out;
    float px = -.15f, py = -.75f, phi = 0; // phi: robot heading vs zone +y, CCW positive
    bool zone_visible = true, zone_predicted = false, gripper_responds = true, servo_responds = true;
    bool multi_evidence=true, all_ordinary=true;
    float py_max=1e9f; bool vref_visible=false, cargo_on=false, cargo_clip=false; float img_frac=std::nanf(""); cv::Point2f cargo_zone{-.15f,.12f}; bool cargo2_on=false; cv::Point2f cargo_zone2{-.15f,.30f};
    bool navigation_ready = true, drop_available = true, live_camera_guard = false;
    int pitch = 0, pitch_still = 0, pitch_rate = 400; // power-on: level
    Inventory load; // what is physically enclosed
    std::vector<PushObservation::HeldObject> load_objects; // detections behind `load` (identity/confidence)
    std::set<PushState> visited;
    static TaskTuning testTune(TaskTuning t) { t.startup_advance_us = 0; t.require_multi_view=false; t.sweep_non_supply = g_non_supply_sweep; t.rush_stall_us = 0; /* sim targets do not close; stall tests re-enable it */ return t; }
    explicit Sim(TaskTuning t = {}) : tune(testTune(t)), task(tune) {
        in.run = in.safety_ok = in.target_valid = in.geometry_valid = true;
        in.target_region_valid = in.zone_identity_verified = true;
        in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.target_id = 7; in.label = "ordinary_supply"; in.distance_m = .2f;
        in.corridor = inv(1); in.corridor_complete = in.corridor_occlusion_free = true;
        in.zone_valid = in.zone_own = true; in.zone_class = "supply";
        in.zone_inventory_complete = in.zone_counts_valid = true; in.heading_valid = true;
    }
    PushOutput tick() {
        const float dt = .05f;
        phi += out.motion.wz_rps * dt;
        px += -std::sin(phi) * out.motion.vx_mps * dt;
        py += std::cos(phi) * out.motion.vx_mps * dt;
        if (py > py_max) py = py_max;
        in.now_us += 50000;
        auto &z = in.zone_estimate;
        z = ZoneEstimate{};
        if (zone_visible) {
            z.valid = true; z.source = zone_predicted ? ZoneEstimate::Source::PREDICTED : ZoneEstimate::Source::MULTI_POINT;
            z.frame_id = in.now_us / 50000; z.timestamp_us = z.observed_us = in.now_us;
            z.zone_label = "red_safe_zone"; z.geometry_id = "rescue2027-inner-v1-red";
            z.inlier_ids = {0, 2, 3, 5}; z.residual_m = .001f; z.position_sigma_m = .002f; z.yaw_sigma_rad = .01f;
            z.predicted_distance_m = 0;
            // p_body = R(-phi)(p_zone - r)  =>  yaw_body = -phi, origin = R(-phi)(-r)
            z.yaw_body_rad = -phi;
            const float c = std::cos(-phi), s = std::sin(-phi);
            z.origin_body_m = {c * -px - s * -py, s * -px + c * -py};
        }
        in.vref_valid=false;
        if (vref_visible) { // coarse visual reference only (assume mode); same geometry as the strict estimate
            auto &v = in.vref_zone; v = ZoneEstimate{};
            v.valid = true; v.timestamp_us = in.now_us; v.yaw_body_rad = -phi;
            const float c = std::cos(-phi), s = std::sin(-phi);
            v.origin_body_m = {c * -px - s * -py, s * -px + c * -py};
            in.vref_valid = true; in.vref_points = 4;
        }
        in.cargo_body.clear(); in.cargo_img.clear(); in.cargo_ts_us=in.now_us;
        if(cargo_on){const float c=std::cos(-phi),sn=std::sin(-phi),dx=cargo_zone.x-px,dy=cargo_zone.y-py;const cv::Point2f cb{c*dx-sn*dy,sn*dx+c*dy};if(cb.y>0){in.cargo_body.push_back(cb);PushObservation::CargoImg ci;ci.bottom_px=400;ci.height_px=100;ci.clipped=cargo_clip;if(std::isfinite(img_frac)){ci.front_px=400+img_frac*100;ci.front_valid=true;}in.cargo_img.push_back(ci);}}
        if(cargo2_on){const float c=std::cos(-phi),sn=std::sin(-phi),dx=cargo_zone2.x-px,dy=cargo_zone2.y-py;const cv::Point2f cb{c*dx-sn*dy,sn*dx+c*dy};if(cb.y>0){in.cargo_body.push_back(cb);PushObservation::CargoImg ci;ci.bottom_px=400;ci.height_px=100;ci.clipped=cargo_clip;if(std::isfinite(img_frac)){ci.front_px=400+img_frac*100;ci.front_valid=true;}in.cargo_img.push_back(ci);}}
        // Simulator supplies explicit validated direct-route/empty-zone evidence.
        in.carry_plan_valid=navigation_ready;in.drop_plan_valid=drop_available;in.navigation_timestamp_us=in.now_us;
        in.drop_centre_zone={task_injured?tune.injured_half_x_m:tune.supply_half_x_m,tune.deposit_y_m};
        const float centre=task_injured?tune.injured_hold_center_y_m:tune.hold_center_y_m;
        in.carry_waypoint_body=z.zoneToBody({in.drop_centre_zone.x,-tune.gate_clearance_m-centre});
        in.heading_rad = phi;
        const int target = out.motion.camera_pitch_cdeg;
        const int moved = servo_responds ? std::clamp(target - pitch, -pitch_rate, pitch_rate) : 0;
        pitch += moved; pitch_still = moved ? 0 : pitch_still + 1;
        in.camera_pitch_cdeg = int16_t(pitch); in.camera_pitch_stable = pitch_still >= 3;
        in.hold_observable = in.camera_pitch_stable && std::abs(pitch - tune.near_pitch_cdeg) <= 100;
        in.held = in.hold_observable ? load : Inventory{};
        in.held_objects = in.hold_observable ? load_objects : std::vector<PushObservation::HeldObject>{};
        in.captured = in.held_complete = in.held.total() > 0;
        in.gripper_closed_observed=gripper_responds && out.motion.gripper_offset==20;
        in.multi_view_finished=multi_evidence && out.state==PushState::VERIFY_CAPTURE && in.captured;
        in.multi_view_verdict=in.multi_view_finished?1:0;
        in.multi_view_inventory=in.held;
        in.gripper_done = gripper_responds;
        in.gripper_feedback_open = gripper_responds ? (out.motion.gripper_offset == 0 ? 1 : 0) : -1;
        const bool mapped = in.camera_pitch_stable && pitch==tune.far_pitch_cdeg;
        if(live_camera_guard) {
            const std::string reason=in.camera_pitch_stable?"pitch_not_calibrated":"pitch_moving";
            if(!(mapped || stationaryPitchWork(out,reason))) std::cerr<<"guard state="<<PushTask::name(out.state)<<" vx="<<out.motion.vx_mps<<" wz="<<out.motion.wz_rps<<" cmd="<<out.motion.camera_pitch_cdeg<<" actual="<<pitch<<" stable="<<in.camera_pitch_stable<<"\n";
            assert(mapped || stationaryPitchWork(out,reason));
        }
        if(all_ordinary)in.clutter_ordinary=in.clutter_body;
        out = task.update(in);
        if(live_camera_guard) {
            inhibitUnmappedMotion(out.motion,mapped);
            if(!mapped)assert(stopped(out.motion));
        }
        commanded.insert(out.motion.camera_pitch_cdeg);
        if (record) frames.push_back(in);
        visited.insert(out.state);
        return out;
    }
    bool until(PushState s, int limit) {
        for (int i = 0; i < limit; ++i) if (tick().state == s) return true;
        return false;
    }
    // Lock, open, inventory the corridor and start the rush.
    void toRush() { assert(until(PushState::RUSH, 80)); }
    void hold(const Inventory &held) { load = held; }
    void toCarry(const Inventory &held = inv(1)) {
        toRush(); hold(held); assert(until(PushState::CARRY, 40));
    }
    // Whole trip with the zone counts reacting once the load is pushed in.
    void deliver(bool counts_react = true) {
        assert(until(PushState::ENTER, 400));
        in.delivery_observed = counts_react;
        assert(until(PushState::BACK_OUT, 400));
        if (counts_react) {
            if (out.batch_size && task_injured) in.zone_injured_count += out.batch_size;
            else in.zone_supply_count += out.batch_size;
        }
        hold({}); // the load stays behind
        assert(until(PushState::VERIFY_DELIVERY, 400));
        assert(until(PushState::TURN_SCAN, 100));
    }
    bool task_injured = false;
    bool record = false;
    std::set<int> commanded;
    std::vector<PushObservation> frames;
};
// Regenerates tests/fixtures/push_delivery.json from the closed-loop simulator, so the
// replay inputs (zone pose, gripper acknowledgements, heading) match the task's own commands.
void writeFixture(const std::string &path) {
    Sim s; s.record = true;
    s.toCarry(); s.deliver();
    s.in.target_valid = false; s.in.target_id = -1; s.in.corridor = {}; // nothing left in view
    assert(s.until(PushState::SCAN, 400));
    for (int i = 0; i < 3; ++i) s.tick();
    assert(s.out.state == PushState::SCAN && s.out.delivered_total == 1 && s.out.first_ordinary_delivered);
    cv::FileStorage f(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    assert(f.isOpened());
    const auto inventory = [&](const char *key, const Inventory &i) {
        f << key << "{" << "ordinary" << i.ordinary << "core" << i.core << "injured" << i.injured
          << "dangerous" << i.dangerous << "unknown" << i.unknown << "}";
    };
    f << "frames" << "[";
    for (const auto &in : s.frames) {
        f << "{" << "now_us" << double(in.now_us) << "run" << int(in.run) << "safety_ok" << int(in.safety_ok)
          << "carry_plan_valid" << int(in.carry_plan_valid) << "drop_plan_valid" << int(in.drop_plan_valid)
          << "navigation_timestamp_us" << double(in.navigation_timestamp_us)
          << "carry_waypoint_body" << "[" << in.carry_waypoint_body.x << in.carry_waypoint_body.y << "]"
          << "drop_centre_zone" << "[" << in.drop_centre_zone.x << in.drop_centre_zone.y << "]"
          << "target_region_valid" << int(in.target_region_valid) << "zone_identity_verified" << int(in.zone_identity_verified)
          << "delivery_observed" << int(in.delivery_observed)
          << "target_valid" << int(in.target_valid) << "geometry_valid" << int(in.geometry_valid)
          << "path_safe" << int(in.path_safe) << "retreat_safe" << int(in.retreat_safe)
          << "opponent_zone_clear" << int(in.opponent_zone_clear)
          << "target_id" << in.target_id << "label" << in.label
          << "distance_m" << double(in.distance_m) << "heading_error" << double(in.heading_error);
        inventory("corridor", in.corridor);
        f << "corridor_complete" << int(in.corridor_complete) << "corridor_occlusion_free" << int(in.corridor_occlusion_free)
          << "hold_observable" << int(in.hold_observable)
          << "captured" << int(in.captured) << "held_complete" << int(in.held_complete)
          << "camera_pitch_cdeg" << int(in.camera_pitch_cdeg) << "camera_pitch_stable" << int(in.camera_pitch_stable);
        inventory("held", in.held);
        f << "gripper_done" << int(in.gripper_done) << "gripper_feedback_open" << in.gripper_feedback_open
          << "zone_valid" << int(in.zone_valid) << "zone_own" << int(in.zone_own) << "zone_class" << in.zone_class
          << "zone_inventory_complete" << int(in.zone_inventory_complete)
          << "zone_counts_valid" << int(in.zone_counts_valid)
          << "zone_supply_count" << in.zone_supply_count << "zone_injured_count" << in.zone_injured_count
          << "heading_valid" << int(in.heading_valid) << "heading_rad" << double(in.heading_rad);
        const auto &z = in.zone_estimate;
        f << "zone_estimate" << "{" << "valid" << int(z.valid) << "source" << "multi_point"
          << "zone_label" << z.zone_label << "geometry_id" << z.geometry_id
          << "frame_id" << double(z.frame_id) << "timestamp_us" << double(z.timestamp_us)
          << "observed_us" << double(z.observed_us)
          << "origin_body_m" << "[" << double(z.origin_body_m.x) << double(z.origin_body_m.y) << "]"
          << "yaw_body_rad" << double(z.yaw_body_rad) << "inlier_ids" << "[";
        for (int id : z.inlier_ids) f << id;
        f << "]" << "residual_m" << double(z.residual_m) << "position_sigma_m" << double(z.position_sigma_m)
          << "yaw_sigma_rad" << double(z.yaw_sigma_rad) << "predicted_distance_m" << double(z.predicted_distance_m)
          << "}" << "}";
    }
    f << "]";
    std::cout << "wrote " << s.frames.size() << " frames to " << path << "\n";
}
void rulesTests() {
    assert(targetKind("ordinary_supply") == TargetKind::ORDINARY && targetKind("core_supply") == TargetKind::CORE);
    assert(targetKind("injured_person") == TargetKind::INJURED && targetKind("dangerous_object") == TargetKind::DANGEROUS);
    assert(targetKind("") == TargetKind::UNKNOWN && targetKind("normal") == TargetKind::UNKNOWN);
    assert(targetSelectable("ordinary_supply", false) && !targetSelectable("core_supply", false));
    assert(!targetSelectable("injured_person", false) && targetSelectable("injured_person", true));
    assert(targetSelectable("core_supply", true) && !targetSelectable("dangerous_object", true));
    assert(!targetSelectable("unknown", true));
    Inventory a; a.add("ordinary_supply"); a.add("core_supply"); a.add("weird");
    assert(a.ordinary == 1 && a.core == 1 && a.unknown == 1 && a.supplies() == 2 && a.total() == 3);
    assert(checkTrip({}, true) == RuleVerdict::EMPTY);
    assert(checkTrip(inv(1), false) == RuleVerdict::OK && checkTrip(inv(2), false) == RuleVerdict::OK);
    assert(checkTrip(inv(3), false) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(4), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(2, 2), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(1, 1), true) == RuleVerdict::OK);
    assert(checkTrip(inv(2, 1), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(1, 1), false) == RuleVerdict::CORE_BEFORE_FIRST);
    assert(checkTrip(inv(0, 0, 1), false) == RuleVerdict::INJURED_BEFORE_FIRST);
    assert(checkTrip(inv(0, 0, 1), true) == RuleVerdict::OK);
    assert(checkTrip(inv(1, 0, 1), true) == RuleVerdict::INJURED_NOT_ALONE);
    assert(checkTrip(inv(0, 0, 2), true) == RuleVerdict::INJURED_NOT_ALONE);
    assert(checkTrip(inv(1, 0, 0, 1), true) == RuleVerdict::DANGEROUS);
    assert(checkTrip(inv(1, 0, 0, 0, 1), true) == RuleVerdict::UNKNOWN_OBJECT);
    // Corridor: the primary must be inside, and before the first delivery (or for an
    // injured person) nothing may be hidden behind it.
    assert(checkCorridor(inv(1), false, true, "ordinary_supply", false) == RuleVerdict::INCOMPLETE);
    assert(checkCorridor(inv(0, 1), true, true, "ordinary_supply", true) == RuleVerdict::INCOMPLETE);
    assert(checkCorridor(inv(1), true, false, "ordinary_supply", false) == RuleVerdict::OCCLUDED);
    assert(checkCorridor(inv(1, 1), true, false, "core_supply", true) == RuleVerdict::OK);
    assert(checkCorridor(inv(0, 0, 1), true, false, "injured_person", true) == RuleVerdict::OCCLUDED);
    assert(checkCorridor(inv(1, 1), true, true, "ordinary_supply", false) == RuleVerdict::CORE_BEFORE_FIRST);
    assert(std::string(requiredHalf(inv(0, 0, 1))) == "injured" && std::string(requiredHalf(inv(2, 1))) == "supply");
    assert(std::string(verdictName(RuleVerdict::TOO_MANY_SUPPLIES)) == "too_many_supplies");
}
void captureTests() {
    CaptureConfig old_view;old_view.holding={{3500,{540,570,820,690}}};
    CaptureMonitor monitor(old_view);
    PushObservation in;
    in.camera_pitch_cdeg = 3500; in.camera_pitch_stable = true; // NEAR, settled
    const auto box = [](int id, const char *label, cv::Rect r, uint64_t t) {
        SegDetection d; d.track_id = id; d.label = label; d.confidence = .9f; d.box = r; d.timestamp_us = t; return d;
    };
    uint64_t t = 1000000;
    // Held only after the box stays inside the closed frame for follow_frames.
    for (int i = 0; i < 2; ++i) { t += 50000; monitor.update(in, {box(1, "ordinary_supply", {600, 600, 40, 40}, t)}, t); assert(!in.captured); }
    t += 50000; monitor.update(in, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
    assert(in.hold_observable && in.captured && in.held_complete && in.held == inv(1));
    // FAR, or a camera still moving: the region is out of view, nothing is claimed either way.
    auto far = in; far.camera_pitch_cdeg = 1200; CaptureMonitor other(old_view);
    for (int i = 0; i < 4; ++i) { other.update(far, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t); assert(!far.hold_observable && !far.captured); }
    auto moving = in; moving.camera_pitch_stable = false;
    other.update(moving, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
    assert(!moving.hold_observable && !moving.captured);
    // A pitch change restarts the follow history: positions at another pitch are not comparable.
    t += 50000; far.camera_pitch_cdeg = 3500; other.update(far, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
    assert(far.hold_observable && !far.captured);
    // A box straddling the frame edge makes the held set incomplete.
    t += 50000;
    monitor.update(in, {box(1, "ordinary_supply", {602, 600, 40, 40}, t), box(2, "core_supply", {800, 600, 60, 40}, t)}, t);
    assert(!in.held_complete);
    // Stale or low-confidence boxes are never held as known targets.
    t += 50000; monitor.update(in, {box(1, "ordinary_supply", {602, 600, 40, 40}, t - 400000)}, t);
    assert(!in.captured && in.held.total() == 0);
    // Corridor: objects in the swept width are inventoried; one without ground position spoils completeness.
    CaptureMonitor corridor(old_view);
    in = {}; in.target_valid = in.geometry_valid = true; in.distance_m = .3f; in.heading_error = 0;
    in.camera_pitch_cdeg = 3500; in.camera_pitch_stable = true;
    auto a = box(5, "ordinary_supply", {600, 300, 40, 40}, t); a.ground_position_valid = true; a.body_xy_m = {0, .3f};
    auto b = box(6, "core_supply", {300, 300, 40, 40}, t); b.ground_position_valid = true; b.body_xy_m = {-.4f, .3f};
    corridor.update(in, {a, b}, t);
    assert(in.corridor_complete && in.corridor_occlusion_free && in.corridor == inv(1));
    auto hidden = box(7, "core_supply", {610, 310, 40, 40}, t); hidden.ground_position_valid = true; hidden.body_xy_m = {.02f, .35f};
    corridor.update(in, {a, hidden}, t);
    assert(in.corridor_complete && !in.corridor_occlusion_free && in.corridor == inv(1, 1));
    // An overlapping box whose ground position is clearly beside the corridor is a neighbour, not occlusion.
    auto neighbour = box(9, "core_supply", {610, 310, 40, 40}, t); neighbour.ground_position_valid = true; neighbour.body_xy_m = {.20f, .3f};
    corridor.update(in, {a, neighbour}, t);
    assert(in.corridor_complete && in.corridor_occlusion_free && in.corridor == inv(1));
    // Two overlapping ordinary supplies in the corridor are a legal pair, not occlusion; a core behind one still is.
    auto pair2 = box(10, "ordinary_supply", {615, 305, 40, 40}, t); pair2.ground_position_valid = true; pair2.body_xy_m = {.03f, .4f};
    corridor.update(in, {a, pair2}, t);
    assert(in.corridor_complete && in.corridor_occlusion_free && in.corridor == inv(2));
    corridor.update(in, {a, pair2, hidden}, t);
    assert(!in.corridor_occlusion_free);
    // An ordinary block cut by the image bottom no longer spoils the corridor; a core cut by it does.
    auto cut = box(11, "ordinary_supply", {600, 690, 40, 40}, t); cut.ground_position_valid = true; cut.body_xy_m = {0, .3f};
    corridor.update(in, {cut}, t);
    assert(in.corridor_occlusion_free && in.corridor == inv(1));
    cut.label = "core_supply"; corridor.update(in, {cut}, t);
    assert(!in.corridor_occlusion_free);
    auto floating = box(8, "ordinary_supply", {900, 300, 40, 40}, t);
    corridor.update(in, {a, floating}, t);
    assert(!in.corridor_complete && !in.corridor_occlusion_free);
    in.camera_pitch_stable = false; corridor.update(in, {a}, t);
    assert(!in.corridor_complete && in.corridor.total() == 0); // no inventory while the camera moves
}
void holding40StaticReviewTests() {
    CaptureConfig config;
    config.holding = {{4000, {440, 210, 930, 720}, 440}};
    const auto detection = [](int id, const char* label, float confidence, cv::Rect rect, uint64_t when) {
        SegDetection d; d.track_id=id; d.label=label; d.confidence=confidence; d.box=rect; d.timestamp_us=when;
        return d;
    };
    const auto check = [&](const std::vector<SegDetection>& sample, const Inventory& expected) {
        CaptureMonitor monitor(config);
        PushObservation in; in.camera_pitch_cdeg=4000; in.camera_pitch_stable=true;
        for(int frame=0;frame<3;++frame) {
            const uint64_t now=1000000+frame*50000;
            auto boxes=sample;
            for(auto& d:boxes)d.timestamp_us=now;
            monitor.update(in,boxes,now);
        }
        assert(in.hold_observable && in.held==expected);
        assert(in.captured==(expected.total()>0));
        if(expected.total()>0)assert(in.held_complete);
    };
    constexpr uint64_t now=1000000;
    check({detection(1,"ordinary_supply",.811f,{589,22,171,210},now)},{});
    check({detection(1,"ordinary_supply",.875f,{614,255,284,306},now),
           detection(2,"core_supply",.176f,{383,110,55,73},now)},inv(1));
    check({detection(1,"ordinary_supply",.854f,{463,399,312,304},now),
           detection(2,"ordinary_supply",.785f,{594,228,254,252},now)},inv(2));
    check({detection(1,"injured_person",.573f,{622,255,290,465},now)},inv(0,0,1));
    check({detection(1,"ordinary_supply",.711f,{545,104,203,180},now),
           detection(2,"ordinary_supply",.429f,{591,274,150,82},now)},{});
    check({detection(1,"ordinary_supply",.862f,{597,141,218,258},now),
           detection(2,"core_supply",.592f,{321,101,104,111},now)},{});
}
// Field 2026-10-05 07:52: one block held at y=0.158 m while blocks at y~0.29-0.31 m crossed the
// top of the 40deg region; those must not make the held set ambiguous.
void holdingOutsideMouthTests() {
    CaptureConfig config;
    config.holding = {{4000, {440, 210, 930, 720}, 440}};
    config.mouth_y_m = .22f;
    const auto det = [](int id, const char* label, cv::Rect r, bool contact, float y) {
        SegDetection d; d.track_id=id; d.label=label; d.confidence=.9f; d.box=r;
        d.ground_contact_valid=d.ground_position_valid=contact; d.body_xy_m={0,y}; return d;
    };
    const auto run = [&](std::vector<SegDetection> boxes, PushObservation& in, CaptureMonitor& m) {
        in.camera_pitch_cdeg=4000; in.camera_pitch_stable=true;
        for(int frame=0;frame<3;++frame) {
            const uint64_t now=1000000+frame*50000;
            for(auto& d:boxes)d.timestamp_us=now;
            m.update(in,boxes,now);
        }
    };
    const auto held = det(8,"ordinary_supply",{560,330,320,390},true,.158f);
    { // In front of the mouth with a measured contact: ignored, the held block is complete.
        CaptureMonitor m(config); PushObservation in;
        run({held,det(13,"ordinary_supply",{580,40,200,230},true,.309f),det(12,"core_supply",{430,0,200,215},true,.347f)},in,m);
        assert(in.hold_observable && in.captured && in.held_complete && in.held==inv(1));
        assert(m.diagnostics().outside.size()==2 && m.diagnostics().ambiguous.empty());
    }
    { // The same box without a measured contact stays ambiguous.
        CaptureMonitor m(config); PushObservation in;
        run({held,det(13,"ordinary_supply",{580,40,200,230},false,.309f)},in,m);
        assert(in.captured && !in.held_complete);
        assert(m.diagnostics().ambiguous.size()==1 && m.diagnostics().ambiguous[0]==13);
    }
    { // Within the margin beyond the mouth: still treated conservatively.
        CaptureMonitor m(config); PushObservation in;
        run({held,det(13,"ordinary_supply",{580,40,200,230},true,.24f)},in,m);
        assert(!in.held_complete);
    }
    { // Uncalibrated mouth (default): old behaviour.
        CaptureConfig old=config; old.mouth_y_m=std::numeric_limits<float>::infinity();
        CaptureMonitor m(old); PushObservation in;
        run({held,det(13,"ordinary_supply",{580,40,200,230},true,.309f)},in,m);
        assert(!in.held_complete);
    }
}
// Field 2026-10-12 01:31: a dangerous block (across 9.5 cm) and an injured person (across -9.8 cm) sat just outside a 9 cm
// centre-point corridor and were found only during RUSH. Values below are the logged body_xy_m of the two failures
// (synthetic frames around logged coordinates, not a full log replay).
void corridorFootprintTests() {
    const auto make = [](int id, const char *label, float conf, float x, float y, uint64_t t, cv::Rect r) {
        SegDetection d; d.track_id=id; d.label=label; d.confidence=conf; d.box=r; d.timestamp_us=t;
        d.ground_contact_valid=d.ground_position_valid=true; d.body_xy_m={x,y}; return d;
    };
    const uint64_t t=2000000;
    const auto observe = [&](CaptureConfig cfg, std::vector<SegDetection> boxes, float dist, float head, PushObservation& in) {
        CaptureMonitor m(cfg);
        in = {}; in.target_valid=in.geometry_valid=true; in.distance_m=dist; in.heading_error=head;
        in.camera_pitch_cdeg=3500; in.camera_pitch_stable=true;
        m.update(in, boxes, t);
        return m.corridorEntries();
    };
    CaptureConfig field; field.corridor_half_width_m=.09f; // frame_20261008.assume.json: 0.15/2 + 0.015
    CaptureConfig centre_only=field; centre_only.footprint_ordinary_m=centre_only.footprint_dangerous_m=centre_only.footprint_core_m=
        centre_only.footprint_injured_m=centre_only.footprint_unknown_m=0.f;
    const auto target1 = make(190,"ordinary_supply",.9f,-.0039f,.2659f,t,{600,400,40,40});
    const auto danger = make(177,"dangerous_object",.85f,.0909f,.2821f,t,{700,380,40,40});
    PushObservation in;
    { // centre-point rule (old behaviour) misses the dangerous block, the footprint rule counts it
        observe(centre_only,{target1,danger},.2659f,.0146f,in);
        assert(in.corridor==inv(1));
        assert(checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,"ordinary_supply",false)==RuleVerdict::OK);
        const auto entries=observe(field,{target1,danger},.2659f,.0146f,in);
        assert(in.corridor==inv(1,0,0,1));
        assert(checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,"ordinary_supply",false)==RuleVerdict::DANGEROUS);
        bool saw=false;
        for(const auto&e:entries) if(e.track_id==177){ saw=true; assert(e.counted && e.footprint_only && std::string(e.reason)=="counted" && e.across_m>.09f); }
        assert(saw);
    }
    { // injured person left of the target
        const auto target2 = make(286,"ordinary_supply",.9f,-.0037f,.2647f,t,{600,400,40,40});
        const auto injured = make(298,"injured_person",.785f,-.1019f,.2767f,t,{480,380,80,40});
        observe(centre_only,{target2,injured},.2647f,.0132f,in); assert(in.corridor==inv(1));
        observe(field,{target2,injured},.2647f,.0132f,in);
        assert(in.corridor==inv(1,0,1));
        assert(checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,"ordinary_supply",false)==RuleVerdict::INJURED_BEFORE_FIRST);
    }
    { // a legal pair of ordinary supplies, one only reached by its footprint, still passes
        const auto pair2 = make(191,"ordinary_supply",.9f,.0999f,.2759f,t,{700,380,40,40});
        observe(field,{target1,pair2},.2659f,.0146f,in);
        assert(in.corridor==inv(2));
        assert(checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,"ordinary_supply",true)==RuleVerdict::OK);
    }
    { // clearly outside, behind, stale or low-confidence neighbours are not counted (and say why)
        const auto far_side = make(201,"dangerous_object",.9f,.20f,.28f,t,{900,380,40,40});
        const auto behind = make(202,"dangerous_object",.9f,0.f,-.10f,t,{600,600,40,40});
        const auto stale = make(203,"dangerous_object",.9f,.05f,.28f,t-400000,{650,380,40,40});
        const auto noise = make(204,"dangerous_object",.05f,.05f,.28f,t,{650,380,40,40});
        const auto entries=observe(field,{target1,far_side,behind,stale,noise},.2659f,.0146f,in);
        assert(in.corridor==inv(1));
        std::set<std::string> reasons;
        for(const auto&e:entries){ assert(e.track_id!=203 && e.track_id!=204); if(e.track_id==201||e.track_id==202) reasons.insert(e.reason); }
        assert(reasons.count("outside_lateral") && reasons.count("behind"));
    }
    { // a neighbour without ground position still makes the evidence incomplete
        auto blind=make(205,"dangerous_object",.9f,.2f,.3f,t,{900,380,40,40}); blind.ground_position_valid=false;
        const auto entries=observe(field,{target1,blind},.2659f,.0146f,in);
        assert(!in.corridor_complete);
        bool saw=false; for(const auto&e:entries) if(e.track_id==205){ saw=true; assert(!e.counted && std::string(e.reason)=="no_ground_position"); }
        assert(saw);
    }
}
// Release retreat when the zone is not visible at FAR (field 2026-10-05 07:52: retreat_unverified).
void blindRetreatTests() {
    // Rush `ticks` frames from 0.3 m, then close on nothing; the zone stays out of view.
    const auto failAfterRush = [](Sim& s, int ticks) {
        s.in.distance_m=.3f; s.toRush(); s.zone_visible=false;
        for(int i=0;i<ticks;++i) { s.tick(); assert(s.out.state==PushState::RUSH); }
        s.in.distance_m=.19f;
        assert(s.until(PushState::CAPTURE_FAIL, 150) && s.out.reason=="capture_unverified");
    };
    for(int ticks : {12, 2}) { // reverse straight by min(abort_back_m, forward since the lock), then rescan
        Sim s; failAfterRush(s, ticks);
        const float forward=s.py+.75f, y0=s.py, phi0=s.phi;
        bool reversed=false;
        for(int i=0;i<400 && s.out.state!=PushState::SCAN;++i) {
            s.tick();
            assert(s.out.state!=PushState::SAFE_STOP);
            if(s.out.motion.vx_mps<0) { reversed=true; assert(s.out.motion.wz_rps==0 && s.out.motion.gripper_offset==0); }
        }
        assert(reversed && s.out.state==PushState::SCAN && s.out.reason=="blind_retreat_complete");
        assert(std::abs(s.phi-phi0)<1e-6f);
        const float expected=std::min(s.tune.abort_back_m, forward);
        assert(std::abs((y0-s.py)-expected)<.01f);
        if(ticks==2) assert(expected<s.tune.abort_back_m);
    }
    { // Nothing driven forward since the lock: nothing to retrace, rescan without reversing.
        Sim s; s.toRush(); s.zone_visible=false;
        assert(s.until(PushState::CAPTURE_FAIL, 150));
        const float y0=s.py;
        assert(s.until(PushState::SCAN, 400) && s.out.reason=="blind_retreat_complete" && std::abs(s.py-y0)<.005f);
    }
    { // Heading drift while reversing blind stops the robot.
        Sim s; failAfterRush(s, 12);
        for(int i=0;i<200 && s.out.motion.vx_mps>=0;++i) s.tick();
        assert(s.out.motion.vx_mps<0);
        s.phi+=.3f; s.tick();
        assert(s.out.state==PushState::SAFE_STOP && s.out.reason=="blind_retreat_heading_drift" && stopped(s.out.motion));
    }
    { // No IMU heading: no blind retreat, explicit stop.
        Sim s; failAfterRush(s, 12);
        s.in.heading_valid=false;
        for(int i=0;i<200 && s.out.state!=PushState::SAFE_STOP;++i) { s.tick(); assert(s.out.motion.vx_mps>=0); }
        assert(s.out.state==PushState::SAFE_STOP && s.out.reason=="retreat_unverified_no_heading");
    }
    { // Zone visible: the measured retreat is unchanged.
        Sim s; s.in.distance_m=.3f; s.toRush();
        for(int i=0;i<12;++i) s.tick();
        s.in.distance_m=.19f;
        assert(s.until(PushState::CAPTURE_FAIL, 150));
        const float y0=s.py;
        assert(s.until(PushState::SCAN, 400) && s.out.reason!="blind_retreat_complete");
        assert(y0-s.py >= s.tune.abort_back_m-.01f);
    }
}
// Regressions for the 01:31 run: tiny-error weaving, moving distance bound,
// blind continuation and sweep forward creep. Synthetic observations, no hardware.
void precisionMotionTests() {
    const auto tune = [] { TaskTuning t; t.field_rush=true; t.precise_rush=true;
        t.min_turn_wz=.4f; t.rush_speed=.32f; return t; };
    {
        Sim s(tune()); s.in.distance_m=.30f; s.toRush();
        const float bound=s.task.travel_limit_;
        s.in.distance_m=.18f; s.in.heading_error=.01745f;
        s.tick(); assert(s.out.state==PushState::RUSH && s.out.motion.vx_mps>0 && s.out.motion.vx_mps<=.16f);
        assert(s.out.motion.wz_rps==0 && s.out.reason=="rush_straight_near");
        for(int i=0;i<10;++i)s.tick();
        assert(s.task.travel_limit_==bound); // a frozen distance cannot move the hard bound forward
        s.in.heading_error=.10f; s.tick();
        assert(s.out.reason=="rush_align_at_rest" && s.out.motion.vx_mps==0);
        assert(s.out.motion.wz_rps>0 && s.out.motion.wz_rps<=.25f);
        s.in.heading_error=-.10f; s.tick(); assert(s.out.motion.vx_mps==0 && s.out.motion.wz_rps<0 && s.out.motion.wz_rps>=-.25f);
        s.in.directional_clearance_valid=true; s.in.turn_safe=false; s.tick(); assert(stopped(s.out.motion));
    }
    for(int mode=0;mode<4;++mode) {
        Sim s(tune()); s.in.distance_m=.30f; s.toRush();
        if(mode==0)s.task.travel_limit_=0; // distance bound
        if(mode==2)s.in.target_valid=false; // lost target cannot authorize blind drive
        if(mode==3){s.task.travel_limit_=0; s.hold(inv(1));} // positive held-region evidence still permits enclosure
        if(mode==1){s.in.now_us+=s.tune.rush_budget_us; s.task.last_us_=s.in.now_us; s.task.phase_us_=s.in.now_us-s.tune.rush_budget_us-1;}
        s.tick(); assert(s.out.state==PushState::RUSH && stopped(s.out.motion));
        assert(s.out.reason== (mode==2?"rush_target_lost_reobserve":"rush_bound_reobserve"));
        bool lowered=false;
        for(int i=0;i<100 && s.out.state==PushState::RUSH;++i){s.tick(); assert(stopped(s.out.motion)); lowered|=s.out.state==PushState::LOWER_FRAME;}
        if(mode==3)assert(lowered && s.out.reason=="rush_stall_in_frame_evidence");
        else assert(!lowered && s.out.state==PushState::CAPTURE_FAIL);
    }
    {
        Sim s(tune());s.in.distance_m=.30f;s.toRush();
        s.in.distance_m=.14f;s.in.heading_error=.08f;s.tick();
        assert(s.out.state==PushState::RUSH && s.out.motion.vx_mps==0); // close enough but not aligned: no drop
        s.in.heading_error=0;s.tick();assert(s.out.state==PushState::LOWER_FRAME);
    }
    for(bool white : {true,false}) { // SCAN: a block seen for more than 3 frames at confidence >= .5 is driven at before its ground projection works
        Sim s;s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.in.target_valid=true;s.in.geometry_valid=false;s.in.pixel_bearing_valid=true;s.in.pixel_bearing=.1f;s.in.pixel_confidence=white?.6f:.4f;
        for(int i=0;i<3;++i){s.tick();assert(s.out.state==PushState::SCAN && s.out.motion.vx_mps==0);}
        s.tick();
        if(white){
            assert(s.out.state==PushState::SCAN && s.out.reason=="visual_chase" && s.out.motion.vx_mps>0);
            s.in.geometry_valid=true;assert(s.until(PushState::APPROACH,10)); // once projectable the normal lock takes over
        } else {
            assert(s.out.state==PushState::SCAN && s.out.motion.vx_mps==0 && s.out.reason=="searching_target"); // low confidence: keep spinning
        }
    }
    { // SCAN: a pixel candidate stops the spin; the lock then collects the 4 frames and visual_chase starts
        Sim s;s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.tick();assert(s.out.motion.wz_rps!=0 && s.out.reason=="searching_target"); // nothing in view: spinning
        s.in.pixel_reason="confirming_three_frames";
        s.tick();assert(s.out.state==PushState::SCAN && s.out.reason=="scan_sighting_hold" && stopped(s.out.motion));
        for(int i=0;i<2;++i){s.tick();assert(s.out.reason=="scan_sighting_hold" && stopped(s.out.motion));}
        s.in.pixel_reason="pixel_ratio_locked";s.in.target_valid=true;s.in.geometry_valid=false;s.in.pixel_bearing_valid=true;s.in.pixel_bearing=.1f;s.in.pixel_confidence=.6f;
        for(int i=0;i<3;++i){s.tick();assert(s.out.reason=="scan_sighting_hold" && stopped(s.out.motion));}
        s.tick();assert(s.out.reason=="visual_chase" && s.out.motion.vx_mps>0);
    }
    { // a candidate that never locks holds the spin for scan_hold_max_us only, then the scan resumes for the cool-down
        Sim s;s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.in.pixel_reason="confirming_three_frames";
        int held=0;for(int i=0;i<60;++i){s.tick();if(s.out.reason=="scan_sighting_hold")++held;else break;}
        assert(held>=25 && held<=32 && s.out.motion.wz_rps!=0);
        for(int i=0;i<20;++i){s.tick();assert(s.out.reason!="scan_sighting_hold" && s.out.motion.wz_rps!=0);}
        for(int i=0;i<20;++i)s.tick();
        s.tick();assert(s.out.reason=="scan_sighting_hold"); // cool-down over: the next sighting holds again
    }
    { // a candidate that disappears frees the spin after scan_hold_lost_us
        Sim s;s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.in.pixel_reason="confirming_three_frames";s.tick();assert(s.out.reason=="scan_sighting_hold");
        s.in.pixel_reason="no_legal_candidate";
        int held=0;for(int i=0;i<30;++i){s.tick();if(s.out.reason=="scan_sighting_hold")++held;else break;}
        assert(held>=6 && held<=10 && s.out.motion.wz_rps!=0);
    }
    for(int phase : {4,5,7}) {
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;t.blind_scatter_passes=1;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_sweep");s.task.sweep_phase_=phase;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_ledger_.setSwung(phase!=7);
        s.task.sweep_start_us_=s.in.now_us;s.task.disperse_rush_m_=.5f;
        s.in.clutter_body.clear();s.in.clutter_ts_us=0;
        s.tick();assert(s.out.state==PushState::SCAN && stopped(s.out.motion));
        assert(s.out.reason=="blind_scatter_observation_lost");
        const auto spent=s.task.sweepSpentUs(s.in.now_us);assert(spent>0);
        s.in.target_valid=true;
        for(int i=0;i<8;++i){s.tick();assert(stopped(s.out.motion)&&s.out.reason=="sweep_reobserve");}
        assert(s.task.sweepSpentUs(s.in.now_us)==spent);
        assert(s.until(PushState::APPROACH,50)); // observation pause is bounded, not a new deadlock
    }
    {
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_swing");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;
        s.task.sweep_yaw0_=s.phi;s.task.disperse_swing_.configure(s.task.disperse_swing_.passLimit(),3);
        s.in.clutter_body={{0,.3f},{.1f,.3f},{-.1f,.3f}};s.in.clutter_ts_us=s.in.now_us;
        s.tick();assert(s.out.motion.vx_mps==0 && s.out.motion.wz_rps!=0); // pure swing, no hidden forward creep
    }
    // Single turn: camera to the 20deg stage first, then one in-place turn toward the side with fewer blocks in view.
    for(int blocks_left : {1,0}) {
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;t.disperse_single_turn=true;t.far_pitch_cdeg=500;t.intermediate_pitch_cdeg=2000;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_single_turn");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;
        s.task.sweep_yaw0_=s.phi;s.task.disperse_turn_.begin(true);s.task.disperse_sign_=blocks_left?1.f:-1.f;
        const auto feed=[&]{ // blocks on the left (x<0) only when requested; right side empty
            s.in.clutter_body.clear();
            if(blocks_left) s.in.clutter_body={{-.2f,.5f},{-.25f,.6f},{-.15f,.45f}};
            else s.in.clutter_body={{.2f,.5f},{.25f,.6f},{.15f,.45f},{.1f,.5f}};
            s.in.clutter_ts_us=s.in.now_us;
        };
        feed();
        const float phi0=s.phi;bool looked=false,turned=false;float dir_seen=0;
        for(int i=0;i<400;++i){
            feed();s.tick();
            if(s.out.reason=="disperse_look"){assert(!turned&&stopped(s.out.motion));looked=true;}
            if(s.out.reason=="disperse_turn"){assert(looked&&s.out.motion.vx_mps==0&&s.out.motion.camera_pitch_cdeg==s.tune.intermediate_pitch_cdeg);turned=true;dir_seen=s.out.motion.wz_rps;}
            if(!s.task.disperse_turn_.active()&&turned)break;
        }
        assert(looked&&turned&&!s.task.disperse_turn_.active());
        assert(dir_seen*(blocks_left?-1.f:1.f)>0); // fewer blocks on the right: turn right; fewer on the left: turn left
        const float done=std::abs(s.phi-phi0);
        assert(done>=s.tune.disperse_turn_rad-s.tune.disperse_swing_tol_rad-.2f && done<=s.tune.disperse_turn_rad+.4f); // one turn, not three swings
        assert(s.task.disperse_swing_.swingCount()==0);
    }
    {   // thin view: stops after the minimum turn instead of the full angle
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;t.disperse_single_turn=true;t.far_pitch_cdeg=500;t.intermediate_pitch_cdeg=2000;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_single_turn_thin");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;
        s.task.sweep_yaw0_=s.phi;s.task.disperse_turn_.begin(true);s.task.disperse_sign_=1.f;
        const float phi0=s.phi;
        for(int i=0;i<400;++i){
            s.in.clutter_body.clear();s.in.clutter_ts_us=s.in.now_us;s.tick();
            if(!s.task.disperse_turn_.active()&&s.out.reason!="disperse_look"&&std::abs(s.phi-phi0)>.1f)break;
        }
        assert(!s.task.disperse_turn_.active());
        const float done=std::abs(s.phi-phi0);
        assert(done>=s.tune.disperse_turn_min_rad-.1f && done<s.tune.disperse_turn_rad-.3f);
    }
    for(int zone_blocks : {0,3}) { // frame-zone check: thin view ahead is not enough, the low camera must also see <=1 block in the zone
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;t.disperse_single_turn=true;t.disperse_turn_frame_check=true;t.disperse_turn_rad=1.8f;t.far_pitch_cdeg=500;t.intermediate_pitch_cdeg=2000;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_frame_zone");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;
        s.task.sweep_yaw0_=s.phi;s.task.disperse_turn_.begin(true);s.task.disperse_sign_=1.f;
        const float phi0=s.phi;bool low_turn=false,low_look=false;
        for(int i=0;i<600;++i){
            s.in.clutter_body.clear();
            if(s.task.disperse_turn_.needsFrameView()&&zone_blocks) for(int k=0;k<zone_blocks;++k) s.in.clutter_body.push_back({.03f*float(k),.2f}); // only the low camera sees the zone
            s.in.clutter_ts_us=s.in.now_us;s.tick();
            if(s.out.reason=="disperse_frame_look"){assert(stopped(s.out.motion));low_look=true;}
            if(s.out.reason=="disperse_turn_frame"){assert(s.out.motion.vx_mps==0&&s.out.motion.camera_pitch_cdeg==s.tune.track_pitch_cdeg);low_turn=true;}
            if(!s.task.disperse_turn_.active()&&low_look)break;
        }
        assert(!s.task.disperse_turn_.active()&&low_look);
        const float done=std::abs(s.phi-phi0);
        if(zone_blocks==0){assert(!low_turn&&done<s.tune.disperse_turn_rad-.3f);} // zone clear at once: stops at the minimum turn
        else {assert(low_turn&&done>=s.tune.disperse_turn_min_rad+s.tune.disperse_turn_frame_extra_rad-.1f&&done<s.tune.disperse_turn_min_rad+s.tune.disperse_turn_frame_extra_rad+.25f);} // zone stays crowded: only the extra allowance past the minimum turn, not the cap
    }
    {   // push-on after the turn: straight forward ~disperse_turn_push_m along the new heading, then the sweep ends
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;t.disperse_single_turn=true;t.disperse_turn_push_m=.2f;t.far_pitch_cdeg=500;t.intermediate_pitch_cdeg=2000;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_single_turn_push");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;
        s.task.sweep_yaw0_=s.phi;s.task.disperse_turn_.begin(true);s.task.disperse_sign_=1.f;
        bool pushed=false;float phi_push=0;
        for(int i=0;i<600;++i){
            s.in.clutter_body.clear();s.in.clutter_ts_us=s.in.now_us;s.tick();
            if(s.out.reason=="disperse_turn_push"){
                if(!pushed)phi_push=s.phi;
                pushed=true;if(s.task.disperse_turn_.phase()==DisperseTurnPhase::Push)assert(s.out.motion.vx_mps>0&&s.out.motion.wz_rps==0); // the last tick finishes the push and ends the sweep
            }
            if(pushed&&!s.task.disperse_turn_.active())break;
        }
        assert(pushed&&!s.task.disperse_turn_.active());
        assert(std::abs(s.phi-phi_push)<.05f); // no turning while pushing
    }
}
// Sweeps with the frame open: open acknowledged before the chassis moves, closed again before it moves on, cargo logic untouched.
void sweepFrameOpenTests() {
    for(int phase : {4,5}) { // SCATTER rush / swing
        TaskTuning t;t.sweep_observation_guard=true;t.sweep_frame_open=true;t.disperse_swing_vx=0;t.blind_scatter_passes=1;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        assert(s.out.motion.gripper_offset==20);
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_sweep");s.task.sweep_phase_=phase;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;s.task.sweep_yaw0_=s.phi;
        s.task.disperse_rush_m_=.5f;s.task.disperse_swing_.configure(s.task.disperse_swing_.passLimit(),2);
        s.in.clutter_body={{0,.3f},{.1f,.3f},{-.1f,.3f}};
        auto step=[&]{s.in.clutter_ts_us=s.in.now_us;s.tick();};
        step();assert(stopped(s.out.motion)&&s.out.reason=="sweep_frame_open_wait"); // nothing moves until the open is acknowledged
        assert(s.out.motion.gripper_offset==0);
        s.gripper_responds=false;step();assert(stopped(s.out.motion));s.gripper_responds=true; // no ack yet: still waiting
        step();assert(!stopped(s.out.motion)&&s.out.motion.gripper_offset==0);
        assert(s.task.frame_raised_==0); // cargo logic still sees a closed frame
        s.in.clutter_body.clear();s.in.clutter_ts_us=0;s.in.target_valid=false; // the sweep ends by observation loss
        s.tick();assert(s.out.state==PushState::SCAN);
        s.in.target_valid=true;
        s.gripper_responds=false;s.tick();assert(stopped(s.out.motion)&&s.out.reason=="sweep_frame_close_wait"&&s.out.motion.gripper_offset==20);
        s.gripper_responds=true;s.tick();assert(s.out.reason!="sweep_frame_close_wait");
    }
    { // a frame that never acknowledges the open: the sweep continues closed and stays closed afterwards
        TaskTuning t;t.sweep_observation_guard=true;t.sweep_frame_open=true;t.disperse_swing_vx=0;t.gripper_timeout_us=500000;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_sweep");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;s.task.sweep_yaw0_=s.phi;s.task.disperse_swing_.configure(s.task.disperse_swing_.passLimit(),2);
        s.in.clutter_body={{0,.3f},{.1f,.3f},{-.1f,.3f}};
        s.gripper_responds=false;
        for(int i=0;i<40;++i){s.in.clutter_ts_us=s.in.now_us;s.tick();}
        assert(s.task.sweep_open_disabled_&&s.out.motion.gripper_offset==20);
        s.gripper_responds=true;s.in.clutter_ts_us=s.in.now_us;s.tick();
        assert(s.out.motion.wz_rps!=0&&s.out.motion.gripper_offset==20);
    }
    { // flag off: the closed-frame sweep is unchanged
        TaskTuning t;t.sweep_observation_guard=true;t.disperse_swing_vx=0;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        s.task.enter(PushState::SCATTER,s.in.now_us,"test_sweep");s.task.sweep_phase_=5;
        s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;s.task.sweep_yaw0_=s.phi;s.task.disperse_swing_.configure(s.task.disperse_swing_.passLimit(),2);
        s.in.clutter_body={{0,.3f},{.1f,.3f},{-.1f,.3f}};s.in.clutter_ts_us=s.in.now_us;
        s.tick();assert(s.out.motion.wz_rps!=0&&s.out.motion.gripper_offset==20);
    }
}
// Default-open frame: open whenever nothing is held and the robot searches/approaches/sweeps; closed demands wait for the wire.
void defaultOpenFrameTests() {
    {
        TaskTuning t;t.frame_open_default=true;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        assert(s.task.frame_raised_==0&&s.out.motion.gripper_offset==0); // SCAN keeps the wire open
        s.task.enter(PushState::CLEAR_PILE,s.in.now_us,"test_closed_state"); // needs the closed wire: held until it is confirmed
        s.gripper_responds=false;s.tick();assert(stopped(s.out.motion)&&s.out.reason=="sweep_frame_close_wait"&&s.out.motion.gripper_offset==20);
        s.gripper_responds=true;s.tick();assert(s.out.reason!="sweep_frame_close_wait"&&s.out.motion.gripper_offset==20);
    }
    { // the logical close hidden by the open overlay is not a new MCU action; LOST_SEARCH may spin with the frame open
        TaskTuning t;t.frame_open_default=true;
        Sim s(t);s.in.target_valid=false;
        uint64_t open_txn=0;bool seen_open=false;
        for(int i=0;i<400&&s.out.state!=PushState::SCAN;++i){s.tick();if(s.out.motion.gripper_offset==0){open_txn=s.out.motion.frame_transaction;seen_open=true;}}
        assert(seen_open&&s.out.state==PushState::SCAN);
        for(int i=0;i<20;++i)s.tick();
        assert(s.out.motion.gripper_offset==0&&s.out.motion.frame_transaction==open_txn);
        s.task.enter(PushState::LOST_SEARCH,s.in.now_us,"test_lost");
        for(int i=0;i<10;++i)s.tick();
        assert(s.out.state==PushState::LOST_SEARCH&&s.out.motion.gripper_offset==0);
    }
    { // flag off: SCAN is closed as before
        TaskTuning t;
        Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
        assert(s.out.motion.gripper_offset==20);
    }
    { // a wider swing angle turns further (default 1.0 rad vs 1.4 rad)
        float turned[2]={0,0};int k=0;
        for(float rad : {1.0f,1.4f}) {
            TaskTuning t;t.disperse_swing_vx=0;t.disperse_turn_rad=rad;t.sweep_observation_guard=true;
            Sim s(t);s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
            s.task.enter(PushState::SCATTER,s.in.now_us,"test_sweep");s.task.sweep_phase_=5;
            s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_start_us_=s.in.now_us;s.task.sweep_yaw0_=s.phi;s.task.disperse_swing_.configure(s.task.disperse_swing_.passLimit(),2);
            s.in.clutter_body={{0,.3f},{.1f,.3f},{-.1f,.3f}};
            const float phi0=s.phi;float peak=0;
            for(int i=0;i<400&&s.task.sweep_phase_==5;++i){s.in.clutter_ts_us=s.in.now_us;s.tick();peak=std::max(peak,std::fabs(s.phi-phi0));}
            turned[k++]=peak;
        }
        assert(turned[1]>turned[0]+.25f);
    }
}
// Requested demo capture timing with real corridor rules still enabled.
void demoCaptureMotionTests() {
    const auto tune=[] { TaskTuning t; t.field_rush=true;t.demo_capture_motion=true;t.precise_rush=false;t.rush_stall_us=0;
        t.far_pitch_cdeg=500;t.intermediate_pitch_cdeg=2000;t.track_pitch_cdeg=t.near_pitch_cdeg=4000;
        t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;t.rush_speed=.32f;
        t.max_wz=.8f;t.min_turn_wz=.4f;t.inplace_turn_boost=2.f;t.rush_budget_us=12000000;return t; };
    const auto prepare=[](Sim& s) {
        s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.36f;for(int i=0;i<30;++i)s.tick();
        assert(s.out.state==PushState::APPROACH && s.out.motion.camera_pitch_cdeg==2000);
        s.in.distance_m=.32f;assert(s.until(PushState::PREPARE,20));
        assert(s.out.motion.gripper_offset==0 && s.out.motion.camera_pitch_cdeg==2000);
    };
    {
        Sim s(tune());prepare(s);s.toRush();assert(!s.tune.assume_all_safe);
        s.in.distance_m=.18f;s.in.heading_error=.02f;s.tick();
        assert(s.out.state==PushState::RUSH&&s.out.motion.vx_mps==.32f&&std::abs(s.out.motion.wz_rps)<=.25f);
        assert(s.out.motion.camera_pitch_cdeg==2000);
        for(int i=0;i<50;++i){s.tick();assert(s.out.state==PushState::RUSH&&s.out.motion.vx_mps==.32f);}
        s.in.target_valid=false;s.in.corridor={};s.in.corridor_complete=false;
        bool drove=false;for(int i=0;i<100&&s.out.state==PushState::RUSH;++i){s.tick();
            if(s.out.state==PushState::RUSH){assert(s.out.reason=="assumed_blind_rush"&&s.out.motion.wz_rps==0);drove|=s.out.motion.vx_mps==.32f;}}
        assert(drove&&s.out.state==PushState::LOWER_FRAME&&s.out.reason=="assumed_enclosure_target_below_view");
        assert(s.out.motion.camera_pitch_cdeg==4000);
    }
    { // 0.25 rad/s does not move the chassis: pure in-place turns in demo APPROACH use the 0.4 floor and escalate to max_wz after 1 s.
        Sim s(tune());s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.30f;s.in.heading_error=.15f;
        float first=0,last=0;bool turned=false;
        for(int i=0;i<80;++i){s.tick();
            if(s.out.state==PushState::APPROACH&&s.out.motion.vx_mps==0&&s.out.motion.wz_rps!=0){
                if(!turned){first=std::abs(s.out.motion.wz_rps);turned=true;}last=std::abs(s.out.motion.wz_rps);}}
        assert(turned&&first>=.4f-1e-4f&&last>first&&last<=.8f+1e-4f);
    }
    { // Recognized danger wins over the blind completion and the bound on the same tick.
        Sim s(tune());prepare(s);s.toRush();s.in.target_valid=false;s.in.corridor=inv(1,0,0,1);s.task.travel_limit_=0;
        s.tick();assert(s.out.state==PushState::CAPTURE_FAIL&&s.out.reason=="rush_corridor_dangerous"&&stopped(s.out.motion));
    }
    { // Opening earlier does not bypass PREPARE's inventory check.
        Sim s(tune());prepare(s);s.in.corridor=inv(1,0,1);
        assert(s.until(PushState::LOST_HOLD,80));assert(!s.visited.count(PushState::RUSH));
    }
    { // Time budget still bounds a continuously visible stalled target, as in demo.
        auto t=tune();t.rush_budget_us=500000;Sim s(t);prepare(s);s.toRush();
        assert(s.until(PushState::LOWER_FRAME,40));assert(s.out.reason=="assumed_enclosure_rush_bound");
    }
    { // Normal visual enclosure threshold still closes without waiting for the fallback.
        Sim s(tune());prepare(s);s.toRush();s.in.distance_m=.14f;s.tick();
        assert(s.out.state==PushState::LOWER_FRAME&&s.out.reason=="frame_enclosure_position");
    }
}
void sweepViewTests() {
    const auto base=[]{ TaskTuning t;t.enable_search_cues=false;t.blind_scatter_min=3;t.blind_scatter_first_turn_rad=0.f;t.blind_scatter_passes=1;
        t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;return t; };
    struct R{int max_pitch=0,rush_pitch=-1,swing_pitch=-1,align_pitch=-1;float max_swing=0;bool done=false,lost=false;};
    const auto run=[&](TaskTuning t,std::vector<cv::Point2f> body_pts){
        Sim s(t);s.in.target_valid=false;s.in.distance_m=.9f;
        const float x0=s.px,y0=s.py;std::vector<cv::Point2f> world;
        for(const auto&b:body_pts)world.push_back({x0+b.x,y0+b.y});
        R r;float phi_rush=0;bool rushed=false;
        for(int i=0;i<2400&&!r.done;++i){
            s.in.clutter_body.clear();const float c=std::cos(s.phi),sn=std::sin(s.phi);
            for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
            s.in.clutter_ts_us=s.in.now_us;s.tick();
            if(s.out.state==PushState::SCATTER){
                r.max_pitch=std::max(r.max_pitch,int(s.out.motion.camera_pitch_cdeg));
                if(s.out.reason=="blind_scatter_align")r.align_pitch=s.out.motion.camera_pitch_cdeg;
                if(s.out.reason=="disperse_rush"){r.rush_pitch=s.out.motion.camera_pitch_cdeg;if(!rushed){rushed=true;phi_rush=s.phi;}}
                if(s.out.reason=="disperse_swing")r.swing_pitch=s.out.motion.camera_pitch_cdeg;
                if(rushed)r.max_swing=std::max(r.max_swing,s.phi-phi_rush);
            }
            if(s.out.reason=="blind_scatter_done")r.done=true;
            if(s.out.reason=="blind_scatter_observation_lost")r.lost=true;
        }
        return r;
    };
    const std::vector<cv::Point2f> group{{.20f,.50f},{.32f,.56f},{.26f,.44f}};
    { // 扫动前视角封顶20°；扫动动作本身（冲入/摆扫）改用5°远景判断。
        TaskTuning off=base();
        TaskTuning cap=base();cap.sweep_view_cap_mid=true;
        TaskTuning far=cap;far.sweep_far_judge=true;
        const auto a=run(off,group),b=run(cap,group),c=run(far,group);
        assert(a.done&&b.done&&c.done);
        assert(b.max_pitch<=2000);
        assert(c.max_pitch<=2000&&c.align_pitch==2000);
        assert(c.rush_pitch==500&&c.swing_pitch==500);
        assert(!c.lost);
    }
    { // 摆扫方向避开框外物块：框外块在前方偏左时，向左的摆角被封顶，先摆向空闲的一侧。
        const cv::Point2f gc{.26f,.50f};const float len=std::hypot(gc.x,gc.y);
        const cv::Point2f u{gc.x/len,gc.y/len},xa{u.y,-u.x};
        const float e=len-.171f+.20f;
        const cv::Point2f blk{u.x*(e+.30f)-xa.x*.20f,u.y*(e+.30f)-xa.y*.20f};
        auto pts=group;pts.push_back(blk);
        TaskTuning on=base();on.sweep_view_cap_mid=true;on.sweep_far_judge=true;
        TaskTuning plan=on;plan.swing_avoid_outside=true;
        const auto free=run(on,pts),guarded=run(plan,pts);
        assert(free.done&&guarded.done);
        assert(free.max_swing>.85f);
        assert(guarded.max_swing<.75f&&free.max_swing-guarded.max_swing>.2f);
    }
    { // 摆角上限只属于本次SCATTER扫动：离开SCATTER后不能残留到后面的APPROACH/MID扫动里。
        TaskTuning t=base();Sim s(t);
        s.task.enter(PushState::SCATTER,s.in.now_us,"test");
#ifdef RESCUE_HAS_RUSH_STAGE
        s.task.disperse_rush_.active_=true;s.task.disperse_rush_.pending_=true;s.task.disperse_rush_.cap_negative_=.3f;s.task.disperse_rush_.cap_positive_=.2f;
#else
        s.task.swing_plan_active_=true;s.task.swing_plan_pending_=true;s.task.swing_cap_neg_=.3f;s.task.swing_cap_pos_=.2f;
#endif
        s.task.enter(PushState::APPROACH,s.in.now_us,"test");
#ifdef RESCUE_HAS_RUSH_STAGE
        assert(!s.task.disperse_rush_.active_&&!s.task.disperse_rush_.pending_&&s.task.disperse_rush_.cap_negative_>=99.f&&s.task.disperse_rush_.cap_positive_>=99.f);
#else
        assert(!s.task.swing_plan_active_&&!s.task.swing_plan_pending_&&s.task.swing_cap_neg_>=99.f&&s.task.swing_cap_pos_>=99.f);
#endif
    }
    { // 有很多目标时不锁定：锁定目标附近成团则进入拆分(SCATTER)而不是APPROACH；关闭开关则照常锁定。
        for(int on=0;on<2;++on){
            TaskTuning t=base();t.sweep_view_cap_mid=true;t.dense_no_lock=on;
            Sim s(t);s.in.target_valid=true;s.in.distance_m=.45f;
            bool split=false,approach=false;
            for(int i=0;i<400;++i){
                s.in.clutter_body={{.12f,.50f},{-.10f,.52f}};s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.state==PushState::SCATTER&&s.out.reason.rfind("dense_group",0)==0)split=true;
                if(s.out.reason.rfind("dense_group",0)==0||s.out.state==PushState::SCATTER)split=split||s.out.state==PushState::SCATTER;
                if(s.out.state==PushState::APPROACH)approach=true;
                if(split||approach)break;
            }
            if(on){assert(split&&!approach);}else{assert(!split&&approach);}
        }
    }
    { // 只有目标少时才锁定：视野里目标数超过lock_max_visible且成对靠近时开框清扫，不论是否成"紧团"；2个以内照常锁定；冷却期内可选择无视20 s间隔。
        const auto run=[&](int max_visible,bool ignore_gap,bool gap_active,std::vector<cv::Point2f> others){
            TaskTuning t=base();t.sweep_view_cap_mid=true;t.dense_no_lock=true;t.lock_max_visible=max_visible;t.dense_multi_ignore_gap=ignore_gap;
            Sim s(t);s.in.target_valid=true;s.in.distance_m=.45f;
            if(gap_active)s.task.sweep_ledger_.setGapUntil(s.in.now_us+20000000);
            std::string why;bool approach=false,split=false;
            for(int i=0;i<400;++i){
                s.in.clutter_body=others;s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.state==PushState::SCATTER){split=true;why=s.out.reason;}
                if(s.out.state==PushState::APPROACH)approach=true;
                if(split||approach)break;
            }
            return std::make_pair(split,approach);
        };
        const std::vector<cv::Point2f> three{{.30f,.50f},{-.30f,.52f}},two{{.30f,.50f}};
        auto r=run(0,false,false,three);assert(!r.first&&r.second);   // 关闭：松散三个照常锁定
        r=run(2,false,false,three);assert(r.first&&!r.second);        // 3个可见 > 2：清扫而不锁定
        r=run(2,false,false,two);assert(!r.first&&r.second);          // 2个可见：照常锁定
        r=run(2,false,true,three);assert(!r.first&&r.second);        // 冷却期内且不无视间隔：回落到锁定
        r=run(2,true,true,three);assert(r.first&&!r.second);         // 无视间隔：冷却期内仍清扫
    }
    { // 锁定后近处被车框遮挡丢失：原地等待新ID，不立刻进入丢失搜索；新ID出现在原位置附近则改绑；超时仍走原来的丢失流程。
        const auto run=[&](uint64_t hold,bool newid){
            TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.occlusion_hold_us=hold;
            Sim s(t);s.in.distance_m=.6f;assert(s.until(PushState::APPROACH,80));
            for(int i=0;i<10;++i)s.tick();
            s.task.approach_rebind_=false; // isolate the occlusion hold from the pitch-change rebind window
            s.in.target_valid=false;bool waited=false,rebound=false;int left=-1;
            for(int i=0;i<220;++i){
                if(newid&&i==20){s.in.target_valid=true;s.in.target_id=99;s.in.distance_m=.58f;}
                s.tick();
                if(s.out.reason=="target_occluded_wait")waited=true;
                if(s.out.reason=="approach_id_rebound_same_object")rebound=true;
                if(left<0&&s.out.state!=PushState::APPROACH)left=i;
            }
            return std::make_tuple(waited,rebound,left);
        };
        auto [w0,b0,l0]=run(0,false);assert(!w0&&l0>=0&&l0<120);
        auto [w1,b1,l1]=run(2500000,false);assert(w1&&l1>=l0+35);
        auto [w2,b2,l2]=run(2500000,true);assert(w2&&b2&&(l2<0||l2>l0+35));
    }
    { // APPROACH里的密集扫动：开关打开后摆扫角速度与首发爆发一致(first_burst_swing_wz)，关闭则为disperse_wz；摆幅两侧同为disperse_turn_rad。
        const auto run=[&](bool burst){
            TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;
            t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;t.disperse_swing_vx=0;t.disperse_turn_rad=1.4f;t.first_burst_swing_wz=5.f;t.disperse_wz=3.2f;
            t.approach_dense_burst_swing=burst;
            Sim s(t);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.25f;float swing_wz=0,lo=0,hi=0;const float phi0=s.phi;bool swung=false;
            for(int i=0;i<600;++i){
                s.in.clutter_body.clear();
                for(int k=0;k<4;++k)s.in.clutter_body.push_back({-.12f+.08f*k,.25f+((k%2)?.06f:-.06f)});
                s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.state==PushState::APPROACH&&s.out.reason=="disperse_swing"){swung=true;swing_wz=std::max(swing_wz,std::fabs(s.out.motion.wz_rps));}
                lo=std::min(lo,s.phi-phi0);hi=std::max(hi,s.phi-phi0);
                if(s.out.reason=="disperse_done")break;
            }
            assert(swung);
            return std::array<float,3>{swing_wz,lo,hi};
        };
        const auto off=run(false),on=run(true);
        assert(on[0]>off[0]+.5f);
        assert(std::fabs(on[1])>1.0f&&on[2]>1.0f);
    }
}
void pileTests() {
    const auto base=[]{ TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;
        t.assume_motion_ratio=1.f;t.pile_clear=true;return t; };
    struct Blk{cv::Point2f p;bool ordinary;};
    struct R{bool entered=false,ended=false;std::string end;std::set<std::string> reasons;float advanced=0,cut_yaw=0,phi_cut=0,y_measure=0,y_end=0;};
    const auto run=[&](TaskTuning t,std::vector<Blk> world_rel,int corridor,float dist,int ticks=2400,const Blk*tgt=nullptr){
        Sim s(t);s.all_ordinary=false;s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
        const float x0=s.px,y0=s.py;R r;PushState prev=s.out.state;
        for(int i=0;i<ticks&&!r.ended;++i){
            s.in.clutter_body.clear();s.in.clutter_ordinary.clear();const float c=std::cos(s.phi),sn=std::sin(s.phi);
            for(const auto&b:world_rel){const float dx=x0+b.p.x-s.px,dy=y0+b.p.y-s.py;const cv::Point2f q{dx*c+dy*sn,-dx*sn+dy*c};
                s.in.clutter_body.push_back(q);if(b.ordinary)s.in.clutter_ordinary.push_back(q);}
            s.in.clutter_ts_us=s.in.now_us;
            if(tgt&&(s.out.state==PushState::PILE||r.entered)){ // locked target kept valid inside PILE, at a fixed world spot
                const float dx=x0+tgt->p.x-s.px,dy=y0+tgt->p.y-s.py;const cv::Point2f q{dx*c+dy*sn,-dx*sn+dy*c};
                s.in.target_valid=true;s.in.distance_m=std::hypot(q.x,q.y);s.in.heading_error=std::atan2(-q.x,q.y);s.in.label="ordinary_supply";}
            else if(s.out.state==PushState::PILE||r.entered){s.in.target_valid=false;}
            else{s.in.target_valid=true;s.in.distance_m=dist;s.in.corridor=inv(corridor);}
            s.tick();
            if(s.out.state==PushState::PILE){
                r.entered=true;r.reasons.insert(s.out.reason);
                if(s.out.reason=="pile_advance")r.advanced=s.py-y0;
                if(s.out.reason=="pile_measure"&&r.y_measure==0)r.y_measure=s.py;
                if(s.out.reason=="pile_cut_align"||s.out.reason=="pile_cut_drive"){r.cut_yaw=s.task.pile_cut_yaw_;r.phi_cut=s.phi;}
            }
            if(prev==PushState::PILE&&s.out.state!=PushState::PILE){r.ended=true;r.end=s.out.reason;r.y_end=s.py;}
            prev=s.out.state;
        }
        return r;
    };
    // 3个普通物资在框内(超过2个会被走廊规则拒绝) + 1个非普通物资在左侧框内。
    const std::vector<Blk> pile_left_bad{{{-.12f,.75f},false},{{-.05f,.75f},true},{{.05f,.75f},true},{{.10f,.75f},true}};
    std::vector<Blk> pile_right_bad;for(const auto&b:pile_left_bad)pile_right_bad.push_back({{-b.p.x,b.p.y},b.ordinary});
    { // 框口走廊>=3：开着框直进0.5 m -> 后退看 -> 量块角度 -> 向非普通物资一侧的反方向...的最优侧转一个物块视角 -> 切入 -> 后退 -> 回搜索。
        const auto r=run(base(),pile_left_bad,3,.5f);
        assert(r.entered&&r.ended);
        for(const char*k:{"pile_advance","pile_view_back","pile_measure","pile_cut_align","pile_cut_drive","pile_cut_back"})
            if(!r.reasons.count(k)){std::cerr<<"missing pile reason "<<k<<"\n";assert(false);}
        assert(r.end=="pile_split_done");
        assert(r.advanced>.47f&&r.advanced<.55f);
        assert(std::abs(r.y_end-r.y_measure)<.08f); // 切入后只退回观察位，不再多退一段
        // 把左边那个非普通物资筛掉、普通物资全留在框里：向右转一个物块视角(约7.6度)，镜像场景向左。
        const float turn=r.cut_yaw-0.f;
        assert(turn<-.09f&&turn>-.2f);
        const auto m=run(base(),pile_right_bad,3,.5f);
        assert(m.end=="pile_split_done"&&m.cut_yaw>.09f&&m.cut_yaw<.2f);
    }
    { // 非普通物资藏在框中间，转一个物块视角还夹着它：默认放弃切；pile_cut_max_steps=3 时多转几个物块视角，先把框里清成同一类型再切。
        const std::vector<Blk> deep_bad{{{0.f,.75f},false},{{.08f,.75f},true},{{.16f,.75f},true},{{.24f,.75f},true}};
        const auto narrow=run(base(),deep_bad,3,.5f);
        assert(narrow.entered&&narrow.ended&&narrow.end=="pile_cut_no_gain");
        TaskTuning wide=base();wide.pile_cut_max_steps=3;
        const auto w=run(wide,deep_bad,3,.5f);
        assert(w.entered&&w.ended&&w.end=="pile_split_done"&&w.reasons.count("pile_cut_drive"));
        assert(w.cut_yaw<-.18f); // 向右转了不止一个物块视角(单步约7.6度=.13 rad)
    }
    { // 被锁定的普通物资紧贴(4 cm)一个非普通物资：不能把它当成重复物资丢掉，否则簇里只剩非普通物资，误判无增益。
        const std::vector<Blk> touch{{{-.065f,.75f},false},{{.067f,.75f},false}};
        const Blk target{{-.107f,.75f},true};
        TaskTuning wide=base();wide.pile_cut_max_steps=3;
        const auto r=run(wide,touch,3,.5f,2400,&target);
        assert(r.entered&&r.ended&&r.end=="pile_split_done"&&r.cut_yaw>.18f);
    }
    { // 默认关闭；走廊<3；目标超出触发距离：都不进入。
        TaskTuning off=base();off.pile_clear=false;
        assert(!run(off,pile_left_bad,3,.5f,600).entered);
        assert(!run(base(),pile_left_bad,2,.5f,600).entered);
        assert(!run(base(),pile_left_bad,3,.9f,600).entered);
    }
    { // 直进之后框里只剩<=2个普通物资：不再多此一举地切，直接回搜索去锁定。
        const std::vector<Blk> ok{{{-.30f,.75f},true},{{-.05f,.75f},true},{{.10f,.75f},true},{{.30f,.75f},true}};
        const auto r=run(base(),ok,3,.5f);
        assert(r.entered&&r.ended&&r.end=="pile_frame_already_ok");
        assert(!r.reasons.count("pile_cut_drive"));
    }
    { // 转一个物块视角也分不开(块都挤在框中间)：不切，也不乱动。
        const std::vector<Blk> tight{{{-.03f,.75f},true},{{0.f,.75f},true},{{.03f,.75f},true},{{.01f,.78f},true}};
        const auto r=run(base(),tight,3,.5f);
        assert(r.entered&&r.ended&&r.end=="pile_cut_no_gain");
    }
    { // PILE chain: pushed-out cluster deeper than one layer -> face, rush in, big-angle turn, back off, re-measure (capped), then the cut; one layer -> straight to the cut.
        const auto chain=[&](std::vector<cv::Point2f> rel,bool on,int max_rounds,float fov=0.f){
            TaskTuning t=base();t.pile_chain=on;t.pile_chain_max=max_rounds;t.disperse_single_turn=true;t.disperse_turn_push_m=.2f;
            Sim s(t);s.all_ordinary=false;s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
            const float x0=s.px,y0=s.py,phi0=s.phi;
            s.task.enter(PushState::PILE,s.in.now_us,"test_pile_chain");
            s.task.pile_yaw0_=s.phi;s.task.pile_phase_=7;s.task.pile_seen_=0;s.task.pile_push0_=-1.f;s.task.pile_chain_n_=0;s.task.pile_phase_us_=s.in.now_us;
            std::set<std::string> reasons;std::string end;int rounds=0;bool left=false;
            for(int i=0;i<6000&&!left;++i){
                s.in.clutter_body.clear();s.in.clutter_ordinary.clear();const float c=std::cos(s.phi),sn=std::sin(s.phi);
                for(const auto&b:rel){const float dx=x0+(b.x*std::cos(phi0)-b.y*std::sin(phi0))-s.px,dy=y0+(b.x*std::sin(phi0)+b.y*std::cos(phi0))-s.py;
                    const cv::Point2f q{dx*c+dy*sn,-dx*sn+dy*c};if(fov>0.f&&std::fabs(std::atan2(-q.x,q.y))>fov)continue;s.in.clutter_body.push_back(q);s.in.clutter_ordinary.push_back(q);}
                s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.state==PushState::PILE)reasons.insert(s.out.reason);else{left=true;end=s.out.reason;}
                rounds=s.task.pile_chain_n_;
            }
            assert(left);
            return std::make_tuple(reasons,end,rounds);
        };
        const std::vector<cv::Point2f> deep{{0.f,.50f},{.04f,.62f},{-.04f,.74f},{0.f,.86f}};
        const std::vector<cv::Point2f> flat{{-.10f,.60f},{0.f,.60f},{.10f,.60f}};
        {   const auto [rs,end,n]=chain(deep,true,2);
            for(const char*k:{"pile_chain_face","disperse_rush","disperse_turn","pile_chain_back"})
                if(!rs.count(k)){std::cerr<<"missing chain reason "<<k<<"\n";assert(false);}
            assert(n==2); // blocks never move in the Sim, so the loop runs to its cap
        }
        {   const auto [rs,end,n]=chain(flat,true,3);
            assert(!rs.count("pile_chain_face")&&!rs.count("disperse_rush")&&n==0); // one layer: no more sweeping
        }
        {   // the turn takes the cluster out of a narrow view: the car must look back (mid-turn, then pre-turn heading) instead of giving up.
            const auto [rs,end,n]=chain(deep,true,1,.5f);
            assert(rs.count("pile_chain_look")&&end!="pile_chain_cluster_clear");
        }
        {   const auto [rs,end,n]=chain(deep,true,0);
            assert(!rs.count("pile_chain_face")&&n==0); // cap 0: measure and cut right away
        }
    }
    { // A swung disperse sweep that ends in APPROACH hands over to PILE (phase 7) only when pile_chain is on.
        for(bool on : {true,false}){
            TaskTuning t=base();t.pile_chain=on;t.disperse_single_turn=true;t.disperse_turn_push_m=.2f;
            Sim s(t);s.all_ordinary=false;s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.task.sweep_ledger_.begin(s.task.t_,s.in.now_us);s.task.sweep_ledger_.setSwung(true);s.task.sweep_phase_=5;s.task.disperse_turn_.phase_=DisperseTurnPhase::Push;
            s.task.sweep_start_us_=s.in.now_us;s.task.sweep_travel0_=s.task.travel_-1.f;
            for(int i=0;i<60&&s.out.state==PushState::APPROACH;++i)s.tick();
            assert((s.out.state==PushState::PILE)==on);
            if(on)assert(s.task.pile_phase_==7);
        }
    }
}
int main(int argc, char **argv) {
    if (argc == 3 && std::string(argv[1]) == "--write-fixture") { writeFixture(argv[2]); return 0; }
    sweepViewTests();
    pileTests();
    precisionMotionTests();
    demoCaptureMotionTests();
    sweepFrameOpenTests();
    defaultOpenFrameTests();
    { // assume_injured_trip starts as if the first ordinary supply was delivered; the default is unchanged.
        for(int on=0;on<2;++on) {
            TaskTuning t;t.assume_injured_trip=on;PushTask task(t);PushObservation in;
            in.run=in.safety_ok=true;in.now_us=100000;
            assert(task.update(in).first_ordinary_delivered==bool(on));
            in.reset=true;in.now_us+=100000;assert(task.update(in).first_ordinary_delivered==bool(on));
        }
    }
    { // Visual closed loop: release/push distances follow the remembered or visible zone, not fixed blind legs.
      for(int lose=0;lose<2;++lose){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::CARRY,300) && s.out.reason=="assumed_capture");
        if(lose){ // zone leaves the view mid-leg: remaining distance is dead-reckoned
            for(int i=0;i<400 && s.py<-.5f;++i)s.tick();
            s.vref_visible=false;
        }
        assert(s.until(PushState::RAISE_RELEASE,600));
        assert(s.out.reason==(lose?"assumed_drop_point_visual_dr":"assumed_drop_point_visual"));
        {const float e=s.py+s.tune.hold_center_y_m+s.tune.assume_release_gap_m; // dead-reckoned leg is pessimistic: may run on past the point, never stop short
         if(lose)assert(e>-.06f&&e<.32f);else assert(std::abs(e)<.06f);}
        s.vref_visible=true;
        assert(s.until(PushState::BACK_OUT,100) && s.out.reason=="assumed_release_back_off");
        const float yb=s.py;
        assert(s.until(PushState::ENTER,300) && s.out.reason=="assumed_release_then_push");
        assert(std::abs((yb-s.py)-s.tune.assume_prepush_back_m)<.04f);
        assert(s.until(PushState::RAISE_RELEASE,600) && s.out.reason=="assumed_push_in_done");
        assert(std::abs(s.py+s.tune.hold_center_y_m-s.tune.assume_push_depth_m)<.06f);
        assert(s.until(PushState::SAFE_STOP,2000) && s.out.reason=="demo_next_trip_unverified");
      }
    }
    { // Blocked by the zone rim near the front edge: release by stall instead of ramming until timeout.
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.py_max=-.03f-s.tune.hold_center_y_m;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::CARRY,300) && s.out.reason=="assumed_capture");
        assert(s.until(PushState::RAISE_RELEASE,600) && s.out.reason=="assumed_drop_point_stalled");
    }
    { // 卡在边框前较远处（第37趟 hold_y≈-0.08）：放宽的卡停窗口内同样按停滞释放，而不是空转到超时。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;t.assume_stall_near_m=.12f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.py_max=-.09f-s.tune.hold_center_y_m;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::CARRY,300) && s.out.reason=="assumed_capture");
        assert(s.until(PushState::RAISE_RELEASE,900) && s.out.reason=="assumed_drop_point_stalled");
    }
    { // 视觉判块：块已在区内（近端距前沿≥done_y）则推入提前结束，核验阶段按视觉判进区。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::RAISE_RELEASE,600) && s.out.reason=="assumed_push_in_done");
        assert(s.py+s.tune.hold_center_y_m<s.tune.assume_push_depth_m-.05f);
        assert(s.py+s.tune.hold_center_y_m>=s.tune.assume_inside_min_hold_y_m-.02f); // 框自身须已越过前沿一定深度才信“块已进区”
        assert(s.until(PushState::TURN_SCAN,1500) && s.out.reason=="delivered_visual");
        const std::string ev=s.task.takeDropZoneEvent();
        assert(ev.find("[CARGO_ZONE] event=push_inside")!=std::string::npos && ev.find("event=verify")!=std::string::npos);
    }
    for(int hard=0;hard<2;++hard){ // 翻框前须确认真进区：推入深度不够时继续下压；超过硬上限才放弃确认
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;t.assume_enter_confirm=true;
        t.assume_enter_hold_y_m=hard?.90f:.22f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        assert(s.until(PushState::RAISE_RELEASE,1500) && s.out.reason=="assumed_push_in_done");
        const float depth=s.py+s.tune.hold_center_y_m;
        const std::string ev=s.task.takeDropZoneEvent();
        assert(ev.find("event=push_unconfirmed")!=std::string::npos);
        if(!hard){assert(depth>=.21f&&ev.find("event=push_unconfirmed_giveup")==std::string::npos);}
        else{assert(depth>=s.tune.assume_enter_max_hold_y_m-.02f&&ev.find("event=push_unconfirmed_giveup")!=std::string::npos);}
    }
    { // 默认关闭：行为与原先一致（深度到 assume_push_depth_m 即翻框）
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;t.assume_enter_hold_y_m=.90f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        assert(s.until(PushState::RAISE_RELEASE,1500) && s.out.reason=="assumed_push_in_done");
        assert(s.py+s.tune.hold_center_y_m<s.tune.assume_push_depth_m+.06f);
        assert(s.task.takeDropZoneEvent().find("push_unconfirmed")==std::string::npos);
    }
    { // 深松手：松开时块已过前沿，后退中视觉看到块在区内则跳过推入直接核验。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500) && s.out.reason=="assumed_inside_after_release");
        assert(s.until(PushState::TURN_SCAN,1500) && s.out.reason=="delivered_visual");
        const std::string ev=s.task.takeDropZoneEvent();
        assert(ev.find("event=inside_after_release")!=std::string::npos && ev.find("event=push_done")==std::string::npos);
    }
    { // 核验只认视觉定位：推算认为块在区内但无定位 → 不判视觉通过，先等、再切远俯仰找角点；等不到按“推算”记并标明。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500) && s.out.reason=="assumed_inside_after_release");
        s.vref_visible=false;
        bool far=false;int n=0;
        for(;n<400 && s.out.state==PushState::VERIFY_DELIVERY;++n){s.tick();if(s.out.motion.camera_pitch_cdeg==s.tune.far_pitch_cdeg)far=true;}
        assert(s.out.state==PushState::TURN_SCAN && s.out.reason=="delivered_unverified");
        assert(far && n*50000ull>=s.tune.assume_verify_fix_wait_us-100000);
        assert(s.task.takeDropZoneEvent().find("fix=0 repush=0 in=1")==std::string::npos);
    }
    { // 定位晚到：先无定位等待，随后拿到定位且块在区内 → 才判 delivered_visual。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500));
        s.vref_visible=false;
        for(int i=0;i<80;++i){s.tick();assert(s.out.state==PushState::VERIFY_DELIVERY);}
        s.vref_visible=true;
        assert(s.until(PushState::TURN_SCAN,200) && s.out.reason=="delivered_visual");
    }
    { // 定位到了但块在区外（压在边框外）→ 重推，而不是记成功。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500));
        s.cargo_zone={s.tune.supply_half_x_m,-.02f};
        assert(s.until(PushState::ENTER,200) && s.out.reason=="assumed_verify_repush");
        assert(s.task.takeDropZoneEvent().find("event=verify_outside_repush")!=std::string::npos);
    }
    { // 块近端刚越过前沿约2cm（日志实测1.4~3cm）→ 算进区，不再重推。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500));
        s.cargo_zone={s.tune.supply_half_x_m,.02f};
        assert(s.until(PushState::TURN_SCAN,200) && s.out.reason=="delivered_visual");
    }
    { // 最近的块在区外，但区内另有块 → 不重推，直接记送达。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500));
        s.cargo_zone={s.tune.supply_half_x_m,-.02f};s.cargo2_on=true;
        assert(s.until(PushState::TURN_SCAN,200) && s.out.reason=="delivered_visual");
        assert(s.task.takeDropZoneEvent().find("event=verify_other_inside")!=std::string::npos);
    }
    { // 后退后前推前的视觉对准：货物偏在侧前方时先原地转向对准，再按方位追踪，框到达时货物须在框正前方。
      for(int align=0;align<2;++align){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;t.assume_align_push=align;t.assume_slant_max_rad=0.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.cargo_on=true;s.cargo_zone={s.px+.12f,.03f};
        bool turned=false;float lastx=9.f;
        for(int i=0;i<900 && s.out.state==PushState::ENTER;++i){
            s.tick();
            if(s.out.reason=="assumed_push_align_turn")turned=true;
            if(!s.in.cargo_body.empty()&&s.in.cargo_body[0].y-s.tune.hold_center_y_m<.15f)lastx=s.in.cargo_body[0].x;
        }
        const std::string ev=s.task.takeDropZoneEvent();
        if(align){assert(turned&&ev.find("event=push_aligned")!=std::string::npos&&std::abs(lastx)<.05f);}
        else assert(!turned);
      }
    }
    { // 卡住重推：块停在前沿、框前进不了；每次重推加长助跑，最多2次后放弃并抬框退出。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.cargo_on=true;s.cargo_zone={s.px,.0f};s.py_max=-.15f-s.tune.hold_center_y_m;
        float back1=0;
        for(int n=0;n<2;++n){
            assert(s.until(PushState::BACK_OUT,1500) && s.out.reason=="assumed_stuck_repush");
            const float yb=s.py;
            assert(s.until(PushState::ENTER,600) && s.out.reason=="assumed_release_then_push");
            const float back=yb-s.py;
            if(n==0)back1=back; else assert(back>back1+.05f);
        }
        assert(s.until(PushState::RAISE_RELEASE,1500) && s.out.reason=="assumed_push_in_done");
        const std::string ev=s.task.takeDropZoneEvent();
        assert(ev.find("event=stuck_giveup")!=std::string::npos && ev.find("event=stuck_repush")!=std::string::npos);
    }
    { // IMU判被边框顶住：近区内按IMU释放（早于视觉/推算的+0.04释放点）；远离区域时IMU误报不释放。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::CARRY,300) && s.out.reason=="assumed_capture");
        s.in.imu_blocked_us=1000000;
        assert(s.until(PushState::RAISE_RELEASE,900) && s.out.reason=="assumed_drop_point_imu_blocked");
        const float e=s.py+s.tune.hold_center_y_m;
        assert(e<-.12f&&e>-.40f);
        assert(s.task.takeDropZoneEvent().find("event=release_stall_imu")!=std::string::npos);
    }
    { // ENTER中IMU判卡住（货物不可见也成立）：重推2次后放弃并抬框，由核验阶段再用视觉判进区。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.in.imu_blocked_us=1000000;
        for(int n=0;n<2;++n){
            assert(s.until(PushState::BACK_OUT,200) && s.out.reason=="assumed_stuck_repush");
            s.in.imu_blocked_us=0;
            assert(s.until(PushState::ENTER,600) && s.out.reason=="assumed_release_then_push");
            s.in.imu_blocked_us=1000000;
        }
        assert(s.until(PushState::RAISE_RELEASE,200));
        const std::string ev=s.task.takeDropZoneEvent();
        assert(ev.find("event=stuck_giveup")!=std::string::npos&&ev.find("event=stuck_repush")!=std::string::npos);
        assert(ev.find("event=push_inside")==std::string::npos);
    }
    { // 视觉定不了货物(截断/看不到)时：IMU堵转且框已深入→视为推到位不重推；框还浅→仍重推；看得清且在区外→仍重推。
      for(int k=0;k<4;++k){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        const bool deep=k==0||k==3;   // 0:截断+深 1:截断+浅 2:看不到+浅 3:看不到+深
        s.py_max=(deep?.13f:.04f)-s.tune.hold_center_y_m;
        if(k<2){s.cargo_on=true;s.cargo_zone={s.px,.20f};s.cargo_clip=true;}
        for(int i=0;i<400 && s.py<s.py_max-.005f && s.out.state==PushState::ENTER;++i)s.tick();
        s.in.imu_blocked_us=1000000;
        s.task.takeDropZoneEvent();
        for(int i=0;i<60 && s.out.state==PushState::ENTER;++i)s.tick();
        const std::string ev=s.task.takeDropZoneEvent();
        if(k==1||k==2){assert(s.out.state==PushState::BACK_OUT && s.out.reason=="assumed_stuck_repush");}
        else{
            assert(s.out.reason!="assumed_stuck_repush");
            assert(ev.find("event=stuck_repush")==std::string::npos);
            assert(s.until(PushState::RAISE_RELEASE,300) && s.out.reason=="assumed_push_in_done");
            assert(s.until(PushState::VERIFY_DELIVERY,2500));
        }
      }
    }
    { // 对准转向期间货物必然不动：不能被当成“卡住”触发重推（run58：对准转了2s+就重推）。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        t.assume_align_push=true;t.assume_align_tol_rad=1e-4f;t.assume_stuck_us=300000;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.cargo_on=true;s.cargo_zone={s.px+.08f,.30f};
        int aligned=0;
        for(int i=0;i<200 && s.out.state==PushState::ENTER;++i){
            s.tick();
            if(s.out.reason=="assumed_push_align_turn"){++aligned;continue;}
            if(aligned)break;
        }
        assert(aligned>=8 && s.out.reason!="assumed_stuck_repush");
    }
    { // B：框被图像底边截断、或块偏离目标半区，都不能当作“已在区内”（跳过推入）。
      for(int k=0;k<2;++k){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m+(k?.20f:0.f),.12f};s.cargo_clip=k==0;
        for(int i=0;i<2500 && s.out.state!=PushState::VERIFY_DELIVERY && s.out.state!=PushState::SAFE_STOP;++i){s.tick();assert(s.out.reason!="assumed_inside_after_release");}
        assert(s.task.takeDropZoneEvent().find("event=inside_after_release")==std::string::npos);
      }
    }
    { // A：核验无区定位时用图像回退：框底高过区前沿一定比例才算进区；贴边/被截断不算，并触发重推。
      for(int k=0;k<3;++k){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::VERIFY_DELIVERY,2500) && s.out.reason=="assumed_inside_after_release");
        s.vref_visible=false;s.img_frac=k==0?.5f:k==1?.02f:.5f;s.cargo_clip=k==2;
        if(k==0){
            assert(s.until(PushState::TURN_SCAN,100) && s.out.reason=="delivered_visual");
            assert(s.until(PushState::SAFE_STOP,400) && s.out.reason=="demo_next_trip_ready");
        } else {
            assert(s.until(PushState::ENTER,100) && s.out.reason=="assumed_verify_repush");
        }
      }
    }
    { // C：直线推进也要有主动偏航探测：进 ENTER 满 1.2 s 后才出现，且左右交替；平时(之前)不发偏航。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;t.assume_probe_gap_us=300000;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.py_max=s.py+.01f;
        const uint64_t t0=s.in.now_us;float first=0,second=0;bool prev=false;
        for(int i=0;i<200 && s.out.state==PushState::ENTER;++i){
            s.tick();
            const bool probe=s.out.reason=="assumed_push_imu_probe"&&s.out.motion.wz_rps!=0;
            if(probe&&!prev){
                assert(s.in.now_us-t0>=s.tune.assume_probe_first_us);
                if(first==0)first=s.out.motion.wz_rps;else if(second==0)second=s.out.motion.wz_rps;
            }
            prev=probe;
        }
        assert(first!=0&&second!=0&&first*second<0);
    }
    { // 未达持续时间的IMU卡住不触发（短暂顿挫不算）。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.in.imu_blocked_us=s.tune.assume_imu_block_us-100000;
        for(int i=0;i<20;++i){s.tick();assert(s.out.reason!="assumed_stuck_repush");}
    }
    { // 目标半区由货物类别决定并在整趟锁定：物资→左(-0.15)，伤员→右(+0.15)，日志可见。
      for(int inj=0;inj<2;++inj){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        if(inj){s.in.label="injured_person";s.in.corridor=inv(0,0,1);s.task.first_=true;}
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::CARRY,300) && s.out.reason=="assumed_capture");
        assert(s.until(PushState::RAISE_RELEASE,800));
        const float want=inj?s.tune.injured_half_x_m:s.tune.supply_half_x_m;
        if(inj)assert(s.px>-.15f+.15f); else assert(std::abs(s.px-want)<.06f); // 右移趋向伤员半区
        assert(s.until(PushState::SAFE_STOP,3000));
        const std::string ev=s.task.takeDropZoneEvent();
        const char *half=inj?"half=injured":"half=supply";
        assert(ev.find("event=lock")!=std::string::npos && ev.find(half)!=std::string::npos);
        assert(ev.find("event=lock")==ev.rfind("event=lock"));
        assert(ev.find("event=release")!=std::string::npos && ev.find("event=push_done")!=std::string::npos);
        assert(ev.find(inj?"half=supply":"half=injured")==std::string::npos);
        assert(ev.find(inj?"target_x=0.150":"target_x=-0.150")!=std::string::npos);
      }
    }
    { // 盲冲IMU偏航保持 + 落框前偏航门：盲冲中被推偏后先原地对正再落框；卡住不转则放弃本次（不拉黑）。
      for(int stuck=0;stuck<2;++stuck){
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;
        s.toRush();s.in.target_valid=false;
        s.phi+=.25f; // 盲区里被推偏约14°；stuck=0时盲冲中的IMU保持自行纠回，stuck=1时底盘不转（phi被钉住）
        bool realign=false;const float phi0=s.phi;
        for(int i=0;i<200&&s.out.state!=PushState::LOWER_FRAME&&s.out.state!=PushState::CAPTURE_FAIL;++i){
            if(stuck){s.tick();s.phi=phi0;} else s.tick();
            if(s.out.reason=="prelower_realign")realign=true;
            if(s.out.reason=="assumed_blind_rush"&&s.out.motion.wz_rps!=0)assert(s.out.motion.wz_rps<0); // 偏左 -> 向右修正
        }
        if(stuck){assert(realign&&s.out.state==PushState::CAPTURE_FAIL&&s.out.reason=="prelower_heading_drift");}
        if(!stuck) assert(s.out.state==PushState::LOWER_FRAME&&std::abs(s.phi-(phi0-.25f))<t.rush_lower_yaw_tol_rad+.02f);
      }
    }
    { // TEMP_ASSUMPTION assume_all_safe: no zone, no multi-view evidence, target lost below the view.
      TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;
      Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
      s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;
      s.toRush();s.in.target_valid=false;
      assert(s.until(PushState::LOWER_FRAME,80) && s.out.reason=="assumed_enclosure_target_below_view");
      assert(s.until(PushState::CARRY,300) && s.out.reason=="assumed_capture" && s.out.batch_size==1);
      const float x0=s.px,y0=s.py;
      assert(s.until(PushState::RAISE_RELEASE,400) && s.out.reason=="assumed_drop_point");
      assert(std::abs(std::hypot(s.px-x0,s.py-y0)-s.tune.assume_carry_m)<.03f);
      assert(s.until(PushState::BACK_OUT,100) && s.out.reason.rfind("assumed_release_back_off",0)==0);
      assert(s.until(PushState::ENTER,600) && s.out.reason=="assumed_release_then_push");
      const float xp=s.px,yp=s.py;
      assert(s.until(PushState::RAISE_RELEASE,400) && s.out.reason=="assumed_push_in_done");
      assert(std::abs(std::hypot(s.px-xp,s.py-yp)-s.tune.assume_push_m)<.03f);
      assert(s.until(PushState::BACK_OUT,100) && s.out.reason=="frame_raised_at_drop");
      const float x1=s.px,y1=s.py;
      assert(s.until(PushState::VERIFY_DELIVERY,400) && s.out.reason=="assumed_backed_out");
      const float back=std::hypot(s.px-x1,s.py-y1);
      assert(back>=s.tune.assume_back_m-.01f && back<=s.tune.assume_back_m+.03f);
      assert(s.until(PushState::TURN_SCAN,200) && s.out.delivered_total==1 && s.out.reason=="delivered_unverified");
      assert(s.until(PushState::SAFE_STOP,400) && s.out.reason=="demo_next_trip_unverified");
      for(auto st:{PushState::CAPTURE_FAIL,PushState::LOST_HOLD,PushState::ABORT_DROP})assert(!s.visited.count(st));
    }
    { // Without the assumption the same evidence gap still stops at VERIFY_CAPTURE.
      TaskTuning t;t.demo_carry_once=true;
      Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
      s.zone_visible=false;s.multi_evidence=false;s.toRush();s.hold(inv(1));
      assert(s.until(PushState::SAFE_STOP,300) && s.out.reason=="multi_view_unconfirmed");
    }
    { Sim s;s.in.target_region_valid=false;
      for(int i=0;i<60;++i)assert(s.tick().state!=PushState::APPROACH);
    }
    { Sim s;s.in.opponent_zone_clear=false;s.in.target_valid=false;
      for(int i=0;i<20;++i)assert(stopped(s.tick().motion));
    }
    { Sim s;s.toCarry();s.in.zone_identity_verified=false;
      for(int i=0;i<20;++i){auto o=s.tick();assert(o.motion.vx_mps==0 && o.state!=PushState::GATE);}
    }
    { Sim s;s.toCarry();s.navigation_ready=false;
      for(int i=0;i<30;++i)assert(s.tick().motion.vx_mps==0);
    }
    { Sim s;s.toCarry();assert(s.until(PushState::RAISE_RELEASE,400));s.drop_available=false;
      for(int i=0;i<10;++i){auto o=s.tick();assert(stopped(o.motion)&&o.motion.gripper_offset==20);}
    }
    { // Live-loop guard permits camera work only at zero speed in explicit phases.
        PushOutput out; out.state=PushState::LOWER_FRAME;
        assert(stationaryPitchWork(out,"pitch_moving"));
        assert(stationaryPitchWork(out,"pitch_not_calibrated"));
        assert(!stationaryPitchWork(out,"imu_not_synchronized"));
        assert(!stationaryPitchWork(out,"stale_frame"));
        out.motion.vx_mps=.1f;assert(!stationaryPitchWork(out,"pitch_moving"));
        inhibitUnmappedMotion(out.motion,false);assert(stopped(out.motion));
        out.state=PushState::SCAN;out.motion.camera_pitch_cdeg=500;
        assert(stationaryPitchWork(out,"pitch_moving"));
        assert(!stationaryPitchWork(out,"pitch_not_calibrated"));
        out.motion.wz_rps=.3f;assert(!stationaryPitchWork(out,"pitch_moving"));
        out.motion.wz_rps=0;out.motion.camera_pitch_cdeg=kCameraPitchInvalid;
        assert(!stationaryPitchWork(out,"pitch_moving"));
    }
    for(bool stalled : {false,true}) { // Actual failure: lose a cue at 40deg, return to SCAN at 5deg.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.55f;
        assert(s.until(PushState::CUE_APPROACH,80));
        for(int i=0;i<20;++i)s.tick();assert(s.pitch==4000);
        s.in.target_valid=false;assert(s.until(PushState::SCAN,90));
        assert(stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==500);
        s.servo_responds=!stalled;
        bool waited=false,rotated=false;
        for(int i=0;i<65;++i) {
            const auto prior=s.out;s.tick();
            if(!s.in.camera_pitch_stable) {
                waited=true;assert(stationaryPitchWork(prior,"pitch_moving"));
                assert(stopped(s.out.motion));
            }
            if(s.out.motion.wz_rps!=0) {
                assert(s.pitch==500&&s.in.camera_pitch_stable);rotated=true;break;
            }
            if(s.out.state==PushState::SAFE_STOP)break;
        }
        if(stalled)assert(s.out.state==PushState::SAFE_STOP&&s.out.reason=="camera_pitch_timeout");
        else assert(waited&&rotated&&s.out.state==PushState::SCAN);
    }
    { // Exercise complete 5 -> 40 -> 5 cycles with live-loop mapping rejection.
        Sim s;s.live_camera_guard=true;s.pitch=500;s.pitch_still=3;s.out.motion.camera_pitch_cdeg=500;
        s.toCarry();s.deliver();assert(s.out.delivered_total==1);
    }
    { // Full first trip: no target or zone at the start, rotate, approach from
      // farther away, capture, then search for a zone that is still out of view.
        TaskTuning t;t.carry_budget_us=90000000;
        Sim s(t);s.live_camera_guard=true;s.pitch=500;s.pitch_still=3;s.out.motion.camera_pitch_cdeg=500;
        s.zone_visible=false;s.in.target_valid=false;
        bool scanned=false;
        for(int i=0;i<30;++i){s.tick();assert(s.out.motion.vx_mps==0);scanned|=s.out.motion.wz_rps>0;}
        assert(scanned&&s.out.state==PushState::SCAN);
        s.in.target_valid=true;s.in.distance_m=.8f;
        bool approached=false;
        for(int i=0;i<300&&s.out.state!=PushState::RUSH;++i) {
            s.in.distance_m-=std::max(0.f,s.out.motion.vx_mps)*.05f;s.tick();
            approached|=s.out.state==PushState::APPROACH&&s.out.motion.vx_mps>0;
        }
        assert(approached&&s.out.state==PushState::RUSH);
        s.hold(inv(1));
        for(int i=0;i<100&&s.out.state!=PushState::CARRY;++i) {
            s.in.distance_m-=std::max(0.f,s.out.motion.vx_mps)*.05f;s.tick();
        }
        assert(s.out.state==PushState::CARRY);
        bool searched=false;
        for(int i=0;i<550;++i) {
            s.tick();assert(s.out.state==PushState::CARRY&&s.out.motion.vx_mps==0&&s.out.motion.gripper_offset==20);
            searched|=s.out.motion.wz_rps>0&&s.out.reason=="searching_own_zone";
        }
        assert(searched); // >25s with no zone no longer aborts the requested first-trip search
        s.zone_visible=true;
        assert(s.until(PushState::GATE,1200));
        s.deliver();assert(s.out.delivered_total==1&&s.out.first_ordinary_delivered);
        assert(s.until(PushState::SCAN,400));
    }
    { // Missing zone eventually stops with the gripper closed, never a guessed route.
        TaskTuning t;t.carry_budget_us=90000000;t.zone_search_budget_us=2000000;
        Sim s(t);s.toCarry();s.zone_visible=false;
        assert(s.until(PushState::SAFE_STOP,100));
        assert(s.out.reason=="own_zone_search_timeout"&&stopped(s.out.motion)&&s.out.motion.gripper_offset==20);
    }
    { // Count a full 10s of forward commands; blocked time is not travelled time.
        TaskTuning t;t.startup_advance_us=10000000;PushTask task(t);PushObservation in;
        in.run=in.safety_ok=in.path_safe=in.opponent_zone_clear=true;
        in.camera_pitch_cdeg=500;in.camera_pitch_stable=true;
        int forward_frames=0;PushOutput out;
        for(int i=0;i<140;++i) {
            in.now_us+=100000;in.path_safe=!(i>=30&&i<40);
            out=task.update(in);
            if(out.motion.vx_mps>0){++forward_frames;assert(out.motion.vx_mps==.1f&&out.motion.wz_rps==0);}
            if(out.state==PushState::SCAN)break;
        }
        assert(out.state==PushState::SCAN&&forward_frames==100);
    }
    { // START_ADVANCE holds the IMU yaw seen at the start: a bump that turns the body right is steered back left.
        TaskTuning t;t.startup_advance_us=10000000;PushTask task(t);PushObservation in;
        in.run=in.safety_ok=in.path_safe=in.opponent_zone_clear=true;
        in.camera_pitch_cdeg=500;in.camera_pitch_stable=true;in.heading_valid=true;in.heading_rad=3.1f; // near the +pi wrap
        PushOutput out;
        for(int i=0;i<3;++i){in.now_us+=100000;out=task.update(in);}
        assert(out.state==PushState::START_ADVANCE&&out.motion.vx_mps>0&&out.motion.wz_rps==0&&out.reason=="startup_forward");
        in.heading_rad=3.1f-.10f; // yaw dropped: body turned right
        in.now_us+=100000;out=task.update(in);
        assert(out.motion.vx_mps>0&&out.motion.wz_rps>0&&out.reason=="startup_forward_hold");
        in.heading_rad=-3.1f+.0f; // wraps past +pi: still a small left-turn error of 0.083 rad
        in.now_us+=100000;out=task.update(in);
        assert(out.motion.wz_rps<0&&out.reason=="startup_forward_hold");
        in.heading_rad=3.1f+.02f; // inside the deadband
        in.now_us+=100000;out=task.update(in);assert(out.motion.wz_rps==0);
        in.heading_valid=false;in.heading_rad=0;in.now_us+=100000;out=task.update(in);assert(out.motion.wz_rps==0&&out.motion.vx_mps>0);
    }
    for(const auto* label:{"ordinary_supply","injured_person","core_supply","dangerous_object"}) {
        TaskTuning t;t.enable_search_cues=true;Sim s(t);
        s.in.label=label;s.in.target_is_search_cue=true;s.in.distance_m=.34f;
        assert(s.until(PushState::SELECT_CARGO,80));
        assert(stopped(s.out.motion)&&s.out.motion.gripper_offset==20);
        assert(s.out.search_cue_id==7&&s.out.capture_target_id==-1&&!s.out.cue_contact_allowed);
        s.in.label="ordinary_supply";s.in.target_id=9;s.in.target_is_search_cue=false;s.in.distance_m=.7f;
        assert(s.until(PushState::APPROACH,10));
        assert(s.out.search_cue_id==7&&s.out.capture_target_id==9&&s.out.target_id==9);
        s.tick();assert(s.out.state==PushState::APPROACH&&s.out.motion.gripper_offset==20);
        s.in.distance_m=.2f;s.toCarry();s.deliver();
        assert(s.out.first_ordinary_delivered&&s.out.delivered_total==1);
        assert(!s.visited.count(PushState::CLEAR_PILE));
    }
    { // No eligible cargo: stop to reselect, then rescan; never push the cue.
        TaskTuning t;t.enable_search_cues=true;Sim s(t);
        s.in.label="dangerous_object";s.in.target_is_search_cue=true;s.in.distance_m=.34f;
        assert(s.until(PushState::SELECT_CARGO,80));
        s.in.target_is_search_cue=false;
        for(int i=0;i<90;++i){
            s.tick();assert(stopped(s.out.motion)&&s.out.motion.gripper_offset==20);
            if(s.out.state==PushState::SCAN)break;
        }
        assert(s.out.state==PushState::SCAN&&s.out.reason=="no_eligible_cargo");
        assert(!s.visited.count(PushState::CLEAR_PILE)&&!s.out.first_ordinary_delivered);
    }
    { // Switch at 0.6m, before the old 0.35m gate; never drive while pitch moves.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.55f;
        assert(s.until(PushState::CUE_APPROACH,80));
        s.tick();assert(stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==4000);
        for(int i=0;i<15;++i){s.tick();if(!s.in.camera_pitch_stable)assert(stopped(s.out.motion));}
        assert(s.pitch==4000);
        s.in.distance_m=.34f;assert(s.out.state==PushState::NEAR_REACQUIRE);
        assert(s.out.motion.camera_pitch_cdeg==4000);
        s.in.target_is_search_cue=false;s.in.target_id=9;s.in.distance_m=.2f;
        s.toCarry();assert(s.out.motion.camera_pitch_cdeg==500);
    }
    { // Re-identify a new green ID at 40 degrees without lifting the camera.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.label="core_supply";s.in.distance_m=.55f;
        assert(s.until(PushState::NEAR_REACQUIRE,80));
        s.in.target_valid=false;
        for(int i=0;i<20;++i){s.tick();assert(stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==4000);}
        s.in.target_valid=true;s.in.target_is_search_cue=false;s.in.target_id=99;s.in.label="ordinary_supply";
        assert(s.until(PushState::APPROACH,10));assert(s.out.capture_target_id==99&&s.out.motion.camera_pitch_cdeg==4000);
        s.in.target_valid=false;assert(s.until(PushState::NEAR_REACQUIRE,20));
        assert(s.out.motion.camera_pitch_cdeg==4000);
        assert(s.until(PushState::SAFE_STOP,90));assert(s.out.reason=="near_reacquire_failed_stop");
    }
    { // Empty near views get one return to FAR; the second failure latches at NEAR.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.55f;
        assert(s.until(PushState::NEAR_REACQUIRE,80));s.in.target_valid=false;
        assert(s.until(PushState::SCAN,100));
        s.in.target_valid=true;s.in.target_id=100;
        assert(s.until(PushState::NEAR_REACQUIRE,80));s.in.target_valid=false;
        assert(s.until(PushState::SAFE_STOP,100));
        for(int i=0;i<10;++i){s.tick();assert(stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==4000);}
    }
    { // 无线索的直接APPROACH：远处保持5°，贴近时停车下压到20°，再到40°（只下压不回抬）。
        TaskTuning t;t.enable_search_cues=false;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.2f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        for(int i=0;i<5;++i){s.tick();assert(s.out.state==PushState::APPROACH&&s.out.motion.camera_pitch_cdeg==500);}
        s.in.distance_m=.39f;s.tick();assert(s.out.state==PushState::APPROACH&&stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==2000);
        for(int i=0;i<12;++i){s.in.distance_m=.45f;s.tick();assert(s.out.motion.camera_pitch_cdeg==2000);} // 到位后远离也不回抬
        s.in.distance_m=.26f;
        for(int i=0;i<12&&s.out.motion.camera_pitch_cdeg!=4000;++i)s.tick();
        assert(s.out.state==PushState::APPROACH&&s.out.motion.camera_pitch_cdeg==4000);
    }
    { // 下压后跟踪器改发新ID：改绑继续接近，不进入丢失搜索。
        TaskTuning t;t.enable_search_cues=false;t.enable_lost_search=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.2f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.39f;s.tick();assert(s.out.motion.camera_pitch_cdeg==2000);
        s.in.target_id+=1;s.in.distance_m=.47f;
        for(int i=0;i<40;++i){s.tick();assert(s.out.state==PushState::APPROACH);}
        assert(s.out.motion.vx_mps>0&&s.out.capture_target_id==s.in.target_id);
    }
    { // 下压后图像无效/重新确认期间（远超 hold_grace_frames）不判丢失；窗口内恢复并换新ID则改绑，窗口耗尽才进丢失搜索。
        for(int late=0;late<2;++late){
            TaskTuning t;t.enable_search_cues=false;t.enable_lost_search=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.2f;
            Sim s(t);s.in.distance_m=.5f;
            assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.39f;s.tick();assert(s.out.motion.camera_pitch_cdeg==2000);
            s.in.target_valid=false;
            const int gap=late?120:25;
            bool lost=false;
            for(int i=0;i<gap;++i){s.tick();if(s.out.state!=PushState::APPROACH){lost=true;break;}}
            if(late){assert(lost&&s.out.state==PushState::LOST_SEARCH);continue;}
            assert(!lost);
            s.in.target_valid=true;s.in.target_id+=1;s.in.distance_m=.47f;
            for(int i=0;i<40;++i){s.tick();assert(s.out.state==PushState::APPROACH);}
            assert(s.out.motion.vx_mps>0&&s.out.capture_target_id==s.in.target_id);
        }
    }
    { // 假设模式：方框张开后跟踪器换新ID，PREPARE改绑继续而不是退回SCAN。
        TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.2f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.19f;assert(s.until(PushState::PREPARE,80));
        s.in.target_id+=1;
        for(int i=0;i<40&&s.out.state==PushState::PREPARE;++i)s.tick();
        assert(s.out.state!=PushState::SCAN&&s.out.capture_target_id==s.in.target_id);
    }
    { // 密集成堆：冲入后高速摆扫分散，之后清掉黑名单并继续接近；稀疏则不触发；冷却期内不重复触发。
        TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        struct D{bool rush=false,swing=false,done=false,prepared=false;float max_phi=0,max_wz=0;};
        const auto run=[&](int n_clutter,bool disperse){
            TaskTuning tt=t;tt.disperse=disperse;
            Sim s(tt);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.25f;D d;
            for(int i=0;i<400&&!d.prepared;++i){
                s.in.clutter_body.clear();
                for(int k=0;k<n_clutter;++k)s.in.clutter_body.push_back({-.12f+.08f*k,.25f+((k%2)?.06f:-.06f)});
                s.in.clutter_ts_us=s.in.now_us;
                s.tick();
                d.max_phi=std::max(d.max_phi,std::abs(s.phi));d.max_wz=std::max(d.max_wz,std::abs(s.out.motion.wz_rps));
                if(s.out.reason=="disperse_rush")d.rush=true;
                if(s.out.reason=="disperse_swing")d.swing=true;
                if(s.out.reason=="disperse_done")d.done=true;
                if(s.out.state==PushState::PREPARE)d.prepared=true;
                if(d.done&&s.out.state==PushState::APPROACH)s.in.distance_m=.19f;
                if(s.out.state!=PushState::APPROACH&&s.out.state!=PushState::PREPARE&&s.out.state!=PushState::SCAN)break;
            }
            return d;
        };
        auto dense=run(4,true);
        assert(dense.rush&&dense.swing&&dense.max_wz>=1.5f&&dense.max_phi>=.5f);
        auto sparse=run(1,true);assert(!sparse.rush&&!sparse.swing);
        auto off=run(4,false);assert(!off.rush&&!off.swing);

        { // 打散后目标丢失：先在20°视角搜索（近处货物），关闭开关则仍从5°开始。
          const auto after=[&](bool mid){
              TaskTuning tm=t;tm.enable_lost_search=true;tm.post_clear_mid_view=mid;
              Sim s(tm);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
              s.in.distance_m=.25f;bool done=false;
              for(int i=0;i<400&&!done;++i){
                  s.in.clutter_body.clear();
                  for(int k=0;k<4;++k)s.in.clutter_body.push_back({-.12f+.08f*k,.25f+((k%2)?.06f:-.06f)});
                  s.in.clutter_ts_us=s.in.now_us;s.tick();
                  if(s.out.reason=="disperse_done")done=true;
              }
              assert(done);
              s.in.clutter_body.clear();s.in.target_valid=false;
              for(int i=0;i<60&&s.out.state!=PushState::LOST_SEARCH;++i)s.tick();
              assert(s.out.state==PushState::LOST_SEARCH);
              return int(s.out.motion.camera_pitch_cdeg);
          };
          assert(after(true)==2000);assert(after(false)==500);
        }
    }
    { // 看不到普通物资但前方有成团的其他物体：转向它、冲入并摆扫（两趟），然后回到搜索；单个也触发；太远/太偏/过期/开关关闭/已有目标都不触发。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_min=1;t.blind_scatter_first_turn_rad=0.f;t.blind_scatter_passes=2; // this block exercises the two-pass mechanics; the default is one pass
        struct B{bool align=false,swing=false,done=false,budget=false,back=false;int starts=0,rushes=0,scans_after=0,pitch_after=-1;float min_phi=0,max_phi=0;};
        const auto run=[&](std::vector<cv::Point2f> body_pts,bool on,bool fresh,bool ordinary,int max_n,int ticks,bool follow=false){
            TaskTuning tt=t;tt.blind_scatter=on;tt.blind_scatter_max=max_n;
            Sim s(tt);s.in.target_valid=ordinary;s.in.distance_m=.9f;
            const float x0=s.px,y0=s.py;
            std::vector<cv::Point2f> world;
            for(const auto&b:body_pts)world.push_back({x0+b.x,y0+b.y});
            B b;std::string prev;
            for(int i=0;i<ticks;++i){
                s.in.clutter_body.clear();
                const float c=std::cos(s.phi),sn=std::sin(s.phi);
                if(follow)s.in.clutter_body=body_pts;
                else for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
                if(fresh)s.in.clutter_ts_us=s.in.now_us;
                s.tick();
                b.min_phi=std::min(b.min_phi,s.phi);b.max_phi=std::max(b.max_phi,s.phi);
                if(s.out.state==PushState::SCATTER){
                    if(s.out.reason=="blind_scatter_align")b.align=true;
                    if(s.out.reason=="disperse_swing")b.swing=true;
                    if(s.out.reason=="disperse_rush"&&prev!="disperse_rush")++b.rushes;
                    if(s.out.reason=="blind_scatter_back")b.back=true;
                }
                if(s.out.reason=="blind_scatter_start")++b.starts;
                if(s.out.reason=="blind_scatter_done")b.done=true;
                if(s.out.reason=="blind_scatter_timeout"||s.out.reason=="blind_scatter_align_timeout")b.budget=true;
                prev=s.out.reason;
                if(b.done&&s.out.state==PushState::SCAN&&++b.scans_after==1)b.pitch_after=s.out.motion.camera_pitch_cdeg;
            }
            return b;
        };
        const std::vector<cv::Point2f> group{{.20f,.50f},{.32f,.56f},{.26f,.44f}};
        auto one=run(group,true,true,false,1,2400);
        assert(one.starts==1&&one.align&&one.swing&&one.done&&!one.budget);
        assert(std::max(-one.min_phi,one.max_phi)>=.35f);
        assert(one.rushes>=2);
        assert(one.scans_after>0);
        assert(one.pitch_after==500); // 默认无20°档时保持5°
        { TaskTuning tp=t;tp.track_pitch_cdeg=4000;tp.intermediate_pitch_cdeg=2000;
          const auto pa=[&](bool mid){ TaskTuning tm=tp;tm.post_clear_mid_view=mid;tm.blind_scatter_max=1;
              Sim s(tm);s.in.target_valid=false;s.in.distance_m=.9f;const float x0=s.px,y0=s.py;int pitch=-1;bool done=false;
              const std::vector<cv::Point2f> world{{x0+.20f,y0+.50f},{x0+.32f,y0+.56f},{x0+.26f,y0+.44f}};
              for(int i=0;i<2400&&pitch<0;++i){
                  s.in.clutter_body.clear();const float c=std::cos(s.phi),sn=std::sin(s.phi);
                  for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
                  s.in.clutter_ts_us=s.in.now_us;s.tick();
                  if(s.out.reason=="blind_scatter_done")done=true;
                  if(done&&s.out.state==PushState::SCAN)pitch=s.out.motion.camera_pitch_cdeg;
              }
              return pitch; };
          assert(pa(true)==2000);assert(pa(false)==500); }
        auto again=run(group,true,true,false,3,6000,true);assert(again.starts>=2&&again.starts<=3);
        auto lone=run({{.20f,.50f}},true,true,false,1,2400);assert(lone.starts==1&&lone.align&&lone.swing&&lone.done&&!lone.budget&&lone.rushes>=2);
        auto wideL=run({{-.70f,1.05f}},true,true,false,1,2400);assert(wideL.starts==1&&wideL.done&&!wideL.budget&&std::max(-wideL.min_phi,wideL.max_phi)>=.35f);
        auto far=run({{.0f,2.2f},{.1f,2.25f}},true,true,false,3,600);assert(far.starts==0);
        auto side=run({{1.1f,.8f}},true,true,false,3,600,true);assert(side.starts==0);
        auto stale=run(group,true,false,false,3,600);assert(stale.starts==0);
        auto off=run(group,false,true,false,3,600);assert(off.starts==0);
        auto have=run(group,true,true,true,3,200);assert(have.starts==0);
        assert(one.back); // backs out after the swings
        for(const float frac:{1.f,.35f}){ // 退出距离=前向行程*blind_scatter_back_frac：1.0回到起点附近；默认只退一部分，不把推开的物块甩远。
          TaskTuning tb=t;tb.blind_scatter_back_budget_us=30000000;tb.blind_scatter_back_frac=frac;Sim s(tb);s.in.target_valid=false;s.in.distance_m=.9f;const float x0=s.px,y0=s.py;float sx=0,sy=0,max_d=0;bool back=false,started=false;
          std::vector<cv::Point2f> world{{x0+.0f,y0+.60f},{x0+.06f,y0+.64f},{x0-.05f,y0+.56f}};
          for(int i=0;i<2400;++i){
              s.in.clutter_body.clear();const float c=std::cos(s.phi),sn=std::sin(s.phi);
              for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
              s.in.clutter_ts_us=s.in.now_us;s.tick();
              if(s.out.reason=="blind_scatter_start"){started=true;sx=s.px;sy=s.py;}
              if(started)max_d=std::max(max_d,std::hypot(s.px-sx,s.py-sy));
              if(s.out.reason=="blind_scatter_back")back=true;
              if(started&&s.out.reason=="blind_scatter_done")break;
          }
          assert(started&&back&&max_d>.3f);
          const float left=std::hypot(s.px-sx,s.py-sy);
          if(frac>=1.f)assert(left<.4f*max_d); else assert(left>.45f*max_d); }
        { // 默认参数：单个/两个非目标物体不打散；成团(>=3)在转够一圈前也不打散；偏出±40°的团不打散。
          TaskTuning td;td.enable_search_cues=false;
          const auto starts=[&](std::vector<cv::Point2f> pts,float first_turn,int ticks){
              TaskTuning tt=td;tt.blind_scatter_first_turn_rad=first_turn;Sim s(tt);s.in.target_valid=false;s.in.distance_m=.9f;int n=0;
              for(int i=0;i<ticks;++i){s.in.clutter_body=pts;s.in.clutter_ts_us=s.in.now_us;s.tick();if(s.out.reason=="blind_scatter_start")++n;}
              return n; };
          assert(starts({{.20f,.50f}},0.f,300)==0);
          assert(starts({{.20f,.50f},{.26f,.44f}},0.f,300)==0);
          assert(starts(group,0.f,300)==1);
          assert(starts(group,1000.f,300)==0);
          assert(starts({{.95f*.8f,.40f},{.78f,.46f},{.74f,.36f}},0.f,300)==0);
          const auto startsMix=[&](int n_ord){
              Sim s(td);s.all_ordinary=false;s.in.target_valid=false;s.in.distance_m=.9f;int n=0;
              for(int i=0;i<300;++i){s.in.clutter_body=group;s.in.clutter_ordinary.assign(group.begin(),group.begin()+n_ord);s.in.clutter_ts_us=s.in.now_us;s.tick();if(s.out.reason=="blind_scatter_start")++n;}
              return n; };
          assert(startsMix(0)==0); // injured / dangerous only: no sweep
          assert(startsMix(1)==1); } // one ordinary inside the cluster is enough
    }
    { // 普通物资只有很少像素（远/小）：不追，较短时间后直接冲入打散；像素足够大则正常锁定；太远不触发；对准途中看到像素足够的才中止。
        TaskTuning t;t.enable_search_cues=false;
        const auto mk=[&](float ratio,float dist,Sim &s){s.in.target_valid=true;s.in.distance_m=dist;s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=ratio;};
        const auto first=[&](Sim &s,const char *why,int ticks){for(int i=0;i<ticks;++i){s.tick();if(s.out.reason==why)return i;}return -1;};
        { Sim s(t);mk(.002f,.9f,s);assert(first(s,"blind_scatter_start",400)>=0);
          assert(first(s,"blind_scatter_done",3000)>=0); }
        { Sim s(t);mk(.006f,.9f,s);assert(first(s,"blind_scatter_start",300)<0&&s.out.state==PushState::APPROACH); }
        { Sim s(t);mk(.002f,1.8f,s);assert(first(s,"blind_scatter_start",300)<0); }
        { TaskTuning tw=t;tw.weak_target_scatter=false;Sim s(tw);mk(.002f,.9f,s);assert(first(s,"blind_scatter_start",300)<0); }
        { Sim s(t);mk(.002f,.9f,s);s.in.heading_error=.6f;assert(first(s,"blind_scatter_start",400)>=0);
          s.tick();assert(s.out.state==PushState::SCATTER&&s.out.reason=="blind_scatter_align");
          mk(.006f,.9f,s);assert(first(s,"blind_scatter_target_seen",200)>=0); }
    }
    { // 很远(>0.8 m)的锁定不可信：不论群还是单个锁定都直接前驱(不被远处的锁定打断)；<=0.8 m恢复原决策。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_max_y_m=.5f;t.blind_scatter_far_max_y_m=4.f;t.blind_scatter_half_x_m=2.5f;t.blind_scatter_first_turn_rad=0.f;t.sweep_confirm_wait_us=0; // legacy: the cluster decision is not held back by a confirming lock
        struct R{bool group=false,far_start=false,approach=false,seen=false,chase=false;};
        const auto run=[&](bool locked,float dist,std::vector<cv::Point2f> pts,int ticks){
            Sim s(t);s.in.target_valid=locked;s.in.distance_m=dist;s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=.006f;R r;
            for(int i=0;i<ticks;++i){s.in.clutter_body=pts;s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.reason=="blind_scatter_far_group")r.group=true;
                if(s.out.reason=="blind_scatter_far_start")r.far_start=true;
                if(s.out.reason=="blind_scatter_approach")r.approach=true;
                if(s.out.reason=="blind_scatter_target_seen")r.seen=true;
                if(s.out.state==PushState::APPROACH)r.chase=true;}
            return r; };
        const std::vector<cv::Point2f> grp{{.20f,2.30f},{-.25f,2.60f}};
        { const R r=run(true,2.5f,grp,400);assert(r.group&&r.approach&&!r.seen); }
        { const R r=run(true,2.5f,{},400);assert(r.group&&r.approach&&!r.seen); }
        { const R r=run(true,.7f,{{.20f,.60f},{-.25f,.80f}},400);assert(!r.group&&!r.approach); }
        { const R r=run(false,2.5f,{{0.f,2.5f},{.20f,2.30f},{-.25f,2.60f}},600);assert(r.far_start&&r.approach); }
        { const R r=run(false,2.5f,{{0.f,2.5f},{1.5f,2.30f},{-1.2f,2.60f}},600);assert(!r.far_start&&!r.group); }
    }
    { // 单个远处锁定(世界固定，杂物视图过期)：直接前驱，锁定进入0.5 m窗口后交给正常APPROACH，不扫。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_max_y_m=.5f;t.blind_scatter_far_max_y_m=4.f;t.blind_scatter_half_x_m=2.5f;t.blind_scatter_first_turn_rad=0.f;t.sweep_confirm_wait_us=0;
        Sim s(t);s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=.006f;
        const float x0=s.px,y0=s.py;const cv::Point2f w{x0+.05f,y0+1.90f};
        bool far=false,chase=false,near_group=false;
        for(int i=0;i<3000&&!chase;++i){
            const float c=std::cos(s.phi),sn=std::sin(s.phi);const float dx=w.x-s.px,dy=w.y-s.py;
            const cv::Point2f lk{dx*c+dy*sn,-dx*sn+dy*c};s.in.clutter_body.clear();
            s.in.target_valid=lk.y>.1f;s.in.distance_m=std::hypot(lk.x,lk.y);s.in.heading_error=std::atan2(-lk.x,lk.y);s.tick();
            if(s.out.reason=="blind_scatter_far_group")far=true;
            if(s.out.reason=="blind_scatter_near_group")near_group=true;
            if(s.out.state==PushState::APPROACH)chase=true;
        }
        assert(far&&chase&&!near_group);
    }
    { // 前驱途中群里的普通物资锁定不打断：群进入0.5 m窗口后直接扫(near_group)，不回单目标追踪。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_max_y_m=.5f;t.blind_scatter_far_max_y_m=4.f;t.blind_scatter_half_x_m=2.5f;t.blind_scatter_first_turn_rad=0.f;t.sweep_confirm_wait_us=0;
        Sim s(t);s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=.006f;
        const float x0=s.px,y0=s.py;
        const std::vector<cv::Point2f> world{{x0+.0f,y0+1.90f},{x0+.08f,y0+1.98f},{x0-.07f,y0+1.84f}};
        bool far=false,near_group=false,seen=false,chase=false,done=false;
        for(int i=0;i<3000&&!done;++i){
            const float c=std::cos(s.phi),sn=std::sin(s.phi);s.in.clutter_body.clear();
            for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
            const cv::Point2f lk=s.in.clutter_body[0];s.in.target_valid=lk.y>.1f;s.in.distance_m=std::hypot(lk.x,lk.y);s.in.heading_error=std::atan2(-lk.x,lk.y);
            s.in.clutter_ts_us=s.in.now_us;s.tick();
            if(s.out.reason=="blind_scatter_far_group"||s.out.reason=="blind_scatter_far_start")far=true;
            if(s.out.reason=="blind_scatter_near_group")near_group=true;
            if(!near_group){if(s.out.reason=="blind_scatter_target_seen")seen=true;if(s.out.state==PushState::APPROACH)chase=true;}
            if(near_group&&s.out.reason=="blind_scatter_done")done=true;
        }
        assert(far&&near_group&&!seen&&!chase&&done);
    }
    { // 开局首次远距离接近(整场一次)：首个远群前驱用高速、剩余0.35 m降回常态；随后首次近距打散用高转速摆框；之后的远驱/打散都是常态；总开关关闭则全程常态。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_max_y_m=.5f;t.blind_scatter_far_max_y_m=4.f;t.blind_scatter_half_x_m=2.5f;t.blind_scatter_first_turn_rad=0.f;
        t.first_burst=true;t.first_burst_speed=1.f;t.first_burst_swing_wz=5.f;t.assume_motion_ratio=1.f;t.sweep_gap_us=0;t.sweep_escalate_max=0;t.blind_scatter_cooldown_us=0;
        struct B{bool fast=false,slow=false,burst_swing=false,plain_swing=false,done=false,impact=false;float fast_vx=0,slow_vx=0,burst_wz=0,plain_wz=0,impact_min=10.f,impact_max=0.f;};
        const auto runGroup=[&](Sim &s){
            const float x0=s.px,y0=s.py;
            const std::vector<cv::Point2f> world{{x0+.0f,y0+1.90f},{x0+.08f,y0+1.98f},{x0-.07f,y0+1.84f}};
            B b;bool scattered=false;
            for(int i=0;i<3000&&!b.done;++i){
                const float c=std::cos(s.phi),sn=std::sin(s.phi);s.in.clutter_body.clear();
                for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
                const cv::Point2f lk=s.in.clutter_body[0];s.in.target_valid=lk.y>.1f;s.in.distance_m=std::hypot(lk.x,lk.y);s.in.heading_error=std::atan2(-lk.x,lk.y);
                s.in.clutter_ts_us=s.in.now_us;s.tick();
                const std::string &r=s.out.reason;
                if(r=="first_burst_approach"){b.fast=true;b.fast_vx=std::max(b.fast_vx,s.out.motion.vx_mps);}
                if(r=="first_burst_impact" && s.out.motion.vx_mps>0){b.impact=true;b.impact_min=std::min(b.impact_min,s.out.motion.vx_mps);b.impact_max=std::max(b.impact_max,s.out.motion.vx_mps);}
                if(r=="blind_scatter_approach"){b.slow=true;b.slow_vx=std::max(b.slow_vx,s.out.motion.vx_mps);}
                if(r=="first_burst_swing"){b.burst_swing=true;b.burst_wz=std::max(b.burst_wz,std::abs(s.out.motion.wz_rps));}
                if(r=="disperse_swing"){b.plain_swing=true;b.plain_wz=std::max(b.plain_wz,std::abs(s.out.motion.wz_rps));}
                if(s.out.state==PushState::SCATTER)scattered=true;
                if(scattered&&r=="blind_scatter_done")b.done=true; // SCAN keeps the previous end reason until it overwrites it
            }
            return b; };
        const auto newSim=[&](TaskTuning tt){Sim s(tt);s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=.006f;return s;};
        { Sim s=newSim(t);
          const B a=runGroup(s);
          assert(a.done&&a.fast&&a.fast_vx==1.f&&a.slow&&a.slow_vx>0&&a.slow_vx<=.4f+1e-4f); // 先高速，后段降回常态
          assert(a.burst_swing&&a.burst_wz==5.f&&!a.plain_swing); // 首次打散整段高转速
          const B b=runGroup(s); // 第二个群：整场只一次
          assert(b.done&&!b.fast&&!b.burst_swing&&b.plain_swing&&b.plain_wz==t.disperse_wz&&b.slow_vx<=.4f+1e-4f); }
        { // Requested opening: keep full speed through near impact, only once; normal sweeps do not inherit it.
          TaskTuning full=t;full.first_burst_full_impact=true;full.sweep_observation_guard=true;
          Sim s=newSim(full);const B a=runGroup(s);
          assert(a.done&&a.fast&&!a.slow&&a.impact&&a.impact_min==1.f&&a.impact_max==1.f&&a.burst_swing);
          const B b=runGroup(s);assert(b.done&&!b.fast&&!b.impact&&!b.burst_swing&&b.plain_swing); }
        { // Speed override must not undo a forward-clearance veto.
          TaskTuning full=t;full.first_burst_full_impact=true;Sim s=newSim(full);
          s.in.target_valid=false;assert(s.until(PushState::SCAN,30));for(int i=0;i<20;++i)s.tick();
          s.task.enter(PushState::SCATTER,s.in.now_us,"test_impact");s.task.sweep_phase_=4;s.task.burst_swing_=true;
          s.task.disperse_rush_m_=.5f;s.task.sweep_start_us_=s.in.now_us;s.in.path_safe=false;
          s.tick();assert(stopped(s.out.motion)); }
        { TaskTuning off=t;off.first_burst=false;Sim s=newSim(off);
          const B a=runGroup(s);assert(a.done&&!a.fast&&!a.burst_swing&&a.plain_swing&&a.plain_wz==off.disperse_wz); }
        { // 驱动距离不足first_burst_min_drive_m：不算首次远驱，不消耗；打散也用常态。
          TaskTuning tm=t;tm.first_burst_min_drive_m=9.f;Sim s=newSim(tm);
          const B a=runGroup(s);assert(a.done&&!a.fast&&!a.burst_swing&&a.plain_swing); }
        { // 远处少像素锁定反复靠近(approach_max次)：只有第一次靠近是高速。
          const auto mk=[&](float ratio,float dist,Sim &s){s.in.target_valid=true;s.in.distance_m=dist;s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=ratio;};
          Sim s(t);mk(.002f,1.1f,s);int fast_stints=0,slow_stints=0;std::string prev;float max_slow=0;
          for(int i=0;i<4000;++i){s.tick();const std::string &r=s.out.reason;
              if(r=="first_burst_approach"&&prev!=r)++fast_stints;
              if(r=="blind_scatter_approach"){max_slow=std::max(max_slow,s.out.motion.vx_mps);if(prev!=r&&fast_stints>=1)++slow_stints;}
              prev=r;}
          assert(fast_stints==1&&max_slow<=.4f+1e-4f); }
        { // 普通锁定先出现(没有远群前驱)：不触发高速接近；之后的首个近距打散也是常态转速。
          Sim s=newSim(t);s.in.target_valid=true;s.in.distance_m=.7f;bool burst=false;
          for(int i=0;i<200;++i){s.tick();if(s.out.reason=="first_burst_approach"||s.out.reason=="first_burst_swing")burst=true;}
          assert(!burst); }
    }
    { // 扫除台账：扫完后20 s间隔内不再扫；每扫完一次所需密度+1(上限+2)；每趟总次数有上限。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_first_turn_rad=0.f;t.blind_scatter_max=10;
        const auto sweeps=[&](std::vector<cv::Point2f> pts,TaskTuning tt,int ticks,std::vector<int> *at=nullptr){
            Sim s(tt);s.in.target_valid=false;s.in.distance_m=.9f;int n=0;
            for(int i=0;i<ticks;++i){s.in.clutter_body=pts;s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.reason=="blind_scatter_start"){++n;if(at)at->push_back(i);}}
            return n; };
        const std::vector<cv::Point2f> g3{{.05f,.30f},{-.10f,.38f},{.12f,.42f}};
        const std::vector<cv::Point2f> g5{{.05f,.30f},{-.10f,.38f},{.12f,.42f},{.00f,.35f},{.08f,.36f}};
        { TaskTuning off=t;off.sweep_gap_us=0;off.sweep_escalate_max=0;off.sweep_total_max=100;off.sweep_time_budget_us=1ull<<60;
          assert(sweeps(g3,off,6000)>=2); } // 无台账：同一堆被反复扫
        assert(sweeps(g3,t,6000)==1); // 有台账：第二次要求4个，3个的堆不再扫
        { std::vector<int> at;const int n=sweeps(g5,t,6000,&at);assert(n>=2);assert((at[1]-at[0])*100>=20000+0); } // 间隔>=20 s(含扫动本身)
        { TaskTuning one=t;one.sweep_total_max=1;one.sweep_escalate_max=0;assert(sweeps(g5,one,6000)==1); }
    }
    { // 追踪丢失时群还在视野里：直接扫(lost_group_sweep)，不进LOST_SEARCH原地转圈；没有群则照常进LOST_SEARCH，转圈中看到群也立刻扫。
        TaskTuning t;t.enable_search_cues=false;t.enable_lost_search=true;t.blind_scatter_max_y_m=.5f;t.blind_scatter_first_turn_rad=0.f;
        const std::vector<cv::Point2f> grp{{.05f,.30f},{-.10f,.38f},{.12f,.42f}};
        const auto toApproach=[&](Sim &s){s.in.target_valid=true;s.in.distance_m=.7f;s.in.clutter_body.clear();
            for(int i=0;i<300;++i){s.tick();if(s.out.state==PushState::APPROACH)return true;}return false;};
        { Sim s(t);assert(toApproach(s));s.in.target_valid=false;bool sweep=false,lost=false;
          for(int i=0;i<80;++i){s.in.clutter_body=grp;s.in.clutter_ts_us=s.in.now_us;s.tick();
              if(s.out.reason=="lost_group_sweep")sweep=true;if(s.out.state==PushState::LOST_SEARCH)lost=true;}
          assert(sweep&&!lost); }
        { Sim s(t);assert(toApproach(s));s.in.target_valid=false;bool lost=false;
          for(int i=0;i<80;++i){s.in.clutter_body.clear();s.tick();if(s.out.state==PushState::LOST_SEARCH)lost=true;}
          assert(lost);
          bool sweep=false;
          for(int i=0;i<40;++i){s.in.clutter_body=grp;s.in.clutter_ts_us=s.in.now_us;s.tick();if(s.out.reason=="lost_group_sweep")sweep=true;}
          assert(sweep&&s.out.state==PushState::SCATTER); }
    }
    { // 远处(0.5~1.4 m)的少像素普通物资：先低速靠近，再从近距离触发冲入打散；不超过常态速度；靠近次数有上限。
        TaskTuning t;t.enable_search_cues=false;t.blind_scatter_max_y_m=.5f;t.max_speed=.4f;t.approach_speed=.4f;
        const auto mk=[&](float ratio,float dist,Sim &s){s.in.target_valid=true;s.in.distance_m=dist;s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=ratio;};
        { Sim s(t);mk(.002f,1.1f,s);bool far_start=false,approach=false,approached=false,start=false;float max_vx=0;
          for(int i=0;i<3000&&!approached;++i){s.tick();
              if(s.out.reason=="blind_scatter_far_start")far_start=true;
              if(s.out.reason=="blind_scatter_approach"){approach=true;max_vx=std::max(max_vx,s.out.motion.vx_mps);}
              if(s.out.reason=="blind_scatter_start")start=true;
              if(s.out.reason=="blind_scatter_approached")approached=true;}
          assert(far_start&&approach&&approached&&!start&&max_vx>0&&max_vx<=.4f+1e-4f&&s.out.state==PushState::SCAN);
          mk(.002f,.45f,s);
          for(int i=0;i<400&&!start;++i){s.tick();if(s.out.reason=="blind_scatter_start")start=true;}
          assert(start); }
        { TaskTuning tn=t;tn.blind_scatter_approach_max=0;Sim s(tn);mk(.002f,1.1f,s);
          for(int i=0;i<600;++i){s.tick();assert(s.out.reason!="blind_scatter_far_start"&&s.out.reason!="blind_scatter_approach");} }
    }
    { // 只有启动段允许超过常态max_speed；进入SCAN后恢复常态上限。
        TaskTuning t;t.startup_advance_us=3000000;t.startup_advance_speed=1.f;t.max_speed=.4f;
        PushTask task(t);PushObservation in;
        in.run=in.safety_ok=in.path_safe=in.opponent_zone_clear=true;
        in.camera_pitch_cdeg=500;in.camera_pitch_stable=true;
        bool fast=false;
        for(int i=0;i<200;++i){in.now_us+=100000;const auto out=task.update(in);
            if(out.state==PushState::START_ADVANCE&&out.motion.vx_mps>0){assert(out.motion.vx_mps==1.f);fast=true;}
            else assert(std::abs(out.motion.vx_mps)<=.4f+1e-4f);}
        assert(fast); }
    { // 扫掠通道内有竖放伤员：低框原地转向扫开（IMU量角），转回后再进入PREPARE；不在通道内/过期/就是目标本身则不动。
        TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        struct R{int outs=0,backs=0,turn_signs=0;bool prepared=false,left_approach_early=false;float first_wz=0,max_phi=0;};
        const auto run=[&](float ux,float uy,bool fresh,bool persist,int ticks){
            Sim s(t);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.19f;R r;bool blocker=true;
            for(int i=0;i<ticks&&!r.prepared;++i){
                if(blocker){s.in.upright_body={{ux,uy}};if(fresh)s.in.upright_ts_us=s.in.now_us;}
                s.tick();
                r.max_phi=std::max(r.max_phi,std::abs(s.phi));
                if(s.out.state==PushState::PREPARE){r.prepared=true;break;}
                if(s.out.state!=PushState::APPROACH){r.left_approach_early=true;break;}
                if(s.out.reason=="sweep_upright_out"&&s.out.motion.wz_rps!=0){++r.outs;if(!r.first_wz)r.first_wz=s.out.motion.wz_rps;assert(s.out.motion.vx_mps==0&&s.out.motion.gripper_offset==20);}
                if(s.out.reason=="sweep_upright_back"&&s.out.motion.wz_rps!=0)++r.backs;
                if(!persist&&r.backs>0)blocker=false;
            }
            r.turn_signs=int(std::abs(s.phi)*1000);
            return r;
        };
        auto right=run(.10f,.19f,true,false,200);
        assert(right.outs>0&&right.backs>0&&right.first_wz<0&&right.max_phi>=.8f&&right.max_phi<=1.1f&&right.prepared&&!right.left_approach_early);
        assert(right.turn_signs<120); // returned to the original heading before PREPARE
        auto left=run(-.10f,.19f,true,false,200);
        assert(left.outs>0&&left.first_wz>0&&left.prepared);
        auto wide=run(.22f,.19f,true,false,200); // outer part of the widened corridor
        assert(wide.outs>0&&wide.first_wz<0&&wide.prepared);
        auto far=run(.40f,.19f,true,false,200);
        assert(far.outs==0&&far.prepared);
        auto behind=run(.10f,.45f,true,false,200); // beyond target + extra reach
        assert(behind.outs==0&&behind.prepared);
        auto self=run(.02f,.19f,true,false,200); // same physical object as the target
        assert(self.outs==0&&self.prepared);
        auto stale=run(.10f,.19f,false,false,200);
        assert(stale.outs==0&&stale.prepared);
        { // sweep time is not charged to the approach budget: no approach_timeout back to SCAN, and the sweep turns at sweep_wz
            TaskTuning b=t;b.approach_budget_us=1500000;
            Sim s(b);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.19f;bool swept=false,prepared=false;float minwz=9;
            for(int i=0;i<200&&!prepared;++i){
                if(!swept)s.in.upright_body={{.10f,.19f}},s.in.upright_ts_us=s.in.now_us;
                s.tick();
                if(s.out.reason=="sweep_upright_out"||s.out.reason=="sweep_upright_back"){swept=true;minwz=std::min(minwz,std::abs(s.out.motion.wz_rps));}
                if(s.out.reason=="sweep_upright_done")s.in.upright_body.clear();
                assert(s.out.state==PushState::APPROACH||s.out.state==PushState::PREPARE);
                prepared=s.out.state==PushState::PREPARE;
            }
            assert(swept&&prepared&&minwz>=b.sweep_wz-1e-4f);
        }
        auto stuck=run(.10f,.19f,true,true,600); // block never leaves: two attempts, then carry on
        assert(stuck.outs>0&&stuck.prepared&&stuck.left_approach_early==false);
    }
    { // 假设模式：20°阶段进入PREPARE/RUSH后保持20°，不为开框切40°。
        TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.near_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.33f;assert(s.until(PushState::PREPARE,120));
        for(int i=0;i<30&&s.out.state==PushState::PREPARE;++i){s.tick();assert(s.out.motion.camera_pitch_cdeg==2000);}
    }
    { // 真实模式：目标在0.33m（>intermediate_to_near）时不进PREPARE，先在20°下贴近，进入范围后才PREPARE。
        TaskTuning t;t.enable_search_cues=false;t.track_pitch_cdeg=4000;t.near_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.33f;
        for(int i=0;i<40;++i){s.tick();assert(s.out.state==PushState::APPROACH);}
        assert(s.out.reason=="approach_to_prepare_range"&&s.out.motion.vx_mps>0&&s.out.motion.camera_pitch_cdeg==2000);
        s.in.distance_m=.25f;assert(s.until(PushState::PREPARE,120));
    }
    { // 距离卡在0.29 m（>0.27、<=0.32）不再前进：3 s后视为到位进PREPARE，不拖到approach_timeout。
        TaskTuning t;t.enable_search_cues=false;t.track_pitch_cdeg=4000;t.near_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.29f;
        bool stalled=false;
        for(int i=0;i<400&&!stalled;++i){s.tick();stalled=s.out.reason=="approach_range_stalled";}
        assert(stalled&&s.out.state==PushState::PREPARE);
    }
    { // 5 -> 20 -> 40, reacquiring a new ID at each stable angle before advancing.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.2f;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));assert(s.out.motion.camera_pitch_cdeg==2000);
        s.in.target_is_search_cue=false;s.in.target_id=101;
        for(int i=0;i<30&&s.out.state!=PushState::MID_APPROACH;++i) {
            s.tick();assert(stopped(s.out.motion));
        }
        assert(s.out.state==PushState::MID_APPROACH&&s.pitch==2000&&s.in.camera_pitch_stable);
        s.tick();assert(s.out.motion.vx_mps>0&&s.out.motion.vx_mps<=.4f);
        s.in.distance_m=.21f;s.tick();assert(s.out.state==PushState::MID_APPROACH);
        s.in.distance_m=.19f;assert(s.until(PushState::NEAR_REACQUIRE,15));assert(stopped(s.out.motion)&&s.out.motion.gripper_offset==0);
        assert(s.out.motion.camera_pitch_cdeg==4000);s.in.target_id=102;
        assert(s.until(PushState::APPROACH,40));assert(s.out.capture_target_id==102&&s.pitch==4000);
        s.in.distance_m=.2f;s.toCarry();assert(s.out.motion.camera_pitch_cdeg==500);
    }
    { // Open at 20deg/0.35m, wait for ACK, then retain open across 40deg reassociation.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.intermediate_to_near_m=.2f;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;s.in.target_id=101;
        assert(s.until(PushState::MID_APPROACH,40));s.in.distance_m=.34f;s.gripper_responds=false;
        for(int i=0;i<5;++i){s.tick();assert(stopped(s.out.motion));}
        assert(s.out.motion.gripper_offset==0&&s.out.motion.camera_pitch_cdeg==2000);
        assert(s.out.reason=="mid_gripper_feedback_wait");
        s.gripper_responds=true;s.tick();assert(s.out.motion.vx_mps>0);
        s.in.distance_m=.19f;s.tick();assert(s.out.state==PushState::NEAR_REACQUIRE&&s.out.motion.gripper_offset==0);
        s.in.target_id=102;assert(s.until(PushState::APPROACH,40));
        assert(s.out.motion.gripper_offset==0&&s.out.motion.camera_pitch_cdeg==4000);
    }
    for(int mode=0;mode<4;++mode) {
        // Field 20deg setup (near switch 0.27). 0: tracked to 0.26 switches to 40deg.
        // 1: open jaw loses the box at 0.29 (image bottom) -> stopped 40deg handoff, not SAFE_STOP.
        // 2: open jaw lost at 0.34 (beyond the mouth range) -> lost-search guard SAFE_STOP.
        // 3: closed jaw lost at 0.38 -> ordinary lost search, no handoff.
        TaskTuning t;t.enable_search_cues=true;t.enable_lost_search=true;t.track_pitch_cdeg=4000;
        t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;s.in.target_id=101;
        assert(s.until(PushState::MID_APPROACH,40));
        if(mode==3){s.in.distance_m=.38f;s.tick();assert(s.out.motion.gripper_offset==20);
            s.in.target_valid=false;assert(s.until(PushState::LOST_SEARCH,20));continue;}
        s.in.distance_m=.34f;
        for(int i=0;i<10&&!(s.out.motion.gripper_offset==0&&s.out.motion.vx_mps>0);++i)s.tick();
        assert(s.out.motion.gripper_offset==0&&s.out.motion.vx_mps>0);
        if(mode==0) {
            s.in.distance_m=.28f;s.tick();assert(s.out.state==PushState::MID_APPROACH&&s.out.motion.vx_mps>0);
            s.in.distance_m=.26f;s.tick();
            assert(s.out.state==PushState::NEAR_REACQUIRE&&stopped(s.out.motion)&&s.out.motion.gripper_offset==0);
            continue;
        }
        if(mode==1){s.in.distance_m=.29f;s.tick();assert(s.out.state==PushState::MID_APPROACH);}
        s.in.target_valid=false;s.tick();
        if(mode==1) {
            assert(s.out.state==PushState::NEAR_REACQUIRE&&stopped(s.out.motion)&&s.out.motion.gripper_offset==0);
            assert(s.out.reason=="open_jaw_lost_at_mouth_near_handoff"&&s.out.motion.camera_pitch_cdeg==4000);
            s.in.target_valid=true;s.in.target_id=102;s.in.distance_m=.22f;
            assert(s.until(PushState::APPROACH,40)&&s.out.motion.gripper_offset==0);
        } else {
            assert(s.until(PushState::SAFE_STOP,20)&&s.out.reason=="lost_search_jaw_or_load_unsafe");
        }
    }
    { // Aligned approach drives straight: a small bearing is not lifted to min_turn_wz.
        Sim s;s.in.distance_m=.8f;s.in.heading_error=.03f;assert(s.until(PushState::APPROACH,30));
        s.tick();assert(s.out.motion.vx_mps>0&&s.out.motion.wz_rps==0);
        s.in.heading_error=-.05f;s.tick();assert(s.out.motion.vx_mps>0&&s.out.motion.wz_rps==0);
        s.in.heading_error=.1f;s.tick();assert(s.out.motion.vx_mps>0&&s.out.motion.wz_rps==.4f);
    }
    for(int mode=0;mode<3;++mode) {
        // Open jaw, corridor changes: 0 one-frame flicker continues, 1 confirmed dangerous
        // object stops, 2 confirmed second ordinary on the first trip is carried as a new batch.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.intermediate_to_near_m=.2f;
        Sim s(t);
        s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;s.in.target_id=101;
        assert(s.until(PushState::MID_APPROACH,40));s.in.distance_m=.34f;
        for(int i=0;i<10&&!(s.out.motion.gripper_offset==0&&s.out.motion.vx_mps>0);++i)s.tick();
        assert(s.out.motion.gripper_offset==0&&s.out.motion.vx_mps>0);
        s.in.corridor=mode==1?inv(1,0,0,1):inv(2);s.tick();
        assert(s.out.state==PushState::MID_APPROACH&&stopped(s.out.motion)&&s.out.reason=="open_jaw_corridor_recheck");
        if(mode==0) {
            s.in.corridor=inv(1);s.tick();
            assert(s.out.state==PushState::MID_APPROACH&&s.out.motion.vx_mps>0&&s.out.motion.gripper_offset==0);
            continue;
        }
        s.tick();assert(stopped(s.out.motion));
        s.tick();
        if(mode==1) {
            assert(s.out.state==PushState::SAFE_STOP&&s.out.reason=="open_jaw_corridor_changed");
            assert(s.out.verdict==RuleVerdict::DANGEROUS&&stopped(s.out.motion));
        } else {
            assert(s.out.state==PushState::MID_APPROACH&&s.out.motion.vx_mps>0&&s.out.batch_size==2);
            s.tick();assert(s.out.motion.vx_mps>0);                  // new batch accepted, no recheck
        }
    }
    { // A blocked closed-jaw approach stops immediately and rejects its cargo ID.
        TaskTuning t;t.enable_search_cues=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;s.in.target_id=101;
        assert(s.until(PushState::MID_APPROACH,40));s.in.path_safe=false;s.tick();
        assert(s.out.state==PushState::MID_REACQUIRE&&stopped(s.out.motion));
        const auto rejected=s.task.rejectedTargets(s.in.now_us);
        assert(std::find(rejected.begin(),rejected.end(),101)!=rejected.end());
    }
    { // 张框距离内航向略超限：底盘小wz(<0.5)转不动(实测)，改用0.55rad/s按IMU量角的短脉冲，转到位再张框；原地小wz会一直卡住。
        struct R{bool prepared=false,blocked_moved=false;int pulse_ticks=0;float first_wz=0,max_dev=0;bool bad_wz=false;};
        const auto run=[&](float e0,bool pulse,int ticks,bool turn_blocked){
            TaskTuning t;t.rush_pulse_turn=pulse;
            Sim s(t);s.in.distance_m=.3f;s.in.heading_error=e0;assert(s.until(PushState::APPROACH,30));
            if(turn_blocked){s.in.directional_clearance_valid=true;s.in.turn_safe=false;s.in.arc_safe=false;}
            const float p0=s.phi;R r;
            for(int i=0;i<ticks&&!r.prepared;++i){
                s.in.heading_error=e0-(s.phi-p0);
                s.tick();
                if(s.out.state==PushState::PREPARE){r.prepared=true;break;}
                assert(s.out.state==PushState::APPROACH);
                if(s.out.reason=="rush_pulse_turn"&&s.out.motion.wz_rps!=0){
                    ++r.pulse_ticks;if(!r.first_wz)r.first_wz=s.out.motion.wz_rps;
                    if(std::abs(s.out.motion.wz_rps)!=t.rush_pulse_wz||s.out.motion.vx_mps!=0)r.bad_wz=true;
                }
                if(turn_blocked&&s.out.motion.wz_rps!=0)r.blocked_moved=true;
                r.max_dev=std::max(r.max_dev,std::abs(s.phi-p0));
                if(std::abs(s.out.motion.wz_rps)<.5f)s.out.motion.wz_rps=0; // base dead band
            }
            return r;
        };
        auto stuck=run(.10f,false,200,false);
        assert(!stuck.prepared&&stuck.pulse_ticks==0&&stuck.max_dev<1e-4f); // old behaviour: commands 0.4, base never turns
        auto left=run(.10f,true,400,false);
        assert(left.prepared&&left.pulse_ticks>0&&left.first_wz>0&&!left.bad_wz&&left.max_dev>.07f&&left.max_dev<.14f);
        auto right=run(-.15f,true,400,false);
        assert(right.prepared&&right.first_wz<0&&!right.bad_wz&&right.max_dev>.11f&&right.max_dev<.20f);
        auto big=run(.30f,true,100,false); // beyond rush_pulse_max_rad: legacy rotate-in-place command, no pulse
        assert(big.pulse_ticks==0);
        auto blocked=run(.10f,true,100,true); // unsafe turn stays blocked even for a pulse
        assert(!blocked.prepared&&!blocked.blocked_moved);
    }
    { // Forward permission alone cannot authorize an unsafe turn or curved approach.
        Sim s;s.in.distance_m=.8f;assert(s.until(PushState::APPROACH,30));
        s.in.directional_clearance_valid=true;s.in.turn_safe=false;s.in.arc_safe=false;
        s.in.heading_error=.6f;s.tick();assert(stopped(s.out.motion)&&s.out.reason=="turn_sweep_blocked"); // in-place turn stays blocked
        s.in.heading_error=.1f;s.tick(); // blocked arc with a verified straight corridor keeps going straight
        assert(s.out.motion.vx_mps>0&&s.out.motion.wz_rps==0&&s.out.reason=="turn_sweep_blocked_straight");
        s.in.path_safe=false;s.tick();assert(stopped(s.out.motion)); // no straight corridor: stop
        s.in.path_safe=true;
        s.in.heading_error=0;s.tick();assert(s.out.motion.vx_mps>0);
    }
    { // Both signs escape the yaw deadband, zero and safety stops remain zero.
        Sim s;s.in.distance_m=.3f;s.in.heading_error=.07f;
        assert(s.until(PushState::APPROACH,30));
        s.tick();assert(s.out.motion.vx_mps==0&&s.out.motion.wz_rps==.4f);
        s.in.heading_error=-.07f;s.tick();assert(s.out.motion.wz_rps==-.4f);
        s.in.path_safe=false;s.tick();assert(stopped(s.out.motion));
        s.in.path_safe=true;s.in.heading_error=0;s.tick();assert(stopped(s.out.motion));
        assert(s.out.state==PushState::PREPARE);
    }
    for(bool lose_feedback : {false,true}) {
        TaskTuning t;t.enable_search_cues=true;t.enable_short_push=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;
        assert(s.until(PushState::MID_APPROACH,40));
        s.in.distance_m=.34f;s.in.corridor=inv(1,0,0,1);s.in.clear_push_safe=true;
        assert(s.until(PushState::CLEAR_PILE,15));
        if(lose_feedback){s.gripper_responds=false;s.tick();assert(s.out.state==PushState::SAFE_STOP&&stopped(s.out.motion));}
        else {
            int moving=0;
            for(int i=0;i<12 && s.out.state!=PushState::CLEAR_PAUSE;++i) {
                s.in.distance_m=std::max(.19f,s.in.distance_m-.04f);
                s.tick();assert(s.out.motion.gripper_offset==20&&s.out.motion.wz_rps==0&&s.out.delivered_total==0);
                if(s.out.motion.vx_mps>0){assert(s.out.motion.vx_mps==.4f);++moving;}
            }
            assert(moving>0&&moving<=6&&s.out.state==PushState::CLEAR_PAUSE&&s.out.clearing_attempts==1);
            assert(s.out.reason=="clear_target_distance_reached");
            assert(s.until(PushState::MID_REACQUIRE,15));
        }
    }
    { // A pushed object maintaining its distance must never cause unlimited forward travel.
        TaskTuning t;t.enable_search_cues=true;t.enable_short_push=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;
        assert(s.until(PushState::MID_APPROACH,40));
        s.in.distance_m=.34f;s.in.corridor=inv(1,0,0,1);s.in.clear_push_safe=true;
        assert(s.until(PushState::CLEAR_PILE,15));
        assert(s.until(PushState::SAFE_STOP,40));
        assert(s.out.reason=="clear_distance_not_reached"&&stopped(s.out.motion));
    }
    { // Repeated blocked paths reselect at rest instead of latching an attempt fault.
        TaskTuning t;t.enable_search_cues=true;t.enable_lost_search=true;
        t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;
        assert(s.until(PushState::MID_APPROACH,40));
        for(int attempt=0;attempt<5;++attempt) {
            s.in.path_safe=false;s.tick();
            assert(s.out.state==PushState::MID_REACQUIRE&&stopped(s.out.motion));
            assert(s.out.reason=="blocked_cargo_reselect");
            s.in.path_safe=true;++s.in.target_id;
            assert(s.until(PushState::MID_APPROACH,40));
        }
    }
    { // Open-jaw corridor rejected by neighbours: small closed-frame swings first; the old reselect path only after the budget is used.
        TaskTuning t;t.enable_search_cues=true;t.enable_lost_search=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.target_is_search_cue=true;s.in.distance_m=.39f;
        assert(s.until(PushState::MID_REACQUIRE,80));s.in.target_is_search_cue=false;
        assert(s.until(PushState::MID_APPROACH,40));
        for(int round=0;round<2;++round) {
            s.in.distance_m=.30f;s.in.corridor=inv(1,0,1);
            for(int i=0;i<10&&s.out.reason!="mid_neighbour_clear";++i)s.tick();
            assert(s.out.reason=="mid_neighbour_clear"&&s.out.motion.gripper_offset==20);
            bool swung=false;float max_dyaw=0;const float yaw0=s.phi;
            for(int i=0;i<400&&s.out.reason!="mid_neighbour_clear_done";++i) {
                s.tick();assert(s.out.state!=PushState::SAFE_STOP&&s.out.motion.gripper_offset==20);
                if(s.out.motion.wz_rps!=0)swung=true;
                max_dyaw=std::max(max_dyaw,std::abs(s.phi-yaw0));
            }
            assert(s.out.reason=="mid_neighbour_clear_done"&&s.out.state==PushState::MID_REACQUIRE&&swung);
            assert(max_dyaw>.3f&&max_dyaw<.8f);
            s.in.corridor=inv(1);
            assert(s.until(PushState::MID_APPROACH,40));
        }
        s.in.distance_m=.30f;s.in.corridor=inv(1,0,1);
        for(int i=0;i<10&&s.out.reason!="mid_open_corridor_reselect";++i)s.tick();
        assert(s.out.reason=="mid_open_corridor_reselect"&&s.out.state==PushState::MID_REACQUIRE);
    }
    { // Lost closed-jaw target: zero translation, bounded turn, new-ID confirmation.
        TaskTuning t;t.enable_search_cues=true;t.enable_lost_search=true;
        Sim s(t);s.in.distance_m=.8f;assert(s.until(PushState::APPROACH,30));
        s.in.target_valid=false;assert(s.until(PushState::LOST_SEARCH,20));
        s.tick();assert(s.out.motion.vx_mps==0&&s.out.motion.wz_rps==.4f);
        s.in.directional_clearance_valid=true;s.in.turn_safe=false;s.tick();assert(stopped(s.out.motion));
        s.in.turn_safe=true;s.in.target_valid=true;s.in.target_id=99;
        assert(s.until(PushState::CUE_APPROACH,5));assert(stopped(s.out.motion)&&s.out.target_id==99);
    }
    { // Repeated successful reacquisitions do not exhaust a lifetime attempt budget.
        TaskTuning t;t.enable_search_cues=true;t.enable_lost_search=true;
        Sim s(t);s.in.distance_m=.8f;assert(s.until(PushState::APPROACH,30));
        for(int attempt=0;attempt<5;++attempt) {
            s.in.target_valid=false;assert(s.until(PushState::LOST_SEARCH,20));
            s.tick();assert(s.out.motion.vx_mps==0&&s.out.motion.wz_rps==.4f);
            s.in.target_valid=true;s.in.target_id=100+attempt;
            assert(s.until(PushState::CUE_APPROACH,5));
            assert(stopped(s.out.motion)&&s.out.target_id==100+attempt);
        }
    }
    { // Searching never becomes endless when no valid target is seen.
        TaskTuning t;t.enable_lost_search=true;t.lost_search_rescan=false;Sim s(t);s.in.distance_m=.8f;
        assert(s.until(PushState::APPROACH,30));s.in.target_valid=false;
        assert(s.until(PushState::LOST_SEARCH,20));assert(s.until(PushState::SAFE_STOP,430));
        assert(s.out.reason=="lost_search_exhausted"&&stopped(s.out.motion));
    }
    { // 默认：两轮丢失搜索都没找到，不再整场停死，而是回到SCAN继续找，之后再看到目标仍能进入APPROACH。
        TaskTuning t;t.enable_lost_search=true;t.enable_search_cues=false;t.blind_scatter=false;Sim s(t);s.in.distance_m=.8f;
        assert(s.until(PushState::APPROACH,30));s.in.target_valid=false;
        assert(s.until(PushState::LOST_SEARCH,20));
        bool rescan=false;
        for(int i=0;i<1200&&!rescan;++i){s.tick();rescan=s.out.reason=="lost_search_rescan";assert(s.out.state!=PushState::SAFE_STOP);}
        assert(rescan&&s.out.state==PushState::SCAN);
        s.in.target_valid=true;s.in.target_id=77;s.in.distance_m=.8f;
        assert(s.until(PushState::APPROACH,40));
    }
    { // FAR sweep finds nothing: stop, tilt to 20deg, sweep once more, then stop for good.
        TaskTuning t;t.enable_lost_search=true;t.lost_search_rescan=false;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.distance_m=.8f;assert(s.until(PushState::APPROACH,80));s.in.target_valid=false;
        assert(s.until(PushState::LOST_SEARCH,20));
        bool far_turn=false;
        for(int i=0;i<430 && s.out.reason!="lost_search_mid_pitch";++i){s.tick();far_turn|=s.out.motion.wz_rps!=0&&s.out.motion.camera_pitch_cdeg==500;}
        assert(far_turn&&s.out.state==PushState::LOST_SEARCH&&stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==2000);
        bool mid_turn=false;
        for(int i=0;i<430 && s.out.state==PushState::LOST_SEARCH;++i){
            s.tick();if(s.out.motion.wz_rps!=0){assert(s.pitch==2000&&s.out.motion.vx_mps==0);mid_turn=true;}
        }
        assert(mid_turn&&s.out.state==PushState::SAFE_STOP&&s.out.reason=="lost_search_exhausted"&&stopped(s.out.motion));
    }
    { // Close cargo seen in the 20deg sweep keeps the 20deg view (MID_REACQUIRE), not FAR.
        TaskTuning t;t.enable_lost_search=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
        Sim s(t);s.in.distance_m=.8f;assert(s.until(PushState::APPROACH,80));s.in.target_valid=false;
        assert(s.until(PushState::LOST_SEARCH,20));
        for(int i=0;i<430 && s.out.reason!="lost_search_mid_pitch";++i)s.tick();
        assert(s.out.reason=="lost_search_mid_pitch");
        for(int i=0;i<200 && s.out.reason!="rotating_for_lost_target_mid";++i)s.tick();
        assert(s.out.reason=="rotating_for_lost_target_mid"&&s.pitch==2000);
        s.in.target_valid=true;s.in.target_id=77;s.in.distance_m=.3f;
        assert(s.until(PushState::MID_REACQUIRE,5));
        assert(stopped(s.out.motion)&&s.out.motion.camera_pitch_cdeg==2000&&s.out.reason=="lost_target_reacquired_mid");
        assert(s.until(PushState::MID_APPROACH,40));
    }
    { // Stop before replacing a blue cue with newly visible green; then confirm its lock.
        TaskTuning t;t.enable_search_cues=true;Sim s(t);
        s.in.label="dangerous_object";s.in.target_is_search_cue=true;s.in.distance_m=.8f;
        assert(s.until(PushState::CUE_APPROACH,40));
        s.in.label="ordinary_supply";s.in.target_id=99;s.tick();
        assert(s.out.state==PushState::SCAN&&stopped(s.out.motion)&&s.out.reason=="green_priority_reselect");
        assert(s.until(PushState::CUE_APPROACH,5));assert(s.out.target_id==99);
    }
    { // A detector cannot activate cue navigation when the feature is disabled.
        Sim s;s.in.label="dangerous_object";s.in.target_is_search_cue=true;
        for(int i=0;i<40;++i)s.tick();assert(s.out.state==PushState::SCAN);
    }
    { // Requested startup duration/speed and angular cap must reach real task output.
        TaskTuning t;t.startup_advance_us=15000000;t.startup_advance_speed=1.f;t.max_speed=1.f;
        t.scan_wz=t.turn_wz=t.max_wz=3.f;PushTask task(t);PushObservation in;
        in.run=in.safety_ok=in.path_safe=in.opponent_zone_clear=true;
        in.camera_pitch_cdeg=500;in.camera_pitch_stable=true;
        int forward_frames=0;PushOutput out;
        for(int i=0;i<440;++i){
            in.now_us+=100000;in.path_safe=!(i>=30&&i<40);out=task.update(in);
            if(out.motion.vx_mps>0){++forward_frames;assert(out.motion.vx_mps==1.f&&out.motion.wz_rps==0);}
            if(out.state==PushState::SCAN)break;
        }
        assert(forward_frames==150 && out.startup_commanded_us==15000000);
        in.now_us+=100000;out=task.update(in);assert(out.motion.vx_mps==0&&out.motion.wz_rps==3.f);
        assert(out.reason=="searching_target");
        in.path_safe=false;in.now_us+=100000;out=task.update(in);
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 && out.reason=="scan_path_blocked");
        in.path_safe=true;in.now_us+=100000;out=task.update(in);
        assert(out.motion.wz_rps==3.f);

    }
    { // CLI speed input rejects out-of-contract values and trailing garbage.
        const auto parse=[](std::vector<std::string> values){
            std::vector<char*> argv;for(auto& v:values)argv.push_back(v.data());
            return parseArgs(int(argv.size()),argv.data());
        };
        auto c=parse({"test","--startup-speed","0.2","--scan-wz","3","--turn-wz","3","--startup-advance-ms","40000"});
        assert(c.startup_speed_mps==.2f&&c.scan_wz_rps==3&&c.turn_wz_rps==3&&c.startup_advance_ms==40000);
        for(const auto& args:std::vector<std::vector<std::string>>{
            {"test","--startup-speed","1.01"},{"test","--scan-wz","nan"},
            {"test","--turn-wz","1junk"},{"test","--scan-wz","3.1"},{"test","--startup-speed","0"}}){
            bool rejected=false;try{parse(args);}catch(const std::exception&){rejected=true;}assert(rejected);
        }
    }
    rulesTests();
    captureTests();
    holding40StaticReviewTests();
    holdingOutsideMouthTests();
    corridorFootprintTests();
    blindRetreatTests();
    { // Interim firmware presets (2026-10: only -25/0/+25 deg): FAR 0, TRACK = NEAR = 2500.
        TaskTuning t; t.far_pitch_cdeg = 0; t.track_pitch_cdeg = t.near_pitch_cdeg = 2500;
        Sim s(t); s.toCarry(); s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered && s.out.reason == "delivered");
        assert(!s.visited.count(PushState::SAFE_STOP) && !s.visited.count(PushState::ABORT_DROP));
        // Every pitch the task commanded is one the MCU has.
        assert((s.commanded == std::set<int>{0, 2500}));
    }
    { // Transport uses a lowered enclosure until the destination, then lifts at rest.
        Sim s; s.toCarry(); assert(s.until(PushState::ENTER,400));
        assert(!s.visited.count(PushState::RAISE_RELEASE));
        bool advanced=false;
        for(int i=0;i<400 && s.out.state==PushState::ENTER;++i) {
            s.tick(); assert(s.out.motion.gripper_offset==20);
            if(s.out.motion.vx_mps>0) advanced=true;
        }
        assert(advanced && s.out.state==PushState::RAISE_RELEASE);
        assert(s.in.zone_estimate.bodyToZone({0,s.tune.hold_center_y_m}).y >= s.out.drop_centre_zone.y);
        s.gripper_responds=false;
        for(int i=0;i<80 && s.out.state!=PushState::SAFE_STOP;++i) {
            s.tick(); assert(stopped(s.out.motion));
            assert(s.out.state!=PushState::BACK_OUT);
        }
        assert(s.out.state==PushState::SAFE_STOP && s.out.motion.gripper_offset==0);
    }
    { // Full first ordinary trip: rush, enclose, carry, carry inside, raise at the drop point, back out, turn.
        Sim s; s.toCarry();
        assert(s.out.batch_size == 1 && s.out.motion.gripper_offset == 20);
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered && s.out.reason == "delivered");
        for (auto st : {PushState::SCAN, PushState::APPROACH, PushState::PREPARE, PushState::RUSH, PushState::LOWER_FRAME,
                        PushState::VERIFY_CAPTURE, PushState::CARRY, PushState::GATE, PushState::RAISE_RELEASE,
                        PushState::ENTER, PushState::BACK_OUT, PushState::VERIFY_DELIVERY, PushState::TURN_SCAN})
            assert(s.visited.count(st));
        assert(!s.visited.count(PushState::ABORT_DROP) && !s.visited.count(PushState::SAFE_STOP));
        assert(s.until(PushState::SCAN, 400));
        assert(s.out.batch_size == 0 && s.out.delivered_total == 1 && s.out.first_ordinary_delivered);
        assert(std::abs(s.phi) >= 1.5f); // turned away from the zone before rescanning
        // After the first delivery a core supply may be taken together with ordinary ones.
        s.in.label = "core_supply"; s.in.target_id = 8; s.in.corridor = inv(1, 1); s.in.corridor_occlusion_free = false;
        s.px = -.15f; s.py = -.75f; s.phi = 0;
        s.toCarry(inv(1, 1));
        assert(s.out.batch_size == 2 && s.out.verdict == RuleVerdict::OK);
        s.deliver();
        assert(s.out.delivered_total == 3);
        assert(s.until(PushState::SCAN, 400));
        // An injured person grabbed together with a supply is dropped, not carried.
        s.in.label = "injured_person"; s.in.target_id = 9; s.in.corridor = inv(0, 0, 1); s.in.corridor_occlusion_free = true;
        s.toRush(); s.hold(inv(1, 0, 1));
        assert(s.until(PushState::ABORT_DROP, 40) && s.out.verdict == RuleVerdict::INJURED_NOT_ALONE);
        s.hold({});
        assert(s.until(PushState::SCAN, 200) && s.out.batch_size == 0 && s.out.delivered_total == 3);
        // Alone, it goes to the injured half.
        s.in.target_id = 10; s.px = .15f; s.py = -.75f; s.phi = 0; s.task_injured = true;
        s.toCarry(inv(0, 0, 1));
        s.deliver();
        assert(s.out.delivered_total == 4 && s.in.zone_injured_count == 1);
        assert(s.px > 0); // pushed into the right (injured) half
    }
    { // A held image is only required after the close feedback, never while the frame is open.
        Sim s; s.toRush();
        assert(s.until(PushState::LOWER_FRAME, 10));
        assert(s.until(PushState::VERIFY_CAPTURE, 30));
        s.hold(inv(1));
        assert(s.until(PushState::CARRY, 15));
    }
    for (const auto *label : {"core_supply", "injured_person", "dangerous_object", "unknown"}) {
        // Before the first ordinary delivery only ordinary supplies are selectable.
        Sim s; s.in.label = label;
        for (int i = 0; i < 20; ++i) s.tick();
        assert(s.out.state == PushState::SCAN && s.out.target_id == -1 && s.out.motion.vx_mps == 0);
    }
    { // First trip that also swept a core supply into the frame: release and back off.
        Sim s; s.toRush(); s.hold(inv(1, 1));
        assert(s.until(PushState::ABORT_DROP, 40) && s.out.verdict == RuleVerdict::CORE_BEFORE_FIRST);
        s.tick(); assert(s.out.motion.gripper_offset == 0);
        s.hold({});
        assert(s.until(PushState::SCAN, 200));
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered);
        for (int i = 0; i < 10; ++i) assert(s.tick().state == PushState::SCAN); // primary blacklisted
    }
    { // A dangerous object enclosed with the target is never carried.
        Sim s; s.toRush(); s.hold(inv(1, 0, 0, 1));
        assert(s.until(PushState::ABORT_DROP, 40) && s.out.verdict == RuleVerdict::DANGEROUS);
    }
    { // Blue in PREPARE must never start a capture rush (pre-check disabled here so PREPARE itself is exercised).
        TaskTuning t; t.corridor_precheck=false; Sim s(t); s.in.corridor = inv(1,0,0,1);
        for (int i=0;i<80 && s.out.reason!="corridor_dangerous_back_off";++i) s.tick();
        assert(s.out.reason=="corridor_dangerous_back_off" && !s.visited.count(PushState::RUSH));
    }
    for (bool at_close : {false,true}) {
        Sim s; s.toRush(); s.in.distance_m=at_close?.19f:.30f;
        s.in.corridor=inv(1,0,0,1); s.tick();
        assert(s.out.state==PushState::CAPTURE_FAIL && stopped(s.out.motion));
        assert(s.out.motion.gripper_offset==0 && s.out.reason=="rush_corridor_dangerous");
        assert(!s.visited.count(PushState::LOWER_FRAME));
    }
    { // Selection-stage pre-check: a fatal corridor vetoes the target before the frame opens; no RUSH, and a legal pair passes.
        for (const auto &bad : {inv(1,0,0,1), inv(1,0,1), inv(1,1), inv(1,0,0,0,1), inv(4)}) {
            Sim s; s.in.corridor = bad;
            for (int i=0;i<60 && !(s.out.state==PushState::SCAN && s.out.reason.rfind("corridor_precheck_",0)==0);++i) s.tick();
            if(!(s.out.reason.rfind("corridor_precheck_",0)==0 && s.out.state==PushState::SCAN)) std::cerr<<"precheck case reason="<<s.out.reason<<" state="<<PushTask::name(s.out.state)<<" ord="<<bad.ordinary<<" core="<<bad.core<<" inj="<<bad.injured<<"\n";
            assert(s.out.reason.rfind("corridor_precheck_",0)==0 && s.out.state==PushState::SCAN);
            assert(!s.visited.count(PushState::PREPARE) && !s.visited.count(PushState::RUSH));
        }
        Sim pair; pair.in.corridor = inv(2); pair.toRush(); // two ordinary supplies are a legal trip
        Sim occluded; occluded.in.corridor_occlusion_free=false; // not a fatal verdict: left to PREPARE
        for (int i=0;i<40 && occluded.out.state!=PushState::PREPARE;++i) occluded.tick();
        assert(occluded.out.state==PushState::PREPARE || occluded.out.state==PushState::LOST_HOLD);
    }
    { // The pre-check cannot deadlock the search: after corridor_precheck_max vetoes PREPARE decides alone.
        TaskTuning t; t.corridor_precheck_max=2; Sim s(t); s.in.corridor = inv(1,0,0,1);
        for (int i=0;i<2000 && !s.visited.count(PushState::PREPARE);++i) s.tick();
        assert(s.visited.count(PushState::PREPARE) && !s.visited.count(PushState::RUSH));
        assert(s.out.reason!="corridor_precheck_dangerous" || s.out.state==PushState::SCAN);
    }
    { // Evidence that is not fatal for two frames then clears does not accumulate towards a veto.
        Sim s; assert(s.until(PushState::APPROACH, 40));
        s.in.corridor = inv(1,0,0,1); s.tick(); s.in.corridor = inv(1); 
        assert(s.until(PushState::RUSH, 80));
    }
    { // Lost corridor evidence also prevents advancing or closing.
        Sim s; s.toRush(); s.in.corridor_complete=false; s.tick();
        assert(s.out.state==PushState::CAPTURE_FAIL && stopped(s.out.motion));
        assert(s.out.reason=="rush_corridor_incomplete" && s.out.motion.gripper_offset==0);
    }
    { // Too many supplies in the corridor: the rush is not started (PREPARE path, pre-check disabled).
        TaskTuning t; t.corridor_precheck=false; Sim s(t); s.in.corridor = inv(4);
        for (int i = 0; i < 40 && s.out.reason != "corridor_too_many_supplies_back_off"; ++i) s.tick();
        assert(s.out.reason == "corridor_too_many_supplies_back_off" && !s.visited.count(PushState::RUSH));
    }
    { // ... nor when four end up held after closing.
        Sim s; s.in.corridor = inv(2); s.toRush(); s.hold(inv(3));
        assert(s.until(PushState::ABORT_DROP, 20) && s.out.verdict == RuleVerdict::TOO_MANY_SUPPLIES);
    }
    { // Before the first delivery an occluded corridor might hide a core supply.
        Sim s; s.in.corridor_occlusion_free = false;
        for (int i = 0; i < 40 && s.out.reason != "corridor_occluded_back_off"; ++i) s.tick();
        assert(s.out.reason == "corridor_occluded_back_off" && !s.visited.count(PushState::RUSH));
        assert(s.out.state == PushState::LOST_HOLD);
    }
    { // Incomplete corridor evidence holds the robot stopped until the prepare budget expires.
        Sim s; s.in.corridor_complete = false;
        assert(s.until(PushState::PREPARE, 20));
        for (int i = 0; i < 20; ++i) { s.tick(); assert(s.out.state == PushState::PREPARE && stopped(s.out.motion)); }
        assert(s.until(PushState::SCAN, 100) && s.out.reason == "corridor_unresolved");
    }
    { // Nothing enclosed within the rush distance: release and retry later.
        Sim s; s.toRush();
        assert(s.until(PushState::CAPTURE_FAIL, 150) && s.out.reason == "capture_unverified");
        assert(s.until(PushState::SCAN, 200));
    }
    { // The load slips out during the carry: unseen at FAR, found by the next stopped NEAR check.
        Sim s; s.toCarry(); s.tick(); s.hold({});
        for (int i = 0; i < 20; ++i) assert(s.tick().state == PushState::CARRY);
        assert(s.until(PushState::LOST_HOLD, 200));
        assert(std::abs(s.in.camera_pitch_cdeg - 4000) <= 100 && stopped(s.out.motion));
        assert(s.until(PushState::SCAN, 200) && s.out.delivered_total == 0 && s.out.batch_size == 0);
    }
    { // The load changes (another object joined): no longer the verified trip.
        Sim s; s.toCarry(); s.hold(inv(1, 1));
        assert(s.until(PushState::LOST_HOLD, 200));
    }
    { // Looking far ahead is not a loss; the hold is re-checked at NEAR, always stopped.
        Sim s; s.toCarry(); s.zone_visible = false;
        int far = 0, near_checks = 0;
        for (int i = 0; i < 200; ++i) {
            s.tick(); assert(s.out.state == PushState::CARRY);
            if (s.out.motion.camera_pitch_cdeg == 4000) { assert(stopped(s.out.motion)); ++near_checks; }
            if (s.out.motion.camera_pitch_cdeg == 500 && s.in.camera_pitch_stable) ++far;
        }
        assert(far > 100 && near_checks > 0);
    }
    { // Nothing moves while the camera is between presets.
        Sim s; s.in.distance_m = .8f; s.pitch_rate = 50;
        for (int i = 0; i < 10; ++i) { s.tick(); assert(stopped(s.out.motion)); }
        assert(s.until(PushState::APPROACH, 10));
    }
    { // A camera that never reaches its preset is a fault.
        Sim s; s.servo_responds = false;
        assert(s.until(PushState::SAFE_STOP, 60) && s.out.reason == "camera_pitch_timeout");
        for (int i = 0; i < 5; ++i) assert(stopped(s.tick().motion));
    }
    { // The gate re-checks the load at NEAR before opening: a load lost on the way is not released as delivered.
        Sim s; s.toCarry();
        assert(s.until(PushState::GATE, 800));
        s.hold({});
        for (int i = 0; i < 100 && s.out.state == PushState::GATE; ++i) { s.tick(); assert(s.out.motion.gripper_offset == 20); }
        assert(s.out.state == PushState::LOST_HOLD && !s.visited.count(PushState::RAISE_RELEASE));
    }
    { // Pushed, but the zone counts never confirm it: no credit, first stays false.
        Sim s; s.toCarry(); s.deliver(false);
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered && s.out.reason == "delivery_unverified");
        assert(s.until(PushState::SCAN, 400));
        s.in.label = "core_supply"; s.in.target_id = 8;
        for (int i = 0; i < 20; ++i) assert(s.tick().state == PushState::SCAN);
    }
    { // Seeing one target enter must not credit a two-object batch.
        Sim s;s.in.corridor=inv(2);s.toCarry(inv(2));
        assert(s.until(PushState::ENTER,400));s.in.delivery_observed=true;
        assert(s.until(PushState::BACK_OUT,400));s.hold({});
        s.in.zone_supply_count=1;
        assert(s.until(PushState::TURN_SCAN,400));
        assert(s.out.delivered_total==1 && s.out.reason=="partial_delivery" && !s.out.first_ordinary_delivered);
    }
    { // A transient target entry, with no inventory increase, gets no credit.
        Sim s;s.toCarry();assert(s.until(PushState::ENTER,400));
        s.in.delivery_observed=true;assert(s.until(PushState::BACK_OUT,400));
        s.in.delivery_observed=false;s.hold({});
        assert(s.until(PushState::TURN_SCAN,400));assert(s.out.delivered_total==0);
    }
    { // Incomplete inventory cannot establish delivery, even with a count increase.
        Sim s;s.toCarry();assert(s.until(PushState::BACK_OUT,800));s.hold({});
        s.in.zone_supply_count=1;s.in.zone_inventory_complete=false;
        assert(s.until(PushState::TURN_SCAN,400));assert(s.out.delivered_total==0);
    }
    { // A count change in the other half invalidates the check.
        Sim s; s.toCarry();
        assert(s.until(PushState::BACK_OUT, 800));
        s.in.zone_supply_count = 1; s.in.zone_injured_count = 1; s.hold({});
        assert(s.until(PushState::TURN_SCAN, 400));
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered);
    }
    { // Forward motion needs the opponent zone to stay clear.
        Sim s; s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 10));
        s.tick(); assert(s.out.motion.vx_mps > 0);
        s.in.opponent_zone_clear = false; s.tick(); assert(stopped(s.out.motion));
        s.in.opponent_zone_clear = true; s.in.path_safe = false; s.tick(); assert(stopped(s.out.motion));
    }
    { // A load is never released where the opponent zone may receive it.
        Sim s; s.toCarry(); s.in.opponent_zone_clear = false; s.hold({});
        assert(s.until(PushState::LOST_HOLD, 200) && s.out.motion.gripper_offset == 20);
        assert(s.until(PushState::SAFE_STOP, 80) && s.out.reason == "drop_blocked_opponent_zone");
        for (int i = 0; i < 5; ++i) assert(stopped(s.tick().motion) && s.out.state == PushState::SAFE_STOP);
    }
    { // Reversing needs explicit rear clearance.
        Sim s; s.toCarry();
        assert(s.until(PushState::BACK_OUT, 800));
        s.in.retreat_safe = false;
        for (int i = 0; i < 5; ++i) assert(stopped(s.tick().motion) && s.out.state == PushState::BACK_OUT);
        s.in.retreat_safe = true; s.tick(); assert(s.out.motion.vx_mps < 0);
    }
    { // A short-term predicted zone pose cannot prove physical retreat or permit more reversing.
        Sim s; s.toCarry(); assert(s.until(PushState::BACK_OUT,800));
        s.zone_predicted=true;
        for(int i=0;i<5;++i)assert(stopped(s.tick().motion) && s.out.state==PushState::BACK_OUT);
    }
    { // No trusted zone estimate: carry only searches in place, never drives blind.
        Sim s; s.toCarry(); s.zone_visible = false;
        for (int i = 0; i < 10; ++i) { s.tick(); assert(s.out.state == PushState::CARRY && s.out.motion.vx_mps == 0); }
        s.zone_visible = true;
        bool resumed=false;
        for(int i=0;i<40;++i) { s.tick(); resumed=resumed || s.out.motion.vx_mps>0; }
        assert(resumed);
    }
    { // Stale frame or safety veto stops; resuming keeps the verified load.
        Sim s; s.toCarry(); s.tick();
        s.in.safety_ok = false; s.tick();
        assert(s.out.state == PushState::WAIT_START && stopped(s.out.motion) && s.out.batch_size == 1);
        assert(s.out.motion.gripper_offset == 20); // a stop never opens the frame
        s.in.safety_ok = true; s.tick(); assert(s.out.state == PushState::CARRY && s.out.batch_size == 1);
        s.in.now_us += 300000; s.tick();
        assert(s.out.state == PushState::WAIT_START && stopped(s.out.motion));
        s.tick(); assert(s.out.state == PushState::CARRY);
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered);
    }
    { // Same timestamp twice is not a fresh frame.
        Sim s; s.toRush(); auto out = s.task.update(s.in); assert(out.state == PushState::WAIT_START && stopped(out.motion));
    }
    { // A pause during the rush releases rather than guessing what is enclosed.
        Sim s; s.toRush(); s.tick(); s.in.run = false; s.tick(); s.in.run = true; s.tick();
        assert(s.out.state == PushState::ABORT_DROP);
    }
    { // Invalid geometry while approaching: no motion toward a guessed position.
        Sim s; s.in.distance_m = .8f; assert(s.until(PushState::APPROACH, 10));
        s.in.distance_m = std::numeric_limits<float>::quiet_NaN(); s.tick(); assert(s.out.motion.vx_mps == 0);
    }
    { // Reset clears the delivery record and the first-delivery permission.
        Sim s; s.toCarry(); s.deliver();
        assert(s.out.first_ordinary_delivered);
        s.in.reset = true; s.tick(); s.in.reset = false;
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered && s.out.state == PushState::WAIT_START);
    }
    { // A gripper that never acknowledges is a fault, not a guess.
        Sim s; s.gripper_responds = false;
        assert(s.until(PushState::SAFE_STOP, 100) && s.out.reason == "gripper_open_timeout");
        for (int i = 0; i < 5; ++i) assert(stopped(s.tick().motion));
        s.in.reset = true; s.tick(); assert(s.out.state == PushState::WAIT_START);
    }
    { // The tracker re-IDs the same cargo (same label, near the last bearing and range): rebind instead of searching.
        Sim s; s.in.distance_m = .8f; assert(s.until(PushState::APPROACH, 20));
        s.tick(); s.in.target_id = 8; s.tick();
        assert(s.out.state == PushState::APPROACH && s.out.reason == "approach_id_rebound_same_object" && s.out.target_id == 8);
    }
    { // The body turned .4 rad (e.g. a rush pulse) between the last tracked frame and the re-ID: the bearing shifts by
      // the same amount, so the world bearing is unchanged and the new ID is still the same object.
        Sim s; s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 20));
        s.tick(); s.in.target_id = 8; s.phi += .4f; s.in.heading_error -= .4f; s.tick();
        assert(s.out.state == PushState::APPROACH && s.out.reason == "approach_id_rebound_same_object" && s.out.target_id == 8);
    }
    { // Other detections do not authorize motion after the locked ID changes.
        Sim s; s.in.distance_m = .8f; assert(s.until(PushState::APPROACH, 20));
        s.tick(); const float h0 = s.in.heading_error; s.in.target_id = 8; s.in.heading_error = h0 + .8f; s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "target_id_mismatch");
        s.in.target_id = 7; s.in.heading_error = h0; s.in.geometry_valid = false; s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "target_geometry_invalid");
        s.in.geometry_valid = true; s.in.path_safe = false; s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "forward_path_blocked");
    }
    { // Waiting for an open ACK is distinguishable from target or path failure.
        Sim s; s.gripper_responds = false;
        assert(s.until(PushState::PREPARE, 30));
        bool saw_wait = false;
        for (int i=0; i<45 && s.out.state==PushState::PREPARE; ++i) {
            s.tick(); saw_wait = saw_wait || s.out.reason == "gripper_feedback_wait";
            assert(stopped(s.out.motion));
        }
        assert(saw_wait);
    }
    { // Entering PREPARE must open in the same stationary output, not one frame later.
        Sim s; assert(s.until(PushState::PREPARE, 40));
        assert(stopped(s.out.motion) && s.out.motion.gripper_offset == 0);
    }
    { // Open at NEAR without ground evidence, then reacquire TRACK before moving.
        Sim s;
        assert(s.until(PushState::PREPARE,80));
        s.live_camera_guard=true;
        assert(s.out.motion.camera_pitch_cdeg==4000 && s.out.motion.gripper_offset==0);
        bool returned=false;
        for(int i=0;i<60;++i) {
            s.in.target_valid=s.in.geometry_valid=false;
            s.tick(); assert(stopped(s.out.motion));
            assert(s.out.state==PushState::PREPARE);
            if(s.out.reason=="open_confirmed_return_to_tracking_pitch") {
                assert(s.pitch==4000 && s.in.camera_pitch_stable);
                assert(s.out.motion.camera_pitch_cdeg==500);
                returned=true;break;
            }
        }
        assert(returned);
        s.in.target_valid=s.in.geometry_valid=true;
        assert(s.until(PushState::RUSH,60));
        assert(s.pitch==500 && s.in.camera_pitch_stable);
    }
    { // Closing uses longitudinal depth, not the longer off-axis radial range.
        Sim s; s.toRush(); s.in.distance_m = .209f; s.in.heading_error = .32f;
        s.tick(); assert(s.out.state == PushState::LOWER_FRAME && stopped(s.out.motion));
    }
    { // Still outside the closing plane: do not close early.
        Sim s; s.toRush(); s.in.distance_m = .21f; s.in.heading_error = .2f;
        s.tick(); assert(s.out.state == PushState::RUSH);
    }
    { // New default: missing multiview evidence holds CLOSE and ends in safe stop.
        Sim s;s.tune.require_multi_view=true;s.task=PushTask(s.tune);s.multi_evidence=false;
        s.toRush();s.load=inv(1);s.in.distance_m=.19f;
        assert(s.until(PushState::VERIFY_CAPTURE,100));
        for(int i=0;i<200&&s.out.state!=PushState::SAFE_STOP;++i){s.tick();assert(stopped(s.out.motion));assert(s.out.motion.gripper_offset==20);}
        assert(s.out.state==PushState::SAFE_STOP&&s.out.reason=="multi_view_unconfirmed");
        assert(!s.visited.count(PushState::CARRY));
    }
    { // Positive injected multiview summary exercises the actual default path.
        Sim s;s.tune.require_multi_view=true;s.task=PushTask(s.tune);s.toCarry();
        assert(s.out.state==PushState::CARRY&&s.out.reason=="multi_view_capture_verified");
    }
    { // Search-only is a terminal decision, never an approach or a pickup.
        TaskTuning t;t.demo_search_only=true;Sim s(t);
        assert(s.until(PushState::SAFE_STOP,100));
        assert(s.out.reason=="demo_target_found"&&stopped(s.out.motion));
        assert(!s.visited.count(PushState::APPROACH)&&!s.visited.count(PushState::RUSH));
    }
    { // One-trip demo stops before the normal post-delivery turn / next search.
        TaskTuning t;t.demo_carry_once=true;Sim s(t);s.toCarry();
        assert(s.until(PushState::ENTER,400));s.in.delivery_observed=true;
        assert(s.until(PushState::BACK_OUT,400));s.in.zone_supply_count+=s.out.batch_size;s.hold({});
        assert(s.until(PushState::VERIFY_DELIVERY,400));assert(s.until(PushState::SAFE_STOP,100));
        assert(s.out.reason=="demo_delivery_complete"&&s.out.delivered_total==1&&stopped(s.out.motion));
        assert(!s.visited.count(PushState::TURN_SCAN));
    }
    assert(makeTargetAreas("red") == std::vector<std::string>{"red_safe_zone"});
    assert(makeTargetAreas("blue") == std::vector<std::string>{"blue_safe_zone"});
    assert(makeTargetBalls("red").size() == 3);
    { // MotionWatch 在 ~9Hz 控制环下也能判静止；原地转向指令下陀螺≈0超过阈值计为转圈卡死，正常转动/停止指令立即清零。
        MotionWatch w;std::array<float,3> still{0,0,9.8f},stalled{0,0,.004f},turning{0,0,.2f};
        uint64_t t=1000000;
        for(int i=0;i<40;++i,t+=105000)w.feed(t,"SCAN",0,.8f,true,true,still,stalled,stalled);
        assert(w.suspect()&&w.spinStallUs()>=3000000);
        assert(w.takeEvents().find("spin_stall_begin state=SCAN")!=std::string::npos);
        w.feed(t,"SCAN",0,.8f,true,true,still,turning,turning);t+=105000;
        assert(w.spinStallUs()==0&&w.takeEvents().find("spin_stall_end")!=std::string::npos);
        MotionWatch f;
        for(int i=0;i<40;++i,t+=105000)f.feed(t,"SCAN",0,.8f,true,true,still,turning,turning);
        assert(f.spinStallUs()==0&&!f.suspect());
        MotionWatch d; // 前进指令不算转圈卡死
        for(int i=0;i<40;++i,t+=105000)d.feed(t,"APPROACH",.3f,.8f,true,true,still,stalled,stalled);
        assert(d.spinStallUs()==0);
    }
    { // SCAN转圈卡死 -> 后退再前进各一段，然后恢复转圈。
        TaskTuning t;t.scan_wz=t.turn_wz=t.max_wz=.4f;PushTask task(t);PushObservation in;
        in.run=in.safety_ok=in.path_safe=in.opponent_zone_clear=in.retreat_safe=true;
        in.camera_pitch_cdeg=500;in.camera_pitch_stable=true;
        PushOutput out;
        for(int i=0;i<400;++i){in.now_us+=100000;out=task.update(in);if(out.state==PushState::SCAN)break;}
        assert(out.state==PushState::SCAN);
        in.now_us+=100000;out=task.update(in);assert(out.motion.wz_rps>0&&out.motion.vx_mps==0);
        in.imu_spin_stall_us=1600000;
        int back=0,fwd=0;
        for(int i=0;i<40;++i){
            in.now_us+=100000;out=task.update(in);
            if(out.reason=="spin_stall_back"){++back;assert(out.motion.vx_mps<0&&out.motion.wz_rps==0);}
            if(out.reason=="spin_stall_forward"){++fwd;assert(out.motion.vx_mps>0&&out.motion.wz_rps==0);}
            in.imu_spin_stall_us=0; // 动作期间 MotionWatch 的计数被清零
        }
        assert(back>=7&&back<=9&&fwd>=7&&fwd<=9);
        in.now_us+=100000;out=task.update(in);assert(out.motion.vx_mps==0&&out.motion.wz_rps>0&&out.reason=="searching_target");
        TaskTuning off=t;off.spin_unstick=false;PushTask task2(off);PushObservation i2=in;i2.now_us=0;i2.imu_spin_stall_us=0;
        for(int i=0;i<400;++i){i2.now_us+=100000;out=task2.update(i2);if(out.state==PushState::SCAN)break;}
        i2.imu_spin_stall_us=9000000;i2.now_us+=100000;out=task2.update(i2);assert(out.motion.vx_mps==0&&out.motion.wz_rps>0);
    }
    { // MotionWatch: commanded but vibration-free for >=1.5 s is flagged; real vibration or no command is not.
        MotionWatch w;std::array<float,3> still{0,0,9.8f},zero{0,0,0};
        uint64_t t=1000000;
        for(int i=0;i<200;++i,t+=10000)w.feed(t,"CARRY",.3f,0,true,true,still,zero,zero);
        assert(w.suspect()&&w.suspectTotalUs()>=400000&&w.stateSuspectUs("CARRY")>0);
        assert(w.takeEvents().find("[MOTION_EVENT] begin state=CARRY")!=std::string::npos);
        MotionWatch v;
        for(int i=0;i<200;++i,t+=10000){std::array<float,3> a{0,0,9.8f+((i&1)?.8f:-.8f)};v.feed(t,"CARRY",.3f,0,true,true,a,zero,zero);}
        assert(!v.suspect()&&v.suspectTotalUs()==0&&v.stateCmdUs("CARRY")>0);
        MotionWatch n;
        for(int i=0;i<200;++i,t+=10000)n.feed(t,"SCAN",0,0,true,true,still,zero,zero);
        assert(!n.suspect()&&n.stateCmdUs("SCAN")==0);
        MotionWatch o;
        for(int i=0;i<200;++i,t+=10000)o.feed(t,"CARRY",.3f,0,false,true,still,zero,zero);
        assert(!o.suspect());
        assert(w.line().find("[MOTION] state=CARRY")!=std::string::npos);
    }
    { // MotionWatch blocked: 前进+指令转向，陀螺响应占比<0.16持续一个窗口才算被顶住；正常转向、直行、无指令、换状态都不算。
        std::array<float,3> acc{0,0,9.8f},turn_slow{0,0,.05f},turn_ok{0,0,.18f},rpy{0,0,0};
        uint64_t t=1000000;
        MotionWatch b;
        for(int i=0;i<300;++i,t+=10000)b.feed(t,"ENTER",.3f,i&64?.55f:-.55f,true,true,acc,turn_slow,rpy);
        assert(b.blockedUs()>=600000&&b.blockRatio()>=0&&b.blockRatio()<.16f);
        assert(b.takeEvents().find("[MOTION_EVENT] blocked_begin state=ENTER")!=std::string::npos);
        assert(b.line().find("blocked_ms=")!=std::string::npos);
        b.feed(t,"BACK_OUT",-.2f,0,true,true,acc,turn_slow,rpy);
        assert(b.blockedUs()==0);
        MotionWatch f;
        for(int i=0;i<300;++i,t+=10000)f.feed(t,"ENTER",.3f,.55f,true,true,acc,turn_ok,rpy);
        assert(f.blockedUs()==0);
        MotionWatch g;
        for(int i=0;i<300;++i,t+=10000)g.feed(t,"ENTER",.3f,0,true,true,acc,turn_slow,rpy);
        assert(g.blockedUs()==0);
        MotionWatch h;
        for(int i=0;i<300;++i,t+=10000)h.feed(t,"ENTER",.3f,.55f,false,true,acc,turn_slow,rpy);
        assert(h.blockedUs()==0);
        MotionWatch k;
        for(int i=0;i<120;++i,t+=10000)k.feed(t,"ENTER",.3f,.55f,true,true,acc,turn_slow,rpy);
        for(int i=0;i<120;++i,t+=10000)k.feed(t,"ENTER",.3f,.55f,true,true,acc,turn_ok,rpy);
        assert(k.blockedUs()==0);
    }
    { // field_carry (main path): transport/release/push logic follows the demo, capture stays strict.
        TaskTuning t;t.field_carry=true;
        { // The load vanishing from the NEAR view during the carry is not a LOST_HOLD; without a usable drop plan the trip keeps searching and never releases blind.
            Sim s(t);s.toCarry();s.tick();s.hold({});s.drop_available=false;
            bool lost=false;
            for(int i=0;i<400&&s.out.state!=PushState::RAISE_RELEASE;++i){s.tick();lost=lost||s.out.state==PushState::LOST_HOLD;}
            assert(!lost && s.out.state==PushState::CARRY && s.out.reason=="searching_own_zone");
        }
        { // No verified own zone (enemy colour / unseen): the load is held and the robot searches, never a blind release.
            Sim s(t);s.toCarry();s.zone_visible=false;
            for(int i=0;i<400;++i){ // 20 s, inside carry_budget_us
                s.tick();
                assert(s.out.state!=PushState::RAISE_RELEASE&&s.out.state!=PushState::ENTER);
                assert(s.out.reason!="assumed_blind_carry"&&s.out.reason!="assumed_drop_point");
            }
            assert(s.out.state==PushState::CARRY&&s.out.reason=="searching_own_zone");
        }
        { // Enemy-colour zone seen while searching: turn about (IMU), drive a long leg away, then resume searching our zone.
            Sim s(t);s.toCarry();s.zone_visible=false;
            for(int i=0;i<200&&s.out.reason!="searching_own_zone";++i)s.tick();assert(s.out.reason=="searching_own_zone");
            const float phi0=s.phi;
            s.in.enemy_zone_seen=true;s.tick();s.tick(); // below the 3-frame debounce
            s.in.enemy_zone_seen=false;s.tick();assert(s.out.reason=="searching_own_zone");
            s.in.enemy_zone_seen=true;for(int i=0;i<3;++i)s.tick();
            s.in.enemy_zone_seen=false;
            assert(s.out.reason=="enemy_zone_turn_away");
            bool turned=false,drove=false;float x0=0,y0=0;
            for(int i=0;i<400&&s.out.reason!="searching_own_zone";++i){
                s.tick();
                if(s.out.reason=="enemy_zone_drive_away"){
                    if(!drove){drove=true;x0=s.px;y0=s.py;turned=std::abs(s.phi-phi0)>=s.tune.enemy_evade_turn_rad;}
                    assert(s.out.motion.vx_mps>0&&s.out.state==PushState::CARRY);
                }
                assert(s.out.state==PushState::CARRY);
            }
            assert(turned&&drove&&s.out.reason=="searching_own_zone");
            assert(std::abs(std::hypot(s.px-x0,s.py-y0)-s.tune.enemy_evade_m*s.tune.assume_motion_ratio)<.08f);
            // Cooldown: an immediate re-sighting does not restart the manoeuvre.
            s.in.enemy_zone_seen=true;for(int i=0;i<4;++i){s.tick();assert(s.out.reason=="searching_own_zone");}
        }
        { // Without the enemy sighting nothing changes; with it disabled the sighting is ignored.
            TaskTuning off=t;off.enemy_evade=false;
            Sim s(off);s.toCarry();s.zone_visible=false;for(int i=0;i<200&&s.out.reason!="searching_own_zone";++i)s.tick();s.in.enemy_zone_seen=true;
            for(int i=0;i<40;++i){s.tick();assert(s.out.reason=="searching_own_zone");}
        }
        { // Zone never found: after carry_search_turns turns the robot drives back along the start line, then spins again.
            TaskTuning q=t;q.carry_search_turns=.5f;q.carry_leg_us=2000000;q.carry_search_respin_turns=.25f;
            Sim s(q);s.toCarry();s.zone_visible=false;
            bool drove=false,respun=false;
            for(int i=0;i<1200&&!respun;++i){
                s.tick();
                assert(s.out.state==PushState::CARRY);
                if(s.out.reason=="carry_drive_start_line"){drove=true;}
                else if(drove&&s.out.reason=="searching_own_zone")respun=true;
            }
            assert(drove&&respun);
        }
        { // A far sighting (no geometry fix) ends the spin: face it and drive; the memory outlives the sighting; an enemy-colour sighting there erases it.
            Sim s(t);s.toCarry();s.zone_visible=false;
            for(int i=0;i<40&&s.out.reason!="searching_own_zone";++i)s.tick();assert(s.out.reason=="searching_own_zone");
            const float zyaw=s.phi+.5f;s.in.zone_sight_valid=true;s.in.zone_sight_bearing=.5f;s.tick();s.in.zone_sight_valid=false;
            bool sighted=false,drove=false;
            for(int i=0;i<200;++i){
                s.tick();assert(s.out.state==PushState::CARRY);
                sighted=sighted||s.out.reason=="carry_drive_to_sighted_zone";
                if(s.out.reason=="carry_drive_to_remembered_zone"&&s.out.motion.vx_mps>0)drove=true;
            }
            assert(sighted&&drove);
            s.in.zone_sight_valid=true;s.in.zone_sight_enemy=true;s.in.zone_sight_bearing=std::remainder(zyaw-s.phi,2*float(CV_PI));s.tick();
            s.in.zone_sight_valid=false;s.in.zone_sight_enemy=false;
            for(int i=0;i<5;++i){s.tick();assert(s.out.reason!="carry_drive_to_remembered_zone"&&s.out.reason!="carry_drive_to_sighted_zone");}
        }
        { // Strict capture is untouched: without multi-view evidence the capture is not credited.
            Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);s.multi_evidence=false;
            s.toRush();s.hold(inv(1));
            for(int i=0;i<200;++i){s.tick();assert(s.out.state!=PushState::CARRY);assert(s.out.reason!="assumed_capture");}
        }
        { // Without field_carry the same loss still ends the trip in LOST_HOLD (strict path unchanged).
            Sim s;s.toCarry();s.tick();s.hold({});
            assert(s.until(PushState::LOST_HOLD,200));
        }
    }
    { // Other objects in the PREPARE corridor: more frames to confirm, then keep the frame up, reverse and search again.
        auto prepareTicks = [](int frames) {
            TaskTuning t; t.corridor_precheck=false; t.corridor_confirm_frames = frames;
            Sim s(t); s.in.corridor = inv(1,0,0,0,1);
            int n = 0;
            while (s.out.state != PushState::LOST_HOLD && n < 400) { s.tick(); ++n; }
            assert(s.out.state == PushState::LOST_HOLD && !s.visited.count(PushState::RUSH));
            return n;
        };
        assert(prepareTicks(6) - prepareTicks(3) == 3);
        TaskTuning ft; ft.corridor_precheck=false; ft.corridor_confirm_frames = 6;
        Sim s(ft); s.in.corridor = inv(1,0,0,0,1);
        bool reversed = false, lowered = false;
        bool held = false;
        for (int i = 0; i < 600 && !(held && s.out.state == PushState::SCAN); ++i) {
            s.tick();
            if (s.out.state == PushState::LOST_HOLD) { held = true; if (s.out.motion.vx_mps < 0) reversed = true; }
            if (s.out.state == PushState::LOWER_FRAME || s.out.state == PushState::RUSH) lowered = true;
        }
        assert(reversed && !lowered && s.out.state == PushState::SCAN);
        assert(s.task.rejectedTargets(s.in.now_us).empty()); // first block: search again, no blacklist
    }
    { // Blocked again and again: the target is blacklisted after corridor_retry_max back-offs.
        TaskTuning pt; pt.corridor_precheck=false; Sim s(pt); s.in.corridor = inv(1,0,0,0,1);
        int backoffs = 0; PushState prev = s.out.state;
        for (int i = 0; i < 4000 && s.task.rejectedTargets(s.in.now_us).empty(); ++i) {
            s.tick();
            if (s.out.state == PushState::LOST_HOLD && prev != PushState::LOST_HOLD) ++backoffs;
            prev = s.out.state;
        }
        assert(!s.task.rejectedTargets(s.in.now_us).empty() && backoffs == s.tune.corridor_retry_max + 1);
    }
    { // A fatal verdict (dangerous in the corridor) blacklists at once and remembers the world bearing.
        TaskTuning pt; pt.corridor_precheck=false; Sim s(pt); s.in.corridor = inv(1,0,0,1);
        for (int i = 0; i < 4000 && s.task.rejectedTargets(s.in.now_us).empty(); ++i) s.tick();
        assert(!s.task.rejectedTargets(s.in.now_us).empty());
        assert(s.task.rejectedBearings(s.in.now_us).size() == 1);
    }
    { // Without zone geometry the back-off is blind: corridor_back_m x motion ratio of commanded reverse; unsafe rear skips it.
        for (bool rear_safe : {true, false}) {
            TaskTuning bt; bt.corridor_precheck=false; bt.back_speed = .32f; // field speed: the 0.8 commanded fits the retreat budget
            Sim s(bt); s.in.corridor = inv(1,0,0,1); s.in.retreat_safe = rear_safe;
            assert(s.until(PushState::PREPARE, 80));
            s.zone_visible = false;
            float back = 0; bool held = false;
            for (int i = 0; i < 800 && !(held && s.out.state == PushState::SCAN); ++i) {
                s.tick();
                if (s.out.state == PushState::LOST_HOLD) { held = true; back += -std::min(0.f, s.out.motion.vx_mps) * .05f; }
            }
            const float want = s.tune.corridor_back_m * s.tune.assume_motion_ratio;
            assert(held && s.out.state == PushState::SCAN);
            if (rear_safe) assert(back >= want - .05f && back <= want + .05f);
            else assert(back < .05f && s.out.reason == "corridor_back_off_unsafe");
        }
    }
    { // field_rush: ID rebound after the frame opens, scaled rush limit, blind finish below view (strict path unchanged).
        for (int mode = 0; mode < 3; ++mode) { // 0 strict, 1 field_rush near, 2 field_rush far (not the same object)
            TaskTuning ft; ft.field_rush = mode > 0; ft.corridor_confirm_frames = 3;
            Sim s(ft); bool switched = false, relost = false;
            for (int i = 0; i < 200 && !relost && !s.visited.count(PushState::RUSH); ++i) {
                s.tick();
                if (!switched && s.out.state == PushState::PREPARE && s.out.reason == "open_confirmed_return_to_tracking_pitch") {
                    switched = true; s.in.target_id = 9; if (mode == 2) s.in.distance_m += .30f;
                }
                if (switched && s.out.state == PushState::SCAN) relost = true;
            }
            assert(switched);
            if (mode == 1) assert(s.visited.count(PushState::RUSH) && !relost);
            else assert(relost && !s.visited.count(PushState::RUSH));
        }
        for (bool fr : {false, true}) {
            TaskTuning ft; ft.field_rush = fr; ft.rush_stall_us = 0; // legacy bound path, watchdog covered separately
            Sim s(ft); s.in.distance_m = .30f;
            s.toRush();
            bool failed = false;
            for (int i = 0; i < 400 && s.out.state != PushState::LOWER_FRAME && !failed; ++i) { s.tick(); failed = s.out.state == PushState::CAPTURE_FAIL; }
            if (fr) assert(s.out.state == PushState::LOWER_FRAME && !failed);
            else assert(failed && s.out.reason == "rush_overrun");
        }
        for (bool fr : {false, true}) {
            TaskTuning ft; ft.field_rush = fr;
            Sim s(ft); s.in.distance_m = .30f;
            s.toRush();
            for (int i = 0; i < 3; ++i) s.tick();
            s.in.target_valid = false;
            bool drove = false, failed = false;
            for (int i = 0; i < 400 && s.out.state == PushState::RUSH; ++i) { s.tick(); drove = drove || s.out.motion.vx_mps > 0; failed = s.out.state == PushState::CAPTURE_FAIL; }
            if (fr) assert(s.out.state == PushState::LOWER_FRAME && drove && s.out.reason == "assumed_enclosure_target_below_view");
            else assert(failed && s.out.reason == "rush_target_lost");
        }
    }
    { // RUSH no-progress watchdog (Stage C). Synthetic frames: frozen/closing distance values, not a replay of the field log.
        const auto run = [](const char *mode) {
            TaskTuning ft; ft.field_rush = true;
            Sim s(ft); s.tune.rush_stall_us = 2000000; s.task = PushTask(s.tune); s.in.distance_m = .30f; s.toRush();
            if (std::string(mode) == "closing") s.in.distance_m = .45f;
            if (std::string(mode) == "in_frame") s.hold(inv(1));
            bool reobserve = false, lowered = false; float moved_in_reobserve = 0;
            for (int i = 0; i < 400 && s.out.state == PushState::RUSH; ++i) {
                if (std::string(mode) == "closing") s.in.distance_m -= .0015f;
                if (std::string(mode) == "flicker" && i % 4 == 0) s.pitch_still = 0;
                s.tick();
                if (s.out.reason == "rush_stall_reobserve") { reobserve = true; moved_in_reobserve += std::abs(s.out.motion.vx_mps) + std::abs(s.out.motion.wz_rps); }
                lowered = lowered || s.out.state == PushState::LOWER_FRAME;
            }
            struct R { PushState st; std::string reason; bool reobserve, lowered; float moved; } r{s.out.state, s.out.reason, reobserve, lowered, moved_in_reobserve};
            return r;
        };
        { const auto r = run("frozen"); // valid stall: stop, re-observe, then recover; the bound must not be credited as enclosure
          assert(r.reobserve && r.moved == 0 && !r.lowered && r.st == PushState::CAPTURE_FAIL && r.reason == "rush_no_progress"); }
        { const auto r = run("in_frame"); // reliable in-frame evidence at the stall allows lowering
          assert(r.st == PushState::LOWER_FRAME && r.reason == "rush_stall_in_frame_evidence"); }
        { const auto r = run("closing"); // real closing (3 cm/s) never raises a stall
          assert(!r.reobserve && r.reason != "rush_no_progress"); }
        { const auto r = run("flicker"); // camera repeatedly unstable: no valid window, so no stall claim
          assert(!r.reobserve && r.reason != "rush_no_progress"); }
    }
    { // VERIFY_CAPTURE false-class re-check (Stage D). Synthetic held-region frames; the field #313 frame itself is not replayed.
        struct R { PushState st; std::string reason; int bad_frames_seen; bool carried; };
        // pattern(k) -> bad objects present in verify frame k (core on a track id, with confidence)
        const auto run = [](const std::function<std::vector<PushObservation::HeldObject>(int)> &pattern, bool recheck_on = true) {
            TaskTuning ft; ft.field_rush = true; ft.verify_held_contents = true;
            Sim s(ft); s.tune.require_multi_view = true; s.tune.held_recheck = recheck_on; s.task = PushTask(s.tune); s.multi_evidence = false;
            s.toRush(); s.hold(inv(1));
            int k = -1; bool carried = false; int bad_seen = 0;
            for (int i = 0; i < 400 && s.out.state != PushState::ABORT_DROP && s.out.state != PushState::CAPTURE_FAIL && !carried; ++i) {
                if (s.out.state == PushState::VERIFY_CAPTURE) {
                    ++k; auto objs = pattern(k);
                    s.load = inv(1); s.load_objects = {{7, "ordinary_supply", .9f}};
                    for (const auto &o : objs) { s.load_objects.push_back(o); if (o.label == "core_supply") s.load.core++; if (o.label == "dangerous_object") s.load.dangerous++; }
                    if (!objs.empty()) ++bad_seen;
                }
                s.tick(); carried = s.out.state == PushState::CARRY;
            }
            return R{s.out.state, s.out.reason, bad_seen, carried};
        };
        const auto core = [](int id, float c) { return PushObservation::HeldObject{id, "core_supply", c}; };
        { const auto r = run([&](int k) { return k >= 2 && k < 4 ? std::vector<PushObservation::HeldObject>{core(99, .54f)} : std::vector<PushObservation::HeldObject>{}; }, false);
          assert(r.st == PushState::ABORT_DROP && r.reason == "held_core_before_first"); } // old behaviour: two frames abort
        { const auto r = run([&](int k) { return k >= 2 && k < 4 ? std::vector<PushObservation::HeldObject>{core(99, .54f)} : std::vector<PushObservation::HeldObject>{}; });
          assert(r.carried && r.reason == "assumed_capture" && r.st == PushState::CARRY); } // brief weak class: re-check clears, flow continues
        { const auto r = run([&](int k) { return k >= 2 ? std::vector<PushObservation::HeldObject>{core(99, .54f)} : std::vector<PushObservation::HeldObject>{}; });
          assert(r.st == PushState::ABORT_DROP && r.reason == "held_core_before_first" && r.bad_frames_seen >= 4); } // stable same identity: confirmed illegal, existing drop
        { const auto r = run([&](int k) { return k >= 2 ? std::vector<PushObservation::HeldObject>{core(90 + k, .54f)} : std::vector<PushObservation::HeldObject>{}; });
          assert(!r.carried && r.st == PushState::CAPTURE_FAIL && r.reason == "held_illegal_unresolved"); } // identity keeps changing: uncertain is not a capture
        { const auto r = run([&](int k) { return k >= 2 ? std::vector<PushObservation::HeldObject>{core(99, .90f)} : std::vector<PushObservation::HeldObject>{}; });
          assert(r.st == PushState::ABORT_DROP && r.bad_frames_seen == 2); } // strong class keeps the prompt block
        { const auto r = run([&](int k) { return k >= 2 ? std::vector<PushObservation::HeldObject>{{98, "dangerous_object", .5f}} : std::vector<PushObservation::HeldObject>{}; });
          assert(r.st == PushState::ABORT_DROP && r.reason == "held_dangerous" && r.bad_frames_seen == 2); } // a dangerous object is never re-checked
    }
    { // field_rush: unconfirmed multi-view credits the capture; a measured inventory conflict stays fatal.
        TaskTuning ft; ft.field_rush = true;
        { Sim s(ft); s.tune.require_multi_view = true; s.task = PushTask(s.tune); s.multi_evidence = false;
          s.toRush(); s.hold(inv(1));
          assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture"); }
        { Sim s(ft); s.tune.require_multi_view = true; s.task = PushTask(s.tune);
          s.toRush(); s.hold(inv(2));
          bool credited = false;
          for (int i = 0; i < 300 && s.out.state != PushState::ABORT_DROP; ++i) { s.tick(); credited = credited || s.out.reason == "assumed_capture"; }
          assert(!credited && s.out.state == PushState::ABORT_DROP && s.out.reason.rfind("multi_grab_", 0) == 0); }
    }
    { // field_rush: after the frame opens the tracker re-IDs the target and the 40deg view clips it (region/geometry invalid): rebind by position, not by validity.
        TaskTuning ft; ft.field_rush = true; ft.corridor_confirm_frames = 3;
        Sim s(ft); bool done = false;
        for (int i = 0; i < 200 && !done; ++i) {
            s.tick();
            if (s.out.state == PushState::PREPARE && s.out.reason == "open_confirmed_return_to_tracking_pitch") {
                s.in.target_id = 9; s.in.target_region_valid = false; s.in.geometry_valid = false;
                for (int k = 0; k < 40 && s.out.reason != "prepare_id_rebound_after_open"; ++k) s.tick();
                assert(s.out.reason == "prepare_id_rebound_after_open" && s.out.target_id == 9); done = true;
            }
        }
        assert(done);
    }
    { // Frame contents seen in VERIFY_CAPTURE: the target must be inside, a dangerous object never, an injured person only after a delivery and alone.
        const auto run = [](bool first, const Inventory &held, PushState want, const char *prefix) {
            TaskTuning ft; ft.field_rush = true; ft.verify_held_contents = true; ft.assume_injured_trip = first;
            Sim s(ft); s.tune.require_multi_view = true; s.task = PushTask(s.tune); s.multi_evidence = false;
            s.toRush(); s.hold(held);
            for (int i = 0; i < 400 && s.out.state != want; ++i) s.tick();
            assert(s.out.state == want && s.out.reason.rfind(prefix, 0) == 0);
        };
        Inventory with_danger = inv(1); with_danger.dangerous = 1;
        Inventory with_injured = inv(1); with_injured.injured = 1;
        run(false, inv(1), PushState::CARRY, "assumed_capture");
        run(false, with_danger, PushState::ABORT_DROP, "held_");
        run(true, with_danger, PushState::ABORT_DROP, "held_");
        run(false, with_injured, PushState::ABORT_DROP, "held_");
        run(true, with_injured, PushState::ABORT_DROP, "held_");
        Inventory only_injured; only_injured.injured = 1;
        run(true, only_injured, PushState::CAPTURE_FAIL, "capture_target_not_in_frame");
        run(true, Inventory{}, PushState::CAPTURE_FAIL, "capture_target_not_in_frame");
    }
    { // 走廊邻居清理：走廊判定不合法且目标前方走廊内有任意类别邻居 -> 低框原地扫开再回正；开关关闭/走廊合法/邻居在目标后方或带外/数据过期则不动。
        const auto run=[&](bool on,Inventory corridor,float ux,float uy,bool fresh){
            TaskTuning t;t.enable_search_cues=false;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
            t.clear_neighbours=on;
            Sim s(t);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.19f;s.in.corridor=corridor;
            int outs=0,backs=0;float first_wz=0;bool left_sweep=false;
            for(int i=0;i<200;++i){
                s.in.clutter_body={{ux,uy}};if(fresh)s.in.clutter_ts_us=s.in.now_us;
                if(backs>0)s.in.corridor=inv(1);
                s.tick();
                if(s.out.state!=PushState::APPROACH){left_sweep=true;break;}
                if(s.out.reason=="sweep_neighbour_out"&&s.out.motion.wz_rps!=0){++outs;if(!first_wz)first_wz=s.out.motion.wz_rps;assert(s.out.motion.vx_mps==0);}
                if(s.out.reason=="sweep_upright_back"&&s.out.motion.wz_rps!=0)++backs;
            }
            struct R{int outs,backs;float wz;bool left;};
            return R{outs,backs,first_wz,left_sweep};
        };
        auto many=run(true,inv(3),.08f,.10f,true);          // 目标前方偏右的普通物资，走廊内共3个
        assert(many.outs>0&&many.backs>0&&many.wz<0&&many.left);
        auto leftside=run(true,inv(3),-.08f,.10f,true);
        assert(leftside.outs>0&&leftside.wz>0);
        auto danger=run(true,inv(1,0,0,1),.06f,.10f,true);  // 走廊内有危险物：同样只扫不搬
        assert(danger.outs>0&&danger.left);
        assert(run(false,inv(3),.08f,.10f,true).outs==0);   // 开关关闭
        assert(run(true,inv(2),.08f,.10f,true).outs==0);    // 走廊合法
        assert(run(true,inv(3),.08f,.30f,true).outs==0);    // 在目标后方
        assert(run(true,inv(3),.30f,.10f,true).outs==0);    // 带外
        assert(run(true,inv(3),.08f,.10f,false).outs==0);   // 数据过期
    }
    { // 远处邻居（框前沿够不到）：先慢速前探到距邻居约.03m，再转出扫开；邻居已贴近则不前探。
        const auto run=[&](float uy){
            TaskTuning t;t.enable_search_cues=false;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
            t.clear_neighbours=true;
            Sim s(t);s.in.distance_m=.5f;assert(s.until(PushState::APPROACH,80));
            s.in.distance_m=.34f;s.in.corridor=inv(3);
            int creeps=0,outs=0;bool order_ok=true;
            for(int i=0;i<300;++i){
                s.in.clutter_body={{.08f,uy}};s.in.clutter_ts_us=s.in.now_us;
                s.tick();
                if(s.out.state!=PushState::APPROACH)break;
                if(s.out.reason=="sweep_creep"&&s.out.motion.vx_mps>0&&s.out.motion.wz_rps==0)++creeps;
                if((s.out.reason=="sweep_neighbour_out"||s.out.reason=="sweep_upright_out")&&s.out.motion.wz_rps!=0){if(!creeps&&uy>.22f)order_ok=false;++outs;}
            }
            return std::make_tuple(creeps,outs,order_ok);
        };
        const auto far=run(.28f);
        assert(std::get<0>(far)>0&&std::get<1>(far)>0&&std::get<2>(far));
        assert(std::get<0>(run(.20f))==0);
    }
    { // 扫动阶段沿用搬运的视角分级：5° → 20°(剩余≤track_near_m) → 40°(剩余≤intermediate_to_near_m)，只降不升；关闭开关则全程5°。
        TaskTuning t;t.enable_search_cues=false;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;
        const auto mk=[&](float ratio,float dist,Sim &s){s.in.target_valid=true;s.in.distance_m=dist;s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=ratio;};
        const auto ladder=[&](bool handoff){
            TaskTuning tt=t;tt.sweep_view_handoff=handoff;
            Sim s(tt);mk(.002f,.9f,s);std::vector<int> seq;
            for(int i=0;i<3000;++i){s.tick();
                if(s.out.state==PushState::SCATTER){const int p=s.out.motion.camera_pitch_cdeg;if(seq.empty()||seq.back()!=p)seq.push_back(p);}
                if(s.out.reason=="blind_scatter_done")break;}
            return seq;
        };
        const auto on=ladder(true);
        assert(on.size()>=2&&on.front()==500);
        for(size_t i=1;i<on.size();++i)assert(on[i]>on[i-1]);
        assert(std::find(on.begin(),on.end(),2000)!=on.end());
        const auto off=ladder(false);
        assert(off.size()==1);
    }
    { // 锁定目标不是普通物资：不抓取，冲入摆扫后拉黑并回到扫描；普通物资单个仍正常抓取；开关关闭则按原流程。
        g_non_supply_sweep=true;
        TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        const auto runOne=[&](const char *label,bool flag){
            g_non_supply_sweep=flag;
            Sim s(t);s.task.first_=true;s.in.label=label;s.in.distance_m=.5f;s.in.corridor=inv(0,1);
            struct R{bool rush=false,swept=false,prepared=false;float max_wz=0;} r;
            if(!s.until(PushState::APPROACH,120)&&!s.until(PushState::MID_APPROACH,1)){return r;}
            s.in.distance_m=.25f;
            for(int i=0;i<600;++i){
                s.tick();
                r.max_wz=std::max(r.max_wz,std::abs(s.out.motion.wz_rps));
                if(s.out.reason=="disperse_rush"||s.out.reason=="non_supply_rush")r.rush=true;
                if(s.out.reason=="non_supply_swept"){r.swept=true;assert(s.out.state==PushState::SCAN);break;}
                if(s.out.state==PushState::PREPARE||s.out.state==PushState::RUSH){r.prepared=true;break;}
            }
            return r;
        };
        const auto core=runOne("core_supply",true);
        assert(core.rush&&core.swept&&!core.prepared&&core.max_wz>=1.5f);
        const auto inj=runOne("injured_person",true);
        assert(inj.rush&&inj.swept&&!inj.prepared);
        const auto ord=runOne("ordinary_supply",true);
        assert(!ord.rush&&!ord.swept);
        const auto off=runOne("core_supply",false);
        assert(!off.rush&&!off.swept);
        g_non_supply_sweep=false;
    }
    { // 清扫账本(Stage A)：所有退出路径只记一次实际清扫时间；远驱+近扫算同一次；总预算约束执行中的清扫；确认等待有界；扫完不连扫。
        const std::vector<cv::Point2f> grp{{.20f,.50f},{.32f,.56f},{.26f,.44f}};
        TaskTuning base;base.enable_search_cues=false;base.blind_scatter_first_turn_rad=0.f;base.assume_motion_ratio=1.f;
        struct L{int starts=0;bool done=false;std::string exit;uint64_t t_start=0,t_end=0;int done_cnt=0;uint64_t spent=0;bool open=true;};
        // 近距成团：持续喂同一个团，跑到第一次扫结束(任何退出)后再多跑extra个tick，检查账本不再变化。
        const auto near=[&](TaskTuning tt,int extra,int ticks){
            Sim s(tt);s.in.target_valid=false;s.in.distance_m=.9f;L l;int after=-1;uint64_t spent_at_end=0;
            for(int i=0;i<ticks;++i){
                s.in.clutter_body=grp;s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.reason=="blind_scatter_start"){++l.starts;if(l.starts==1)l.t_start=s.in.now_us;}
                if(l.starts>=1&&!l.done&&s.out.state!=PushState::SCATTER){l.done=true;l.exit=s.out.reason;l.t_end=s.in.now_us;spent_at_end=s.task.sweepSpentUs(s.in.now_us);after=0;}
                if(after>=0){++after;if(after>=extra)break;}
            }
            l.done_cnt=s.task.sweepDone();l.spent=s.task.sweepSpentUs(s.in.now_us);l.open=s.task.sweepChainOpen();
            assert(!l.done||l.spent==spent_at_end); // 退出后继续跑，账本不变(不重复计时)
            return l; };
        { // 完整近扫：计1次，用时≈实际时长，链已关闭。
            const L l=near(base,200,2400);
            assert(l.starts>=1&&l.done&&l.exit=="blind_scatter_done"&&l.done_cnt==1&&!l.open);
            const double dur=double(l.t_end-l.t_start)/1e6,sp=double(l.spent)/1e6;
            assert(sp>=dur-.2&&sp<=dur+.2); }
        { // 超时退出：用时记入但不计次数；sweep_gap(20 s)内不再起扫，间隔过后才允许下一次，且两次用时各记一次。
            TaskTuning t=base;t.blind_scatter_budget_us=600000;
            Sim s(t);s.in.target_valid=false;s.in.distance_m=.9f;int starts=0,starts_in_gap=0;bool timeout=false;
            for(int i=0;i<600;++i){
                s.in.clutter_body=grp;s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(s.out.reason=="blind_scatter_start"){++starts;if(i<380)++starts_in_gap;}
                if(s.out.reason=="blind_scatter_timeout")timeout=true;
                if(i==379){assert(timeout&&starts==1&&s.task.sweepDone()==0&&!s.task.sweepChainOpen());
                    const double sp=double(s.task.sweepSpentUs(s.in.now_us))/1e6;assert(sp>=.5&&sp<=.9);}
            }
            assert(starts_in_gap==1&&starts==2&&s.task.sweepDone()==0&&!s.task.sweepChainOpen());
            const double sp2=double(s.task.sweepSpentUs(s.in.now_us))/1e6;assert(sp2>=1.0&&sp2<=1.8); }
        { // 总预算在清扫执行中生效：3 s预算切断近扫，用时不超过预算+1个tick，原因sweep_budget_exhausted。
            TaskTuning t=base;t.sweep_time_budget_us=3000000;
            const L l=near(t,20,2400);
            assert(l.done&&l.exit=="sweep_budget_exhausted"&&!l.open);
            const double sp=double(l.spent)/1e6;assert(sp>=2.9&&sp<=3.2);
            // 对照：默认预算下同一场景走完整扫且用时大于3 s
            const L d=near(base,0,2400);assert(d.exit=="blind_scatter_done"&&double(d.spent)/1e6>3.3); }
        { // 退场不安全：back阶段retreat_safe=false -> 立即结束且不下发倒车。
            Sim s(base);s.in.target_valid=false;s.in.distance_m=.9f;bool back=false,ended=false;float min_vx=0;
            for(int i=0;i<2400&&!ended;++i){
                s.in.clutter_body=grp;s.in.clutter_ts_us=s.in.now_us;
                if(s.out.reason=="blind_scatter_back")back=true;
                s.in.retreat_safe=!back;s.tick();
                if(!s.in.retreat_safe)min_vx=std::min(min_vx,s.out.motion.vx_mps);
                if(back&&s.out.reason=="blind_scatter_back_unsafe"){ended=true;assert(s.out.state==PushState::SCAN);}
            }
            assert(back&&ended&&min_vx>=-1e-4f&&!s.task.sweepChainOpen()); }
        // 远驱+近扫：世界固定的远群(约1.9 m)，同一条链。
        struct F{bool far=false,near_group=false,done=false;std::string exit;uint64_t t_far=0,t_near=0,t_end=0,spent=0;int done_cnt=0;bool open=true;};
        const auto farChain=[&](TaskTuning tt){
            Sim s(tt);s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=.006f;
            const float x0=s.px,y0=s.py;
            const std::vector<cv::Point2f> world{{x0+.0f,y0+1.90f},{x0+.08f,y0+1.98f},{x0-.07f,y0+1.84f}};
            F f;
            for(int i=0;i<3000&&!f.done;++i){
                const float c=std::cos(s.phi),sn=std::sin(s.phi);s.in.clutter_body.clear();
                for(const auto&w:world){const float dx=w.x-s.px,dy=w.y-s.py;s.in.clutter_body.push_back({dx*c+dy*sn,-dx*sn+dy*c});}
                const cv::Point2f lk=s.in.clutter_body[0];s.in.target_valid=lk.y>.1f;s.in.distance_m=std::hypot(lk.x,lk.y);s.in.heading_error=std::atan2(-lk.x,lk.y);
                s.in.clutter_ts_us=s.in.now_us;s.tick();
                if(!f.far&&(s.out.reason=="blind_scatter_far_group"||s.out.reason=="blind_scatter_far_start")){f.far=true;f.t_far=s.in.now_us;}
                if(!f.near_group&&s.out.reason=="blind_scatter_near_group"){f.near_group=true;f.t_near=s.in.now_us;}
                if(f.far&&s.out.state!=PushState::SCATTER){f.done=true;f.exit=s.out.reason;f.t_end=s.in.now_us;}
            }
            f.spent=s.task.sweepSpentUs(s.in.now_us);f.done_cnt=s.task.sweepDone();f.open=s.task.sweepChainOpen();
            return f; };
        TaskTuning fc=base;fc.blind_scatter_max_y_m=.5f;fc.blind_scatter_far_max_y_m=4.f;fc.blind_scatter_half_x_m=2.5f;fc.sweep_confirm_wait_us=0;
        F ref;
        { // 远驱->近扫算同一次：计1次，用时覆盖远驱起点到结束，而不是只有近扫段。
            ref=farChain(fc);
            assert(ref.far&&ref.near_group&&ref.done&&ref.exit=="blind_scatter_done"&&ref.done_cnt==1&&!ref.open);
            const double whole=double(ref.t_end-ref.t_far)/1e6,near_only=double(ref.t_end-ref.t_near)/1e6,sp=double(ref.spent)/1e6;
            assert(sp>=whole-.2&&sp<=whole+.2&&sp>near_only+.5); }
        { // 远->近切换不能重置整次作业期限：作业预算=远驱段+2.5 s，近扫被sweep_op_timeout切断，总时长≈预算。
            const double far_part=double(ref.t_near-ref.t_far)/1e6;
            TaskTuning t=fc;t.sweep_op_budget_us=uint64_t((far_part+2.5)*1e6);
            const F f=farChain(t);
            assert(f.far&&f.near_group&&f.done&&f.exit=="sweep_op_timeout"&&!f.open);
            const double whole=double(f.t_end-f.t_far)/1e6;assert(whole<=double(t.sweep_op_budget_us)/1e6+.2&&whole>=far_part+2.3); }
        // 确认等待：普通物资锁定(2.5 m)+远群。默认等待内由3帧确认的单目标追踪优先于2帧群决策；等待为0则回到群追。
        struct Cf{bool group=false,approach=false,seen=false;uint64_t t_group=0;};
        const auto confirm=[&](TaskTuning tt,bool flip,bool burst){
            tt.first_burst=burst;
            Sim s(tt);s.in.pixel_mode="box_area";s.in.pixel_ratio_smoothed=s.in.pixel_ratio_raw=.006f;
            s.in.target_valid=true;s.in.distance_m=2.5f;Cf c;
            const std::vector<cv::Point2f> world{{0.f,2.5f},{.20f,2.30f},{-.25f,2.60f}};
            for(int i=0;i<400&&!c.group&&!c.approach;++i){
                s.in.clutter_body=world;s.in.clutter_ts_us=s.in.now_us;
                s.in.target_id=flip?(i%2?7:8):s.in.target_id;
                s.tick();
                if(s.out.reason=="blind_scatter_far_group"){c.group=true;c.t_group=s.in.now_us;}
                if(s.out.state==PushState::APPROACH)c.approach=true;
                if(s.out.reason=="blind_scatter_target_seen")c.seen=true;
            }
            return c; };
        TaskTuning ct=fc;ct.sweep_confirm_wait_us=1200000;
        { const Cf c=confirm(ct,false,false);assert(c.approach&&!c.group); }
        { TaskTuning z=ct;z.sweep_confirm_wait_us=0;const Cf c=confirm(z,false,false);assert(c.group); }
        { // 目标ID来回跳(永远凑不够3帧)：等待有界，约1.2 s后仍交给群决策，不无限挂起。
            const Cf c=confirm(ct,true,false);assert(c.group&&c.t_group>=900000&&c.t_group<=2500000); }
        { // 开局首次远驱(first_burst且未用过)不被确认等待拦住。
            const Cf c=confirm(ct,true,true);assert(c.group&&c.t_group<=500000); }
    }
    std::cout << "Capture, transport rules, carry, delivery, drop and stop checks passed\n";
}
