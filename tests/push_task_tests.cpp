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
#define private public // test-only: seed first_ so an injured target is selectable
#include "rescue/push_task.hpp"
#undef private
#include "rescue/capture_monitor.hpp"
#include "rescue/config.hpp"
#include "rescue/motion_readiness.hpp"
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
struct Sim {
    TaskTuning tune;
    PushTask task{tune};
    PushObservation in;
    PushOutput out;
    float px = -.15f, py = -.75f, phi = 0; // phi: robot heading vs zone +y, CCW positive
    bool zone_visible = true, zone_predicted = false, gripper_responds = true, servo_responds = true;
    bool multi_evidence=true;
    float py_max=1e9f; bool vref_visible=false, cargo_on=false; cv::Point2f cargo_zone{-.15f,.12f};
    bool navigation_ready = true, drop_available = true, live_camera_guard = false;
    int pitch = 0, pitch_still = 0, pitch_rate = 400; // power-on: level
    Inventory load; // what is physically enclosed
    std::set<PushState> visited;
    static TaskTuning testTune(TaskTuning t) { t.startup_advance_us = 0; t.require_multi_view=false; return t; }
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
        in.cargo_body.clear(); in.cargo_ts_us=in.now_us;
        if(cargo_on){const float c=std::cos(-phi),sn=std::sin(-phi),dx=cargo_zone.x-px,dy=cargo_zone.y-py;const cv::Point2f cb{c*dx-sn*dy,sn*dx+c*dy};if(cb.y>0)in.cargo_body.push_back(cb);}
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
int main(int argc, char **argv) {
    if (argc == 3 && std::string(argv[1]) == "--write-fixture") { writeFixture(argv[2]); return 0; }
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
        assert(s.until(PushState::SAFE_STOP,2000) && s.out.reason=="demo_next_trip_ready");
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
    { // 视觉判块：块已在区内（近端距前沿≥done_y）则推入提前结束，核验阶段按视觉判进区。
        TaskTuning t;t.assume_all_safe=true;t.demo_carry_once=true;t.assume_motion_ratio=1.f;
        Sim s(t);s.tune.require_multi_view=true;s.task=PushTask(s.tune);
        s.zone_visible=false;s.multi_evidence=false;s.in.distance_m=.30f;s.vref_visible=true;
        s.toRush();s.in.target_valid=false;
        assert(s.until(PushState::ENTER,2000) && s.out.reason=="assumed_release_then_push");
        s.cargo_on=true;s.cargo_zone={s.tune.supply_half_x_m,.12f};
        assert(s.until(PushState::RAISE_RELEASE,600) && s.out.reason=="assumed_push_in_done");
        assert(s.py+s.tune.hold_center_y_m<s.tune.assume_push_depth_m-.05f);
        assert(s.until(PushState::TURN_SCAN,1500) && s.out.reason=="delivered_visual");
        const std::string ev=s.task.takeDropZoneEvent();
        assert(ev.find("[CARGO_ZONE] event=push_inside")!=std::string::npos && ev.find("event=verify")!=std::string::npos);
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
      assert(s.until(PushState::TURN_SCAN,100) && s.out.delivered_total==1);
      assert(s.until(PushState::SAFE_STOP,400) && s.out.reason=="demo_next_trip_ready");
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
    { // 假设模式：20°阶段进入PREPARE/RUSH后保持20°，不为开框切40°。
        TaskTuning t;t.enable_search_cues=false;t.assume_all_safe=true;t.track_pitch_cdeg=4000;t.near_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;t.track_near_m=.4f;t.intermediate_to_near_m=.27f;t.rush_start_m=.35f;
        Sim s(t);s.in.distance_m=.5f;
        assert(s.until(PushState::APPROACH,80));
        s.in.distance_m=.33f;assert(s.until(PushState::PREPARE,120));
        for(int i=0;i<30&&s.out.state==PushState::PREPARE;++i){s.tick();assert(s.out.motion.camera_pitch_cdeg==2000);}
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
    { // Forward permission alone cannot authorize an unsafe turn or curved approach.
        Sim s;s.in.distance_m=.8f;assert(s.until(PushState::APPROACH,30));
        s.in.directional_clearance_valid=true;s.in.turn_safe=false;s.in.arc_safe=false;
        s.in.heading_error=.1f;s.tick();assert(stopped(s.out.motion)&&s.out.reason=="turn_sweep_blocked");
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
        TaskTuning t;t.enable_lost_search=true;Sim s(t);s.in.distance_m=.8f;
        assert(s.until(PushState::APPROACH,30));s.in.target_valid=false;
        assert(s.until(PushState::LOST_SEARCH,20));assert(s.until(PushState::SAFE_STOP,430));
        assert(s.out.reason=="lost_search_exhausted"&&stopped(s.out.motion));
    }
    { // FAR sweep finds nothing: stop, tilt to 20deg, sweep once more, then stop for good.
        TaskTuning t;t.enable_lost_search=true;t.track_pitch_cdeg=4000;t.intermediate_pitch_cdeg=2000;
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
    { // Blue in PREPARE must never start a capture rush.
        Sim s; s.in.corridor = inv(1,0,0,1);
        for (int i=0;i<80 && s.out.reason!="corridor_dangerous";++i) s.tick();
        assert(s.out.reason=="corridor_dangerous" && !s.visited.count(PushState::RUSH));
    }
    for (bool at_close : {false,true}) {
        Sim s; s.toRush(); s.in.distance_m=at_close?.19f:.30f;
        s.in.corridor=inv(1,0,0,1); s.tick();
        assert(s.out.state==PushState::CAPTURE_FAIL && stopped(s.out.motion));
        assert(s.out.motion.gripper_offset==0 && s.out.reason=="rush_corridor_dangerous");
        assert(!s.visited.count(PushState::LOWER_FRAME));
    }
    { // Lost corridor evidence also prevents advancing or closing.
        Sim s; s.toRush(); s.in.corridor_complete=false; s.tick();
        assert(s.out.state==PushState::CAPTURE_FAIL && stopped(s.out.motion));
        assert(s.out.reason=="rush_corridor_incomplete" && s.out.motion.gripper_offset==0);
    }
    { // Too many supplies in the corridor: the rush is not started.
        Sim s; s.in.corridor = inv(4);
        for (int i = 0; i < 40 && s.out.reason != "corridor_too_many_supplies"; ++i) s.tick();
        assert(s.out.reason == "corridor_too_many_supplies" && !s.visited.count(PushState::RUSH));
    }
    { // ... nor when four end up held after closing.
        Sim s; s.in.corridor = inv(2); s.toRush(); s.hold(inv(3));
        assert(s.until(PushState::ABORT_DROP, 20) && s.out.verdict == RuleVerdict::TOO_MANY_SUPPLIES);
    }
    { // Before the first delivery an occluded corridor might hide a core supply.
        Sim s; s.in.corridor_occlusion_free = false;
        for (int i = 0; i < 40 && s.out.reason != "corridor_occluded"; ++i) s.tick();
        assert(s.out.reason == "corridor_occluded" && !s.visited.count(PushState::RUSH));
        assert(s.out.state == PushState::SCAN);
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
    { // Other detections do not authorize motion after the locked ID changes.
        Sim s; s.in.distance_m = .8f; assert(s.until(PushState::APPROACH, 20));
        s.tick(); s.in.target_id = 8; s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "target_id_mismatch");
        s.in.target_id = 7; s.in.geometry_valid = false; s.tick();
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
    std::cout << "Capture, transport rules, carry, delivery, drop and stop checks passed\n";
}
