#include "rescue/push_task.hpp"
#include "rescue/capture_monitor.hpp"
#include "rescue/config.hpp"
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
    bool zone_visible = true, gripper_responds = true, servo_responds = true;
    bool navigation_ready = true, drop_available = true;
    int pitch = 0, pitch_still = 0, pitch_rate = 400; // power-on: level
    Inventory load; // what is physically enclosed
    std::set<PushState> visited;
    explicit Sim(TaskTuning t = {}) : tune(t), task(t) {
        in.run = in.safety_ok = in.target_valid = in.geometry_valid = true;
        in.target_region_valid = in.zone_identity_verified = true;
        in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.target_id = 7; in.label = "ordinary_supply"; in.distance_m = .2f;
        in.corridor = inv(1); in.corridor_complete = in.corridor_occlusion_free = true;
        in.zone_valid = in.zone_own = true; in.zone_class = "supply";
        in.zone_counts_valid = true; in.heading_valid = true;
    }
    PushOutput tick() {
        const float dt = .05f;
        phi += out.motion.wz_rps * dt;
        px += -std::sin(phi) * out.motion.vx_mps * dt;
        py += std::cos(phi) * out.motion.vx_mps * dt;
        in.now_us += 50000;
        auto &z = in.zone_estimate;
        z = ZoneEstimate{};
        if (zone_visible) {
            z.valid = true; z.source = ZoneEstimate::Source::MULTI_POINT;
            z.frame_id = in.now_us / 50000; z.timestamp_us = z.observed_us = in.now_us;
            z.zone_label = "red_safe_zone"; z.geometry_id = "rescue2027-inner-v1-red";
            z.inlier_ids = {0, 2, 3, 5}; z.residual_m = .001f; z.position_sigma_m = .002f; z.yaw_sigma_rad = .01f;
            z.predicted_distance_m = 0;
            // p_body = R(-phi)(p_zone - r)  =>  yaw_body = -phi, origin = R(-phi)(-r)
            z.yaw_body_rad = -phi;
            const float c = std::cos(-phi), s = std::sin(-phi);
            z.origin_body_m = {c * -px - s * -py, s * -px + c * -py};
        }
        // Simulator supplies explicit validated direct-route/empty-zone evidence.
        in.carry_plan_valid=navigation_ready;in.drop_plan_valid=drop_available;in.navigation_timestamp_us=in.now_us;
        in.drop_centre_zone={task_injured?tune.injured_half_x_m:tune.supply_half_x_m,tune.deposit_y_m};
        in.carry_waypoint_body=z.zoneToBody({in.drop_centre_zone.x,-tune.gate_clearance_m-tune.hold_center_y_m});
        in.heading_rad = phi;
        const int target = out.motion.camera_pitch_cdeg;
        const int moved = servo_responds ? std::clamp(target - pitch, -pitch_rate, pitch_rate) : 0;
        pitch += moved; pitch_still = moved ? 0 : pitch_still + 1;
        in.camera_pitch_cdeg = int16_t(pitch); in.camera_pitch_stable = pitch_still >= 3;
        in.hold_observable = in.camera_pitch_stable && std::abs(pitch - tune.near_pitch_cdeg) <= 100;
        in.held = in.hold_observable ? load : Inventory{};
        in.captured = in.held_complete = in.held.total() > 0;
        in.gripper_done = gripper_responds;
        in.gripper_feedback_open = gripper_responds ? out.motion.gripper_open : -1;
        out = task.update(in);
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
    assert(checkTrip(inv(1), false) == RuleVerdict::OK && checkTrip(inv(3), false) == RuleVerdict::OK);
    assert(checkTrip(inv(4), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(2, 2), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(1, 2), true) == RuleVerdict::OK);
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
    CaptureMonitor monitor;
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
    auto far = in; far.camera_pitch_cdeg = 1200; CaptureMonitor other;
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
    CaptureMonitor corridor;
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
int main(int argc, char **argv) {
    if (argc == 3 && std::string(argv[1]) == "--write-fixture") { writeFixture(argv[2]); return 0; }
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
    { Sim s;s.toCarry();assert(s.until(PushState::OPEN_RELEASE,400));s.drop_available=false;
      for(int i=0;i<10;++i){auto o=s.tick();assert(stopped(o.motion)&&o.motion.gripper_open==0);}
    }
    rulesTests();
    captureTests();
    { // Interim firmware presets (2026-10: only -25/0/+25 deg): FAR 0, TRACK = NEAR = 2500.
        TaskTuning t; t.far_pitch_cdeg = 0; t.track_pitch_cdeg = t.near_pitch_cdeg = 2500;
        Sim s(t); s.toCarry(); s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered && s.out.reason == "delivered");
        assert(!s.visited.count(PushState::SAFE_STOP) && !s.visited.count(PushState::ABORT_DROP));
        // Every pitch the task commanded is one the MCU has.
        assert((s.commanded == std::set<int>{0, 2500}));
    }
    { // Full first ordinary trip: rush, enclose, carry, release at the gate, push in, back out, turn.
        Sim s; s.toCarry();
        assert(s.out.batch_size == 1 && s.out.motion.gripper_open == 0);
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered && s.out.reason == "delivered");
        for (auto st : {PushState::SCAN, PushState::APPROACH, PushState::PREPARE, PushState::RUSH, PushState::CLOSE,
                        PushState::VERIFY_CAPTURE, PushState::CARRY, PushState::GATE, PushState::OPEN_RELEASE,
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
        assert(s.until(PushState::ABORT_DROP, 10) && s.out.verdict == RuleVerdict::INJURED_NOT_ALONE);
        s.hold({});
        assert(s.until(PushState::SCAN, 200) && s.out.batch_size == 0 && s.out.delivered_total == 3);
        // Alone, it goes to the injured half.
        s.in.target_id = 10; s.px = .15f; s.py = -.75f; s.phi = 0; s.task_injured = true;
        s.toCarry(inv(0, 0, 1));
        s.deliver();
        assert(s.out.delivered_total == 4 && s.in.zone_injured_count == 1);
        assert(s.px > 0); // pushed into the right (injured) half
    }
    for (const auto *label : {"core_supply", "injured_person", "dangerous_object", "unknown"}) {
        // Before the first ordinary delivery only ordinary supplies are selectable.
        Sim s; s.in.label = label;
        for (int i = 0; i < 20; ++i) s.tick();
        assert(s.out.state == PushState::SCAN && s.out.target_id == -1 && s.out.motion.vx_mps == 0);
    }
    { // First trip that also swept a core supply into the frame: release and back off.
        Sim s; s.toRush(); s.hold(inv(1, 1));
        assert(s.until(PushState::ABORT_DROP, 10) && s.out.verdict == RuleVerdict::CORE_BEFORE_FIRST);
        assert(s.out.motion.gripper_open == 1);
        s.hold({});
        assert(s.until(PushState::SCAN, 200));
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered);
        for (int i = 0; i < 10; ++i) assert(s.tick().state == PushState::SCAN); // primary blacklisted
    }
    { // A dangerous object enclosed with the target is never carried.
        Sim s; s.toRush(); s.hold(inv(1, 0, 0, 1));
        assert(s.until(PushState::ABORT_DROP, 10) && s.out.verdict == RuleVerdict::DANGEROUS);
    }
    { // Too many supplies in the corridor: the rush is not started.
        Sim s; s.in.corridor = inv(4);
        for (int i = 0; i < 40 && s.out.reason != "corridor_too_many_supplies"; ++i) s.tick();
        assert(s.out.reason == "corridor_too_many_supplies" && !s.visited.count(PushState::RUSH));
    }
    { // ... nor when four end up held after closing.
        Sim s; s.in.corridor = inv(3); s.toRush(); s.hold(inv(4));
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
        assert(s.until(PushState::CAPTURE_FAIL, 100) && s.out.reason == "rush_overrun");
        assert(s.until(PushState::SCAN, 200));
    }
    { // The load slips out during the carry: unseen at FAR, found by the next stopped NEAR check.
        Sim s; s.toCarry(); s.tick(); s.hold({});
        for (int i = 0; i < 20; ++i) assert(s.tick().state == PushState::CARRY);
        assert(s.until(PushState::LOST_HOLD, 200));
        assert(std::abs(s.in.camera_pitch_cdeg - 3500) <= 100 && stopped(s.out.motion));
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
            if (s.out.motion.camera_pitch_cdeg == 3500) { assert(stopped(s.out.motion)); ++near_checks; }
            if (s.out.motion.camera_pitch_cdeg == 1200 && s.in.camera_pitch_stable) ++far;
        }
        assert(far > 100 && near_checks > 0);
    }
    { // Nothing moves while the camera is between presets.
        Sim s; s.in.distance_m = .8f; s.pitch_rate = 50;
        for (int i = 0; i < 25; ++i) { s.tick(); assert(stopped(s.out.motion) && s.out.state == PushState::SCAN); }
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
        for (int i = 0; i < 100 && s.out.state == PushState::GATE; ++i) { s.tick(); assert(s.out.motion.gripper_open == 0); }
        assert(s.out.state == PushState::LOST_HOLD && !s.visited.count(PushState::OPEN_RELEASE));
    }
    { // Pushed, but the zone counts never confirm it: no credit, first stays false.
        Sim s; s.toCarry(); s.deliver(false);
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered && s.out.reason == "delivery_unverified");
        assert(s.until(PushState::SCAN, 400));
        s.in.label = "core_supply"; s.in.target_id = 8;
        for (int i = 0; i < 20; ++i) assert(s.tick().state == PushState::SCAN);
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
        assert(s.until(PushState::LOST_HOLD, 200) && s.out.motion.gripper_open == 0);
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
    { // No trusted zone estimate: carry only searches in place, never drives blind.
        Sim s; s.toCarry(); s.zone_visible = false;
        for (int i = 0; i < 10; ++i) { s.tick(); assert(s.out.state == PushState::CARRY && s.out.motion.vx_mps == 0); }
        s.zone_visible = true; s.tick(); assert(s.out.motion.vx_mps > 0);
    }
    { // Stale frame or safety veto stops; resuming keeps the verified load.
        Sim s; s.toCarry(); s.tick();
        s.in.safety_ok = false; s.tick();
        assert(s.out.state == PushState::WAIT_START && stopped(s.out.motion) && s.out.batch_size == 1);
        assert(s.out.motion.gripper_open == 0); // a stop never opens the frame
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
    assert(makeTargetAreas("red") == std::vector<std::string>{"red_safe_zone"});
    assert(makeTargetAreas("blue") == std::vector<std::string>{"blue_safe_zone"});
    assert(makeTargetBalls("red").size() == 3);
    std::cout << "Capture, transport rules, carry, delivery, drop and stop checks passed\n";
}
