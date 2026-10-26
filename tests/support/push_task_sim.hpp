#pragma once

#include <array>
#include <tuple>
#include "rescue/task/transport_rules.hpp"
#include "rescue/core/types.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include "rescue/geometry/zone_dead_reckoning.hpp"
#include "rescue/geometry/planner.hpp"
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
#include "rescue/task/push_task.hpp"
#undef private
#include "rescue/perception/capture_monitor.hpp"
#include "rescue/config/config.hpp"
#include "rescue/control/motion_readiness.hpp"
#include "rescue/control/motion_watch.hpp"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <iostream>
using namespace rescue;
inline bool stopped(const MotionCommand& m) {
    return m.vx_mps == 0 && m.wz_rps == 0;
}
inline Inventory
inv(int ordinary, int core = 0, int injured = 0, int dangerous = 0, int unknown = 0) {
    Inventory i;
    i.ordinary = ordinary;
    i.core = core;
    i.injured = injured;
    i.dangerous = dangerous;
    i.unknown = unknown;
    return i;
}
// Kinematic world: robot pose in our zone frame (x right facing the entry, y into the zone).
// The previous command is integrated before every update; gripper feedback lags one frame.
// The camera servo follows the commanded pitch at pitch_rate per frame and reads back stable
// after three still frames; the closed frame is in view only at NEAR (like CaptureMonitor).
inline bool g_non_supply_sweep =
    false; // only the dedicated tests exercise the sweep of non-supply targets
struct Sim {
    TaskTuning tune;
    PushTask task{tune};
    PushObservation in;
    PushOutput out;
    float px = -.15f, py = -.75f, phi = 0; // phi: robot heading vs zone +y, CCW positive
    bool zone_visible = true, zone_predicted = false, gripper_responds = true,
         servo_responds = true;
    bool multi_evidence = true, all_ordinary = true;
    float py_max = 1e9f;
    bool vref_visible = false, cargo_on = false, cargo_clip = false;
    float img_frac = std::nanf("");
    cv::Point2f cargo_zone{-.15f, .12f};
    bool cargo2_on = false;
    cv::Point2f cargo_zone2{-.15f, .30f};
    bool navigation_ready = true, drop_available = true, live_camera_guard = false;
    int pitch = 0, pitch_still = 0, pitch_rate = 400; // power-on: level
    Inventory load;                                   // what is physically enclosed
    std::vector<PushObservation::HeldObject>
        load_objects; // detections behind `load` (identity/confidence)
    std::set<PushState> visited;
    static TaskTuning testTune(TaskTuning t) {
        t.startup_advance_us = 0;
        t.require_multi_view = false;
        t.sweep_non_supply = g_non_supply_sweep;
        t.rush_stall_us = 0; /* sim targets do not close; stall tests re-enable it */
        return t;
    }
    explicit Sim(TaskTuning t = {}) : tune(testTune(t)), task(tune) {
        in.run = in.safety_ok = in.target_valid = in.geometry_valid = true;
        in.target_region_valid = in.zone_identity_verified = true;
        in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.target_id = 7;
        in.label = "ordinary_supply";
        in.distance_m = .2f;
        in.corridor = inv(1);
        in.corridor_complete = in.corridor_occlusion_free = true;
        in.zone_valid = in.zone_own = true;
        in.zone_class = "supply";
        in.zone_inventory_complete = in.zone_counts_valid = true;
        in.heading_valid = true;
    }
    PushOutput tick() {
        const float dt = .05f;
        phi += out.motion.wz_rps * dt;
        px += -std::sin(phi) * out.motion.vx_mps * dt;
        py += std::cos(phi) * out.motion.vx_mps * dt;
        if (py > py_max)
            py = py_max;
        in.now_us += 50000;
        auto& z = in.zone_estimate;
        z = ZoneEstimate{};
        if (zone_visible) {
            z.valid = true;
            z.source = zone_predicted ? ZoneEstimate::Source::PREDICTED
                                      : ZoneEstimate::Source::MULTI_POINT;
            z.frame_id = in.now_us / 50000;
            z.timestamp_us = z.observed_us = in.now_us;
            z.zone_label = "red_safe_zone";
            z.geometry_id = "rescue2027-inner-v1-red";
            z.inlier_ids = {0, 2, 3, 5};
            z.residual_m = .001f;
            z.position_sigma_m = .002f;
            z.yaw_sigma_rad = .01f;
            z.predicted_distance_m = 0;
            // p_body = R(-phi)(p_zone - r)  =>  yaw_body = -phi, origin = R(-phi)(-r)
            z.yaw_body_rad = -phi;
            const float c = std::cos(-phi), s = std::sin(-phi);
            z.origin_body_m = {c * -px - s * -py, s * -px + c * -py};
        }
        in.vref_valid = false;
        if (vref_visible) { // coarse visual reference only (assume mode); same geometry as the
                            // strict estimate
            auto& v = in.vref_zone;
            v = ZoneEstimate{};
            v.valid = true;
            v.timestamp_us = in.now_us;
            v.yaw_body_rad = -phi;
            const float c = std::cos(-phi), s = std::sin(-phi);
            v.origin_body_m = {c * -px - s * -py, s * -px + c * -py};
            in.vref_valid = true;
            in.vref_points = 4;
        }
        in.cargo_body.clear();
        in.cargo_img.clear();
        in.cargo_ts_us = in.now_us;
        if (cargo_on) {
            const float c = std::cos(-phi), sn = std::sin(-phi), dx = cargo_zone.x - px,
                        dy = cargo_zone.y - py;
            const cv::Point2f cb{c * dx - sn * dy, sn * dx + c * dy};
            if (cb.y > 0) {
                in.cargo_body.push_back(cb);
                PushObservation::CargoImg ci;
                ci.bottom_px = 400;
                ci.height_px = 100;
                ci.clipped = cargo_clip;
                if (std::isfinite(img_frac)) {
                    ci.front_px = 400 + img_frac * 100;
                    ci.front_valid = true;
                }
                in.cargo_img.push_back(ci);
            }
        }
        if (cargo2_on) {
            const float c = std::cos(-phi), sn = std::sin(-phi), dx = cargo_zone2.x - px,
                        dy = cargo_zone2.y - py;
            const cv::Point2f cb{c * dx - sn * dy, sn * dx + c * dy};
            if (cb.y > 0) {
                in.cargo_body.push_back(cb);
                PushObservation::CargoImg ci;
                ci.bottom_px = 400;
                ci.height_px = 100;
                ci.clipped = cargo_clip;
                if (std::isfinite(img_frac)) {
                    ci.front_px = 400 + img_frac * 100;
                    ci.front_valid = true;
                }
                in.cargo_img.push_back(ci);
            }
        }
        // Simulator supplies explicit validated direct-route/empty-zone evidence.
        in.carry_plan_valid = navigation_ready;
        in.drop_plan_valid = drop_available;
        in.navigation_timestamp_us = in.now_us;
        in.drop_centre_zone = {task_injured ? tune.injured_half_x_m : tune.supply_half_x_m,
                               tune.deposit_y_m};
        const float centre = task_injured ? tune.injured_hold_center_y_m : tune.hold_center_y_m;
        in.carry_waypoint_body =
            z.zoneToBody({in.drop_centre_zone.x, -tune.gate_clearance_m - centre});
        in.heading_rad = phi;
        const int target = out.motion.camera_pitch_cdeg;
        const int moved = servo_responds ? std::clamp(target - pitch, -pitch_rate, pitch_rate) : 0;
        pitch += moved;
        pitch_still = moved ? 0 : pitch_still + 1;
        in.camera_pitch_cdeg = int16_t(pitch);
        in.camera_pitch_stable = pitch_still >= 3;
        in.hold_observable =
            in.camera_pitch_stable && std::abs(pitch - tune.near_pitch_cdeg) <= 100;
        in.held = in.hold_observable ? load : Inventory{};
        in.held_objects =
            in.hold_observable ? load_objects : std::vector<PushObservation::HeldObject>{};
        in.captured = in.held_complete = in.held.total() > 0;
        in.gripper_closed_observed = gripper_responds && out.motion.gripper_offset == 20;
        in.multi_view_finished =
            multi_evidence && out.state == PushState::VERIFY_CAPTURE && in.captured;
        in.multi_view_verdict = in.multi_view_finished ? 1 : 0;
        in.multi_view_inventory = in.held;
        in.gripper_done = gripper_responds;
        in.gripper_feedback_open = gripper_responds ? (out.motion.gripper_offset == 0 ? 1 : 0) : -1;
        const bool mapped = in.camera_pitch_stable && pitch == tune.far_pitch_cdeg;
        if (live_camera_guard) {
            const std::string reason =
                in.camera_pitch_stable ? "pitch_not_calibrated" : "pitch_moving";
            if (!(mapped || stationaryPitchWork(out, reason)))
                std::cerr << "guard state=" << PushTask::name(out.state)
                          << " vx=" << out.motion.vx_mps << " wz=" << out.motion.wz_rps
                          << " cmd=" << out.motion.camera_pitch_cdeg << " actual=" << pitch
                          << " stable=" << in.camera_pitch_stable << "\n";
            assert(mapped || stationaryPitchWork(out, reason));
        }
        if (all_ordinary)
            in.clutter_ordinary = in.clutter_body;
        out = task.update(in);
        if (live_camera_guard) {
            inhibitUnmappedMotion(out.motion, mapped);
            if (!mapped)
                assert(stopped(out.motion));
        }
        commanded.insert(out.motion.camera_pitch_cdeg);
        if (record)
            frames.push_back(in);
        visited.insert(out.state);
        return out;
    }
    bool until(PushState s, int limit) {
        for (int i = 0; i < limit; ++i)
            if (tick().state == s)
                return true;
        return false;
    }
    // Lock, open, inventory the corridor and start the rush.
    void toRush() {
        assert(until(PushState::RUSH, 80));
    }
    void hold(const Inventory& held) {
        load = held;
    }
    void toCarry(const Inventory& held = inv(1)) {
        toRush();
        hold(held);
        assert(until(PushState::CARRY, 40));
    }
    // Whole trip with the zone counts reacting once the load is pushed in.
    void deliver(bool counts_react = true) {
        assert(until(PushState::ENTER, 400));
        in.delivery_observed = counts_react;
        assert(until(PushState::BACK_OUT, 400));
        if (counts_react) {
            if (out.batch_size && task_injured)
                in.zone_injured_count += out.batch_size;
            else
                in.zone_supply_count += out.batch_size;
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
