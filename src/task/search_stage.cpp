#include "rescue/task/search_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rescue {
void SearchStage::resetScanTracking() {
    vis_hits_ = vis_miss_ = 0;
    vis_chase_t0_ = 0;
    scan_hold_t0_ = scan_hold_cool_until_us_ = 0;
}

void SearchStage::resetScanHeading() {
    scan_last_valid_ = false;
}

void SearchStage::update(SearchAction state,
                         const TaskTuning& t,
                         const PushObservation& in,
                         const SearchContext& c,
                         const SearchServices& s) {
    switch (state) {
        case SearchAction::Scan:
            scan(t, in, c, s);
            break;
        case SearchAction::LostSearch:
            lostSearch(t, in, c, s);
            break;
        case SearchAction::MidReacquire:
            reacquire(t, in, c, s, true);
            break;
        case SearchAction::NearReacquire:
            reacquire(t, in, c, s, false);
            break;
        case SearchAction::SelectCargo:
            selectCargo(t, in, c, s);
            break;
        case SearchAction::MidApproach:
            midApproach(t, in, c, s);
            break;
        case SearchAction::Approach:
            approach(t, in, c, s);
            break;
        default:
            break;
    }
}

void SearchStage::scan(const TaskTuning& t,
                       const PushObservation& in,
                       const SearchContext& c,
                       const SearchServices& s) {
    int& target_id = c.trip_data.target_id_;
    std::string& label = c.trip_data.label_;
    int& confirmations = c.progress.confirmations;
    int& capture_target_id = c.trip_data.capture_target_id_;
    float& scan_turn_acc = c.search.scan_turn_acc_;
    float& approach_forward = c.trip_data.approach_forward_;
    std::string& reason = c.reason;
    const uint64_t now = c.now_us;

    s.command_frame(0, now);
    approach_forward = 0; // scans turn in place: the retreat bound restarts at the lock
    const bool cand = s.candidate(in, now);
    if (in.heading_valid && std::isfinite(in.heading_rad)) {
        if (scan_last_valid_)
            scan_turn_acc += std::abs(wrapAngle(in.heading_rad - scan_last_heading_));
        scan_last_heading_ = in.heading_rad;
        scan_last_valid_ = true;
    } else
        scan_last_valid_ = false;
    if (!cand) {
        confirmations = 0;
        target_id = -1;

        reason = c.scan_ok ? "searching_target" : "scan_path_blocked";
        if (s.unstick())
            return;
        if (t.visual_chase) {
            const bool seen = in.target_valid && in.target_id >= 0 && in.pixel_bearing_valid &&
                              std::isfinite(in.pixel_bearing) &&
                              in.pixel_confidence >= t.visual_chase_min_conf &&
                              targetSelectable(in.label, c.first_ordinary) && !in.target_in_zone;
            if (seen) {
                ++vis_hits_;
                vis_miss_ = 0;
                vis_bearing_ = in.pixel_bearing;
                vis_last_seen_us_ = now;
            } else if (++vis_miss_ > t.visual_chase_miss_frames) {
                vis_hits_ = 0;
                vis_chase_t0_ = 0;
            }
            if (vis_hits_ >= t.visual_chase_frames &&
                now - vis_last_seen_us_ <= t.visual_chase_hold_us && now >= vis_cool_until_us_ &&
                c.scan_ok) {
                if (!vis_chase_t0_)
                    vis_chase_t0_ = now;
                if (now - vis_chase_t0_ > t.visual_chase_budget_us) {
                    vis_hits_ = 0;
                    vis_chase_t0_ = 0;
                    vis_cool_until_us_ = now + t.visual_chase_cool_us;
                } else if (std::abs(vis_bearing_) > t.rotate_in_place_rad) {
                    reason = "visual_chase_turn";
                    s.drive(0, t.heading_gain * vis_bearing_);
                    return;
                } else if (c.forward_ok) {
                    reason = "visual_chase";
                    s.drive(t.visual_chase_speed,
                            std::abs(vis_bearing_) > t.heading_deadband_rad
                                ? t.heading_gain * vis_bearing_
                                : 0.f);
                    return;
                }
            }
        }
        if (t.scan_sighting_hold && now >= vis_cool_until_us_) {
            const bool sighted =
                in.pixel_reason == "confirming_three_frames" ||
                in.pixel_reason == "pixel_ratio_locked" ||
                in.pixel_reason == "lock_grace_waiting" ||
                (vis_hits_ > 0 && now - vis_last_seen_us_ <= t.visual_chase_hold_us);
            if (sighted)
                scan_hold_last_sight_us_ = now;
            if (!scan_hold_t0_ && sighted && now >= scan_hold_cool_until_us_)
                scan_hold_t0_ = now;
            if (scan_hold_t0_) {
                if (now - scan_hold_t0_ <= t.scan_hold_max_us &&
                    now - scan_hold_last_sight_us_ <= t.scan_hold_lost_us) {
                    reason = "scan_sighting_hold";
                    s.drive(0, 0);
                    return;
                }
                scan_hold_t0_ = 0;
                scan_hold_cool_until_us_ = now + t.scan_hold_cool_us;
            }
        }
        s.scan_spin();
        return;
    }
    if (target_id != in.target_id || label != in.label) {
        target_id = capture_target_id = in.target_id;
        label = in.label;
        confirmations = 0;
    }
    if (++confirmations >= t.confirm_frames) {
        if (t.demo_search_only) {
            s.fail_stop("demo_target_found");
        } else
            s.enter(SearchAction::Approach, now, "target_locked");
    }
}

void SearchStage::lostSearch(const TaskTuning& t,
                             const PushObservation& in,
                             const SearchContext& c,
                             const SearchServices& s) {
    int& target_id = c.trip_data.target_id_;
    std::string& label = c.trip_data.label_;
    int& confirmations = c.progress.confirmations;
    int& capture_target_id = c.trip_data.capture_target_id_;
    int& step = c.progress.step;
    uint64_t& phase_us = c.progress.started_us;
    float& turned = c.progress.turned;
    bool& heading_seen = c.progress.heading_seen;
    float& last_heading = c.progress.last_heading;
    int& lost_round = c.search.lost_round_;
    float& approach_forward = c.trip_data.approach_forward_;
    std::string& reason = c.reason;
    const uint64_t now = c.now_us;

    approach_forward = 0;
    // pitchWait above finishes FAR transition before wheels can turn.
    if (step == 0) {
        step = 1;
        phase_us = now;
        turned = 0;
        heading_seen = false;
    }
    if ((!in.gripper_closed_observed && !c.sweep_open) || !in.heading_valid ||
        !std::isfinite(in.heading_rad)) { // spinning is allowed with the frame closed
                                          // (confirmed) or deliberately open
        s.fail_stop("lost_search_feedback_missing");
        return;
    }
    if (now - phase_us >= 20000000 || turned >= 2 * float(CV_PI)) {
        // A FAR sweep misses cargo right in front of the jaw; stop, tilt to 20deg, sweep
        // again.
        if (!lost_round && c.mid_pitch) {
            lost_round = 1;
            s.enter(SearchAction::LostSearch, now, "lost_search_mid_pitch");
            return;
        }
        if (t.lost_search_rescan && c.frame_raised == 0 && c.trip_empty) {
            // Nothing found in two views: return to normal far-view scanning.
            lost_round = 0;
            target_id = -1;
            confirmations = 0;
            s.enter(SearchAction::Scan, now, "lost_search_rescan");
            return;
        }
        s.fail_stop("lost_search_exhausted");
        return;
    }
    if (heading_seen)
        turned += std::abs(wrapAngle(in.heading_rad - last_heading));
    last_heading = in.heading_rad;
    heading_seen = true;
    if (s.candidate(in, now)) {
        if (target_id != in.target_id || label != in.label) {
            target_id = in.target_id;
            label = in.label;
            confirmations = 0;
        }
        if (++confirmations >= t.confirm_frames) {
            if (lost_round && in.distance_m <= t.track_near_m) {
                // Close cargo found at 20deg: keep this view instead of tilting back to
                // FAR.
                target_id = capture_target_id = -1;

                s.enter(SearchAction::MidReacquire, now, "lost_target_reacquired_mid");
                return;
            }
            capture_target_id = target_id;
            s.enter(SearchAction::Approach, now, "lost_target_reacquired");
        }
        return; // stop while confirming a newly seen cargo candidate
    }
    target_id = -1;
    confirmations = 0;
    if (s.unstick())
        return;
    reason = c.scan_ok ? (lost_round ? "rotating_for_lost_target_mid" : "rotating_for_lost_target")
                       : "lost_search_turn_blocked";
    s.scan_spin();
}

void SearchStage::reacquire(const TaskTuning& t,
                            const PushObservation& in,
                            const SearchContext& c,
                            const SearchServices& s,
                            bool mid) {
    int& target_id = c.trip_data.target_id_;
    std::string& label = c.trip_data.label_;
    int& confirmations = c.progress.confirmations;
    int& capture_target_id = c.trip_data.capture_target_id_;
    int& step = c.progress.step;
    uint64_t& phase_us = c.progress.started_us;
    int& near_failures = c.search.near_failures_;
    std::string& reason = c.reason;
    const uint64_t now = c.now_us;

    // Preserve an already open jaw across a camera change; never close blindly.
    // Global pitchWait has completed. Give re-identification a full 3s at stable TRACK.
    if (step == 0) {
        step = 1;
        phase_us = now;
        confirmations = 0;
    }
    if (now - phase_us >= 3000000) {
        if (c.frame_raised == 1) {
            s.fail_stop("open_jaw_reacquire_timeout");
            return;
        }
        if (t.enable_lost_search) {
            s.search_lost();
            return;
        }
        ++near_failures;
        if (near_failures >= 2) {
            s.fail_stop("near_reacquire_failed_stop");
            return;
        }
        s.scan_after_clear("near_reacquire_retry_search");
        return;
    }
    if (!s.candidate(in, now)) {
        confirmations = 0;
        reason = mid ? "mid_no_valid_cargo" : "near_no_valid_cargo";
        return;
    }
    if (c.frame_raised == 1 && (!in.corridor_complete || in.corridor != c.trip_data.pending_ ||
                                checkTrip(in.corridor, c.first_ordinary) != RuleVerdict::OK)) {
        confirmations = 0;
        reason = "near_open_jaw_batch_unconfirmed";
        return;
    }
    if (!c.forward_ok) {
        confirmations = 0;
        reason = mid ? "mid_cargo_path_blocked" : "near_cargo_path_blocked";
        return;
    }
    if (capture_target_id != in.target_id || label != in.label) {
        capture_target_id = in.target_id;
        label = in.label;
        confirmations = 0;
    }
    if (++confirmations >= t.confirm_frames) {
        target_id = capture_target_id;
        s.enter(mid ? SearchAction::MidApproach : SearchAction::Approach,
                now,
                mid ? "mid_cargo_reacquired" : "near_cargo_reacquired");
    }
}

void SearchStage::selectCargo(const TaskTuning& t,
                              const PushObservation& in,
                              const SearchContext& c,
                              const SearchServices& s) {
    int& target_id = c.trip_data.target_id_;
    std::string& label = c.trip_data.label_;
    int& confirmations = c.progress.confirmations;
    int& capture_target_id = c.trip_data.capture_target_id_;
    std::string& reason = c.reason;
    const uint64_t now = c.now_us;

    s.command_frame(0, now);
    if (c.elapsed_us > t.prepare_budget_us) {
        s.blacklist(now);
        s.scan_after_clear("no_eligible_cargo");
        return;
    }
    if (!s.candidate(in, now)) {
        confirmations = 0;
        reason = "waiting_eligible_cargo";
        return;
    }
    if (capture_target_id != in.target_id || label != in.label) {
        capture_target_id = in.target_id;
        label = in.label;
        confirmations = 0;
    }
    if (++confirmations >= t.confirm_frames) {
        target_id = capture_target_id;
        s.enter(SearchAction::Approach, now, "cargo_locked");
    }
}

void SearchStage::midApproach(const TaskTuning& t,
                              const PushObservation& in,
                              const SearchContext& c,
                              const SearchServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    const bool forward_ok = c.forward_ok;
    const int frame_raised_ = c.frame_raised;
    const bool first_ = c.first_ordinary;
    auto& pending_ = c.trip_data.pending_;
    auto& misses_ = c.progress.misses;
    auto& target_id_ = c.trip_data.target_id_;
    auto& label_ = c.trip_data.label_;
    auto& confirmations_ = c.progress.confirmations;
    auto& capture_target_id_ = c.trip_data.capture_target_id_;
    auto& near_failures_ = c.search.near_failures_;
    auto& reason_ = c.reason;
    auto& last_range_ = c.progress.last_range;
    auto& verdict_ = c.trip_data.verdict_;
    auto& seen_ = c.trip_data.seen_;
    const auto drive = [&](float vx, float wz) {
        s.drive(vx, wz);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto failStop = [&](const char* why) {
        s.fail_stop(why);
    };
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto searchLost = [&] {
        s.search_lost();
    };
    const auto frameWait = [&](const PushObservation& o, uint8_t open, uint64_t at) {
        return s.frame_wait(o, open, at);
    };
    const auto blacklist = [&](uint64_t at) {
        s.blacklist(at);
    };
    const auto lostTarget = [&] {
        return s.lost_target();
    };
    const auto tracking = [&](const PushObservation& o) {
        return s.tracking(o);
    };
    const auto trackingReason = [&] {
        return s.tracking_reason();
    };
    const auto steer = [&](float speed, float heading) {
        s.steer(speed, heading);
    };
    do {
        if (!tracking(in)) {
            reason_ = trackingReason();
            if (frame_raised_ == 1 && last_range_ <= t_.open_jaw_handoff_m) {
                target_id_ = capture_target_id_ = -1;
                enter(SearchAction::NearReacquire, now, "open_jaw_lost_at_mouth_near_handoff");
                break;
            }
            if (lostTarget()) {
                if (t_.enable_lost_search) {
                    searchLost();
                    break;
                }
                if (frame_raised_ == 1) {
                    failStop("open_jaw_target_lost");
                } else if (++near_failures_ >= 2) {
                    failStop("mid_tracking_failed_stop");
                } else {
                    target_id_ = capture_target_id_ = -1;
                    enter(SearchAction::MidReacquire, now, "mid_target_lost_reacquire");
                }
            }
            break;
        }
        misses_ = 0;
        last_range_ = in.distance_m;

        if (!forward_ok) {
            if (frame_raised_ == 1) {
                failStop("open_jaw_path_blocked");
                break;
            }
            blacklist(now);
            target_id_ = capture_target_id_ = -1;
            enter(SearchAction::MidReacquire, now, "blocked_cargo_reselect");
            break;
        }
        if (elapsed > t_.approach_budget_us) {
            failStop("mid_approach_timeout");
            break;
        }
        if (frame_raised_ == 0 && in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) > t_.rush_heading_rad) {
                drive(0, t_.heading_gain * in.heading_error);
                break;
            }

            verdict_ = checkCorridor(
                in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_);
            if (verdict_ != RuleVerdict::OK) {

                blacklist(now);
                target_id_ = capture_target_id_ = -1;
                if (++near_failures_ >= 2) {
                    failStop("mid_open_corridor_rejected");
                } else
                    enter(SearchAction::MidReacquire, now, "mid_open_corridor_reselect");
                break;
            }
            if (confirmations_ == 0 || seen_ != in.corridor) {
                seen_ = in.corridor;
                confirmations_ = 0;
            }
            if (++confirmations_ < t_.confirm_frames) {
                reason_ = "mid_open_corridor_confirm";
                break;
            }
            pending_ = in.corridor;
            commandFrame(1, now);
            reason_ = "mid_open_command";
            break;
        }
        if (frame_raised_ == 1) {
            const int g = frameWait(in, 1, now);
            if (g < 0) {
                failStop("mid_gripper_open_timeout");
                break;
            }
            if (g == 0) {
                reason_ = "mid_gripper_feedback_wait";
                break;
            }
            verdict_ = checkCorridor(
                in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_);
            // Corridor differs from the batch the jaw opened for: stop and confirm the change
            // for confirm_frames. A confirmed legal batch replaces it; a confirmed illegal or
            // incomplete one stops the task.
            if (verdict_ != RuleVerdict::OK || in.corridor != pending_) {
                if (confirmations_ == 0 || seen_ != in.corridor || seen_verdict_ != verdict_) {
                    seen_ = in.corridor;
                    seen_verdict_ = verdict_;
                    confirmations_ = 0;
                }
                if (++confirmations_ < t_.confirm_frames) {
                    reason_ = "open_jaw_corridor_recheck";
                    break;
                }
                if (verdict_ != RuleVerdict::OK) {
                    failStop("open_jaw_corridor_changed");
                    break;
                }
                pending_ = in.corridor;
            }
            confirmations_ = 0;
            if (in.distance_m <= t_.intermediate_to_near_m) {
                target_id_ = capture_target_id_ = -1;
                enter(SearchAction::NearReacquire, now, "near_reacquire_pitch_wait");
                break;
            }
        }
        steer(t_.approach_speed, in.heading_error);
        break;
    } while (false);
}

void SearchStage::approach(const TaskTuning& t,
                           const PushObservation& in,
                           const SearchContext& c,
                           const SearchServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    const bool first_ = c.first_ordinary;
    const int16_t pitch_cmd_ = c.pitch_cmd;
    float& motion_wz = c.motion.wz_rps;
    auto& misses_ = c.progress.misses;
    auto& track_close_ = c.progress.track_close;
    auto& target_id_ = c.trip_data.target_id_;
    auto& label_ = c.trip_data.label_;
    auto& capture_target_id_ = c.trip_data.capture_target_id_;
    auto& phase_us_ = c.progress.started_us;
    auto& near_failures_ = c.search.near_failures_;
    auto& reason_ = c.reason;
    auto& approach_last_id_ = c.search.approach_last_id_;
    auto& rush_pulse_phase_ = c.search.rush_pulse_phase_;
    auto& rush_pulse_count_ = c.search.rush_pulse_count_;
    auto& precheck_seen_ = c.search.precheck_seen_;
    auto& precheck_rejects_ = c.search.precheck_rejects_;
    auto& approach_last_heading_ = c.search.approach_last_heading_;
    auto& approach_last_distance_ = c.search.approach_last_distance_;
    auto& approach_last_world_yaw_ = c.search.approach_last_world_yaw_;
    auto& approach_rebind_ = c.search.approach_rebind_;
    auto& approach_last_world_valid_ = c.search.approach_last_world_valid_;
    auto& approach_pitch_ = c.search.approach_pitch_;
    auto& verdict_ = c.trip_data.verdict_;
    const auto candidate = [&](const PushObservation& o, uint64_t at) {
        return s.candidate(o, at);
    };
    const auto drive = [&](float vx, float wz) {
        s.drive(vx, wz);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto failStop = [&](const char* why) {
        s.fail_stop(why);
    };
    const auto scanAfterClear = [&](const char* why) {
        s.scan_after_clear(why);
    };
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto searchLost = [&] {
        s.search_lost();
    };
    const auto clearTrip = [&] {
        s.clear_trip();
    };
    const auto blacklist = [&](uint64_t at) {
        s.blacklist(at);
    };
    const auto lostTarget = [&] {
        return s.lost_target();
    };
    const auto tracking = [&](const PushObservation& o) {
        return s.tracking(o);
    };
    const auto trackingReason = [&] {
        return s.tracking_reason();
    };
    const auto approachHeadingShift = [&](const PushObservation& o) {
        return s.approach_heading_shift(o);
    };
    const auto desiredPitch = [&]() -> int16_t {
        return s.desired_pitch();
    };
    const auto midPitch = [&] {
        return s.mid_pitch();
    };
    const auto steer = [&](float speed, float heading) {
        s.steer(speed, heading);
    };
    do {
        // Keep jaw state from MID_APPROACH; a camera change must not close on unseen cargo.
        if (rush_pulse_phase_) {
            // IMU-angle pulse then a short stop; vision re-checks the bearing afterwards. Pulse
            // time is not charged to the approach budget.
            const auto pulseEnd = [&](const char* why) {
                phase_us_ += now - rush_pulse_t0_us_;
                rush_pulse_phase_ = 0;
                reason_ = why;
            };
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                rush_pulse_count_ = t_.rush_pulse_max;
                pulseEnd("rush_pulse_no_heading");
                break;
            }
            const uint64_t pe = now - rush_pulse_start_us_;
            if (rush_pulse_phase_ == 1) {
                const float done = wrapAngle(in.heading_rad - rush_pulse_yaw0_) *
                                   (rush_pulse_delta_ > 0 ? 1.f : -1.f);
                if (done >= t_.rush_pulse_done_frac * std::abs(rush_pulse_delta_) ||
                    pe > t_.rush_pulse_budget_us) {
                    rush_pulse_phase_ = 2;
                    rush_pulse_start_us_ = now;
                    reason_ = "rush_pulse_settle";
                    break;
                }
                reason_ = "rush_pulse_turn";
                drive(0, std::copysign(t_.rush_pulse_wz, rush_pulse_delta_));
                if (motion_wz != 0)
                    motion_wz = std::copysign(t_.rush_pulse_wz, rush_pulse_delta_);
                break;
            }
            if (pe < t_.rush_pulse_settle_us) {
                reason_ = "rush_pulse_settle";
                break;
            }
            pulseEnd("rush_pulse_done");
        }
        if (!tracking(in)) {
            reason_ = trackingReason();
            // 下压相机后跟踪器会给同一货物分配新ID：到位后若看到同标签的有效货物，改绑新ID继续接近。
            if (approach_rebind_ && now - rebind_since_us_ > t_.approach_rebind_window_us)
                approach_rebind_ = false;
            if (approach_rebind_ && desiredPitch() == pitch_cmd_ && in.camera_pitch_stable &&
                in.target_valid && in.target_id != target_id_ && in.label == label_ &&
                candidate(in, now)) {
                target_id_ = capture_target_id_ = in.target_id;
                misses_ = 0;
                reason_ = "approach_id_rebound_after_pitch";
                break;
            }
            // 分级下压后相机转动/重新选目标期间图像无效、跟踪器要重新确认：窗口内停车等待，不算丢目标。
            if (approach_rebind_) {
                misses_ = 0;
                break;
            }
            // 跟踪器对同一货物重新分配ID：同标签、方位与距离接近上次跟踪值的合法候选，直接改绑而不是进入丢失搜索。
            if (approach_last_id_ == target_id_ && target_id_ >= 0 && in.target_valid &&
                in.target_id != target_id_ && in.label == label_ && candidate(in, now) &&
                std::abs(approachHeadingShift(in)) <= .3f &&
                std::abs(in.distance_m - approach_last_distance_) <= .35f) {
                target_id_ = capture_target_id_ = in.target_id;
                misses_ = 0;
                approach_last_id_ = target_id_;
                reason_ = "approach_id_rebound_same_object";
                break;
            }
            // 近处丢失多半是被车框挡住：原地等跟踪器给出新ID（选择器优先锁消失点附近的目标），不立即进入丢失搜索。
            if (t_.occlusion_hold_us > 0 && approach_last_id_ == target_id_ && target_id_ >= 0 &&
                approach_last_us_ > 0 && now >= approach_last_us_ &&
                now - approach_last_us_ <= t_.occlusion_hold_us &&
                approach_last_distance_ <= t_.occlusion_near_m) {
                misses_ = 0;
                reason_ = "target_occluded_wait";
                break;
            }
            if (lostTarget()) {
                if (t_.enable_lost_search) {
                    searchLost();
                    break;
                }
                if (t_.track_pitch_cdeg != t_.far_pitch_cdeg) {
                    if (++near_failures_ >= 2) {
                        failStop("near_tracking_failed_stop");
                    } else {
                        target_id_ = capture_target_id_ = -1;

                        enter(SearchAction::NearReacquire, now, "near_target_lost_reacquire");
                    }
                } else {
                    scanAfterClear("target_lost");
                }
            }
            break;
        }
        misses_ = 0;
        approach_last_id_ = target_id_;
        approach_last_heading_ = in.heading_error;
        approach_last_distance_ = in.distance_m;
        approach_last_us_ = now;
        approach_last_world_valid_ =
            in.heading_valid && std::isfinite(in.heading_rad) && std::isfinite(in.heading_error);
        if (approach_last_world_valid_)
            approach_last_world_yaw_ = wrapAngle(in.heading_rad + in.heading_error);
        if (elapsed > t_.approach_budget_us) {
            blacklist(now);
            scanAfterClear("approach_timeout");
            break;
        }
        // Tilt down to TRACK when close (hysteresis keeps it from toggling), stopped while it
        // moves.
        if (in.distance_m <= t_.track_near_m)
            track_close_ = true;
        else if (in.distance_m > t_.track_near_m + .1f)
            track_close_ = false;
        // 目标贴近（像素接近画面底边）时分级下压相机继续跟踪：只下压不回抬，停车等待到位。
        if (midPitch() && !(t_.demo_capture_motion && approach_pitch_ != t_.far_pitch_cdeg &&
                            in.distance_m <= t_.rush_start_m &&
                            std::abs(in.heading_error) <= t_.rush_heading_rad)) {
            const int16_t before = approach_pitch_;
            if (in.distance_m <= t_.intermediate_to_near_m)
                approach_pitch_ = t_.track_pitch_cdeg;
            else if (in.distance_m <= t_.track_near_m && approach_pitch_ == t_.far_pitch_cdeg)
                approach_pitch_ = t_.intermediate_pitch_cdeg;
            if (approach_pitch_ != before) {
                approach_rebind_ = true;
                rebind_since_us_ = now;
            }
        }
        if (desiredPitch() != pitch_cmd_) {
            reason_ = "camera_pitch_wait";
            break;
        }
        reason_ = "approach_target";
        if (in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) <= t_.rush_heading_rad) {
                if (t_.corridor_precheck && !t_.assume_all_safe &&
                    precheck_rejects_ < t_.corridor_precheck_max && in.corridor_complete) {
                    const auto v = checkCorridor(
                        in.corridor, true, in.corridor_occlusion_free, label_, first_);
                    const bool fatal =
                        v == RuleVerdict::DANGEROUS || v == RuleVerdict::UNKNOWN_OBJECT ||
                        v == RuleVerdict::CORE_BEFORE_FIRST ||
                        v == RuleVerdict::INJURED_BEFORE_FIRST ||
                        v == RuleVerdict::INJURED_NOT_ALONE || v == RuleVerdict::TOO_MANY_SUPPLIES;
                    if (fatal) {
                        if (++precheck_seen_ < t_.corridor_precheck_confirm) {
                            reason_ = "corridor_precheck_confirm";
                            break;
                        }
                        precheck_seen_ = 0;
                        ++precheck_rejects_;
                        verdict_ = v;
                        blacklist(now);
                        clearTrip();
                        const std::string why = std::string("corridor_precheck_") + verdictName(v);
                        enter(SearchAction::Scan, now, why.c_str());
                        break;
                    }
                    precheck_seen_ = 0;
                }
                // PREPARE tilts the camera to the tracking pitch; a block farther than this is
                // cut off by the image top (box_at_image_edge), so close in first.
                if (!t_.assume_all_safe && !t_.demo_capture_motion && midPitch() &&
                    in.distance_m > t_.intermediate_to_near_m) {
                    if (now - prep_last_us_ > 500000 ||
                        in.distance_m < prep_ref_m_ - t_.approach_stall_progress_m) {
                        prep_ref_m_ = in.distance_m;
                        prep_ref_us_ = now;
                    }
                    prep_last_us_ = now;
                    if (now - prep_ref_us_ >= t_.approach_stall_us &&
                        in.distance_m <= t_.approach_stall_accept_m) {
                        prep_last_us_ = 0;
                        enter(SearchAction::Prepare, now, "approach_range_stalled");
                        commandFrame(1, now);
                        break;
                    }
                    reason_ = "approach_to_prepare_range";
                    steer(t_.approach_speed, in.heading_error);
                    break;
                }
                prep_last_us_ = 0;
                enter(SearchAction::Prepare, now, "rush_aligned");
                commandFrame(1, now); // PREPARE output is stationary and opens immediately.
            } else if (t_.rush_pulse_turn && rush_pulse_count_ < t_.rush_pulse_max &&
                       in.heading_valid && std::isfinite(in.heading_rad) &&
                       std::abs(in.heading_error) <= t_.rush_pulse_max_rad) {
                ++rush_pulse_count_;
                rush_pulse_phase_ = 1;
                rush_pulse_yaw0_ = in.heading_rad;
                rush_pulse_delta_ = in.heading_error;
                rush_pulse_start_us_ = rush_pulse_t0_us_ = now;
                reason_ = "rush_pulse_turn";
                drive(0, std::copysign(t_.rush_pulse_wz, in.heading_error));
                if (motion_wz != 0)
                    motion_wz = std::copysign(t_.rush_pulse_wz, in.heading_error);
            } else
                drive(0, t_.heading_gain * in.heading_error);
        } else
            steer(t_.approach_speed, in.heading_error);
        break;
    } while (false);
}
} // namespace rescue
