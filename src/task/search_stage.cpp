#include "rescue/task/search_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
void SearchStage::resetScanTracking() {
    confirm_since_us_ = 0;
    vis_hits_ = vis_miss_ = 0;
    vis_chase_t0_ = 0;
    scan_hold_t0_ = scan_hold_cool_until_us_ = 0;
}

void SearchStage::resetScanHeading() {
    scan_last_valid_ = false;
}

void SearchStage::resetConfirmSince() {
    confirm_since_us_ = 0;
}

void SearchStage::update(PushState state,
                         const TaskTuning& t,
                         const PushObservation& in,
                         const SearchContext& c,
                         const SearchServices& s) {
    switch (state) {
        case PushState::SCAN:
            scan(t, in, c, s);
            break;
        case PushState::LOST_SEARCH:
            lostSearch(t, in, c, s);
            break;
        case PushState::MID_REACQUIRE:
            reacquire(t, in, c, s, true);
            break;
        case PushState::NEAR_REACQUIRE:
            reacquire(t, in, c, s, false);
            break;
        case PushState::SELECT_CARGO:
            selectCargo(t, in, c, s);
            break;
        case PushState::MID_APPROACH:
            midApproach(t, in, c, s);
            break;
        case PushState::APPROACH:
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
    int& target_id = *c.target_id;
    std::string& label = *c.label;
    int& confirmations = *c.confirmations;
    bool& cue_mode = *c.cue_mode;
    int& search_cue_id = *c.search_cue_id;
    int& capture_target_id = *c.capture_target_id;
    float& scan_turn_acc = *c.scan_turn_acc;
    uint64_t& scatter_idle_since_us = *c.scatter_idle_since_us;
    bool& burst_drive_used = *c.burst_drive_used;
    bool& burst_swing_armed = *c.burst_swing_armed;
    float& approach_forward = *c.approach_forward;
    std::string& reason = *c.reason;
    const uint64_t now = c.now_us;

    s.command_frame(0, now);
    approach_forward = 0; // scans turn in place: the retreat bound restarts at the lock
    const bool cue = s.cue_candidate();
    const bool cand = s.candidate(in, now);
    if (in.heading_valid && std::isfinite(in.heading_rad)) {
        if (scan_last_valid_)
            scan_turn_acc += std::abs(wrapAngle(in.heading_rad - scan_last_heading_));
        scan_last_heading_ = in.heading_rad;
        scan_last_valid_ = true;
    } else
        scan_last_valid_ = false;
    cv::Point2f weak_pt(0.f, 0.f);
    bool weak_hold = false;
    if (cand && !cue && c.scatter_ok && s.weak_candidate(in)) {
        weak_pt = s.target_body();
        weak_hold = weak_pt.y >= t.blind_scatter_min_y_m && weak_pt.y <= c.far_y &&
                    std::abs(weak_pt.x) <= t.blind_scatter_half_x_m &&
                    std::abs(std::atan2(-weak_pt.x, weak_pt.y)) <= t.blind_scatter_max_bearing_rad;
    }
    // A lock this far out is not trusted (class / position are off at this range): a group
    // of several objects is enough to drive toward; once it is within
    // blind_scatter_far_chase_m the normal decision takes over.
    if (cand && !cue && !weak_hold && c.scatter_ok && c.approach_ok &&
        std::isfinite(in.distance_m) && in.distance_m > t.blind_scatter_far_chase_m) {
        std::vector<cv::Point2f> pts = in.clutter_body;
        pts.emplace_back(s.target_body());
        s.add_ordinary_extra(pts.back());
        if (!c.clutter_fresh) {
            const cv::Point2f lock_pt = pts.back();
            pts.assign(1, lock_pt);
        }
        const BestGroup best = s.find_best(pts, c.far_y, t.blind_scatter_far_radius_m);
        // An ordinary supply that is being confirmed (confirm_frames) is not pre-empted by
        // the shorter cluster count. The hold starts at the first held tick and ends after
        // sweep_confirm_wait_us whatever the lock does (an id that flips every frame cannot
        // hold the cluster back for ever); the opening burst drive is never held back.
        const bool pending =
            t.sweep_confirm_wait_us > 0 && confirmations > 0 && confirmations < t.confirm_frames &&
            targetKind(in.label) == TargetKind::ORDINARY && !(t.first_burst && !burst_drive_used);
        if (best.n >= t.blind_scatter_far_lock_min && best.d > t.blind_scatter_max_y_m) {
            bool hold = false;
            if (pending) {
                if (!confirm_since_us_)
                    confirm_since_us_ = now;
                hold = now - confirm_since_us_ < t.sweep_confirm_wait_us;
            }
            if (hold) {
                far_seen_ = 0;
                reason = "sweep_defer_target_confirm";
            } else if (++far_seen_ >= t.sweep_confirm) {
                far_seen_ = 0;
                confirm_since_us_ = 0;
                s.begin_scatter(true, false, "blind_scatter_far_group");
                return;
            }
        } else {
            far_seen_ = 0;
            confirm_since_us_ = 0;
        }
    } else if (t.dense_no_lock && cand && !cue && !weak_hold &&
               (c.scatter_ok || c.scatter_multi_ok) && c.clutter_fresh &&
               std::isfinite(in.distance_m)) {
        // Several objects together: do not lock one of them, push on and split the group
        // (the committed sweep ignores the lock).
        far_seen_ = 0;
        confirm_since_us_ = 0;
        std::vector<cv::Point2f> pts = in.clutter_body;
        pts.emplace_back(s.target_body());
        s.add_ordinary_extra(pts.back());
        // Lock only while few targets are in view: beyond lock_max_visible any pair that
        // sits together is swept open-frame instead.
        int visible_n = 0;
        for (const auto& q : pts)
            if (std::isfinite(q.x) && std::isfinite(q.y) && q.y >= t.blind_scatter_min_y_m &&
                q.y <= c.far_y)
                ++visible_n;
        const bool multi =
            t.lock_max_visible > 0 && visible_n > t.lock_max_visible && c.scatter_multi_ok;
        int g = 0;
        if (multi)
            g = s.eval_group(
                pts, t.lock_multi_group_min, t.lock_multi_group_min, t.lock_multi_radius_m);
        const bool multi_hit = g != 0;
        if (!g && c.scatter_ok)
            g = s.eval_group(pts, t.blind_scatter_min, t.blind_scatter_far_min, -1.f);
        if (g) {
            if (++dense_seen_ < t.sweep_confirm) {
                reason = "dense_group_confirm";
                return;
            }
            dense_seen_ = 0;
            s.begin_scatter(g == 2,
                            true,
                            multi_hit ? (g == 2 ? "multi_visible_far" : "multi_visible_split")
                                      : (g == 2 ? "dense_group_far" : "dense_group_split"));
            return;
        }
        dense_seen_ = 0;
    } else {
        far_seen_ = 0;
        confirm_since_us_ = 0;
        dense_seen_ = 0;
    }
    if ((!cand && !cue) || weak_hold) {
        confirmations = 0;
        target_id = -1;
        cue_mode = false;
        reason = c.scan_ok ? "searching_target" : "scan_path_blocked";
        if (c.scan_ok && in.target_is_search_cue && *c.clear_attempts >= t.max_clear_attempts)
            reason = "cue_attempt_limit";
        if (weak_hold)
            reason = "weak_target_scatter_wait";
        if (s.unstick())
            return;
        if (t.visual_chase && !cue) {
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
        // Nothing pickable in view but other objects are: they may be wrapping the ordinary
        // supplies. Plough through the densest group. A far / few-pixel ordinary sighting
        // is not worth chasing either: it joins the group and is ploughed through.
        if (c.scatter_ok) {
            if (!scatter_idle_since_us)
                scatter_idle_since_us = now;
            const bool looked = weak_hold || *c.scatter_count > 0 ||
                                scan_turn_acc >= t.blind_scatter_first_turn_rad;
            const bool fresh =
                weak_hold || (looked && in.clutter_ts_us && now >= in.clutter_ts_us &&
                              now - in.clutter_ts_us <= t.sweep_fresh_us);
            if (fresh && now - scatter_idle_since_us >=
                             (weak_hold ? t.weak_scatter_wait_us : t.blind_scatter_wait_us)) {
                std::vector<cv::Point2f> pts = in.clutter_body;
                if (weak_hold) {
                    pts.push_back(weak_pt);
                    s.add_ordinary_extra(weak_pt);
                }
                const int need_n = weak_hold ? 1 : t.blind_scatter_min;
                const int g =
                    s.eval_group(pts, need_n, weak_hold ? 1 : t.blind_scatter_far_min, -1.f);
                if (g) {
                    if (++*c.scatter_seen < t.sweep_confirm) {
                        reason = "blind_scatter_confirm";
                        return;
                    }
                    *c.scatter_seen = 0;
                    // far: too far for the rush budget, drive closer first, then the normal
                    // trigger fires from the short range
                    s.begin_scatter(
                        g == 2, false, g == 2 ? "blind_scatter_far_start" : "blind_scatter_start");
                    return;
                }
                *c.scatter_seen = 0;
            }
        }
        if (t.scan_sighting_hold && !cue && now >= vis_cool_until_us_) {
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
    scatter_idle_since_us = 0;
    *c.scatter_seen = 0;
    if (confirmations + 1 >= t.confirm_frames)
        *c.scatter_approach_n = 0;
    if (target_id != in.target_id || label != in.label || cue_mode != cue) {
        target_id = in.target_id;
        label = in.label;
        confirmations = 0;
        cue_mode = cue;
        if (cue) {
            search_cue_id = target_id;
            capture_target_id = -1;
        } else {
            capture_target_id = target_id;
            search_cue_id = -1;
        }
    }
    if (++confirmations >= t.confirm_frames) {
        burst_swing_armed =
            false; // a normal lock ends the opening: no fast swing later in the match
        if (t.demo_search_only) {
            s.fail_stop("demo_target_found");
        } else
            s.enter(cue ? PushState::CUE_APPROACH : PushState::APPROACH,
                    now,
                    cue ? "search_cue_locked" : "target_locked");
    }
}

void SearchStage::lostSearch(const TaskTuning& t,
                             const PushObservation& in,
                             const SearchContext& c,
                             const SearchServices& s) {
    int& target_id = *c.target_id;
    std::string& label = *c.label;
    int& confirmations = *c.confirmations;
    bool& cue_mode = *c.cue_mode;
    int& search_cue_id = *c.search_cue_id;
    int& capture_target_id = *c.capture_target_id;
    int& step = *c.step;
    uint64_t& phase_us = *c.phase_us;
    float& turned = *c.turned;
    bool& heading_seen = *c.heading_seen;
    float& last_heading = *c.last_heading;
    int& lost_round = *c.lost_round;
    float& approach_forward = *c.approach_forward;
    std::string& reason = *c.reason;
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
            s.enter(PushState::LOST_SEARCH, now, "lost_search_mid_pitch");
            return;
        }
        if (t.lost_search_rescan && c.frame_raised == 0 && c.trip_empty) {
            // Nothing found in two sweeps: go back to the normal SCAN (far pitch, blind
            // scatter) instead of idling for the rest of the match.
            lost_round = 0;
            target_id = -1;
            confirmations = 0;
            s.enter(PushState::SCAN, now, "lost_search_rescan");
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
                cue_mode = false;
                s.enter(PushState::MID_REACQUIRE, now, "lost_target_reacquired_mid");
                return;
            }
            if (!t.enable_search_cues) {
                // 搜索线索关闭（演示模式）：CUE_APPROACH永远不会成立，重新找回后直接回到APPROACH。
                capture_target_id = target_id;
                search_cue_id = -1;
                cue_mode = false;
                s.enter(PushState::APPROACH, now, "lost_target_reacquired");
                return;
            }
            search_cue_id = target_id;
            capture_target_id = -1;
            cue_mode = true;
            s.enter(PushState::CUE_APPROACH, now, "lost_target_reacquired");
        }
        return; // stop while confirming a newly seen cargo candidate
    }
    target_id = -1;
    confirmations = 0;
    if (c.frame_raised == 0 && c.trip_empty && c.scatter_ok &&
        c.clutter_fresh) { // a group in view needs a sweep, not more spinning
        const int g =
            s.eval_group(in.clutter_body, t.blind_scatter_min, t.blind_scatter_far_min, -1.f);
        if (g) {
            if (++*c.scatter_seen < t.sweep_confirm) {
                reason = "lost_group_confirm";
                return;
            }
            *c.scatter_seen = 0;
            s.begin_scatter(g == 2, true, g == 2 ? "lost_group_far" : "lost_group_sweep");
            return;
        }
    }
    *c.scatter_seen = 0;
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
    int& target_id = *c.target_id;
    std::string& label = *c.label;
    int& confirmations = *c.confirmations;
    int& capture_target_id = *c.capture_target_id;
    int& step = *c.step;
    uint64_t& phase_us = *c.phase_us;
    int& near_failures = *c.near_failures;
    std::string& reason = *c.reason;
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
    if (c.frame_raised == 1 && (!in.corridor_complete || in.corridor != *c.pending ||
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
        s.enter(mid ? PushState::MID_APPROACH : PushState::APPROACH,
                now,
                mid ? "mid_cargo_reacquired" : "near_cargo_reacquired");
    }
}

void SearchStage::selectCargo(const TaskTuning& t,
                              const PushObservation& in,
                              const SearchContext& c,
                              const SearchServices& s) {
    int& target_id = *c.target_id;
    std::string& label = *c.label;
    int& confirmations = *c.confirmations;
    int& capture_target_id = *c.capture_target_id;
    std::string& reason = *c.reason;
    const uint64_t now = c.now_us;

    s.command_frame(0, now);
    if (c.elapsed_us > t.prepare_budget_us) {
        s.reject_search_cue(now);
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
        s.enter(PushState::APPROACH, now, "cargo_locked");
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
    const bool clutter_fresh = c.clutter_fresh;
    const bool sweepOver = c.sweep_over;
    const bool sweepTimeOk = c.sweep_time_ok;
    const bool sweepLedgerOk = c.sweep_ledger_ok;
    const float target_forward_m = c.target_forward_m;
    const int frame_raised_ = c.frame_raised;
    const bool first_ = c.first_ordinary;
    auto& pending_ = *c.pending;
    auto& misses_ = *c.misses;
    auto& travel_ = *c.travel;
    auto& target_id_ = *c.target_id;
    auto& label_ = *c.label;
    auto& confirmations_ = *c.confirmations;
    auto& capture_target_id_ = *c.capture_target_id;
    auto& near_failures_ = *c.near_failures;
    auto& scan_turn_acc_ = *c.scan_turn_acc;
    auto& clear_attempts_ = *c.clear_attempts;
    auto& reason_ = *c.reason;
    auto& sweep_phase_ = *c.sweep_phase;
    auto& mid_clear_n_ = *c.mid_clear_n;
    auto& sweep_travel0_ = *c.sweep_travel0;
    auto& last_range_ = *c.last_range;
    auto& disperse_sign_ = *c.disperse_sign;
    auto& disperse_rush_m_ = *c.disperse_rush_m;
    auto& scan_mid_limit_ = *c.scan_mid_limit;
    auto& sweep_start_us_ = *c.sweep_start_us;
    auto& sweep_t0_us_ = *c.sweep_t0_us;
    auto& disperse_cool_until_us_ = *c.disperse_cool_until_us;
    auto& sweep_non_supply_active_ = *c.sweep_non_supply_active;
    auto& post_disperse_mid_ = *c.post_disperse_mid;
    auto& disperse_small_ = *c.disperse_small;
    auto& verdict_ = *c.verdict;
    auto& seen_verdict_ = *c.seen_verdict;
    auto& seen_ = *c.seen;
    auto& sweep_ledger_ = *c.sweep_ledger;
    auto& disperse_swing_ = *c.disperse_swing;
    const auto candidate = [&](const PushObservation& o, uint64_t at) {
        return s.candidate(o, at);
    };
    const auto targetBody = [&] {
        return s.target_body();
    };
    const auto drive = [&](float vx, float wz) {
        s.drive(vx, wz);
    };
    const auto enter = [&](PushState st, uint64_t at, const char* why) {
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
    const auto clearTrip = [&] {
        s.clear_trip();
    };
    const auto blacklist = [&](uint64_t at) {
        s.blacklist(at);
    };
    const auto disperseStep = [&] {
        return s.disperse_step();
    };
    const auto beginPileFromSweep = [&](const char* why) {
        return s.begin_pile_from_sweep(why);
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
        if (sweep_phase_ >= 4) {
            if (t_.sweep_observation_guard && !clutter_fresh && !candidate(in, now)) {
                sweep_ledger_.close(t_, now);
                sweep_phase_ = 0;
                disperse_small_ = false;
                clearTrip();
                enter(PushState::MID_REACQUIRE, now, "sweep_observation_lost");
                break;
            }
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                sweep_ledger_.close(t_, now);
                sweep_phase_ = 0;
                disperse_small_ = false;
                reason_ = "mid_clear_no_heading";
                break;
            }
            if (sweepOver || disperseStep()) {
                const bool swung = sweep_ledger_.close(t_, now);
                if (sweep_non_supply_active_) { // swept aside, not grabbed: veto it and search
                                                // again at the close view
                    disperse_small_ = false;
                    blacklist(now);
                    clearTrip();
                    scan_mid_limit_ = scan_turn_acc_ + 2.f * float(CV_PI);
                    disperse_cool_until_us_ = now + t_.disperse_cooldown_us;
                    sweep_ledger_.setGapUntil(now + t_.sweep_gap_us);
                    if (!sweepOver && swung && beginPileFromSweep("pile_chain_non_supply"))
                        break;
                    enter(PushState::SCAN, now, "non_supply_swept");
                    break;
                }
                sweep_phase_ = 0;
                disperse_small_ = false;
                s.clear_rejected();
                s.clear_rejected_bearings();
                post_disperse_mid_ = t_.post_clear_mid_view;
                target_id_ = capture_target_id_ = -1;
                enter(PushState::MID_REACQUIRE, now, "mid_neighbour_clear_done");
            }
            break;
        }
        if (!tracking(in)) {
            reason_ = trackingReason();
            if (frame_raised_ == 1 && last_range_ <= t_.open_jaw_handoff_m) {
                target_id_ = capture_target_id_ = -1;
                enter(PushState::NEAR_REACQUIRE, now, "open_jaw_lost_at_mouth_near_handoff");
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
                    enter(PushState::MID_REACQUIRE, now, "mid_target_lost_reacquire");
                }
            }
            break;
        }
        misses_ = 0;
        last_range_ = in.distance_m;
        if (!forward_ok && frame_raised_ == 0 && t_.enable_short_push && clear_attempts_ < 2 &&
            in.clear_push_safe && std::abs(in.heading_error) <= t_.rush_heading_rad) {
            enter(PushState::CUE_PREPARE, now, "closed_push_prepare");
            break;
        }
        if (!forward_ok) {
            if (frame_raised_ == 1) {
                failStop("open_jaw_path_blocked");
                break;
            }
            blacklist(now);
            target_id_ = capture_target_id_ = -1;
            enter(PushState::MID_REACQUIRE, now, "blocked_cargo_reselect");
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
            if (t_.sweep_non_supply && t_.disperse && sweepTimeOk &&
                targetKind(label_) != TargetKind::ORDINARY && in.heading_valid &&
                std::isfinite(in.heading_rad)) {
                if (++confirmations_ < t_.confirm_frames) {
                    reason_ = "non_supply_sweep_confirm";
                    break;
                }
                confirmations_ = 0;
                sweep_non_supply_active_ = true;
                disperse_small_ = false;
                disperse_sign_ = 1.f;
                disperse_rush_m_ =
                    std::clamp(target_forward_m - t_.sweep_frame_front_m + t_.disperse_depth_m,
                               0.f,
                               t_.disperse_max_m);
                disperse_swing_.configure(t_.disperse_passes, t_.disperse_swings);
                sweep_phase_ = 4;
                sweep_start_us_ = sweep_t0_us_ = now;
                sweep_travel0_ = travel_;
                sweep_ledger_.begin(t_, now);
                reason_ = "non_supply_rush";
                break;
            }
            verdict_ = checkCorridor(
                in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_);
            if (verdict_ != RuleVerdict::OK) {
                if (t_.enable_short_push && clear_attempts_ < 2 && in.clear_push_safe &&
                    in.corridor_complete && in.corridor.unknown == 0 &&
                    (in.corridor.dangerous > 0 || in.corridor.injured > 0 ||
                     in.corridor.core > 0)) {
                    enter(PushState::CUE_PREPARE, now, "closed_push_prepare");
                    break;
                }
                const bool neighbour_verdict =
                    verdict_ == RuleVerdict::TOO_MANY_SUPPLIES ||
                    verdict_ == RuleVerdict::DANGEROUS || verdict_ == RuleVerdict::UNKNOWN_OBJECT ||
                    verdict_ == RuleVerdict::CORE_BEFORE_FIRST ||
                    verdict_ == RuleVerdict::INJURED_BEFORE_FIRST ||
                    verdict_ == RuleVerdict::INJURED_NOT_ALONE || verdict_ == RuleVerdict::OCCLUDED;
                if (t_.mid_neighbour_clear && t_.disperse && sweepLedgerOk && neighbour_verdict &&
                    mid_clear_n_ < t_.mid_neighbour_clear_max && in.heading_valid &&
                    std::isfinite(in.heading_rad)) {
                    const cv::Point2f tp = targetBody();
                    float sum_x = 0;
                    if (in.clutter_ts_us && now >= in.clutter_ts_us &&
                        now - in.clutter_ts_us <= t_.sweep_fresh_us)
                        for (const auto& u : in.clutter_body) {
                            if (!std::isfinite(u.x) || !std::isfinite(u.y))
                                continue;
                            const float r = float(cv::norm(u - tp));
                            if (r < t_.sweep_min_sep_m || r > t_.disperse_radius_m)
                                continue;
                            sum_x += u.x - tp.x;
                        }
                    ++mid_clear_n_;
                    disperse_small_ = true;
                    disperse_sign_ = sum_x > 0 ? -1.f : 1.f;
                    disperse_rush_m_ = std::clamp(target_forward_m - t_.sweep_frame_front_m -
                                                      t_.mid_neighbour_clear_gap_m,
                                                  0.f,
                                                  t_.mid_neighbour_clear_max_rush_m);
                    disperse_swing_.configure(1, t_.mid_neighbour_clear_swings);
                    sweep_phase_ = 4;
                    sweep_start_us_ = sweep_t0_us_ = now;
                    sweep_travel0_ = travel_;
                    confirmations_ = 0;
                    sweep_ledger_.begin(t_, now);
                    reason_ = "mid_neighbour_clear";
                    break;
                }
                blacklist(now);
                target_id_ = capture_target_id_ = -1;
                if (++near_failures_ >= 2) {
                    failStop("mid_open_corridor_rejected");
                } else
                    enter(PushState::MID_REACQUIRE, now, "mid_open_corridor_reselect");
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
                enter(PushState::NEAR_REACQUIRE, now, "near_reacquire_pitch_wait");
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
    const bool scan_ok = c.scan_ok;
    const bool clutter_fresh = c.clutter_fresh;
    const bool sweepOver = c.sweep_over;
    const bool sweepTimeOk = c.sweep_time_ok;
    const bool sweepLedgerOk = c.sweep_ledger_ok;
    const bool sweep_open_disabled_ = c.sweep_open_disabled;
    const int sweep_esc = c.sweep_esc;
    const float target_forward_m = c.target_forward_m;
    const int frame_raised_ = c.frame_raised;
    const bool first_ = c.first_ordinary;
    const int16_t pitch_cmd_ = c.pitch_cmd;
    float& motion_wz = *c.motion_wz;
    auto& trip_ = *c.trip;
    auto& misses_ = *c.misses;
    auto& track_close_ = *c.track_close;
    auto& travel_ = *c.travel;
    auto& target_id_ = *c.target_id;
    auto& label_ = *c.label;
    auto& cue_mode_ = *c.cue_mode;
    auto& capture_target_id_ = *c.capture_target_id;
    auto& phase_us_ = *c.phase_us;
    auto& near_failures_ = *c.near_failures;
    auto& scan_turn_acc_ = *c.scan_turn_acc;
    auto& reason_ = *c.reason;
    auto& sweep_phase_ = *c.sweep_phase;
    auto& sweep_attempts_ = *c.sweep_attempts;
    auto& sweep_seen_ = *c.sweep_seen;
    auto& pile_runs_ = *c.pile_runs;
    auto& pile_seen_ = *c.pile_seen;
    auto& approach_last_id_ = *c.approach_last_id;
    auto& rush_pulse_phase_ = *c.rush_pulse_phase;
    auto& rush_pulse_count_ = *c.rush_pulse_count;
    auto& precheck_seen_ = *c.precheck_seen;
    auto& precheck_rejects_ = *c.precheck_rejects;
    auto& sweep_creep_m_ = *c.sweep_creep_m;
    auto& sweep_travel0_ = *c.sweep_travel0;
    auto& sweep_yaw0_ = *c.sweep_yaw0;
    auto& sweep_sign_ = *c.sweep_sign;
    auto& approach_last_heading_ = *c.approach_last_heading;
    auto& approach_last_distance_ = *c.approach_last_distance;
    auto& approach_last_world_yaw_ = *c.approach_last_world_yaw;
    auto& rush_pulse_yaw0_ = *c.rush_pulse_yaw0;
    auto& rush_pulse_delta_ = *c.rush_pulse_delta;
    auto& prep_ref_m_ = *c.prep_ref_m;
    auto& disperse_sign_ = *c.disperse_sign;
    auto& disperse_rush_m_ = *c.disperse_rush_m;
    auto& scan_mid_limit_ = *c.scan_mid_limit;
    auto& sweep_start_us_ = *c.sweep_start_us;
    auto& sweep_t0_us_ = *c.sweep_t0_us;
    auto& rebind_since_us_ = *c.rebind_since_us;
    auto& approach_last_us_ = *c.approach_last_us;
    auto& rush_pulse_start_us_ = *c.rush_pulse_start_us;
    auto& rush_pulse_t0_us_ = *c.rush_pulse_t0_us;
    auto& pile_cool_until_us_ = *c.pile_cool_until_us;
    auto& prep_last_us_ = *c.prep_last_us;
    auto& prep_ref_us_ = *c.prep_ref_us;
    auto& disperse_cool_until_us_ = *c.disperse_cool_until_us;
    auto& sweep_non_supply_active_ = *c.sweep_non_supply_active;
    auto& post_disperse_mid_ = *c.post_disperse_mid;
    auto& approach_rebind_ = *c.approach_rebind;
    auto& approach_last_world_valid_ = *c.approach_last_world_valid;
    auto& approach_pitch_ = *c.approach_pitch;
    auto& verdict_ = *c.verdict;
    auto& sweep_ledger_ = *c.sweep_ledger;
    auto& disperse_turn_ = *c.disperse_turn;
    auto& disperse_swing_ = *c.disperse_swing;
    const auto candidate = [&](const PushObservation& o, uint64_t at) {
        return s.candidate(o, at);
    };
    const auto targetBody = [&] {
        return s.target_body();
    };
    const auto drive = [&](float vx, float wz) {
        s.drive(vx, wz);
    };
    const auto enter = [&](PushState st, uint64_t at, const char* why) {
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
    const auto sweepFrameReady = [&] {
        return s.sweep_frame_ready();
    };
    const auto disperseStep = [&] {
        return s.disperse_step();
    };
    const auto beginPileFromSweep = [&](const char* why) {
        return s.begin_pile_from_sweep(why);
    };
    const auto beginPile = [&](const char* why) {
        s.begin_pile(why);
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
    const auto uprightInCorridor =
        [&](const PushObservation& o, uint64_t at, float& lat, float* along) {
            return s.upright_in_corridor(o, at, lat, along);
        };
    const auto neighbourInCorridor =
        [&](const PushObservation& o, uint64_t at, float& lat, float* along) {
            return s.neighbour_in_corridor(o, at, lat, along);
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
    const auto motionRatio = [&] {
        return s.motion_ratio();
    };
    do {
        // Keep jaw state from MID_APPROACH; a camera change must not close on unseen cargo.
        if (sweep_phase_) {
            if (t_.sweep_observation_guard && !clutter_fresh && !candidate(in, now) &&
                !(disperse_turn_.active() && sweep_phase_ == 5)) {
                sweep_ledger_.close(t_, now);
                sweep_phase_ = 0;
                scanAfterClear("sweep_observation_lost");
                break;
            }
            // In-place sweep of an on-end block beside the target: turn out by IMU angle, then
            // back. Tracking may drop meanwhile. The base turns at roughly a quarter of the
            // command, so sweep_wz bypasses max_wz and the sweep time is not charged to the
            // approach budget.
            const auto sweepDrive = [&](float wz) {
                drive(0, wz);
                if (motion_wz != 0)
                    motion_wz = std::copysign(t_.sweep_wz, wz);
            };
            const auto sweepEnd = [&](const char* why) {
                sweep_ledger_.close(t_, now);
                phase_us_ += now - sweep_t0_us_;
                sweep_phase_ = 0;
                ++sweep_attempts_;
                sweep_seen_ = 0;
                if (t_.sweep_observation_guard)
                    sweep_ledger_.setGapUntil(now + t_.sweep_gap_us);
                reason_ = why;
            };
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                sweepEnd("sweep_upright_no_heading");
                break;
            }
            if (sweep_phase_ >= 4) {
                const bool cut = sweepOver;
                if (cut || disperseStep()) {
                    const bool swung = sweep_ledger_.close(t_, now);
                    if (sweep_non_supply_active_) { // swept aside, not grabbed: veto it and
                                                    // search again at the close view
                        blacklist(now);
                        clearTrip();
                        scan_mid_limit_ = scan_turn_acc_ + 2.f * float(CV_PI);
                        disperse_cool_until_us_ = now + t_.disperse_cooldown_us;
                        sweep_ledger_.setGapUntil(now + t_.sweep_gap_us);
                        if (!cut && swung && beginPileFromSweep("pile_chain_non_supply"))
                            break;
                        enter(PushState::SCAN, now, "non_supply_swept");
                        break;
                    }
                    phase_us_ += now - sweep_t0_us_;
                    sweep_phase_ = 0;
                    sweep_seen_ = 0;
                    reason_ = cut ? "sweep_budget_exhausted" : "disperse_done";
                    disperse_cool_until_us_ = now + t_.disperse_cooldown_us;
                    sweep_ledger_.setGapUntil(now + t_.sweep_gap_us);
                    s.clear_rejected();
                    s.clear_rejected_bearings(); // the pile has moved: earlier vetoes no longer
                                                 // apply
                    post_disperse_mid_ =
                        t_.post_clear_mid_view; // if the target is lost after the sweep, search
                                                // at 20deg first (cargo is close)
                    if (!cut && swung)
                        beginPileFromSweep("pile_chain_disperse");
                }
                break;
            }
            if (sweep_phase_ < 4 && sweepOver) {
                sweepEnd("sweep_budget_exhausted");
                break;
            }
            if (!sweepFrameReady())
                break;
            if (sweep_phase_ == 3) {
                if (travel_ - sweep_travel0_ >= sweep_creep_m_ * motionRatio() ||
                    now - sweep_start_us_ > t_.sweep_creep_budget_us) {
                    sweep_phase_ = 1;
                    sweep_yaw0_ = in.heading_rad;
                    sweep_start_us_ = now;
                    sweep_ledger_.setSwung(true);
                } else {
                    reason_ = "sweep_creep";
                    drive(t_.sweep_creep_speed, 0);
                    break;
                }
            }
            const float dyaw = wrapAngle(in.heading_rad - sweep_yaw0_);
            const uint64_t se = now - sweep_start_us_;
            if (sweep_phase_ == 1 &&
                (std::abs(dyaw) >= t_.sweep_turn_rad || se > t_.sweep_budget_us)) {
                sweep_phase_ = 2;
                sweep_start_us_ = now;
            }
            if (sweep_phase_ == 1) {
                reason_ = "sweep_upright_out";
                sweepDrive(sweep_sign_ * t_.sweep_wz);
                break;
            }
            if (std::abs(dyaw) <= t_.sweep_return_tol_rad ||
                now - sweep_start_us_ > t_.sweep_budget_us) {
                sweepEnd("sweep_upright_done");
                break;
            }
            reason_ = "sweep_upright_back";
            sweepDrive(dyaw > 0 ? -t_.sweep_wz : t_.sweep_wz);
            break;
        }
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
                        cue_mode_ = false;
                        enter(PushState::NEAR_REACQUIRE, now, "near_target_lost_reacquire");
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
        // Many blocks in the corridor ahead of the frame mouth: carry the pile away and split
        // the last layer instead of poking at it.
        if (t_.pile_clear && trip_.total() == 0 && frame_raised_ == 0 &&
            now >= pile_cool_until_us_ && pile_runs_ < t_.pile_max_runs && scan_ok &&
            in.heading_valid && std::isfinite(in.heading_rad) && in.corridor_complete &&
            in.corridor.total() >= t_.pile_corridor_min && in.distance_m <= t_.pile_trigger_max_m) {
            if (++pile_seen_ < t_.pile_confirm) {
                reason_ = "pile_confirm";
                break;
            }
            pile_seen_ = 0;
            beginPile("pile_many_in_frame");
            break;
        }
        pile_seen_ = 0;
        if (in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) <= t_.rush_heading_rad) {
                const bool non_supply = t_.sweep_non_supply && t_.disperse &&
                                        targetKind(label_) != TargetKind::ORDINARY;
                if (t_.disperse &&
                    (non_supply ? sweepTimeOk
                                : (now >= disperse_cool_until_us_ && sweepLedgerOk)) &&
                    in.heading_valid && std::isfinite(in.heading_rad) &&
                    std::isfinite(in.distance_m)) {
                    // Count everything else around the target (any class); a dense pile is
                    // scattered, not picked at.
                    const bool use_clutter = in.clutter_ts_us && now >= in.clutter_ts_us &&
                                             now - in.clutter_ts_us <= t_.sweep_fresh_us;
                    const auto& pts = use_clutter ? in.clutter_body : in.upright_body;
                    const uint64_t pts_ts = use_clutter ? in.clutter_ts_us : in.upright_ts_us;
                    const cv::Point2f tp = targetBody();
                    int near_n = 0;
                    float sum_x = 0;
                    if (pts_ts && now >= pts_ts && now - pts_ts <= t_.sweep_fresh_us)
                        for (const auto& u : pts) {
                            if (!std::isfinite(u.x) || !std::isfinite(u.y))
                                continue;
                            const float r = float(cv::norm(u - tp));
                            if (r < t_.sweep_min_sep_m || r > t_.disperse_radius_m)
                                continue;
                            ++near_n;
                            sum_x += u.x - tp.x;
                        }
                    if (non_supply || near_n >= t_.disperse_dense_min + sweep_esc) {
                        if (++sweep_seen_ < t_.sweep_confirm) {
                            reason_ = non_supply ? "non_supply_sweep_confirm" : "disperse_confirm";
                            break;
                        }
                        sweep_seen_ = 0;
                        sweep_non_supply_active_ = non_supply;
                        disperse_sign_ =
                            sum_x > 0 ? -1.f : 1.f; // first swing toward the heavier side
                        disperse_rush_m_ = std::clamp(target_forward_m - t_.sweep_frame_front_m +
                                                          t_.disperse_depth_m,
                                                      0.f,
                                                      t_.disperse_max_m);
                        disperse_swing_.configure(t_.disperse_passes, t_.disperse_swings);
                        sweep_phase_ = 4;
                        sweep_start_us_ = sweep_t0_us_ = now;
                        sweep_travel0_ = travel_;
                        sweep_ledger_.begin(t_, now);
                        reason_ = "disperse_rush";
                        break;
                    }
                }
                float lat = 0, nearest_y = 0;
                const bool upright_hit =
                    t_.sweep_upright && uprightInCorridor(in, now, lat, &nearest_y);
                const bool neighbour_hit =
                    !upright_hit && neighbourInCorridor(in, now, lat, &nearest_y);
                if ((upright_hit || neighbour_hit) && sweepLedgerOk &&
                    sweep_attempts_ < t_.sweep_max_attempts && in.heading_valid &&
                    std::isfinite(in.heading_rad)) {
                    if (++sweep_seen_ < t_.sweep_confirm) {
                        reason_ =
                            neighbour_hit ? "sweep_neighbour_confirm" : "sweep_upright_confirm";
                        break;
                    }
                    sweep_yaw0_ = in.heading_rad;
                    sweep_sign_ = lat > 0 ? -1.f : 1.f;
                    sweep_start_us_ = sweep_t0_us_ = now;
                    sweep_seen_ = 0;
                    sweep_ledger_.begin(t_, now);
                    sweep_creep_m_ = t_.sweep_creep
                                         ? std::clamp(nearest_y - (t_.sweep_frame_front_m +
                                                                   t_.sweep_creep_gap_m),
                                                      0.f,
                                                      t_.sweep_creep_max_m)
                                         : 0.f;
                    if (sweep_creep_m_ >= .02f) {
                        sweep_phase_ = 3;
                        sweep_travel0_ = travel_;
                        reason_ = "sweep_creep";
                        break;
                    }
                    sweep_phase_ = 1;
                    sweep_ledger_.setSwung(true);
                    reason_ = neighbour_hit ? "sweep_neighbour_out" : "sweep_upright_out";
                    if ((t_.sweep_frame_open || t_.frame_open_default) && !sweep_open_disabled_)
                        break; // open the frame first; the next tick turns
                    drive(0, sweep_sign_ * t_.sweep_wz);
                    if (motion_wz != 0)
                        motion_wz = sweep_sign_ * t_.sweep_wz;
                    break;
                }
                if (sweep_seen_ > 0)
                    --sweep_seen_;
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
                        enter(PushState::SCAN, now, why.c_str());
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
                        enter(PushState::PREPARE, now, "approach_range_stalled");
                        commandFrame(1, now);
                        break;
                    }
                    reason_ = "approach_to_prepare_range";
                    steer(t_.approach_speed, in.heading_error);
                    break;
                }
                prep_last_us_ = 0;
                enter(PushState::PREPARE, now, "rush_aligned");
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
