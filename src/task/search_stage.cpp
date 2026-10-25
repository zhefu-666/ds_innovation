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
} // namespace rescue
