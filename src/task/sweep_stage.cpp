#include "rescue/task/sweep_stage.hpp"

#include "rescue/task/push_task.hpp"

#include <algorithm>

#include <cmath>

#include <cstring>

#include <iostream>

namespace rescue {

void SweepStage::update(SearchAction state,
                        const TaskTuning& t,
                        const PushObservation& in,
                        const SweepContext& c,
                        const SweepServices& s) {
    switch (state) {
        case SearchAction::Scatter:
            scatter(t, in, c, s);
            break;
        case SearchAction::Pile:
            pile(t, in, c, s);
            break;
        case SearchAction::CueApproach:
            cueApproach(t, in, c, s);
            break;
        case SearchAction::CuePrepare:
            cuePrepare(t, in, c, s);
            break;
        case SearchAction::ClearPile:
            clearPile(t, in, c, s);
            break;
        case SearchAction::ClearPause:
            clearPause(t, in, c, s);
            break;
        default:
            break;
    }
}

void SweepStage::scatter(const TaskTuning& t,
                         const PushObservation& in,
                         const SweepContext& c,
                         const SweepServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    float& motion_vx = c.motion.vx_mps;
    float& motion_wz = c.motion.wz_rps;
    const bool clutter_fresh = c.clutter_fresh;
    const bool scatter_ok = c.scatter_ok;
    const bool sweepOver = c.sweep_over;
    const bool sweep_op_timeout = c.sweep_operation_timeout;
    auto& approach_forward_ = c.trip_data.approach_forward_;
    auto& burst_fast_ = c.search.burst_fast_;
    auto& burst_swing_ = c.search.burst_swing_;
    auto& burst_swing_armed_ = c.search.burst_swing_armed_;
    auto& disperse_rush_m_ = c.search.disperse_rush_m_;
    auto& reason_ = c.reason;
    auto& scan_mid_limit_ = c.search.scan_mid_limit_;
    auto& scan_turn_acc_ = c.search.scan_turn_acc_;
    auto& scatter_approach_m_ = c.search.scatter_approach_m_;
    auto& scatter_approach_n_ = c.search.scatter_approach_n_;
    auto& scatter_commit_ = c.search.scatter_commit_;
    auto& scatter_cool_until_us_ = c.search.scatter_cool_until_us_;
    auto& scatter_far_ = c.search.scatter_far_;
    auto& scatter_idle_since_us_ = c.search.scatter_idle_since_us_;
    auto& scatter_range_ = c.search.scatter_range_;
    auto& scatter_seen_ = c.search.scatter_seen_;
    auto& scatter_yaw_ = c.search.scatter_yaw_;
    auto& sweep_phase_ = c.search.sweep_phase_;
    auto& sweep_pitch_ = c.search.sweep_pitch_;
    auto& sweep_start_us_ = c.search.sweep_start_us_;
    auto& sweep_t0_us_ = c.search.sweep_t0_us_;
    auto& sweep_travel0_ = c.search.sweep_travel0_;
    auto& travel_ = c.progress.travel;
    auto& sweep_ledger_ = c.ledger;
    auto& disperse_turn_ = c.search.disperse_turn_;
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto drive = [&](float vx, float wz) {
        s.drive(vx, wz);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto candidate = [&](const PushObservation& o, uint64_t at) {
        return s.candidate(o, at);
    };
    const auto weakCandidate = [&](const PushObservation& o) {
        return s.weak_candidate(o);
    };
    const auto targetBody = [&] {
        return s.target_body();
    };
    const auto findBest = [&](const std::vector<cv::Point2f>& pts, float max_y, float radius) {
        return s.find_best(pts, max_y, radius);
    };
    const auto beginScatter = [&](bool far, bool commit, const char* why) {
        s.begin_scatter(far, commit, why);
    };
    const auto disperseStep = [&] {
        return s.disperse_step();
    };
    const auto beginPileFromSweep = [&](const char* why) {
        return s.begin_pile_from_sweep(why);
    };
    const auto midPitch = [&] {
        return s.mid_pitch();
    };
    const auto motionRatio = [&] {
        return s.motion_ratio();
    };
    do {
        commandFrame(0, now);
        approach_forward_ = 0;
        const auto scatterEnd = [&](const char* why) {
            burst_fast_ = burst_swing_ = burst_swing_armed_ = false;
            scatter_cool_until_us_ = now + t_.blind_scatter_cooldown_us;
            const bool swung = sweep_ledger_.close(t_, now);
            if (swung || std::strcmp(why, "blind_scatter_target_seen") != 0)
                sweep_ledger_.setGapUntil(
                    now + t_.sweep_gap_us); // a target seen before any swing leaves the
                                            // dense-pile check open for APPROACH
            scan_mid_limit_ =
                scan_turn_acc_ +
                2.f * float(CV_PI); // first full turn after the scatter looks at the close range
            s.clear_rejected();
            s.clear_rejected_bearings(); // the group has moved: earlier vetoes no longer apply
            if (swung && std::strcmp(why, "blind_scatter_done") == 0 &&
                beginPileFromSweep("pile_chain_scatter"))
                return;
            enter(SearchAction::Scan, now, why);
        };
        const auto approachEnd =
            [&](const char* why) { // no sweep happened: keep the scatter budget, retrigger from
                                   // the short range at once
                scatter_far_ = false;
                ++scatter_approach_n_;
                burst_fast_ = false;
                sweep_ledger_.pause(now);
                enter(SearchAction::Scan, now, why);
                scatter_idle_since_us_ =
                    now > t_.blind_scatter_wait_us ? now - t_.blind_scatter_wait_us : 1;
            };
        const auto sightedTarget = [&] { // driving to a far group: a lock does not stop the drive
                                         // until it is inside the short window (the group sweep
                                         // itself checks first); a committed sweep ignores locks
            return !scatter_commit_ && candidate(in, now) && !weakCandidate(in) &&
                   !(scatter_far_ && in.distance_m > t_.blind_scatter_max_y_m);
        };
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
            scatter_far_ = false;
            scatterEnd("blind_scatter_no_heading");
            break;
        }
        if (elapsed > t_.blind_scatter_budget_us) {
            scatter_far_ = false;
            scatterEnd("blind_scatter_timeout");
            break;
        }
        if (sweepOver) {
            scatter_far_ = false;
            scatterEnd(sweep_op_timeout ? "sweep_op_timeout" : "sweep_budget_exhausted");
            break;
        }
        if (t_.sweep_view_handoff && midPitch() &&
            sweep_phase_ != 6) { // carry's camera ladder toward the group: down only, the base
                                 // waits while it tilts
            const float left = scatter_range_ - travel_ / motionRatio();
            if (t_.sweep_view_cap_mid) {
                if (sweep_phase_ < 4 && (!scatter_far_ || left <= t_.track_near_m) &&
                    sweep_pitch_ < t_.intermediate_pitch_cdeg)
                    sweep_pitch_ = t_.intermediate_pitch_cdeg;
            } else if (left <= t_.intermediate_to_near_m)
                sweep_pitch_ = t_.track_pitch_cdeg;
            else if (left <= t_.track_near_m && sweep_pitch_ == t_.far_pitch_cdeg)
                sweep_pitch_ = t_.intermediate_pitch_cdeg;
        }
        if (sweep_phase_ == 6) { // back out the distance driven in
            if (sightedTarget()) {
                scatterEnd("blind_scatter_target_seen");
                break;
            }
            if (travel_ - sweep_travel0_ >= scatter_back_ ||
                now - sweep_start_us_ > t_.blind_scatter_back_budget_us) {
                scatterEnd("blind_scatter_done");
                break;
            }
            if (!in.retreat_safe) {
                scatterEnd("blind_scatter_back_unsafe");
                break;
            } // the rear is blind: no reversing without the adapter's say-so, the sweep ends
              // where it is
            reason_ = "blind_scatter_back";
            drive(-t_.blind_scatter_back_speed, 0);
            if (motion_vx < 0)
                motion_vx = -t_.blind_scatter_back_speed;
            break;
        }
        if (t_.sweep_observation_guard && !clutter_fresh && !candidate(in, now) &&
            !(t_.sweep_far_judge && sweep_phase_ >= 4) &&
            !(disperse_turn_.active() && sweep_phase_ == 5)) {
            scatterEnd("blind_scatter_observation_lost");
            break;
        }
        if (sweep_phase_ == 0) {
            if (sightedTarget()) {
                scatter_far_ = false;
                scatterEnd("blind_scatter_target_seen");
                break;
            }
            if (elapsed > t_.blind_scatter_align_budget_us) {
                scatter_far_ = false;
                scatterEnd("blind_scatter_align_timeout");
                break;
            }
            const float err =
                wrapAngle(scatter_yaw_ - in.heading_rad); // body yaw grows on left turns
            if (std::abs(err) > t_.blind_scatter_align_tol_rad) {
                reason_ = "blind_scatter_align";
                drive(0, err);
                if (motion_wz != 0)
                    motion_wz = std::copysign(std::abs(err) > .3f ? t_.blind_scatter_align_wz
                                                                  : .5f * t_.blind_scatter_align_wz,
                                              err);
                break;
            }
            if (scatter_far_) {
                sweep_phase_ = 7;
                sweep_start_us_ = now;
                sweep_travel0_ = travel_;
            } else {
                disperse_rush_m_ =
                    std::clamp(scatter_range_ - t_.sweep_frame_front_m + t_.blind_scatter_depth_m,
                               0.f,
                               t_.blind_scatter_max_m);
                sweep_phase_ = 4;
                sweep_start_us_ = sweep_t0_us_ = now;
                sweep_travel0_ = travel_;
            }
        }
        if (sweep_phase_ == 7) { // drive toward the far group at normal speed with IMU yaw hold
            const bool ordinary_lock =
                t_.sweep_confirm_wait_us > 0 && sightedTarget() &&
                targetKind(in.label) == TargetKind::ORDINARY &&
                (!burst_swing_armed_ ||
                 (t_.sweep_observation_guard &&
                  !t_.first_burst_full_impact)); // first opening pile keeps its impact; a lone
                                                 // target still exits through sightedTarget
                                                 // below (SCAN, then APPROACH decides on the
                                                 // pile)
            if (scatter_ok && clutter_fresh &&
                !ordinary_lock) { // the group reached the short window: sweep it now, whatever
                                  // single lock is in view
                std::vector<cv::Point2f> pts = in.clutter_body;
                if (candidate(in, now) && std::isfinite(in.distance_m)) {
                    pts.emplace_back(targetBody());
                    s.add_ordinary_extra(pts.back());
                }
                const SweepBest best =
                    findBest(pts, t_.blind_scatter_max_y_m, t_.disperse_radius_m);
                if (best.n >= t_.blind_scatter_min) {
                    if (++scatter_seen_ < t_.sweep_confirm) {
                        reason_ = "blind_scatter_near_confirm";
                        break;
                    }
                    scatter_seen_ = 0;
                    beginScatter(false, true, "blind_scatter_near_group");
                    break;
                }
            }
            scatter_seen_ = 0;
            if (sightedTarget()) {
                scatter_far_ = false;
                scatterEnd("blind_scatter_target_seen");
                break;
            }
            if (travel_ - sweep_travel0_ >= scatter_approach_m_ * motionRatio()) {
                approachEnd("blind_scatter_approached");
                break;
            }
            if (now - sweep_start_us_ > t_.blind_scatter_approach_budget_us) {
                approachEnd("blind_scatter_approach_timeout");
                break;
            }
            const float err = wrapAngle(scatter_yaw_ - in.heading_rad);
            const float hold_wz =
                std::abs(err) > t_.startup_hold_deadband_rad ? t_.startup_hold_gain * err : 0.f;
            const bool fast =
                burst_fast_ && (t_.first_burst_full_impact ||
                                scatter_approach_m_ - (travel_ - sweep_travel0_) / motionRatio() >
                                    t_.first_burst_slow_m);
            reason_ = fast ? "first_burst_approach" : "blind_scatter_approach";
            drive(fast ? t_.first_burst_speed : t_.approach_speed, hold_wz);
            if (fast && motion_vx > 0)
                motion_vx = t_.first_burst_speed; // drive() caps at max_speed; the chassis
                                                  // packs up to 1.0 m/s
            break;
        }
        if (disperseStep()) {
            if (t_.blind_scatter_return && travel_ > .05f * motionRatio()) {
                scatter_back_ = travel_ * t_.blind_scatter_back_frac;
                sweep_phase_ = 6;
                sweep_start_us_ = now;
                sweep_travel0_ = travel_;
            } else
                scatterEnd("blind_scatter_done");
        }
        break;
    } while (false);
}

void SweepStage::pile(const TaskTuning& t,
                      const PushObservation& in,
                      const SweepContext& c,
                      const SweepServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    float& motion_vx = c.motion.vx_mps;
    float& motion_wz = c.motion.wz_rps;
    const bool clutter_fresh = c.clutter_fresh;
    auto& approach_forward_ = c.trip_data.approach_forward_;
    auto& burst_swing_ = c.search.burst_swing_;
    auto& confirmations_ = c.progress.confirmations;
    auto& cue_mode_ = c.search.cue_mode_;
    auto& disperse_cool_until_us_ = c.search.disperse_cool_until_us_;
    auto& disperse_rush_m_ = c.search.disperse_rush_m_;
    auto& disperse_small_ = c.search.disperse_small_;
    auto& pile_chain_n_ = c.search.pile_chain_n_;
    auto& pile_cool_until_us_ = c.search.pile_cool_until_us_;
    auto& pile_phase_ = c.search.pile_phase_;
    auto& pile_phase_us_ = c.search.pile_phase_us_;
    auto& pile_push0_ = c.search.pile_push0_;
    auto& pile_runs_ = c.search.pile_runs_;
    auto& pile_seen_ = c.search.pile_seen_;
    auto& pile_yaw0_ = c.search.pile_yaw0_;
    auto& reason_ = c.reason;
    auto& scan_mid_limit_ = c.search.scan_mid_limit_;
    auto& scan_turn_acc_ = c.search.scan_turn_acc_;
    auto& scatter_cool_until_us_ = c.search.scatter_cool_until_us_;
    auto& sweep_phase_ = c.search.sweep_phase_;
    auto& sweep_start_us_ = c.search.sweep_start_us_;
    auto& sweep_travel0_ = c.search.sweep_travel0_;
    auto& sweep_yaw0_ = c.search.sweep_yaw0_;
    auto& target_id_ = c.trip_data.target_id_;
    auto& travel_ = c.progress.travel;
    auto& sweep_ledger_ = c.ledger;
    auto& disperse_rush_ = c.search.disperse_rush_;
    auto& disperse_turn_ = c.search.disperse_turn_;
    auto& disperse_swing_ = c.search.disperse_swing_;
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto drive = [&](float vx, float wz) {
        s.drive(vx, wz);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto targetBody = [&] {
        return s.target_body();
    };
    const auto sweepFrameReady = [&] {
        return s.sweep_frame_ready();
    };
    const auto disperseStep = [&] {
        return s.disperse_step();
    };
    const auto motionRatio = [&] {
        return s.motion_ratio();
    };
    do {
        commandFrame(0, now);
        approach_forward_ = 0;
        const auto pileEnd = [&](const char* why) {
            const uint64_t until = now + t_.pile_cool_us;
            pile_cool_until_us_ = until;
            scatter_cool_until_us_ = std::max(scatter_cool_until_us_, until);
            disperse_cool_until_us_ = std::max(disperse_cool_until_us_, until);
            sweep_ledger_.extendGapUntil(until);
            target_id_ = -1;
            confirmations_ = 0;
            cue_mode_ = false;
            sweep_phase_ = 0;
            disperse_small_ = false;
            s.clear_rejected();
            s.clear_rejected_bearings();
            scan_mid_limit_ = scan_turn_acc_ + 2.f * float(CV_PI);
            std::cout << "[PILE] end why=" << why << " runs=" << pile_runs_ << "\n";
            enter(SearchAction::Scan, now, why);
        };
        const auto setPhase = [&](int ph) {
            pile_phase_ = ph;
            pile_phase_us_ = now;
            pile_seen_ = 0;
            pile_travel0_ = travel_;
            pile_push0_ = -1.f;
        };
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
            pileEnd("pile_no_heading");
            break;
        }
        if (elapsed > t_.pile_budget_us) {
            pileEnd("pile_timeout");
            break;
        }
        const float yaw = in.heading_rad;
        struct Blk {
            cv::Point2f p;
            bool ordinary = false;
        };
        // Visible blocks in body coordinates; the locked target can be missing from the clutter
        // list.
        const auto pileBlocks = [&] {
            std::vector<Blk> v;
            if (clutter_fresh)
                for (const auto& u : in.clutter_body) {
                    if (!std::isfinite(u.x) || !std::isfinite(u.y))
                        continue;
                    Blk b;
                    b.p = u;
                    for (const auto& o : in.clutter_ordinary)
                        if (std::abs(o.x - u.x) < 1e-4f && std::abs(o.y - u.y) < 1e-4f) {
                            b.ordinary = true;
                            break;
                        }
                    v.push_back(b);
                }
            if (in.target_valid && in.geometry_valid && !in.target_in_zone &&
                std::isfinite(in.distance_m) && std::isfinite(in.heading_error)) {
                const cv::Point2f tp = targetBody();
                const bool tord = targetKind(in.label) == TargetKind::ORDINARY;
                bool dup = false;
                for (const auto& u : v)
                    if (u.ordinary == tord && cv::norm(u.p - tp) < .03) {
                        dup = true;
                        break;
                    } // a touching block of another type is a different block
                if (!dup) {
                    Blk b;
                    b.p = tp;
                    b.ordinary = tord;
                    v.push_back(b);
                }
            }
            return v;
        };
        // The cluster straight ahead: the block nearest the heading and everything within
        // pile_cluster_radius_m of it.
        const auto pileCluster = [&] {
            std::vector<Blk> c;
            const auto v = pileBlocks();
            int seed = -1;
            float bb = 1e9f;
            for (size_t i = 0; i < v.size(); ++i) {
                if (v[i].p.y < .08f || v[i].p.y > 1.2f ||
                    std::abs(v[i].p.x) > t_.pile_cluster_half_x_m)
                    continue;
                const float b = std::abs(std::atan2(-v[i].p.x, v[i].p.y));
                if (b < bb) {
                    bb = b;
                    seed = int(i);
                }
            }
            if (seed < 0)
                return c;
            for (const auto& u : v)
                if (std::abs(u.p.x) <= t_.pile_cluster_half_x_m &&
                    cv::norm(u.p - v[seed].p) <= t_.pile_cluster_radius_m)
                    c.push_back(u);
            return c;
        };
        const auto faceYaw =
            [&](float target_yaw,
                float tol,
                const char* why) { // in-place turn to a world yaw; true when aligned
                const float err = wrapAngle(target_yaw - yaw);
                if (std::abs(err) <= tol)
                    return true;
                reason_ = why;
                const float wz = std::copysign(
                    std::abs(err) > .5f ? t_.pile_turn_wz : .5f * t_.pile_turn_wz, err);
                drive(0, wz);
                if (motion_wz != 0)
                    motion_wz = wz;
                return false;
            };
        const auto backTo = [&](float metres,
                                uint64_t budget) { // reverse a measured distance; true when done
            if (!in.retreat_safe) {
                reason_ = "pile_back_unsafe";
                return true;
            }
            if (travel_ - pile_travel0_ >= metres * motionRatio() || now - pile_phase_us_ > budget)
                return true;
            drive(-t_.blind_scatter_back_speed, 0);
            if (motion_vx < 0)
                motion_vx = -t_.blind_scatter_back_speed;
            return false;
        };
        switch (pile_phase_) {
            case 0: { // frame open (the rest of the flow carries it open), stand still for a
                      // moment
                if (!sweepFrameReady())
                    break;
                if (now - pile_phase_us_ < 400000) {
                    reason_ = "pile_settle";
                    break;
                }
                std::cout << "[PILE] start yaw0=" << pile_yaw0_
                          << " corridor=" << in.corridor.total() << " distance=" << in.distance_m
                          << "\n";
                setPhase(1);
                break;
            }
            case 1: { // face the pile
                if (now - pile_phase_us_ > t_.pile_advance_budget_us) {
                    pileEnd("pile_face_pile_timeout");
                    break;
                }
                if (faceYaw(pile_yaw0_, .6f * t_.pile_align_tol_rad, "pile_face_pile"))
                    setPhase(2);
                break;
            }
            case 2: { // straight into the pile with the frame open: pile_advance_m at
                      // pile_advance_speed
                if ((travel_ - pile_travel0_) >= t_.pile_advance_m * motionRatio() ||
                    now - pile_phase_us_ > t_.pile_advance_budget_us) {
                    setPhase(3);
                    break;
                }
                const float err = wrapAngle(pile_yaw0_ - yaw);
                reason_ = "pile_advance";
                drive(t_.pile_advance_speed, std::abs(err) > .04f ? t_.heading_gain * err : 0.f);
                break;
            }
            case 3: { // short back-off so the camera sees the remaining cluster
                if (backTo(t_.pile_view_back_m, t_.pile_back_budget_us)) {
                    setPhase(4);
                    break;
                }
                reason_ = "pile_view_back";
                break;
            }
            case 4: { // measure the cluster, turn by one block's angular size to the better
                      // side and cut in
                if (now - pile_phase_us_ < t_.pile_measure_us) {
                    reason_ = "pile_measure";
                    break;
                }
                const auto c = pileCluster();
                if (c.size() < 2) {
                    if (++pile_seen_ < 8) {
                        reason_ = "pile_measure_wait";
                        break;
                    }
                    pileEnd("pile_cluster_clear");
                    break;
                }
                float cx = 0, cy = 0, ymin = 1e9f;
                for (const auto& b : c) {
                    cx += b.p.x / float(c.size());
                    cy += b.p.y / float(c.size());
                    ymin = std::min(ymin, b.p.y);
                }
                const float range = std::max(.10f, float(std::hypot(cx, cy)));
                const float theta =
                    2.f * std::atan(t_.pile_block_half_m /
                                    range); // one block as the camera sees it at this range
                struct Eval {
                    int ord_in = 0, other_in = 0, ord_shed = 0, other_shed = 0;
                    float along_min = 1e9f;
                    float need = 0, cost = 0;
                };
                const auto eval = [&](float beta) { // frame edge line through the origin at
                                                    // body bearing beta (left positive)
                    Eval e;
                    const float un = -std::sin(beta), uy = std::cos(beta), nx = std::cos(beta),
                                ny = std::sin(beta);
                    for (const auto& b : c) {
                        const float along = b.p.x * un + b.p.y * uy, lat = b.p.x * nx + b.p.y * ny;
                        const bool in_frame = along > 0 && std::abs(lat) <= t_.pile_frame_half_m;
                        if (in_frame) {
                            (b.ordinary ? e.ord_in : e.other_in)++;
                            e.along_min = std::min(e.along_min, along);
                        } else
                            (b.ordinary ? e.ord_shed : e.other_shed)++;
                    }
                    // Keep the ordinary supplies in the frame, drop what the corridor rules
                    // would reject, and no more than kMaxSuppliesPerTrip supplies enclosed.
                    e.need = 100.f * float(e.other_in) +
                             20.f * float(std::max(0, e.ord_in - kMaxSuppliesPerTrip));
                    e.cost = e.need + 3.f * float(e.ord_shed);
                    return e;
                };
                const Eval e0 = eval(0.f), el = eval(theta), er = eval(-theta);
                std::cout << "[PILE] measure n=" << c.size() << " range=" << range
                          << " block_deg=" << theta * 57.2958f << " straight(cost=" << e0.cost
                          << " in=" << e0.ord_in << "+" << e0.other_in << ") left(cost=" << el.cost
                          << " in=" << el.ord_in << "+" << el.other_in << " shed=" << el.ord_shed
                          << "+" << el.other_shed << ") right(cost=" << er.cost
                          << " in=" << er.ord_in << "+" << er.other_in << " shed=" << er.ord_shed
                          << "+" << er.other_shed << ")\n";
                // Same-type first: a mixed frame is the dominant cost (other_in), so widen the
                // cut one block at a time until the enclosed set is clean; ties keep the
                // smaller turn, then the left side.
                bool left = el.cost <= er.cost;
                int best_k = 1;
                Eval best = left ? el : er;
                for (int k = 2; k <= t_.pile_cut_max_steps && best.other_in > 0; ++k) {
                    const Eval wl = eval(k * theta), wr = eval(-k * theta);
                    for (const bool wleft :
                         {true, false}) { // a wide turn can sweep the frame clear of every block:
                                          // only cuts that still enclose an ordinary supply count
                        const Eval& wb = wleft ? wl : wr;
                        if (wb.ord_in > 0 && wb.cost < best.cost) {
                            best = wb;
                            left = wleft;
                            best_k = k;
                        }
                    }
                }
                if (e0.need <= 0) {
                    pileEnd("pile_frame_already_ok");
                    break;
                } // nothing the corridor rules would reject is enclosed any more
                if (!(best.need < e0.need)) {
                    pileEnd("pile_cut_no_gain");
                    break;
                }
                pile_cut_yaw_ = wrapAngle(yaw + (left ? 1.f : -1.f) * float(best_k) * theta);
                pile_cut_m_ = std::clamp(
                    best.along_min - t_.sweep_frame_front_m + t_.pile_cut_depth_m, 0.f, .5f);
                std::cout << "[PILE] cut side=" << (left ? "left" : "right") << " steps=" << best_k
                          << " turn_deg=" << best_k * theta * 57.2958f << " in=" << best.ord_in
                          << "+" << best.other_in << " travel=" << pile_cut_m_ << "\n";
                setPhase(5);
                break;
            }
            case 5: { // turn, then drive the edge in
                if (now - pile_phase_us_ > t_.pile_cut_budget_us) {
                    pile_cut_m_ = pile_push0_ >= 0 ? (travel_ - pile_push0_) / motionRatio() : 0.f;
                    setPhase(6);
                    break;
                }
                if (pile_push0_ >= 0) {
                    if (travel_ - pile_push0_ >= pile_cut_m_ * motionRatio()) {
                        setPhase(6);
                        break;
                    }
                    const float err = wrapAngle(pile_cut_yaw_ - yaw);
                    reason_ = "pile_cut_drive";
                    drive(t_.pile_cut_speed, std::abs(err) > .04f ? t_.heading_gain * err : 0.f);
                    break;
                }
                if (faceYaw(pile_cut_yaw_, .1f, "pile_cut_align"))
                    pile_push0_ = travel_;
                break;
            }
            case 6: { // back out to the viewing position; SCAN picks the target from what is
                      // left
                if (backTo(pile_cut_m_, t_.pile_back_budget_us)) {
                    pileEnd("pile_split_done");
                    break;
                }
                reason_ = "pile_cut_back";
                break;
            }
            case 7: { // chain: is the pushed-out cluster still more than one layer deep?
                if (!sweepFrameReady())
                    break;
                if (now - pile_phase_us_ < t_.pile_measure_us) {
                    reason_ = "pile_chain_measure";
                    break;
                }
                const auto c = pileCluster();
                if (c.size() < 2) {
                    if (++pile_seen_ < 8) {
                        reason_ = "pile_chain_wait";
                        break;
                    }
                    if (pile_chain_n_ > 0 &&
                        pile_chain_look_ <
                            2) { // the turn took the blocks out of view: look at the middle of
                                 // the turn, then at the heading it rushed in on
                        pile_yaw0_ =
                            pile_chain_look_ == 0 ? pile_chain_yaw_mid_ : pile_chain_yaw_pre_;
                        ++pile_chain_look_;
                        std::cout << "[PILE] chain relook " << pile_chain_look_
                                  << " yaw=" << pile_yaw0_ << "\n";
                        setPhase(11);
                        break;
                    }
                    pileEnd("pile_chain_cluster_clear");
                    break;
                }
                pile_chain_look_ = 0;
                float cx = 0, cy = 0, ymin = 1e9f, ymax = -1e9f;
                for (const auto& b : c) {
                    cx += b.p.x / float(c.size());
                    cy += b.p.y / float(c.size());
                    ymin = std::min(ymin, b.p.y);
                    ymax = std::max(ymax, b.p.y);
                }
                const float depth = ymax - ymin;
                const bool one_layer = depth <= t_.pile_layer_depth_m;
                std::cout << "[PILE] chain n=" << c.size() << " depth=" << depth
                          << " one_layer=" << one_layer << " round=" << pile_chain_n_ << "/"
                          << t_.pile_chain_max << "\n";
                if (one_layer || pile_chain_n_ >= t_.pile_chain_max) {
                    setPhase(4);
                    break;
                }
                ++pile_chain_n_;
                pile_yaw0_ = wrapAngle(yaw + std::atan2(-cx, cy));
                pile_chain_yaw_pre_ = pile_yaw0_;
                setPhase(8);
                break;
            }
            case 8: { // chain: face the cluster, then arm the disperse step (rush in +
                      // big-angle turn)
                if (now - pile_phase_us_ > t_.pile_advance_budget_us) {
                    pileEnd("pile_chain_face_timeout");
                    break;
                }
                if (!faceYaw(pile_yaw0_, .6f * t_.pile_align_tol_rad, "pile_chain_face"))
                    break;
                sweep_phase_ = 4;
                sweep_start_us_ = now;
                sweep_travel0_ = travel_;
                disperse_rush_m_ = t_.pile_chain_rush_m;
                disperse_small_ = false;
                disperse_rush_.clearFlags();
                burst_swing_ = false;
                disperse_swing_.beginSwing();
                disperse_turn_.clearConfirmation();
                disperse_swing_.restartPass();
                setPhase(9);
                break;
            }
            case 9: { // chain: one disperse step
                if (disperseStep()) {
                    sweep_phase_ = 0;
                    sweep_ledger_.setSwung(false);
                    pile_chain_yaw_mid_ =
                        wrapAngle(sweep_yaw0_ + .5f * wrapAngle(yaw - sweep_yaw0_));
                    pile_chain_look_ = 0;
                    pile_chain_in_m_ = std::max(0.f, travel_ - pile_travel0_) / motionRatio();
                    setPhase(10);
                }
                break;
            }
            case 10: { // chain: back off part of the way in so the camera sees what is left
                if (backTo(std::max(pile_chain_in_m_ * t_.pile_chain_back_frac,
                                    t_.pile_chain_min_back_m),
                           t_.pile_back_budget_us)) {
                    pile_yaw0_ = pile_chain_yaw_mid_;
                    setPhase(11);
                    break;
                }
                reason_ = "pile_chain_back";
                break;
            }
            case 11: { // chain: turn to where the pushed-out blocks went, then measure
                if (now - pile_phase_us_ > t_.pile_advance_budget_us) {
                    pileEnd("pile_chain_look_timeout");
                    break;
                }
                if (faceYaw(pile_yaw0_, .6f * t_.pile_align_tol_rad, "pile_chain_look"))
                    setPhase(7);
                break;
            }
            default:
                pileEnd("pile_bad_phase");
                break;
        }
        break;
    } while (false);
}

void SweepStage::cueApproach(const TaskTuning& t,
                             const PushObservation& in,
                             const SweepContext& c,
                             const SweepServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    auto& capture_target_id_ = c.trip_data.capture_target_id_;
    auto& cue_mode_ = c.search.cue_mode_;
    auto& label_ = c.trip_data.label_;
    auto& misses_ = c.progress.misses;
    auto& reason_ = c.reason;
    auto& search_cue_id_ = c.search.search_cue_id_;
    auto& target_id_ = c.trip_data.target_id_;
    auto& track_close_ = c.progress.track_close;
    const bool first_ = c.first_ordinary;
    const int16_t pitch_cmd_ = c.pitch_cmd;
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto scanAfterClear = [&](const char* why) {
        s.scan_after_clear(why);
    };
    const auto blacklist = [&](uint64_t at) {
        s.blacklist(at);
    };
    const auto cueCandidate = [&] {
        return s.cue_candidate();
    };
    const auto searchLost = [&] {
        s.search_lost();
    };
    const auto lostTarget = [&] {
        return s.lost_target();
    };
    const auto steer = [&](float speed, float heading) {
        s.steer(speed, heading);
    };
    const auto desiredPitch = [&]() -> int16_t {
        return s.desired_pitch();
    };
    do {
        commandFrame(0, now);
        // First-trip green preempts only a search cue, never an active cargo batch.
        if (!first_ && label_ != "ordinary_supply" && in.label == "ordinary_supply" &&
            cueCandidate()) {
            scanAfterClear("green_priority_reselect");
            break;
        }
        if (!cueCandidate() || in.target_id != search_cue_id_ || in.label != label_) {
            if (lostTarget()) {
                if (t_.enable_lost_search)
                    searchLost();
                else {
                    blacklist(now);
                    scanAfterClear("search_cue_lost");
                }
            }
            break;
        }
        misses_ = 0;
        if (elapsed > t_.cue_approach_budget_us) {
            blacklist(now);
            scanAfterClear("cue_approach_timeout");
            break;
        }
        if (in.distance_m <= t_.track_near_m && t_.track_pitch_cdeg != t_.far_pitch_cdeg) {
            target_id_ = capture_target_id_ = -1;
            cue_mode_ = false;
            const bool mid = t_.intermediate_pitch_cdeg > t_.far_pitch_cdeg &&
                             t_.intermediate_pitch_cdeg < t_.track_pitch_cdeg;
            enter(mid ? SearchAction::MidReacquire : SearchAction::NearReacquire,
                  now,
                  mid ? "mid_reacquire_pitch_wait" : "near_reacquire_pitch_wait");
            break;
        }
        if (in.distance_m <= t_.track_near_m)
            track_close_ = true;
        if (desiredPitch() != pitch_cmd_) {
            reason_ = "near_camera_pitch_wait";
            break;
        }
        if (in.distance_m <= t_.rush_start_m) {
            target_id_ = -1;
            cue_mode_ = false;
            enter(SearchAction::SelectCargo, now, "select_cargo_at_near_range");
            break;
        }
        steer(t_.cue_approach_speed, in.heading_error);
        break;
    } while (false);
}

void SweepStage::cuePrepare(const TaskTuning& t,
                            const PushObservation& in,
                            const SweepContext& c,
                            const SweepServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    auto& clear_attempts_ = c.search.clear_attempts_;
    auto& clear_commanded_us_ = c.search.clear_commanded_us_;
    auto& confirmations_ = c.progress.confirmations;
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto failStop = [&](const char* why) {
        s.fail_stop(why);
    };
    do {
        commandFrame(0, now);
        if (!t_.enable_short_push || clear_attempts_ >= 2) {
            failStop("clear_attempt_limit");
            break;
        }
        if (!in.clear_push_safe) {
            failStop("clear_path_or_boundary_unverified");
            break;
        }
        if (!in.gripper_closed_observed || !in.heading_valid) {
            if (elapsed > t_.gripper_timeout_us) {
                failStop("clear_closed_feedback_missing");
            }
            break;
        }
        if (++confirmations_ < t_.confirm_frames)
            break;
        clear_heading_rad_ = in.heading_rad;
        clear_commanded_us_ = 0;
        ++clear_attempts_;
        enter(SearchAction::ClearPile, now, "closed_push_010");
        break;
    } while (false);
}

void SweepStage::clearPile(const TaskTuning& t,
                           const PushObservation& in,
                           const SweepContext& c,
                           const SweepServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const TaskTuning& t_ = t;
    float& motion_vx = c.motion.vx_mps;
    float& motion_wz = c.motion.wz_rps;
    const float target_forward_m = c.target_forward_m;
    auto& travel_ = c.progress.travel;
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto failStop = [&](const char* why) {
        s.fail_stop(why);
    };
    const auto tracking = [&](const PushObservation& o) {
        return s.tracking(o);
    };
    do {
        commandFrame(0, now);
        if (!t_.enable_short_push || !in.clear_push_safe || !in.gripper_closed_observed ||
            !in.heading_valid || std::abs(wrapAngle(in.heading_rad - clear_heading_rad_)) > .10f) {
            failStop("clear_evidence_lost");
            break;
        }
        if (!tracking(in)) {
            failStop("clear_target_unverified");
            break;
        }
        if (target_forward_m <= t_.grasp_trigger_y_m) {
            enter(SearchAction::ClearPause, now, "clear_target_distance_reached");
            break;
        }
        // Time/integrated command only bounds an attempt; it never proves arrival.
        if (elapsed >= t_.clear_wall_budget_us || travel_ >= t_.clear_max_commanded_m) {
            failStop("clear_distance_not_reached");
            break;
        }
        motion_vx = std::min(t_.clear_speed, t_.max_speed);
        motion_wz = 0;
        break;
    } while (false);
}

void SweepStage::clearPause(const TaskTuning& t,
                            const PushObservation& in,
                            const SweepContext& c,
                            const SweepServices& s) {
    const uint64_t now = c.now_us;
    const uint64_t elapsed = c.elapsed_us;
    const auto commandFrame = [&](uint8_t open, uint64_t at) {
        s.command_frame(open, at);
    };
    const auto enter = [&](SearchAction st, uint64_t at, const char* why) {
        s.enter(st, at, why);
    };
    const auto clearTrip = [&] {
        s.clear_trip();
    };
    (void)t;
    (void)in;
    do {
        commandFrame(0, now);
        if (elapsed >= 500000) {
            clearTrip();
            enter(SearchAction::MidReacquire, now, "clear_reobserve_cargo");
        }
    } while (false);
}

} // namespace rescue
