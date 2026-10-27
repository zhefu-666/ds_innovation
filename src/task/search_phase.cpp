#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::updateSearchPhase(TaskTick& tick) {
    switch (search_phase_.action) {
        case SearchAction::Scan:
        case SearchAction::LostSearch:
        case SearchAction::MidReacquire:
        case SearchAction::NearReacquire:
        case SearchAction::SelectCargo:
        case SearchAction::MidApproach:
        case SearchAction::Approach: {
            SearchContext context{search_phase_,
                                  actionProgress(),
                                  trip_context_,
                                  tick.motion,
                                  sweep_ledger_,
                                  reason_};
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.scan_ok = tick.scan_ok;
            context.forward_ok = tick.forward_ok;
            context.scatter_ok = tick.scatter_ok;
            context.clutter_fresh = tick.clutter_fresh;
            context.scatter_multi_ok = tick.scatter_multi_ok;
            context.approach_ok = tick.approach_ok;
            context.mid_pitch = midPitch();
            context.trip_empty = trip_context_.trip_.total() == 0;
            context.sweep_open = sweep_open_;
            context.first_ordinary = first_;
            context.frame_raised = frame_raised_;
            context.far_y = tick.far_y;

            SearchServices services;
            services.candidate = [&](const PushObservation& obs, uint64_t at) {
                return candidate(obs, at);
            };
            services.weak_candidate = [&](const PushObservation& obs) {
                return weakCandidate(obs);
            };
            services.cue_candidate = [&] {
                return cueCandidate(tick);
            };
            services.target_body = [&] {
                return targetBody(tick);
            };
            services.find_best =
                [&](const std::vector<cv::Point2f>& pts, float max_y, float radius) {
                    const auto best = findBest(tick, pts, max_y, radius);
                    return BestGroup{best.n, best.d};
                };
            services.eval_group =
                [&](const std::vector<cv::Point2f>& pts, int need_n, int far_need, float radius) {
                    return evalGroup(tick, pts, need_n, far_need, radius);
                };
            services.add_ordinary_extra = [&](const cv::Point2f& pt) {
                tick.cluster.addOrdinaryExtra(pt);
            };
            services.begin_scatter = [&](bool far, bool commit, const char* why) {
                beginScatter(tick, far, commit, why);
            };
            services.drive = [&](float vx, float wz) {
                drive(tick, vx, wz);
            };
            services.scan_spin = [&] {
                scanSpin(tick);
            };
            services.unstick = [&] {
                return unstick(tick);
            };
            services.enter = [&](SearchAction next, uint64_t at, const char* why) {
                enterSearch(next, at, why);
            };
            services.fail_stop = [&](const char* why) {
                failStop(tick, why);
            };
            services.scan_after_clear = [&](const char* why) {
                scanAfterClear(tick, why);
            };
            services.command_frame = [&](uint8_t open, uint64_t at) {
                commandFrame(open, at);
            };
            services.reject_search_cue = [&](uint64_t at) {
                if (search_phase_.search_cue_id_ >= 0)
                    rejected_[search_phase_.search_cue_id_] = at + t_.blacklist_us;
            };
            services.search_lost = [&] {
                searchLost(tick);
            };

            context.pitch_cmd = pitch_cmd_;
            context.sweep_over = tick.sweepOver;
            context.sweep_time_ok = tick.sweepTimeOk;
            context.sweep_ledger_ok = tick.sweepLedgerOk;
            context.sweep_open_disabled = sweep_open_disabled_;
            context.sweep_esc = tick.sweep_esc;
            context.target_forward_m = tick.target_forward_m;

            services.frame_wait = [&](const PushObservation& o, uint8_t open, uint64_t at) {
                return frameWait(o, open, at);
            };
            services.clear_trip = [&] {
                clearTrip();
            };
            services.blacklist = [&](uint64_t at) {
                blacklist(at);
            };
            services.clear_rejected = [&] {
                rejected_.clear();
            };
            services.clear_rejected_bearings = [&] {
                rejected_bearings_.clear();
            };
            services.sweep_frame_ready = [&] {
                return sweepFrameReady(tick);
            };
            services.disperse_step = [&] {
                return disperseStep(tick);
            };
            services.begin_pile_from_sweep = [&](const char* why) {
                return beginPileFromSweep(tick, why);
            };
            services.begin_pile = [&](const char* why) {
                beginPile(tick, why);
            };
            services.lost_target = [&] {
                return lostTarget(tick);
            };
            services.tracking = [&](const PushObservation& o) {
                return tracking(o);
            };
            services.tracking_reason = [&] {
                return trackingReason(tick);
            };
            services.approach_heading_shift = [&](const PushObservation& o) {
                return approachHeadingShift(o);
            };
            services.upright_in_corridor =
                [&](const PushObservation& o, uint64_t at, float& lat, float* along) {
                    return uprightInCorridor(o, at, lat, along);
                };
            services.neighbour_in_corridor =
                [&](const PushObservation& o, uint64_t at, float& lat, float* along) {
                    return neighbourInCorridor(o, at, lat, along);
                };
            services.desired_pitch = [&]() -> int16_t {
                return desiredPitch();
            };
            services.mid_pitch = [&] {
                return midPitch();
            };
            services.steer = [&](float speed, float heading) {
                steer(tick, speed, heading);
            };
            services.motion_ratio = [&] {
                return motionRatio();
            };

            search_phase_.search_.update(search_phase_.action, t_, tick.in, context, services);
            break;
        }
        case SearchAction::Scatter:
        case SearchAction::Pile:
        case SearchAction::CueApproach:
        case SearchAction::CuePrepare:
        case SearchAction::ClearPile:
        case SearchAction::ClearPause: {
            SweepContext context{search_phase_,
                                 actionProgress(),
                                 trip_context_,
                                 tick.motion,
                                 sweep_ledger_,
                                 reason_};
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.clutter_fresh = tick.clutter_fresh;
            context.scatter_ok = tick.scatter_ok;
            context.first_ordinary = first_;
            context.pitch_cmd = pitch_cmd_;
            context.sweep_over = tick.sweepOver;
            context.sweep_operation_timeout = tick.sweepBudget.operation_timeout;
            context.target_forward_m = tick.target_forward_m;

            SweepServices services;
            services.command_frame = [&](uint8_t open, uint64_t at) {
                commandFrame(open, at);
            };
            services.drive = [&](float vx, float wz) {
                drive(tick, vx, wz);
            };
            services.enter = [&](SearchAction next, uint64_t at, const char* why) {
                enterSearch(next, at, why);
            };
            services.fail_stop = [&](const char* why) {
                failStop(tick, why);
            };
            services.scan_after_clear = [&](const char* why) {
                scanAfterClear(tick, why);
            };
            services.blacklist = [&](uint64_t at) {
                blacklist(at);
            };
            services.clear_trip = [&] {
                clearTrip();
            };
            services.candidate = [&](const PushObservation& o, uint64_t at) {
                return candidate(o, at);
            };
            services.weak_candidate = [&](const PushObservation& o) {
                return weakCandidate(o);
            };
            services.cue_candidate = [&] {
                return cueCandidate(tick);
            };
            services.target_body = [&] {
                return targetBody(tick);
            };
            services.find_best =
                [&](const std::vector<cv::Point2f>& pts, float max_y, float radius) {
                    const auto best = findBest(tick, pts, max_y, radius);
                    return SweepBest{best.n, best.d};
                };
            services.eval_group =
                [&](const std::vector<cv::Point2f>& pts, int need, int far_need, float radius) {
                    return evalGroup(tick, pts, need, far_need, radius);
                };
            services.add_ordinary_extra = [&](const cv::Point2f& pt) {
                tick.cluster.addOrdinaryExtra(pt);
            };
            services.clear_rejected = [&] {
                rejected_.clear();
            };
            services.clear_rejected_bearings = [&] {
                rejected_bearings_.clear();
            };
            services.begin_scatter = [&](bool far, bool commit, const char* why) {
                beginScatter(tick, far, commit, why);
            };
            services.sweep_frame_ready = [&] {
                return sweepFrameReady(tick);
            };
            services.disperse_step = [&] {
                return disperseStep(tick);
            };
            services.begin_pile_from_sweep = [&](const char* why) {
                return beginPileFromSweep(tick, why);
            };
            services.search_lost = [&] {
                searchLost(tick);
            };
            services.lost_target = [&] {
                return lostTarget(tick);
            };
            services.steer = [&](float speed, float heading) {
                steer(tick, speed, heading);
            };
            services.desired_pitch = [&]() -> int16_t {
                return desiredPitch();
            };
            services.tracking = [&](const PushObservation& o) {
                return tracking(o);
            };
            services.mid_pitch = [&] {
                return midPitch();
            };
            services.motion_ratio = [&] {
                return motionRatio();
            };

            search_phase_.sweep_.update(search_phase_.action, t_, tick.in, context, services);
            break;
        }
        case SearchAction::Prepare: {
            // Stopped: open the frame, then inventory everything the rush would sweep.
            commandFrame(1, tick.now);
            const int p = pitchWait(tick.in, tick.now);
            if (p < 0) {
                fail("camera_pitch_timeout");
                break;
            }
            if (p == 0) {
                reason_ = "camera_pitch_wait";
                break;
            }
            const int g = frameWait(tick.in, 1, tick.now);
            // NEAR is a stationary opening check, not a calibrated driving view.
            // Do not require ground geometry while observing at 40 degrees.
            if (actionProgress().step == 0) {
                if (g < 0) {
                    fail("gripper_open_timeout");
                    break;
                }
                if (g == 0) {
                    reason_ = "gripper_feedback_wait";
                    break;
                }
                actionProgress().step = 1;
                actionProgress().started_us = tick.now;
                actionProgress().confirmations = 0;
                actionProgress().misses = 0;
                reason_ = "open_confirmed_return_to_tracking_pitch";
                break;
            }
            if (!tracking(tick.in)) {
                reason_ = trackingReason(tick);
                // TEMP_ASSUMPTION：方框张开后跟踪器会给同一货物发新ID；同标签有效货物直接改绑，不退回SCAN重来。
                const bool near_last =
                    t_.assume_all_safe ||
                    (last_target_valid_ && std::isfinite(tick.in.distance_m) &&
                     std::isfinite(tick.in.heading_error) &&
                     cv::norm(targetBody(tick) - last_target_xy_) <= t_.rebind_max_shift_m);
                if (rushLogic() && tick.in.target_valid &&
                    tick.in.target_id != trip_context_.target_id_ &&
                    tick.in.label == trip_context_.label_ &&
                    ((candidate(tick.in, tick.now) && near_last) ||
                     sameObject(tick.in, tick.now))) {
                    trip_context_.target_id_ = trip_context_.capture_target_id_ = tick.in.target_id;
                    actionProgress().misses = 0;
                    reason_ = "prepare_id_rebound_after_open";
                    break;
                }
                if (lostTarget(tick)) {
                    scanAfterClear(tick, "target_lost");
                }
                break;
            }
            actionProgress().misses = 0;
            if (g < 0) {
                fail("gripper_open_timeout");
                break;
            }
            if (tick.elapsed > t_.prepare_budget_us) {
                blacklist(tick.now);
                scanAfterClear(tick, "corridor_unresolved");
                break;
            }
            if (g == 0) {
                reason_ = "gripper_feedback_wait";
                break;
            }
            if (t_.assume_all_safe) {
                // TEMP_ASSUMPTION: an unresolved corridor counts as holding only the locked target.
                if (++actionProgress().confirmations < t_.confirm_frames) {
                    reason_ = "assumed_corridor_confirm";
                    break;
                }
                Inventory only;
                only.add(trip_context_.label_);
                const bool seen_ok = checkCorridor(tick.in.corridor,
                                                   tick.in.corridor_complete,
                                                   tick.in.corridor_occlusion_free,
                                                   trip_context_.label_,
                                                   first_) == RuleVerdict::OK;
                trip_context_.pending_ = seen_ok ? tick.in.corridor : only;
                trip_context_.verdict_ = RuleVerdict::OK;
                requestPhase(PhaseOutcome::CaptureReady,
                             seen_ok ? "corridor_ok" : "assumed_corridor_ok");
                assume_rush_end_ = std::max(0.f, tick.target_forward_m - t_.assume_rush_trigger_m) *
                                   t_.assume_motion_ratio;
                actionProgress().travel_limit =
                    assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
                break;
            }
            const auto verdict = checkCorridor(tick.in.corridor,
                                               tick.in.corridor_complete,
                                               tick.in.corridor_occlusion_free,
                                               trip_context_.label_,
                                               first_);
            trip_context_.verdict_ = verdict;
            if (verdict == RuleVerdict::INCOMPLETE) {
                actionProgress().confirmations = 0;
                break;
            }
            if (actionProgress().confirmations == 0 || tick.in.corridor != trip_context_.seen_) {
                trip_context_.seen_ = tick.in.corridor;
                actionProgress().confirmations = 0;
            }
            if (++actionProgress().confirmations < t_.corridor_confirm_frames)
                break;
            if (verdict != RuleVerdict::OK) {
                const std::string why =
                    std::string("corridor_") + verdictName(verdict) + "_back_off";
                const bool fatal = verdict == RuleVerdict::DANGEROUS ||
                                   verdict == RuleVerdict::CORE_BEFORE_FIRST ||
                                   verdict == RuleVerdict::INJURED_BEFORE_FIRST ||
                                   verdict == RuleVerdict::INJURED_NOT_ALONE;
                if (fatal || ++search_phase_.corridor_retries_ > t_.corridor_retry_max) {
                    blacklist(tick.now);
                    search_phase_.corridor_retries_ = 0;
                }
                trip_context_.verdict_ = verdict;
                enter(PushState::LOST_HOLD, tick.now, why.c_str());
                corridor_retreat_ = true;
                break;
            }
            search_phase_.corridor_retries_ = 0;
            trip_context_.pending_ = tick.in.corridor;
            requestPhase(PhaseOutcome::CaptureReady, "corridor_ok");
            if (rushLogic()) {
                assume_rush_end_ = std::max(0.f, tick.target_forward_m - t_.assume_rush_trigger_m) *
                                   t_.assume_motion_ratio;
                actionProgress().travel_limit =
                    assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
            } else
                actionProgress().travel_limit =
                    std::max(0.f, tick.target_forward_m - t_.grasp_trigger_y_m) + t_.rush_extra_m;
            break;
        }
        case SearchAction::TurnAway: {
            commandFrame(0, tick.now);
            // A turn is complete only with measured IMU heading changes.
            if (tick.in.heading_valid && std::isfinite(tick.in.heading_rad)) {
                if (actionProgress().heading_seen)
                    actionProgress().turned +=
                        std::abs(wrapAngle(tick.in.heading_rad - actionProgress().last_heading));
                actionProgress().last_heading = tick.in.heading_rad;
                actionProgress().heading_seen = true;
            } else
                actionProgress().heading_seen = false;
            if (actionProgress().turned >= t_.turn_min_rad) {
                if (t_.demo_carry_once) {
                    failStop(tick,
                             transport_phase_.delivery_visual_ ? "demo_next_trip_ready"
                                                               : "demo_next_trip_unverified");
                    break;
                }
                sweep_ledger_.newTrip(
                    t_, tick.now); // a new trip starts with a fresh per-trip sweep
                                   // ledger (the match-wide time ledger keeps running)
                scanAfterClear(tick, "");
                break;
            }
            if (tick.elapsed > t_.turn_budget_us) {
                fail("turn_heading_unverified");
                break;
            }
            drive(tick, 0, t_.turn_wz);
            break;
        }
        default:
            break;
    }
}
} // namespace rescue
