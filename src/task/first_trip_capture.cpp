#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <iostream>

namespace rescue {
namespace {
cv::Point2f centre(const cv::Rect& box) {
    return {box.x + .5f * box.width, box.y + .5f * box.height};
}
bool upperCornerTarget(const PushObservation& in) {
    // Retaining an injured person already inside the calibrated frame uses the
    // turn-before-retreat sequence, including the front half and retry rounds.
    if (in.label == "injured_person" && in.target_grasp_range_valid &&
        in.target_grasp_range_inside)
        return false;
    // A complete near-view target may overlap the inventory with its centre above
    // the calibrated grasp range. Separation must precede approach in that case.
    return in.target_center_px_valid && !in.capture_area_px.empty() &&
           std::isfinite(in.target_center_px.y) && in.target_center_px.y >= 0 &&
           std::isfinite(in.pixel_grasp_error_px) &&
           (in.label == "injured_person" || std::abs(in.pixel_grasp_error_px) > 12.f) &&
           in.target_center_px.y < in.capture_area_px.y + .5f * in.capture_area_px.height;
}
bool positionMatch(const cv::Rect& old, const cv::Rect& box) {
    if (old.empty() || box.empty())
        return false;
    const float scale = float(box.area()) / old.area();
    const float gate = .5f * std::max({old.width, old.height, box.width, box.height});
    return scale >= .5f && scale <= 2.f && cv::norm(centre(old) - centre(box)) <= gate;
}
} // namespace

bool PushTask::captureLoadAllows(const std::string& label) const {
    return !capture_load_.active || capture_load_.choice.inventory.count(targetKind(label)) > 0;
}

void PushTask::reobserveCaptureLoad(uint64_t now) {
    if (!capture_load_.active)
        return;
    // A chassis turn changes image coordinates. Keep the load decision and its
    // deadline, but replace old identities only after stopped fresh observations.
    capture_load_.reobserve_after_us = now;
    capture_load_.stopped_us = capture_load_.sample_us = 0;
    capture_load_.confirmations = 0;
}

void PushTask::beginPixelSplitReview(TaskTick& tick) {
    auto& split = search_phase_.first_trip_split_;
    split.recovery_stage = SearchPhaseData::FirstTripSplit::RecoveryStage::Verify;
    split.recovery_view_us = tick.now;
    split.recovery_turn_us = 0;
    split.align_stage = "ui140_verify";
    split.target_reselecting = true;
    split.target_candidate_id = split.target_id = -1;
    trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
    pixel_alignment_ = {};
    capture_scene_.target_id = -1;
    capture_scene_.status = "incomplete";
    capture_scene_.detail = "split_target_reselection";
    capture_scene_.confirmations = 0;
    capture_scene_.stopped_us = tick.now;
    reobserveCaptureLoad(tick.now);
}

bool PushTask::reviewCaptureLoad(TaskTick& tick) {
    if (!t_.pixel_grasp_enabled || !first_)
        return true;
    const auto& in = tick.in;
    auto& plan = capture_load_;
    Inventory available;
    for (const auto& object : in.capture_image_objects)
        if (object.selectable && in.captureInventoryOverlaps(object.box, object.label))
            available.add(object.label);
    if (!plan.active && !(available.injured > 0 && available.supplies() > 0)) {
        plan = {};
        return true;
    }
    tick.immediate_motion_stop = true;
    if (!plan.started_us)
        plan.started_us = tick.now;
    if (tick.now - plan.started_us > t_.pile_budget_us) {
        failStop(tick, "capture_score_review_timeout");
        return false;
    }
    const auto wait = [&](const char* why) {
        plan.confirmations = 0;
        plan.stopped_us = tick.now;
        reason_ = why;
        return false;
    };
    if (!captureReviewStopped(in) || !in.camera_pitch_stable || !in.pixel_grasp_near_view ||
        !in.capture_scene_valid ||
        (!in.capture_path_valid && !search_phase_.first_trip_split_.pending) || !in.capture_scene_us ||
        in.capture_scene_us > tick.now || tick.now - in.capture_scene_us > t_.frame_timeout_us ||
        in.capture_dangerous_side_pending)
        return wait("capture_score_observation_wait");
    for (const auto& object : in.capture_image_objects)
        if ((in.splitInventoryOverlaps(object.box, object.label) ||
             in.capturePathOverlaps(object.box, object.label)) &&
            (!object.complete || targetKind(object.label) == TargetKind::UNKNOWN))
            return wait("capture_score_incomplete_inventory");
    if (plan.reobserve_after_us) {
        if (!plan.stopped_us)
            plan.stopped_us = tick.now;
        if (in.capture_scene_us > std::max(plan.reobserve_after_us, plan.stopped_us) &&
            in.capture_scene_us > plan.sample_us) {
            plan.sample_us = in.capture_scene_us;
            ++plan.confirmations;
        }
        if (plan.confirmations < t_.corridor_confirm_frames ||
            tick.now - plan.stopped_us < 250000) {
            reason_ = "capture_score_new_view_confirm_wait";
            return false;
        }
        plan.objects.clear();
        plan.observed = {};
        plan.reobserve_after_us = 0;
        plan.confirmations = 0;
    }
    // A missing previously enclosed object is not proof that it left. Permit a
    // unique same-class positional re-ID; otherwise wait for a complete view.
    std::vector<PushObservation::CaptureImageObject> tracked;
    std::set<int> used;
    for (const auto& old : plan.objects) {
        const PushObservation::CaptureImageObject* match = nullptr;
        int matches = 0;
        for (const auto& object : in.capture_image_objects)
            if (object.id == old.id && object.label == old.label) {
                match = &object;
                matches = 1;
                break;
            }
        if (!match)
            for (const auto& object : in.capture_image_objects)
                if (object.label == old.label && positionMatch(old.box, object.box)) {
                    match = &object;
                    ++matches;
                }
        if (matches != 1 || !match || !match->complete || used.count(match->id)) {
            if (in.captureInventoryOverlaps(old.box, old.label))
                return wait("capture_score_member_missing");
            continue; // It was visibly outside before disappearing.
        }
        tracked.push_back(*match);
        used.insert(match->id);
    }
    for (const auto& object : in.capture_image_objects)
        if (object.selectable && targetSelectable(object.label, first_) &&
            in.captureInventoryOverlaps(object.box, object.label) && !used.count(object.id)) {
            tracked.push_back(object);
            used.insert(object.id);
        }
    plan.objects = std::move(tracked);
    if (plan.observed != available) {
        plan.observed = available;
        plan.confirmations = 0;
        plan.stopped_us = tick.now;
        plan.sample_us = in.capture_scene_us;
    }
    if (!plan.stopped_us)
        plan.stopped_us = tick.now;
    if (in.capture_scene_us > plan.stopped_us && in.capture_scene_us > plan.sample_us) {
        plan.sample_us = in.capture_scene_us;
        ++plan.confirmations;
    } else {
        reason_ = "capture_score_confirm_wait";
        return false;
    }
    if (plan.confirmations < t_.corridor_confirm_frames ||
        tick.now - plan.stopped_us < 250000) {
        reason_ = "capture_score_confirm_wait";
        return false;
    }
    // Once physical separation starts for an injured person, retain that group
    // through re-observation and search, including temporary target absence.
    if (plan.keep_injured)
        return true;
    const auto choice = chooseTrip(available, first_);
    // When every retained object is visibly outside, reacquire the selected group
    // through UI120 instead of granting permission to capture an empty frame.
    if (!choice.inventory.total())
        return true;
    if (!plan.active || plan.choice.inventory != choice.inventory) {
        std::cout << "[CAPTURE_SCORE] ordinary=" << available.ordinary
                  << " core=" << available.core << " injured=" << available.injured
                  << " supply_score=" << choice.supply_score
                  << " injured_score=" << choice.injured_score
                  << " keep=" << (choice.inventory.injured ? "injured" : "supply")
                  << " keep_ordinary=" << choice.inventory.ordinary
                  << " keep_core=" << choice.inventory.core
                  << " keep_injured=" << choice.inventory.injured
                  << " tie=injured\n";
    }
    plan.choice = choice;
    plan.active = true;
    return true;
}

bool PushTask::nominateCaptureLoad(TaskTick& tick) {
    if (!capture_load_.active || captureLoadAllows(trip_context_.label_))
        return true;
    const PushObservation::CaptureImageObject* primary = nullptr;
    for (const auto& object : tick.in.capture_image_objects)
        if (object.complete && object.selectable && captureLoadAllows(object.label) &&
            tick.in.captureInventoryOverlaps(object.box, object.label) &&
            (!primary || centre(object.box).y > centre(primary->box).y))
            primary = &object;
    tick.immediate_motion_stop = true;
    reason_ = "capture_score_target_confirm_wait";
    if (primary) {
        auto& split = search_phase_.first_trip_split_;
        split.target_reselecting = true;
        split.target_candidate_id = primary->id;
    }
    return false;
}

bool PushTask::pixelCaptureReady(const PushObservation& in) const {
    if (in.pixel_grasp_ready)
        return true;
    const auto& split = search_phase_.first_trip_split_;
    // After reverse-turn separation, keep the one block already inside the calibrated
    // grasp range. Advancing again to Y337 brings the rear neighbour back into the jaw.
    if (!t_.pixel_grasp_enabled || !split.capture_retry || split.runs == 0 || split.pending ||
        split.active || split.reacquire || !targetSelectable(trip_context_.label_, first_) ||
        !tracking(in) || !in.pixel_grasp_near_view || !in.camera_pitch_stable ||
        !in.target_grasp_range_valid || !in.target_grasp_range_inside || !in.capture_scene_valid ||
        !in.capture_scene_us || in.capture_dangerous_side_pending || in.capture_scene_us > in.now_us ||
        in.now_us - in.capture_scene_us > t_.frame_timeout_us || in.layout_count_region_px.empty())
        return false;
    Inventory inventory;
    bool target_inside = false;
    for (const auto& object : in.capture_image_objects) {
        if (!in.captureInventoryOverlaps(object.box, object.label))
            continue;
        if (!object.complete)
            return false;
        inventory.add(object.label);
        target_inside |= object.id == in.target_id && object.label == trip_context_.label_;
    }
    return checkTrip(inventory, first_) == RuleVerdict::OK && target_inside;
}

bool PushTask::captureReviewStopped(const PushObservation& in) const {
    return in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
           std::isfinite(in.sent_wz_rps) && std::abs(in.sent_vx_mps) <= t_.pixel_align_stopped_vx &&
           std::abs(in.sent_wz_rps) <= t_.pixel_align_stopped_wz;
}

PushTask::CaptureSceneVerdict PushTask::evaluateCaptureScene(const PushObservation& in) {
    auto& scene = capture_scene_;
    scene.blocker_id = -1;
    scene.blocker_box = {};
    scene.association.clear();
    const auto incomplete = [&](const char* why) {
        scene.status = "incomplete";
        scene.detail = why;
        scene.confirmations = 0;
        scene.stopped_us = in.now_us;
        return CaptureSceneVerdict::Incomplete;
    };
    if (!in.capture_scene_valid || !in.capture_path_valid || in.layout_count_region_px.empty() ||
        !in.camera_pitch_stable || !in.pixel_grasp_near_view || !in.capture_scene_us ||
        in.capture_scene_us > in.now_us || in.now_us - in.capture_scene_us > t_.frame_timeout_us ||
        !in.target_valid || !in.target_grasp_range_valid)
        return incomplete("scene_or_target_missing");
    if (scene.pitch != kCameraPitchInvalid && scene.pitch != in.camera_pitch_cdeg)
        return incomplete("cross_view_reassociation_required");
    if (scene.sample_us && in.capture_scene_us < scene.sample_us)
        return incomplete("scene_timestamp_reversed");
    const auto region_relevant = [&](const PushObservation::CaptureImageObject& object) {
        return object.id == in.target_id ||
               in.splitInventoryOverlaps(object.box, object.label) ||
               in.capturePathOverlaps(object.box, object.label);
    };
    const auto relevant = [&](const PushObservation::CaptureImageObject& object) {
        return object.dangerous_side_pending || region_relevant(object);
    };
    std::vector<PushObservation::CaptureImageObject> current;
    std::set<int> ids;
    for (const auto& object : in.capture_image_objects) {
        if (!ids.insert(object.id).second)
            return incomplete("duplicate_object_identity");
        current.push_back(object);
    }
    const auto selected = std::find_if(current.begin(), current.end(), [&](const auto& object) {
        return object.id == in.target_id;
    });
    if (selected == current.end() || !selected->complete)
        return incomplete("target_box_incomplete");
    if (selected->label != in.label || !targetSelectable(selected->label, first_))
        return incomplete("target_class_not_selectable");
    if (scene.objects.empty()) {
        scene.objects = current;
        scene.target_id = in.target_id;
        scene.pitch = in.camera_pitch_cdeg;
    } else {
        std::set<int> used;
        // Commit the new scene only after every relevant original member is found.
        for (const auto& old : scene.objects) {
            // Outside neighbours may leave the view or remain clipped without affecting
            // capture. Keep all current detections for telemetry and split review below.
            if (old.id != scene.target_id && !relevant(old))
                continue;
            const PushObservation::CaptureImageObject* match = nullptr;
            int matches = 0;
            for (const auto& object : current) {
                if (object.label != old.label ||
                    ((old.dangerous_side_pending || old.dangerous_side_clear) && object.id != old.id) ||
                    !positionMatch(old.box, object.box))
                    continue;
                match = &object;
                ++matches;
            }
            if (matches != 1 || !match || (region_relevant(*match) && !match->complete) ||
                !used.insert(match->id).second) {
                scene.blocker_id = old.id;
                scene.blocker_box = match ? match->box : old.box;
                scene.association = matches == 0 ? "missing" : matches > 1 ? "ambiguous"
                                    : !match->complete ? "clipped" : "identity_reused";
                return incomplete("neighbor_missing_or_association_ambiguous");
            }
            if (old.id == scene.target_id) {
                if (match->id != in.target_id)
                    return incomplete("selected_target_identity_conflict");
            }
        }
        scene.target_id = in.target_id;
        scene.objects = current;
    }
    scene.sample_us = in.capture_scene_us;
    const auto target =
        std::find_if(scene.objects.begin(), scene.objects.end(), [&](const auto& object) {
            return object.id == scene.target_id;
        });
    if (target == scene.objects.end())
        return incomplete("target_association_missing");
    Inventory inventory, split_inventory;
    bool path_risk = false;
    bool target_inside = false, split_target_inside = false;
    for (const auto& object : scene.objects) {
        const bool inside = in.splitInventoryOverlaps(object.box, object.label);
        const bool in_path =
            in.capturePathOverlaps(object.box, object.label);
        if (!inside && !in_path)
            continue;
        if (!object.complete) {
            scene.blocker_id = object.id;
            scene.blocker_box = object.box;
            scene.association = "clipped";
            return incomplete("neighbor_box_clipped_or_uncertain");
        }
        if (targetKind(object.label) == TargetKind::UNKNOWN)
            return incomplete("unknown_in_capture_region");
        if (inside) {
            split_inventory.add(object.label);
            split_target_inside |= object.id == scene.target_id;
            if (in.captureInventoryOverlaps(object.box, object.label)) {
                inventory.add(object.label);
                target_inside |= object.id == scene.target_id;
            }
        } else if (object.id != scene.target_id) {
            if (!in.capture_path_lateral_measured &&
                (centre(object.box).x < target->box.x || centre(object.box).x > target->box.br().x))
                return incomplete("neighbor_lateral_range_unmeasured");
            path_risk = true;
        }
    }
    // Before the target reaches the inventory it remains the planned primary.
    // This permits normal approach; closure still requires its calibrated range.
    if (!target_inside)
        inventory.add(target->label);
    if (!split_target_inside)
        split_inventory.add(target->label);
    const auto verdict = checkTrip(split_inventory, first_);
    const bool load_mismatch = capture_load_.active &&
        (inventory != capture_load_.choice.inventory || !captureLoadAllows(target->label));
    const bool risk = path_risk || verdict != RuleVerdict::OK || load_mismatch;
    if (!risk && in.capture_dangerous_side_pending) {
        incomplete("dangerous_side_boundary_pending");
        scene.detail += ":" + in.capture_dangerous_side_detail;
        return CaptureSceneVerdict::Incomplete;
    }
    // Risk confirms separation against the split region; clear confirms capture
    // against the calibrated jaw region. Crossing only the jaw boundary must not
    // restart a stable split review, and split-only neighbours must not restart
    // an otherwise clear capture review.
    const bool inventory_changed = risk ? scene.split_inventory != split_inventory
                                        : scene.inventory != inventory;
    if (scene.status != (risk ? "risk" : "clear") || inventory_changed) {
        scene.confirmations = 0;
        scene.stopped_us = in.now_us;
        scene.confirm_us = in.capture_scene_us;
    }
    scene.inventory = inventory;
    scene.split_inventory = split_inventory;
    scene.status = risk ? "risk" : "clear";
    scene.detail = verdict == RuleVerdict::TOO_MANY_SUPPLIES && !first_
                       ? "neighbor_box_overlaps_ordinary_inventory"
                   : verdict != RuleVerdict::OK ? verdictName(verdict)
                   : load_mismatch              ? "capture_score_inventory_mismatch"
                   : path_risk                  ? "neighbor_inside_capture_path"
                                                : "neighbors_visible_outside_capture_regions";
    if (!risk && !in.capture_dangerous_side_detail.empty())
        scene.detail += ":" + in.capture_dangerous_side_detail;
    return risk ? CaptureSceneVerdict::Risk : CaptureSceneVerdict::Clear;
}

bool PushTask::confirmCaptureReview(const PushObservation& in) {
    auto& scene = capture_scene_;
    if (!captureReviewStopped(in)) {
        scene.stopped_us = in.now_us;
        scene.confirmations = 0;
        return false;
    }
    if (!scene.stopped_us)
        scene.stopped_us = in.now_us;
    if (in.capture_scene_us > scene.stopped_us && in.capture_scene_us > scene.confirm_us) {
        scene.confirm_us = in.capture_scene_us;
        ++scene.confirmations;
    }
    return scene.confirmations >= t_.corridor_confirm_frames &&
           in.now_us - scene.stopped_us >= 250000;
}

void PushTask::recordCaptureFailure(const std::string& reason) {
    failure_source_ = capture_scene_.stage == "alignment"   ? "alignment"
                      : capture_scene_.stage == "preclose"  ? "preclose_review"
                      : capture_scene_.stage == "postclose" ? "postclose_verification"
                      : pixel_alignment_.stage == PixelAlignment::Stage::Correct &&
                              pixel_alignment_.last_sample_us
                          ? "alignment"
                          : "approach";
    original_reason_ = reason;
}

bool PushTask::splitRetreatCleared(
    const PushObservation&, const PushObservation::CaptureImageObject& object) const {
    // Lower corners track the wanted block to Y270; upper corners keep Y280.
    const float stop_y = search_phase_.first_trip_split_.upper_corner ? 280.f : 270.f;
    return !object.box.empty() && object.box.y + .5f * object.box.height <= stop_y;
}

void PushTask::selectPixelSplitDirection(const PushObservation& in) {
    // Image right -> left turn; image left -> right turn.
    // The centre band keeps the previous direction, initially right.
    auto& split = search_phase_.first_trip_split_;
    split.keep_injured = trip_context_.label_ == "injured_person";
    split.upper_corner = upperCornerTarget(in);
    if (split.keep_injured && first_) {
        if (!capture_load_.active) {
            Inventory injured;
            injured.injured = 1;
            capture_load_.choice = chooseTrip(injured, first_);
            capture_load_.active = true;
            capture_load_.started_us = in.now_us;
        }
        capture_load_.keep_injured = true;
    }
    // Latch unwanted identities before the retained target can leave the near view.
    split.retreat_objects.clear();
    split.retreat_reference_id = -1;
    split.retreat_reference_y = 0;
    split.margin_stage = SearchPhaseData::FirstTripSplit::MarginStage::Idle;
    split.margin_used_us = split.margin_tick_us = split.margin_scene_us = 0;
    split.margin_sent_reverse = false;
    for (const auto& object : in.capture_image_objects) {
        if (!split.upper_corner) {
            // Lower corners reverse against the retained cargo, not its neighbours.
            if (object.id != in.target_id || !targetSelectable(object.label, first_))
                continue;
        } else if (object.id == in.target_id ||
                   (!in.splitInventoryOverlaps(object.box, object.label) &&
                    !in.capturePathOverlaps(object.box, object.label))) {
            continue;
        }
        split.retreat_objects.push_back({object, object.complete && splitRetreatCleared(in, object)});
    }
    if (in.target_center_px_valid && std::isfinite(in.pixel_grasp_error_px)) {
        if (in.pixel_grasp_error_px < -12.f)
            split.recovery_direction = 1;
        else if (in.pixel_grasp_error_px > 12.f)
            split.recovery_direction = -1;
    }
    if (split.keep_injured) {
        // Left and centre always turn right; do not inherit a previous left turn.
        split.recovery_direction = in.target_center_px_valid &&
            std::isfinite(in.pixel_grasp_error_px) && in.pixel_grasp_error_px < -12.f ? 1 : -1;
        split.injured_turn_goal = wrapAngle(in.heading_rad + split.recovery_direction * .15f);
    }
}

void PushTask::beginPixelSplit(TaskTick& tick, bool reobserve) {
    auto& split = search_phase_.first_trip_split_;
    if (!tick.in.heading_valid || !std::isfinite(tick.in.heading_rad)) {
        reason_ = "first_trip_review_heading_missing";
        return;
    }
    capture_scene_.before_split = capture_scene_.objects;
    selectPixelSplitDirection(tick.in);
    // No unwanted block was latched: reobserve instead of waiting for a retreat
    // reference that cannot appear. Missing already-latched blocks still stop.
    reobserve = reobserve || split.retreat_objects.empty();
    split.upper_corner = split.upper_corner && !reobserve;
    if (t_.pile_max_runs > 0 && split.runsFor(split.upper_corner) >= t_.pile_max_runs) {
        failStop(tick, "first_trip_split_limit");
        return;
    }
    split.pending = split.capture_retry = split.vertical_swing = true;
    split.active = split.reacquire = false;
    split.target_id = trip_context_.target_id_;
    // Re-observation and subsequent split retries share the original review budget.
    split.started_us = capture_scene_.started_us ? capture_scene_.started_us : tick.now;
    split.recovery_stage = reobserve ? SearchPhaseData::FirstTripSplit::RecoveryStage::Turn
        : split.keep_injured && !split.upper_corner
            ? SearchPhaseData::FirstTripSplit::RecoveryStage::InjuredTurn
            : SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat;
    split.recovery_turn_us = split.recovery_view_us = 0;
    split.target_reselecting = false;
    split.target_candidate_id = -1;
    split.swing_clear_frames = 0;
    split.swing_sample_us = split.swing_stopped_us = 0;
    ++split.runs;
    if (split.upper_corner) ++split.corner_runs;
    capture_scene_.confirmations = 0;
    capture_scene_.stopped_us = tick.now;
    capture_scene_.confirm_us = tick.in.capture_scene_us;
    capture_scene_.waiting = true;
    trip_context_.pending_ = {};
    capture_scene_.reobserve_stopped_us = capture_scene_.reobserve_sample_us = 0;
    capture_scene_.reobserve_frames = 0;
    enter(PushState::ABORT_DROP, tick.now,
          reobserve ? "first_trip_scene_reobserve_turn" : "first_trip_confirmed_capture_path_split");
    capture_scene_.stage = "postsplit_before_approach";
}

bool PushTask::reviewFirstTripCapture(TaskTick& tick, const char* stage, bool preclose) {
    if (!t_.pixel_grasp_enabled || !targetSelectable(trip_context_.label_, first_))
        return true;
    auto& scene = capture_scene_;
    scene.stage = stage;
    auto verdict = evaluateCaptureScene(tick.in);
    if (verdict != CaptureSceneVerdict::Incomplete &&
        (!reviewCaptureLoad(tick) || !nominateCaptureLoad(tick)))
        return false;
    if (capture_load_.active && verdict != CaptureSceneVerdict::Incomplete)
        verdict = evaluateCaptureScene(tick.in);
    if (verdict == CaptureSceneVerdict::Clear && !scene.waiting && !preclose) {
        trip_context_.pending_ = scene.inventory;
        return true;
    }
    tick.immediate_motion_stop = true;
    if (!scene.waiting) {
        scene.waiting = true;
        scene.started_us = scene.stopped_us = tick.now;
        scene.confirm_us = tick.in.capture_scene_us;
        scene.confirmations = 0;
    }
    if (!scene.started_us)
        scene.started_us = tick.now;
    if (tick.now - scene.started_us > t_.pile_budget_us) {
        const auto why = "first_trip_review_timeout_" + scene.detail;
        failStop(tick, why.c_str());
        return false;
    }
    // Incomplete association resets the inventory confirmation counter. Use a separate
    // stopped/fresh-frame window to recover instead of waiting on that counter forever.
    // Never infer clearance from a missing box: turn at UI120 using the existing
    // clearance gates, then reacquire and confirm a new stopped UI140 scene.
    const bool reobservable = scene.detail == "target_box_incomplete" ||
                              scene.detail == "neighbor_missing_or_association_ambiguous" ||
                              scene.detail == "neighbor_box_clipped_or_uncertain";
    const bool fresh_target = tracking(tick.in) && tick.in.camera_pitch_stable &&
                              tick.in.pixel_grasp_near_view && tick.in.capture_scene_valid &&
                              tick.in.capture_path_valid && tick.in.capture_scene_us &&
                              tick.in.capture_scene_us <= tick.now &&
                              tick.now - tick.in.capture_scene_us <= t_.frame_timeout_us;
    if (!captureReviewStopped(tick.in) || !fresh_target || verdict == CaptureSceneVerdict::Clear) {
        scene.reobserve_stopped_us = scene.reobserve_sample_us = 0;
        scene.reobserve_frames = 0;
    } else if (reobservable || scene.reobserve_stopped_us) {
        if (!scene.reobserve_stopped_us)
            scene.reobserve_stopped_us = tick.now;
        if (tick.in.capture_scene_us > scene.reobserve_stopped_us &&
            tick.in.capture_scene_us > scene.reobserve_sample_us) {
            scene.reobserve_sample_us = tick.in.capture_scene_us;
            ++scene.reobserve_frames;
        }
    }
    if (verdict == CaptureSceneVerdict::Incomplete) {
        reason_ = "first_trip_stop_review_" + scene.detail;
        if (reobservable && fresh_target && scene.reobserve_stopped_us &&
            scene.reobserve_frames >= t_.corridor_confirm_frames &&
            tick.now - scene.reobserve_stopped_us >= 250000)
            beginPixelSplit(tick, true);
        return false;
    }
    if (!confirmCaptureReview(tick.in)) {
        reason_ = "first_trip_stop_review_" + scene.detail;
        return false;
    }
    if (verdict == CaptureSceneVerdict::Risk) {
        // Upper corners can lie above the calibrated grasp range: split before
        // approaching them. Other targets retain the existing Y-only entry gate.
        const float y = tick.in.target_center_px.y;
        if (!tick.in.target_center_px_valid || tick.in.capture_area_px.empty() ||
            (!upperCornerTarget(tick.in) &&
             !(y >= tick.in.capture_area_px.y &&
               y <= tick.in.capture_area_px.y + tick.in.capture_area_px.height))) {
            reason_ = "first_trip_stop_review_target_not_at_grasp";
            return false;
        }
        beginPixelSplit(tick);
        return false;
    }
    const bool position_ready = search_entry_.handoff ? entryPositionValid(tick.in)
                                                     : pixelCaptureReady(tick.in);
    if (preclose &&
        (!tracking(tick.in) || !position_ready || !pixelLateralAccepted(tick.in))) {
        reason_ = "first_trip_stop_review_target_not_ready";
        return false;
    }
    trip_context_.pending_ = scene.inventory;
    scene.waiting = false;
    scene.started_us = scene.stopped_us = scene.confirm_us = 0;
    scene.confirmations = 0;
    reason_ = preclose ? "first_trip_preclose_confirmed" : "first_trip_approach_recheck_confirmed";
    return true;
}
} // namespace rescue
