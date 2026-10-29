#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace rescue {
namespace {
cv::Point2f centre(const cv::Rect& box) {
    return {box.x + .5f * box.width, box.y + .5f * box.height};
}
bool positionMatch(const cv::Rect& old, const cv::Rect& box) {
    if (old.empty() || box.empty())
        return false;
    const float scale = float(box.area()) / old.area();
    const float gate = .5f * std::max({old.width, old.height, box.width, box.height});
    return scale >= .5f && scale <= 2.f && cv::norm(centre(old) - centre(box)) <= gate;
}
} // namespace

bool PushTask::pixelCaptureReady(const PushObservation& in) const {
    if (in.pixel_grasp_ready)
        return true;
    const auto& split = search_phase_.first_trip_split_;
    // After reverse-turn separation, keep the one block already inside the calibrated
    // grasp range. Advancing again to Y337 brings the rear neighbour back into the jaw.
    if (!t_.pixel_grasp_enabled || first_ || !split.capture_retry || split.runs == 0 ||
        split.pending || split.active || split.reacquire ||
        targetKind(trip_context_.label_) != TargetKind::ORDINARY || !tracking(in) ||
        !in.pixel_grasp_near_view || !in.camera_pitch_stable || !in.target_grasp_range_valid ||
        !in.target_grasp_range_inside || !in.capture_scene_valid || !in.capture_scene_us ||
        in.capture_scene_us > in.now_us || in.now_us - in.capture_scene_us > t_.frame_timeout_us ||
        in.layout_count_region_px.empty())
        return false;
    int ordinary_inside = 0;
    bool target_inside = false;
    for (const auto& object : in.capture_image_objects) {
        if (object.label != "ordinary_supply" || !in.ordinaryInventoryOverlaps(object.box))
            continue;
        ++ordinary_inside;
        target_inside |= object.id == in.target_id && object.complete;
    }
    return ordinary_inside == 1 && target_inside;
}

bool PushTask::captureReviewStopped(const PushObservation& in) const {
    return in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
           std::isfinite(in.sent_wz_rps) && std::abs(in.sent_vx_mps) <= t_.pixel_align_stopped_vx &&
           std::abs(in.sent_wz_rps) <= t_.pixel_align_stopped_wz;
}

PushTask::CaptureSceneVerdict PushTask::evaluateCaptureScene(const PushObservation& in) {
    auto& scene = capture_scene_;
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
    std::vector<PushObservation::CaptureImageObject> current;
    std::set<int> ids;
    for (const auto& object : in.capture_image_objects)
        if (object.label == "ordinary_supply") {
            if (!ids.insert(object.id).second)
                return incomplete("duplicate_object_identity");
            current.push_back(object);
        }
    const auto selected = std::find_if(current.begin(), current.end(), [&](const auto& object) {
        return object.id == in.target_id;
    });
    if (selected == current.end() || !selected->complete)
        return incomplete("target_box_incomplete");
    if (scene.objects.empty()) {
        scene.objects = current;
        scene.target_id = in.target_id;
        scene.pitch = in.camera_pitch_cdeg;
    } else {
        std::set<int> used;
        auto associated = scene.objects; // commit only after every original member is found
        for (auto& old : associated) {
            const PushObservation::CaptureImageObject* match = nullptr;
            int matches = 0;
            for (const auto& object : current) {
                if (!positionMatch(old.box, object.box))
                    continue;
                match = &object;
                ++matches;
            }
            if (matches != 1 || !match || !match->complete || !used.insert(match->id).second)
                return incomplete("neighbor_missing_or_association_ambiguous");
            if (old.id == scene.target_id) {
                if (match->id != in.target_id)
                    return incomplete("selected_target_identity_conflict");
            }
            old = *match;
        }
        scene.target_id = in.target_id;
        for (const auto& object : current)
            if (!used.count(object.id))
                associated.push_back(object);
        scene.objects = std::move(associated);
    }
    scene.sample_us = in.capture_scene_us;
    const auto target =
        std::find_if(scene.objects.begin(), scene.objects.end(), [&](const auto& object) {
            return object.id == scene.target_id;
        });
    if (target == scene.objects.end())
        return incomplete("target_association_missing");
    bool risk = false, inventory_risk = false;
    for (const auto& object : scene.objects) {
        if (!object.complete)
            return incomplete("neighbor_box_clipped_or_uncertain");
        if (object.id == scene.target_id)
            continue;
        const bool in_inventory = in.ordinaryInventoryOverlaps(object.box);
        if (in_inventory ||
            PushObservation::imageBoxOverlapsRegion(object.box, in.capture_path_px)) {
            // An inventory overlap already proves a conservative multi-supply risk, not
            // permission to close. Only the unmeasured full-image path needs the side gate.
            if (!in_inventory && !in.capture_path_lateral_measured &&
                (centre(object.box).x < target->box.x || centre(object.box).x > target->box.br().x))
                return incomplete("neighbor_lateral_range_unmeasured");
            risk = true;
            inventory_risk = inventory_risk || in_inventory;
        }
    }
    for (const auto& object : in.capture_image_objects)
        if (object.label != "ordinary_supply" && in.capture_path_px.contains(centre(object.box)))
            return incomplete("nonordinary_or_unknown_in_capture_path");
    if (scene.status != (risk ? "risk" : "clear")) {
        scene.confirmations = 0;
        scene.stopped_us = in.now_us;
        scene.confirm_us = in.capture_scene_us;
    }
    scene.status = risk ? "risk" : "clear";
    scene.detail = inventory_risk ? "neighbor_box_overlaps_ordinary_inventory"
                   : risk         ? "neighbor_inside_capture_path"
                                  : "neighbors_visible_outside_capture_regions";
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

void PushTask::beginPixelSplit(TaskTick& tick) {
    auto& split = search_phase_.first_trip_split_;
    if (split.runs >= t_.pile_max_runs) {
        failStop(tick, "first_trip_split_limit");
        return;
    }
    if (!tick.in.heading_valid || !std::isfinite(tick.in.heading_rad)) {
        reason_ = "first_trip_review_heading_missing";
        return;
    }
    capture_scene_.before_split = capture_scene_.objects;
    split.pending = split.capture_retry = split.vertical_swing = true;
    split.active = split.reacquire = false;
    split.target_id = trip_context_.target_id_;
    split.started_us = tick.now;
    split.recovery_stage = SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat;
    split.recovery_turn_us = split.recovery_view_us = 0;
    split.target_reselecting = false;
    split.target_candidate_id = -1;
    split.swing_clear_frames = 0;
    split.swing_sample_us = split.swing_stopped_us = 0;
    ++split.runs;
    capture_scene_.confirmations = 0;
    capture_scene_.stopped_us = tick.now;
    capture_scene_.confirm_us = tick.in.capture_scene_us;
    capture_scene_.waiting = true;
    trip_context_.pending_ = {};
    enter(PushState::ABORT_DROP, tick.now, "first_trip_confirmed_capture_path_split");
    capture_scene_.stage = "postsplit_before_approach";
}

bool PushTask::reviewFirstTripCapture(TaskTick& tick, const char* stage, bool preclose) {
    if (!t_.pixel_grasp_enabled || first_ ||
        targetKind(trip_context_.label_) != TargetKind::ORDINARY)
        return true;
    auto& scene = capture_scene_;
    scene.stage = stage;
    const auto verdict = evaluateCaptureScene(tick.in);
    if (verdict == CaptureSceneVerdict::Clear && !scene.waiting && !preclose)
        return true;
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
    if (verdict == CaptureSceneVerdict::Incomplete) {
        reason_ = "first_trip_stop_review_" + scene.detail;
        return false;
    }
    if (!confirmCaptureReview(tick.in)) {
        reason_ = "first_trip_stop_review_" + scene.detail;
        return false;
    }
    if (verdict == CaptureSceneVerdict::Risk) {
        // Split entry uses only the calibrated Y range. X alignment belongs to
        // capture after separation, and must not interrupt the stopped review here.
        const float y = tick.in.target_center_px.y;
        if (!tick.in.target_center_px_valid || tick.in.capture_area_px.empty() ||
            !(y >= tick.in.capture_area_px.y &&
              y <= tick.in.capture_area_px.y + tick.in.capture_area_px.height)) {
            reason_ = "first_trip_stop_review_target_not_at_grasp";
            return false;
        }
        beginPixelSplit(tick);
        return false;
    }
    if (preclose &&
        (!tracking(tick.in) || !pixelCaptureReady(tick.in) || !pixelLateralAccepted(tick.in))) {
        reason_ = "first_trip_stop_review_target_not_ready";
        return false;
    }
    scene.waiting = false;
    scene.started_us = scene.stopped_us = scene.confirm_us = 0;
    scene.confirmations = 0;
    reason_ = preclose ? "first_trip_preclose_confirmed" : "first_trip_approach_recheck_confirmed";
    return true;
}
} // namespace rescue
