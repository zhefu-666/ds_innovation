#include "rescue/perception/capture_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <sstream>
#include <opencv2/imgproc.hpp>

namespace rescue {
namespace {
float iou(const cv::Rect& a, const cv::Rect& b) {
    const float inter = float((a & b).area());
    const float uni = float(a.area() + b.area()) - inter;
    return uni > 0 ? inter / uni : 0.f;
}
} // namespace
const HoldingView* CaptureMonitor::view(int16_t pitch, bool stable) const {
    if (!stable || pitch == kCameraPitchInvalid)
        return nullptr;
    for (const auto& v : c_.holding)
        if (std::abs(int(pitch) - int(v.pitch_cdeg)) <= c_.pitch_tolerance_cdeg)
            return &v;
    return nullptr;
}
float CaptureMonitor::footprintHalf(const std::string& label) const {
    switch (targetKind(label)) {
        case TargetKind::ORDINARY:
            return c_.footprint_ordinary_m;
        case TargetKind::DANGEROUS:
            return c_.footprint_dangerous_m;
        case TargetKind::CORE:
            return c_.footprint_core_m;
        case TargetKind::INJURED:
            return c_.footprint_injured_m;
        default:
            return c_.footprint_unknown_m;
    }
}
void CaptureMonitor::update(PushObservation& in,
                            const std::vector<SegDetection>& detections,
                            uint64_t now,
                            const std::string& capture_label) {
    in.split_layout_valid = in.image_layout_vertical_dominant = false;
    in.split_vertical_pairs = in.split_ordinary_count = in.split_ordinary_visible_count = 0;
    in.split_layout_us = 0;
    in.capture_scene_valid = in.capture_path_valid = false;
    in.capture_path_lateral_measured = false;
    in.capture_center_membership = false;
    in.capture_scene_us = 0;
    in.capture_area_px = in.capture_path_px = in.layout_count_region_px = {};
    in.capture_dangerous_front_region_px = {};
    in.capture_image_objects.clear();
    in.capture_dangerous_side_pending = false;
    in.capture_dangerous_pending_ids.clear();
    in.capture_dangerous_side_detail.clear();
    in.capture_class_regions_px.clear();
    in.pixel_grasp_view = in.pixel_grasp_near_view = false;
    in.pixel_grasp_target_valid = in.pixel_grasp_ready = in.pixel_grasp_aligned = false;
    in.pixel_grasp_steering = in.pixel_grasp_error_px = 0;
    in.pixel_grasp_sample_us = 0;
    in.pixel_grasp_image_height_px = 0;
    in.target_center_px = {};
    in.target_center_px_valid = in.pixel_view_handoff_ready = false;
    if (c_.pixel_grasp_enabled && in.camera_pitch_cdeg != kCameraPitchInvalid) {
        in.pixel_grasp_view = std::abs(int(in.camera_pitch_cdeg) -
                                      int(c_.pixel_approach_pitch_cdeg)) <= c_.pitch_tolerance_cdeg;
        for (const auto& range : c_.grasp_center_ranges)
            if (std::abs(int(in.camera_pitch_cdeg) - int(range.pitch_cdeg)) <= c_.pitch_tolerance_cdeg)
                in.pixel_grasp_near_view = true;
        in.pixel_grasp_view = in.pixel_grasp_view || in.pixel_grasp_near_view;
    }
    in.target_grasp_range_applicable = false;
    in.target_grasp_range_valid = in.target_grasp_range_inside = false;
    in.target_grasp_range_lateral = in.target_grasp_range_past_rear = false;
    in.target_grasp_range_lateral_measured = false;
    in.held = {};
    in.held_objects.clear();
    in.captured = false;
    in.held_complete = false;
    in.hold_observable = false;
    in.corridor = {};
    in.corridor_complete = false;
    in.corridor_occlusion_free = false;
    diag_ = {};
    corridor_diag_.clear();
    const auto& range_label = capture_label.empty() ? in.label : capture_label;
    for (const auto& range : c_.grasp_center_ranges)
        if (range.label == range_label && in.camera_pitch_cdeg != kCameraPitchInvalid &&
            std::abs(int(in.camera_pitch_cdeg) - int(range.pitch_cdeg)) <= c_.pitch_tolerance_cdeg)
            in.target_grasp_range_applicable = true;
    // Image positions are only comparable at one fixed, settled camera pitch.
    if (!in.camera_pitch_stable || in.camera_pitch_cdeg == kCameraPitchInvalid) {
        history_.clear();
        view_ = -1;
        for (auto& entry : dangerous_side_) {
            entry.second.clear_frames = 0;
            entry.second.clear_us = 0;
            entry.second.pending = true;
            entry.second.pitch = kCameraPitchInvalid;
        }
        return;
    }
    std::vector<const SegDetection*> fresh;
    for (const auto& d : detections) {
        if (d.label == "blue_safe_zone" || d.label == "red_safe_zone")
            continue;
        if (d.timestamp_us == 0 || now < d.timestamp_us || now - d.timestamp_us > c_.max_age_us)
            continue;
        if (!std::isfinite(d.confidence) || d.confidence < c_.noise_confidence)
            continue;
        if (d.box.width <= 0 || d.box.height <= 0)
            continue;
        fresh.push_back(&d);
    }
    if (c_.pixel_grasp_enabled)
        for (const auto* d : fresh) {
            const auto r = std::find_if(c_.grasp_center_ranges.begin(), c_.grasp_center_ranges.end(),
                [&](const GraspCenterRange& range) { return range.label == d->label; });
            if (r == c_.grasp_center_ranges.end() || !in.target_valid ||
                d->track_id != in.target_id || d->label != in.label ||
                d->confidence < c_.min_confidence || d->box.x < 0 || d->box.y < 0 ||
                d->box.br().x > r->image_width_px || d->box.br().y > r->image_height_px)
                continue;
            in.target_center_px_valid = true;
            in.target_center_px = d->center();
            // View-management policy, not a grasp boundary or ground calibration:
            // retain the current view until the centre reaches its lower quarter.
            // At 1280x720 this is Y540 for both FAR->UI120 and UI120->UI140.
            in.pixel_view_handoff_ready = in.target_center_px.y >= .75f * r->image_height_px;
            in.pixel_grasp_target_valid = in.pixel_grasp_view;
            // FAR also needs fresh horizontal error before changing to UI120.
            const float target_x = in.pixel_grasp_near_view ? r->target_center_px.x
                                                           : r->image_width_px * .5f;
            const float error = target_x - in.target_center_px.x;
            in.pixel_grasp_error_px = error;
            in.pixel_grasp_sample_us = d->timestamp_us;
            in.pixel_grasp_steering = error / (r->image_width_px * .5f);
            in.pixel_grasp_aligned = in.pixel_grasp_view &&
                                    std::abs(error) <= c_.pixel_align_tolerance_px;
            break;
        }
    // These samples describe a 140-degree view, not a ground-contact estimate.
    for (const auto& range : c_.grasp_center_ranges) {
        if (range.label != range_label ||
            std::abs(int(in.camera_pitch_cdeg) - int(range.pitch_cdeg)) > c_.pitch_tolerance_cdeg)
            continue;
        in.target_grasp_range_applicable = true;
        for (const auto* d : fresh) {
            if (!in.target_valid || d->track_id != in.target_id || d->label != range_label ||
                d->confidence < c_.min_confidence || d->box.x < 0 || d->box.y < 0 ||
                d->box.br().x > range.image_width_px || d->box.br().y > range.image_height_px)
                continue;
            const auto center = d->center();
            in.target_grasp_range_valid = true;
            in.target_grasp_range_lateral_measured = range.lateral_measured;
            in.target_grasp_range_lateral = center.x >= range.area.x1 && center.x <= range.area.x2;
            in.target_grasp_range_past_rear = center.y > range.area.y2;
            in.target_grasp_range_inside = range.containsCenter(d->box);
            // Unmeasured lateral bounds span the image only for inventory bookkeeping;
            // they must never authorize off-centre closure.
            in.pixel_grasp_ready = in.pixel_grasp_near_view &&
                                    in.target_grasp_range_inside &&
                                    (range.lateral_measured || in.pixel_grasp_aligned) &&
                                    center.y >= range.target_center_px.y;
            break;
        }
        break;
    }
    const auto labelOf = [&](const SegDetection& d) {
        return d.confidence >= c_.min_confidence ? d.label : std::string("unknown");
    };
    if (c_.pixel_grasp_enabled && !c_.grasp_center_ranges.empty()) {
        const auto& ref = c_.grasp_center_ranges.front();
        in.pixel_grasp_image_height_px = ref.image_height_px;
        in.capture_scene_valid = true;
        in.capture_scene_us = now; // observation_builder replaces this with source image time
        for (const auto* d : fresh) {
            in.capture_image_objects.push_back(
                {d->track_id,
                 labelOf(*d),
                 d->box,
                 d->track_id >= 0 && d->confidence >= c_.min_confidence &&
                     captureBoxComplete(d->box, {ref.image_width_px, ref.image_height_px})});
        }
        if (in.pixel_grasp_near_view && in.target_grasp_range_valid)
            for (const auto& range : c_.grasp_center_ranges) {
                if (range.label != range_label)
                    continue;
                // Unmeasured lateral bounds remain conservative full-image bounds.
                in.capture_area_px = {range.area.x1, range.area.y1,
                                      range.area.x2 - range.area.x1,
                                      range.area.y2 - range.area.y1};
                // Capture-path review starts at Y205; ordinary neighbours use box overlap.
                // Keep the calibrated closure area and its rear/lateral bounds separate.
                in.capture_path_px = {range.area.x1, 205.f,
                                      range.area.x2 - range.area.x1, range.area.y2 - 205.f};
                in.capture_path_valid = true;
                in.capture_path_lateral_measured = range.lateral_measured;
                break;
            }
    }
    // UI130 transport review keeps the existing horizontal image-strip policy.
    // This is inspection only: UI140 grasp/inventory boundaries remain unavailable.
    if (in.transport_checks_enabled && in.camera_pitch_cdeg == kTransportReviewPitchCdeg)
        for (const auto& range : c_.grasp_center_ranges)
            if (range.label == "ordinary_supply" && range.lateral_measured)
                in.layout_count_region_px = {range.area.x1, 0.f,
                    range.area.x2 - range.area.x1, float(range.image_height_px)};
    // At UI140 the supplied box-centre limits replace ground projection for both
    // the capture inventory and post-close image evidence. UI120 never borrows them.
    if (in.pixel_grasp_near_view) {
        in.capture_center_membership = c_.pixel_center_membership;
        if (in.capture_center_membership) {
            dangerous_side_.clear(); // Old bbox-edge history cannot override centre calibration.
            in.capture_dangerous_front_region_px = c_.dangerous_front_risk_region_px;
        }
        const auto& ref = c_.grasp_center_ranges.front();
        float left = float(ref.image_width_px), right = 0;
        for (const auto& range : c_.grasp_center_ranges) {
            if (range.lateral_measured) {
                left = std::min(left, range.area.x1);
                right = std::max(right, range.area.x2);
            }
        }
        for (const auto& range : c_.grasp_center_ranges)
            if (range.label == "ordinary_supply") {
                const float ordinary_left = range.lateral_measured ? range.area.x1 : left;
                const float ordinary_right = range.lateral_measured ? range.area.x2 : right;
                in.layout_count_region_px =
                    {ordinary_left, 180.f, ordinary_right - ordinary_left, range.area.y2 - 180.f};
            }
        for (const auto& range : c_.grasp_center_ranges)
                in.capture_class_regions_px[range.label] = {range.area.x1,
                                                            range.area.y1,
                                                            range.area.x2 - range.area.x1,
                                                            range.area.y2 - range.area.y1};
        // Boundary evidence is shared by inventory, path and held review. Every
        // region suppressed by dangerous_side_clear must agree on the entering edge.
        std::vector<cv::Rect2f> dangerous_regions = {in.layout_count_region_px, in.capture_path_px};
        if (const auto region = in.capture_class_regions_px.find("dangerous_object");
            region != in.capture_class_regions_px.end())
            dangerous_regions.push_back(region->second);
        if (dangerous_side_target_id_ != in.target_id) {
            for (auto it = dangerous_side_.begin(); it != dangerous_side_.end();) {
                if (!it->second.pending && !it->second.risk)
                    it = dangerous_side_.erase(it);
                else {
                    it->second.clear_frames = 0;
                    it->second.clear_us = 0;
                    ++it;
                }
            }
            dangerous_side_target_id_ = in.target_id;
        }
        std::set<int> seen_side_ids;
        std::map<int, int> identity_count;
        for (const auto* d : fresh)
            ++identity_count[d->track_id];
        const bool stopped = in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                             std::isfinite(in.sent_wz_rps) &&
                             std::abs(in.sent_vx_mps) <= .001f &&
                             std::abs(in.sent_wz_rps) <= .001f;
        for (const auto* d : fresh) {
            if (in.capture_center_membership || labelOf(*d) != "dangerous_object")
                continue;
            const auto& region = in.layout_count_region_px;
            const bool overlap = std::any_of(dangerous_regions.begin(), dangerous_regions.end(),
                [&](const auto& area) { return PushObservation::imageBoxOverlapsRegion(d->box, area); });
            const float cx = d->center().x;
            const bool right = cx > region.x + region.width;
            const bool left = cx < region.x;
            const float penetration = right ? region.x + region.width - d->box.x
                                    : left ? d->box.br().x - region.x : d->box.width;
            const float ratio = std::max(0.f, penetration / d->box.width);
            const bool entering_edge_visible = right ? d->box.x > 0 && d->box.x < ref.image_width_px
                                             : left ? d->box.br().x > 0 &&
                                                      d->box.br().x < ref.image_width_px : false;
            const bool valid_identity = d->track_id >= 0 && identity_count[d->track_id] == 1;
            const bool calibrated = in.capture_path_lateral_measured && !region.empty() &&
                region.x == in.capture_path_px.x && region.width == in.capture_path_px.width;
            // The edge toward regions below the object is its bottom. Clipping the
            // opposite top at Y0 does not obscure this entering edge. Take the nearest
            // upper bound of every horizontally relevant review region, including a
            // configured dangerous class region, rather than clearing one region only.
            float upper_front = std::numeric_limits<float>::infinity();
            for (const auto& area : dangerous_regions)
                if (!area.empty() && d->box.x <= area.x + area.width && d->box.br().x >= area.x)
                    upper_front = std::min(upper_front, area.y);
            const bool upper_geometry = std::isfinite(upper_front) && d->center().y < upper_front;
            const float upper_penetration = d->box.br().y - upper_front;
            const float upper_jitter_px = c_.dangerous_upper_jitter_px * ref.image_height_px / 720.f;
            const bool bottom_visible = d->box.y >= 0 && d->box.br().y > 0 &&
                d->box.br().y < ref.image_height_px && d->box.x > 0 &&
                d->box.br().x < ref.image_width_px;
            const bool upper_shallow = overlap && upper_geometry && bottom_visible &&
                upper_penetration <= upper_jitter_px;
            const bool side_shallow = overlap && (right || left) && entering_edge_visible &&
                ratio <= c_.dangerous_side_overlap_ratio;
            const bool shallow = calibrated && valid_identity && (side_shallow || upper_shallow);
            auto found = dangerous_side_.find(d->track_id);
            if (!upper_geometry && !shallow && !overlap && found == dangerous_side_.end())
                continue;
            if (!valid_identity || !calibrated) {
                // Invalid objects keep the conservative whole-box rule.
                continue;
            }
            auto& side = dangerous_side_[d->track_id];
            const bool was_upper = side.upper;
            if (upper_geometry && !was_upper) {
                side.upper = true;
                side.pending = true;
                side.clear_frames = 0;
                side.clear_us = 0;
            }
            seen_side_ids.insert(d->track_id);
            const float scale = side.box.empty() ? 1.f : float(d->box.area()) / side.box.area();
            const bool continuous = side.box.empty() || (scale >= .5f && scale <= 2.f &&
                cv::norm(d->center() - cv::Point2f(side.box.x + .5f * side.box.width,
                    side.box.y + .5f * side.box.height)) <=
                    .5f * std::max({side.box.width, side.box.height, d->box.width, d->box.height}));
            const bool ordered = !side.sample_us || d->timestamp_us > side.sample_us;
            const bool complete_box = d->box.x > 0 && d->box.y > 0 &&
                d->box.br().x < ref.image_width_px && d->box.br().y < ref.image_height_px;
            const bool clear_edge_visible = side.upper ? upper_geometry && bottom_visible
                : entering_edge_visible || (!overlap && complete_box);
            const bool target_context = !side.upper || (in.target_valid && in.target_id >= 0 &&
                in.capture_path_valid && in.target_grasp_range_valid);
            const bool context = stopped && target_context && clear_edge_visible && continuous &&
                (side.pitch == kCameraPitchInvalid || side.pitch == in.camera_pitch_cdeg) &&
                (!side.now_us || now >= side.now_us) &&
                (!side.sample_us || (d->timestamp_us >= side.sample_us &&
                  d->timestamp_us - side.sample_us <= c_.max_age_us));
            // A shallow upper touch can retain, but cannot create, outside evidence.
            // Any invalid current observation revokes that retention immediately.
            const bool keep_upper_outside = was_upper && upper_shallow && !side.pending &&
                !side.risk && context && ordered;
            if (!context || !ordered || (overlap && !keep_upper_outside)) {
                side.clear_frames = 0;
                side.clear_us = 0;
                side.pending = true;
            }
            if (overlap && !shallow) {
                side.risk = true; // A single shallow frame cannot erase definite intrusion.
                side.pending = false;
            } else if (!overlap && context && ordered) {
                if (!side.clear_us)
                    side.clear_us = d->timestamp_us;
                ++side.clear_frames;
                if (side.clear_frames >= c_.dangerous_side_clear_frames &&
                    d->timestamp_us >= side.clear_us &&
                    d->timestamp_us - side.clear_us >= c_.dangerous_side_clear_us) {
                    side.risk = side.pending = false;
                }
            }
            side.sample_us = d->timestamp_us;
            side.now_us = now;
            side.box = d->box;
            side.pitch = in.camera_pitch_cdeg;
            for (auto& object : in.capture_image_objects)
                if (object.id == d->track_id && object.label == "dangerous_object") {
                    object.dangerous_side_pending = side.pending && (!side.risk || !overlap);
                    object.dangerous_side_clear = !side.pending && !side.risk;
                    object.dangerous_side_ratio = ratio;
                }
            if (side.pending && (!side.risk || !overlap)) {
                in.capture_dangerous_side_pending = true;
                in.capture_dangerous_pending_ids.push_back(d->track_id);
            }
            std::ostringstream detail;
            detail << "dangerous_id=" << d->track_id << ",side_ratio=" << ratio
                   << ",boundary=" << (side.upper ? "upper" : "side")
                   << ",bottom_y=" << d->box.br().y
                   << ",upper_penetration_px=" << upper_penetration
                   << ",upper_jitter=" << keep_upper_outside
                   << ",reset=" << (!stopped ? "sent_velocity_not_stopped"
                       : !target_context ? "target_invalid"
                       : !clear_edge_visible ? "entering_edge_unavailable"
                       : !continuous ? "identity_geometry_discontinuous"
                       : !ordered ? "image_not_new"
                       : !context ? "view_or_time_discontinuous"
                       : overlap && !keep_upper_outside ? "overlap" : "none")
                   << ",side_state=" << (side.risk ? "risk" : side.pending ? "pending" : "outside")
                   << ",clear_frames=" << side.clear_frames;
            in.capture_dangerous_side_detail += detail.str() + ";";
        }
        for (auto it = dangerous_side_.begin(); it != dangerous_side_.end();) {
            auto& side = it->second;
            if (!seen_side_ids.count(it->first)) {
                if (!side.pending && !side.risk && !side.upper) {
                    it = dangerous_side_.erase(it);
                    continue; // Preserve the existing lateral outside-neighbour lifetime.
                }
                side.clear_frames = 0;
                side.clear_us = 0;
                side.pending = true;
                in.capture_dangerous_side_pending = true;
                in.capture_dangerous_pending_ids.push_back(it->first);
            }
            ++it;
        }
        bool complete = in.pixel_grasp_target_valid, occlusion_free = true, followed = true;
        bool ambiguous = false;
        std::vector<const SegDetection*> frame_members;
        std::vector<const SegDetection*> members;
        std::map<int, std::vector<cv::Point2f>> next;
        if (view_ != -2 || !in.gripper_closed_observed)
            history_.clear();
        view_ = -2;
        bool layout_complete = true;
        for (const auto* d : fresh) {
            const auto center = d->center();
            // Split layout uses its own region; held evidence keeps the jaw calibration.
            const bool split_ordinary = labelOf(*d) == "ordinary_supply";
            if (split_ordinary)
                ++in.split_ordinary_visible_count;
            const auto range = std::find_if(c_.grasp_center_ranges.begin(), c_.grasp_center_ranges.end(),
                [&](const GraspCenterRange& r) { return r.label == labelOf(*d); });
            const bool layout_member = in.splitInventoryOverlaps(d->box, labelOf(*d));
            if (layout_member) {
                frame_members.push_back(d);
                if (split_ordinary)
                    ++in.split_ordinary_count;
                if (d->confidence < c_.min_confidence || d->box.x < 0 || d->box.y < 0 ||
                    d->box.br().x >= ref.image_width_px || d->box.br().y >= ref.image_height_px)
                    layout_complete = false;
            }
            const bool primary = d->track_id == in.target_id && d->label == in.label;
            // Use the same bounded regions and feature as review and split selection.
            // A neighbour below the calibrated rear bound must not start an empty retreat.
            const bool swept = primary || layout_member ||
                               in.capturePathOverlaps(d->box, labelOf(*d));
            if (swept) {
                in.corridor.add(labelOf(*d));
                members.push_back(d);
                if (d->box.x < 0 || d->box.y < 0 || d->box.br().x >= ref.image_width_px ||
                    d->box.br().y >= ref.image_height_px)
                    complete = false;
                CorridorEntry e;
                e.track_id = d->track_id; e.label = labelOf(*d); e.confidence = d->confidence;
                e.age_us = now - d->timestamp_us; e.counted = true; e.reason = "pixel_capture_corridor";
                corridor_diag_.push_back(e);
            }
            if (!in.gripper_closed_observed)
                continue;
            // Separation candidates outside the jaw must never become held cargo.
            const bool held = in.captureInventoryOverlaps(d->box, labelOf(*d)) ||
                              (range == c_.grasp_center_ranges.end() && swept);
            if (!held)
                continue;
            if (d->box.x < 0 || d->box.y < 0 || d->box.br().x >= ref.image_width_px ||
                d->box.br().y >= ref.image_height_px)
                ambiguous = true;
            in.held.add(labelOf(*d));
            in.held_objects.push_back({d->track_id, labelOf(*d), d->confidence});
            auto& history = next[d->track_id];
            if (const auto it = history_.find(d->track_id); it != history_.end())
                history = it->second;
            history.push_back(center);
            if (int(history.size()) > c_.follow_frames)
                history.erase(history.begin());
            bool still = d->track_id >= 0 && int(history.size()) >= c_.follow_frames;
            for (const auto& point : history)
                still = still && cv::norm(point - center) <= c_.follow_tolerance_px;
            followed = followed && still;
        }
        for (size_t i = 0; i < members.size(); ++i)
            for (size_t j = i + 1; j < members.size(); ++j)
                if (iou(members[i]->box, members[j]->box) > c_.occlusion_iou &&
                    !(labelOf(*members[i]) == "ordinary_supply" && labelOf(*members[j]) == "ordinary_supply"))
                    occlusion_free = false;
        int horizontal_pairs = 0;
        for (size_t i = 0; i < frame_members.size(); ++i)
            for (size_t j = i + 1; j < frame_members.size(); ++j) {
                const auto& a = *frame_members[i];
                const auto& b = *frame_members[j];
                if (labelOf(a) != "ordinary_supply" && labelOf(b) != "ordinary_supply")
                    continue;
                const auto delta = a.center() - b.center();
                if (std::abs(delta.y) > std::abs(delta.x))
                    ++in.split_vertical_pairs;
                else
                    ++horizontal_pairs;
            }
        in.split_layout_valid = layout_complete;
        in.split_layout_us = now;
        in.image_layout_vertical_dominant = in.split_vertical_pairs > horizontal_pairs;
        history_ = std::move(next);
        in.hold_observable = in.gripper_closed_observed;
        in.captured = in.hold_observable && in.held.total() > 0 && followed;
        in.held_complete = in.captured && !ambiguous && occlusion_free &&
                           (!in.capture_dangerous_side_pending || in.held.dangerous > 0);
        in.corridor_complete = complete && !in.capture_dangerous_side_pending;
        in.corridor_occlusion_free = in.corridor_complete && occlusion_free;
        return;
    }
    dangerous_side_.clear();
    dangerous_side_target_id_ = -1;
    // Holding region.
    const HoldingView* v = view(in.camera_pitch_cdeg, true);
    const int index = v ? int(v - c_.holding.data()) : -1;
    if (index != view_) {
        history_.clear();
        view_ = index;
    }
    bool ambiguous = false, followed = true;
    std::map<int, std::vector<cv::Point2f>> next;
    for (const auto* d : v ? fresh : std::vector<const SegDetection*>{}) {
        const auto& h = v->area;
        const cv::Rect& b = d->box;
        const auto c = d->center();
        bool inside = h.contains(float(b.x), float(b.y)) &&
                      h.contains(float(b.x + b.width), float(b.y + b.height));
        if (!v->polygon.empty()) {
            for (const cv::Point2f p : std::vector<cv::Point2f>{{float(b.x), float(b.y)},
                                                                {float(b.br().x), float(b.y)},
                                                                {float(b.x), float(b.br().y)},
                                                                {float(b.br().x), float(b.br().y)}})
                inside = inside && cv::pointPolygonTest(v->polygon, p, true) > c_.edge_margin_px;
        }
        const bool touches =
            b.x <= h.x2 + c_.edge_margin_px && b.x + b.width >= h.x1 - c_.edge_margin_px &&
            b.y <= h.y2 + c_.edge_margin_px && b.y + b.height >= h.y1 - c_.edge_margin_px;
        if (!touches)
            continue;
        // Measured contact ahead of the mouth: an object in front of the jaw (field 2026-10-05:
        // blocks at y~0.29-0.31 m crossed the 40deg region top while one block was held).
        if (d->ground_contact_valid && d->ground_position_valid && std::isfinite(d->body_xy_m.y) &&
            d->body_xy_m.y > c_.mouth_y_m + c_.outside_margin_m) {
            diag_.outside.push_back(d->track_id);
            continue;
        }
        if (!inside && !h.contains(c.x, c.y)) {
            ambiguous = true;
            diag_.ambiguous.push_back(d->track_id);
            continue;
        } // straddles from outside
        if (!inside) {
            ambiguous = true;
            diag_.ambiguous.push_back(d->track_id);
        } // centre inside but box crosses the edge
        if (b.y + b.height < v->min_visible_bottom_y_px) {
            ambiguous = true;
            diag_.ambiguous.push_back(d->track_id);
            continue;
        }
        in.held.add(labelOf(*d));
        in.held_objects.push_back({d->track_id, labelOf(*d), d->confidence});
        if (d->track_id < 0) {
            followed = false;
            diag_.unfollowed.push_back(-1);
            continue;
        }
        auto& track = next[d->track_id];
        const auto it = history_.find(d->track_id);
        if (it != history_.end())
            track = it->second;
        track.push_back(c);
        if (int(track.size()) > c_.follow_frames)
            track.erase(track.begin());
        bool still = int(track.size()) >= c_.follow_frames;
        for (const auto& p : track)
            if (cv::norm(p - c) > c_.follow_tolerance_px)
                still = false;
        if (!still) {
            followed = false;
            diag_.unfollowed.push_back(d->track_id);
        }
    }
    history_ = std::move(next);
    in.hold_observable = v != nullptr;
    in.captured = in.held.total() > 0 && followed;
    in.held_complete = in.captured && !ambiguous;

    // Rush corridor to the primary target.
    if (!in.target_valid || !in.geometry_valid || !std::isfinite(in.distance_m))
        return;
    const float reach = in.distance_m + c_.corridor_extra_m;
    const float c = std::cos(in.heading_error), s = std::sin(in.heading_error);
    bool complete = true, occlusion_free = true;
    std::vector<const SegDetection*> members;
    for (const auto* d : fresh) {
        CorridorEntry e;
        e.track_id = d->track_id;
        e.label = d->label;
        e.confidence = d->confidence;
        e.age_us = now - d->timestamp_us;
        if (!d->ground_position_valid || !std::isfinite(d->body_xy_m.x) ||
            !std::isfinite(d->body_xy_m.y)) {
            complete = false;
            e.reason = "no_ground_position";
            corridor_diag_.push_back(e);
            continue; // cannot tell whether it lies in the corridor
        }
        // Rotate into the rush direction: heading_error is the bearing, positive left.
        const float along = c * d->body_xy_m.y - s * d->body_xy_m.x;
        const float across = s * d->body_xy_m.y + c * d->body_xy_m.x;
        e.along_m = along;
        e.across_m = across;
        e.footprint_m = footprintHalf(labelOf(*d));
        const bool lateral_centre = std::abs(across) <= c_.corridor_half_width_m;
        const bool lateral_foot = std::abs(across) <= c_.corridor_half_width_m + e.footprint_m;
        if (along < -e.footprint_m || along > reach + e.footprint_m || !lateral_foot) {
            e.reason = along < -e.footprint_m          ? "behind"
                       : along > reach + e.footprint_m ? "beyond_reach"
                                                       : "outside_lateral";
            corridor_diag_.push_back(e);
            continue;
        }
        e.counted = true;
        e.footprint_only = !lateral_centre || along < 0 || along > reach;
        e.reason = "counted";
        corridor_diag_.push_back(e);
        in.corridor.add(labelOf(*d));
        members.push_back(d);
        // Cut by the image bottom: part of the object, or one behind it, may be unseen.
        if (d->box.y + d->box.height >= c_.image_height_px - 2 &&
            targetKind(labelOf(*d)) != TargetKind::ORDINARY)
            occlusion_free = false;
    }
    // Image overlap only matters for something that can really sit in or behind the corridor: a box
    // whose measured ground position is clearly off to the side (or well beyond reach) is a
    // neighbour, not a hidden object.
    const auto beside = [&](const SegDetection* d) {
        if (!d->ground_position_valid || !std::isfinite(d->body_xy_m.x) ||
            !std::isfinite(d->body_xy_m.y))
            return false;
        const float along = c * d->body_xy_m.y - s * d->body_xy_m.x;
        const float across = s * d->body_xy_m.y + c * d->body_xy_m.x;
        return std::abs(across) > c_.corridor_half_width_m + c_.occlusion_side_margin_m ||
               along > reach + c_.occlusion_side_margin_m;
    };
    for (size_t i = 0; i < members.size(); ++i)
        for (size_t j = 0; j < fresh.size(); ++j)
            if (fresh[j] != members[i] && !beside(fresh[j]) &&
                iou(members[i]->box, fresh[j]->box) > c_.occlusion_iou) {
                // Two overlapping ordinary supplies are just a pair; a hidden third is still caught
                // by the trip limit.
                const bool pair =
                    targetKind(labelOf(*members[i])) == TargetKind::ORDINARY &&
                    std::find(members.begin(), members.end(), fresh[j]) != members.end() &&
                    targetKind(labelOf(*fresh[j])) == TargetKind::ORDINARY;
                if (!pair)
                    occlusion_free = false;
            }
    in.corridor_complete = complete;
    in.corridor_occlusion_free = complete && occlusion_free;
}
} // namespace rescue
