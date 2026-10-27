#include "rescue/perception/capture_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
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
                            uint64_t now) {
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
    // Image positions are only comparable at one fixed, settled camera pitch.
    if (!in.camera_pitch_stable || in.camera_pitch_cdeg == kCameraPitchInvalid) {
        reset();
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
    const auto labelOf = [&](const SegDetection& d) {
        return d.confidence >= c_.min_confidence ? d.label : std::string("unknown");
    };
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
