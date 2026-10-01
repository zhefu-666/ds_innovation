#include "rescue/capture_monitor.hpp"
#include <cmath>
#include <cstdlib>

namespace rescue {
namespace {
float iou(const cv::Rect &a, const cv::Rect &b) {
    const float inter = float((a & b).area());
    const float uni = float(a.area() + b.area()) - inter;
    return uni > 0 ? inter / uni : 0.f;
}
}
const HoldingView *CaptureMonitor::view(int16_t pitch, bool stable) const {
    if (!stable || pitch == kCameraPitchInvalid) return nullptr;
    for (const auto &v : c_.holding)
        if (std::abs(int(pitch) - int(v.pitch_cdeg)) <= c_.pitch_tolerance_cdeg) return &v;
    return nullptr;
}
void CaptureMonitor::update(PushObservation &in, const std::vector<SegDetection> &detections, uint64_t now) {
    in.held = {}; in.captured = false; in.held_complete = false; in.hold_observable = false;
    in.corridor = {}; in.corridor_complete = false; in.corridor_occlusion_free = false;
    // Image positions are only comparable at one fixed, settled camera pitch.
    if (!in.camera_pitch_stable || in.camera_pitch_cdeg == kCameraPitchInvalid) { reset(); return; }
    std::vector<const SegDetection *> fresh;
    for (const auto &d : detections) {
        if (d.timestamp_us == 0 || now < d.timestamp_us || now - d.timestamp_us > c_.max_age_us) continue;
        if (!std::isfinite(d.confidence) || d.confidence < c_.noise_confidence) continue;
        if (d.box.width <= 0 || d.box.height <= 0) continue;
        fresh.push_back(&d);
    }
    const auto labelOf = [&](const SegDetection &d) {
        return d.confidence >= c_.min_confidence ? d.label : std::string("unknown");
    };
    // Holding region.
    const HoldingView *v = view(in.camera_pitch_cdeg, true);
    const int index = v ? int(v - c_.holding.data()) : -1;
    if (index != view_) { history_.clear(); view_ = index; }
    bool ambiguous = false, followed = true;
    std::map<int, std::vector<cv::Point2f>> next;
    for (const auto *d : v ? fresh : std::vector<const SegDetection *>{}) {
        const auto &h = v->area;
        const cv::Rect &b = d->box;
        const auto c = d->center();
        const bool inside = h.contains(float(b.x), float(b.y)) && h.contains(float(b.x + b.width), float(b.y + b.height));
        const bool touches = b.x <= h.x2 + c_.edge_margin_px && b.x + b.width >= h.x1 - c_.edge_margin_px &&
                             b.y <= h.y2 + c_.edge_margin_px && b.y + b.height >= h.y1 - c_.edge_margin_px;
        if (!touches) continue;
        if (!inside && !h.contains(c.x, c.y)) { ambiguous = true; continue; } // straddles from outside
        if (!inside) ambiguous = true; // centre inside but box crosses the edge
        in.held.add(labelOf(*d));
        if (d->track_id < 0) { followed = false; continue; }
        auto &track = next[d->track_id];
        const auto it = history_.find(d->track_id);
        if (it != history_.end()) track = it->second;
        track.push_back(c);
        if (int(track.size()) > c_.follow_frames) track.erase(track.begin());
        if (int(track.size()) < c_.follow_frames) followed = false;
        for (const auto &p : track)
            if (cv::norm(p - c) > c_.follow_tolerance_px) followed = false;
    }
    history_ = std::move(next);
    in.hold_observable = v != nullptr;
    in.captured = in.held.total() > 0 && followed;
    in.held_complete = in.captured && !ambiguous;

    // Rush corridor to the primary target.
    if (!in.target_valid || !in.geometry_valid || !std::isfinite(in.distance_m)) return;
    const float reach = in.distance_m + c_.corridor_extra_m;
    const float c = std::cos(in.heading_error), s = std::sin(in.heading_error);
    bool complete = true, occlusion_free = true;
    std::vector<const SegDetection *> members;
    for (const auto *d : fresh) {
        if (!d->ground_position_valid || !std::isfinite(d->body_xy_m.x) || !std::isfinite(d->body_xy_m.y)) {
            complete = false; continue; // cannot tell whether it lies in the corridor
        }
        // Rotate into the rush direction: heading_error is the bearing, positive left.
        const float along = c * d->body_xy_m.y - s * d->body_xy_m.x;
        const float across = s * d->body_xy_m.y + c * d->body_xy_m.x;
        if (along < 0 || along > reach || std::abs(across) > c_.corridor_half_width_m) continue;
        in.corridor.add(labelOf(*d));
        members.push_back(d);
        // Cut by the image bottom: part of the object, or one behind it, may be unseen.
        if (d->box.y + d->box.height >= c_.image_height_px - 2) occlusion_free = false;
    }
    for (size_t i = 0; i < members.size(); ++i)
        for (size_t j = 0; j < fresh.size(); ++j)
            if (fresh[j] != members[i] && iou(members[i]->box, fresh[j]->box) > c_.occlusion_iou) occlusion_free = false;
    in.corridor_complete = complete;
    in.corridor_occlusion_free = complete && occlusion_free;
}
} // namespace rescue
