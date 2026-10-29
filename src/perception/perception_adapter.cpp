#include "rescue/perception/perception_adapter.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
namespace rescue {
PushObservation makePushObservation(const std::vector<SegDetection>& detections,
                                    uint64_t now,
                                    bool first,
                                    float threshold,
                                    int locked_id,
                                    const std::vector<int>& rejected,
                                    bool search,
                                    bool locked_is_search_cue,
                                    bool require_geometry,
                                    const std::string& prefer_label) {
    PushObservation in;
    in.now_us = now;
    const SegDetection* best = nullptr;
    const auto rank = [&](const SegDetection& d) {
        if (!prefer_label.empty() && d.label == prefer_label)
            return -1;
        if (!first)
            return d.label == "ordinary_supply" ? 0 : 1;
        return d.label == "core_supply"       ? 0
               : d.label == "ordinary_supply" ? 1
               : d.label == "injured_person"  ? 2
                                              : 3;
    };
    const auto distance = [](const SegDetection& d) {
        return d.ground_position_valid ? cv::norm(d.body_xy_m)
                                       : std::numeric_limits<double>::infinity();
    };
    const bool locked_green = std::any_of(detections.begin(), detections.end(), [&](const auto& d) {
        return d.track_id == locked_id && d.label == "ordinary_supply";
    });
    const bool allow_green_preemption = search && locked_is_search_cue && !first && !locked_green;
    for (const auto& d : detections) {
        if ((locked_id >= 0 && d.track_id != locked_id &&
             !(allow_green_preemption && d.label == "ordinary_supply")) ||
            std::find(rejected.begin(), rejected.end(), d.track_id) != rejected.end())
            continue;
        if (d.injured_upright_rejected)
            continue; // on-end injured block cannot be gripped sideways
        if ((search || require_geometry) &&
            (!d.ground_position_valid || !d.ground_contact_valid || !std::isfinite(d.body_xy_m.x) ||
             !std::isfinite(d.body_xy_m.y) || d.body_xy_m.y <= 0))
            continue;
        if (search ? targetKind(d.label) == TargetKind::UNKNOWN : !targetSelectable(d.label, first))
            continue;
        if (d.track_id < 0 || !std::isfinite(d.confidence) || d.confidence < threshold ||
            !d.timestamp_us || now < d.timestamp_us || now - d.timestamp_us > 200000 ||
            d.box.width <= 0 || d.box.height <= 0)
            continue;
        if (!best || rank(d) < rank(*best) ||
            (rank(d) == rank(*best) &&
             (distance(d) < distance(*best) ||
              (distance(d) == distance(*best) && d.confidence > best->confidence))))
            best = &d;
    }
    if (best) {
        in.target_valid = true;
        in.target_id = best->track_id;
        in.label = best->label;
        in.target_is_search_cue = search;
    }
    return in;
}
void applySplitRetreatBox(PushObservation& in,
                          const std::vector<SegDetection>& detections,
                          cv::Size image_size,
                          uint64_t now,
                          float threshold,
                          int preferred_id) {
    in.split_ordinary_box = {};
    in.split_ordinary_box_us = 0;
    in.split_ordinary_box_id = -1;
    in.split_image_width_px = image_size.width;
    if (image_size.width <= 0 || image_size.height <= 0)
        return;
    const cv::Rect image(0, 0, image_size.width, image_size.height);
    const SegDetection* best = nullptr;
    const auto offset = [&](const SegDetection& d) {
        return std::abs(d.box.x + .5f * d.box.width - .5f * image_size.width);
    };
    for (const auto& d : detections) {
        if (d.label != "ordinary_supply" || !std::isfinite(d.confidence) ||
            d.confidence < threshold || d.confidence > 1 || !d.timestamp_us ||
            d.timestamp_us > now || now - d.timestamp_us > 200000 || d.box.width <= 0 ||
            d.box.height <= 0 || (d.box & image).empty())
            continue;
        const bool preferred = preferred_id >= 0 && d.track_id == preferred_id;
        const bool best_preferred = best && preferred_id >= 0 && best->track_id == preferred_id;
        if (!best || (preferred && !best_preferred) ||
            (preferred == best_preferred && offset(d) < offset(*best)))
            best = &d;
    }
    if (best) {
        in.split_ordinary_box = best->box & image;
        in.split_ordinary_box_us = best->timestamp_us;
        in.split_ordinary_box_id = best->track_id;
    }
}
std::vector<int> targetsNearOwnZone(const std::vector<SegDetection>& detections,
                                    const std::string& team,
                                    cv::Size image_size,
                                    uint64_t now,
                                    float threshold) {
    std::vector<int> rejected;
    if ((team != "blue" && team != "red") || image_size.width <= 0 || image_size.height <= 0)
        return rejected;
    const cv::Rect image(0, 0, image_size.width, image_size.height);
    const double gap = kTargetZoneGapImageFrac * std::min(image_size.width, image_size.height);
    const auto usable = [&](const SegDetection& d) {
        return std::isfinite(d.confidence) && d.confidence >= threshold && d.confidence <= 1 &&
               d.timestamp_us && now >= d.timestamp_us && now - d.timestamp_us <= 200000 &&
               d.box.width > 0 && d.box.height > 0;
    };
    std::vector<cv::Rect> zones;
    for (const auto& d : detections)
        if (d.label == team + "_safe_zone" && usable(d)) {
            const auto box = d.box & image;
            if (!box.empty())
                zones.push_back(box);
        }
    if (zones.empty())
        return rejected;
    for (const auto& d : detections) {
        const auto kind = targetKind(d.label);
        if ((kind != TargetKind::ORDINARY && kind != TargetKind::CORE &&
             kind != TargetKind::INJURED) ||
            d.track_id < 0 || !usable(d))
            continue;
        const auto box = d.box & image;
        if (box.empty())
            continue;
        for (const auto& zone : zones) {
            const double dx =
                std::max({0, box.x - (zone.x + zone.width), zone.x - (box.x + box.width)});
            const double dy =
                std::max({0, box.y - (zone.y + zone.height), zone.y - (box.y + box.height)});
            if (std::hypot(dx, dy) <= gap) {
                rejected.push_back(d.track_id);
                break;
            }
        }
    }
    return rejected;
}
bool applyZoneDetectionSight(PushObservation& in,
                             const std::vector<SegDetection>& detections,
                             const std::string& team, cv::Size image_size,
                             const cv::Mat& k, int16_t pitch_cdeg,
                             bool pitch_stable, uint64_t now, float threshold) {
    in.zone_sight_valid = in.zone_sight_enemy = in.enemy_zone_seen = false;
    in.zone_sight_bearing = 0;
    if ((team != "blue" && team != "red") || !pitch_stable ||
        pitch_cdeg == kCameraPitchInvalid || image_size.width <= 0 || image_size.height <= 0 ||
        k.type() != CV_64F || k.rows != 3 || k.cols != 3)
        return false;
    const double fx = k.at<double>(0,0), fy = k.at<double>(1,1),
                 cx = k.at<double>(0,2), cy = k.at<double>(1,2);
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
        !std::isfinite(cx) || !std::isfinite(cy))
        return false;
    const SegDetection* best = nullptr;
    const std::string own = team + "_safe_zone";
    for (const auto& d : detections) {
        if ((d.label != "blue_safe_zone" && d.label != "red_safe_zone") ||
            !std::isfinite(d.confidence) || d.confidence < threshold || d.confidence > 1 ||
            !d.timestamp_us || now < d.timestamp_us || now - d.timestamp_us > 200000 ||
            d.box.width <= 0 || d.box.height <= 0)
            continue;
        const cv::Point2f center(d.box.x + d.box.width * .5f, d.box.y + d.box.height * .5f);
        if (center.x < 0 || center.y < 0 || center.x >= image_size.width || center.y >= image_size.height)
            continue;
        if (!best || (d.label == own && best->label != own) ||
            (d.label == best->label && d.confidence > best->confidence))
            best = &d;
    }
    if (!best)
        return false;
    const cv::Point2f center(best->box.x + best->box.width * .5f,
                             best->box.y + best->box.height * .5f);
    const double x = (center.x - cx) / fx, y = (center.y - cy) / fy;
    const double pitch = double(pitch_cdeg) * .01 * CV_PI / 180.;
    const double forward = std::cos(pitch) - y * std::sin(pitch);
    if (!std::isfinite(forward) || forward <= .05)
        return false;
    in.zone_sight_valid = true;
    in.zone_sight_enemy = in.enemy_zone_seen = best->label != own;
    in.zone_sight_bearing = float(std::atan2(-x, forward));
    return true;
}
} // namespace rescue
