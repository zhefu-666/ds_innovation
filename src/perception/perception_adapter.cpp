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
                                    const std::string& prefer_label, float ui140_core_confidence) {
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
        if (d.track_id < 0 || !std::isfinite(d.confidence) || d.confidence < (d.label == "core_supply" ? std::min(threshold, ui140_core_confidence) : threshold) ||
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
void applyStartupSight(PushObservation& in, const std::vector<SegDetection>& detections,
                       cv::Size size, uint64_t now, float threshold, int preferred_id,
                       bool ordinary_only, int rejected_id, const StartupSightReference& reference) {
    in.startup_rebound_from = -1;
    in.startup_visible_classes = 0;
    in.startup_visible_count = 0;
    in.startup_scene_us = 0;
    in.target_valid = in.geometry_valid = in.target_center_px_valid = false;
    in.pixel_grasp_target_valid = in.pixel_view_handoff_ready = false;
    in.pixel_grasp_sample_us = 0;
    in.target_id = -1;
    in.label.clear();
    if (size.width <= 0 || size.height <= 0 || !in.camera_pitch_stable)
        return;
    const cv::Rect image(0, 0, size.width, size.height);
    const SegDetection* best = nullptr;
    const SegDetection* rebound = nullptr;
    int matches = 0;
    const bool can_rebind = preferred_id >= 0 && reference.id == preferred_id &&
        !reference.box.empty() && reference.pitch == in.camera_pitch_cdeg &&
        reference.sample_us && now >= reference.sample_us && now - reference.sample_us <= kStartupTargetLossHoldUs;
    for (const auto& d : detections) {
        const unsigned bit = d.label == "ordinary_supply" ? 1u : d.label == "core_supply" ? 2u
                             : d.label == "injured_person" ? 4u
                             : d.label == "dangerous_object" ? 8u : 0u;
        if (!bit || !std::isfinite(d.confidence) || d.confidence < threshold ||
            !d.timestamp_us || now < d.timestamp_us || now - d.timestamp_us > 200000 ||
            (d.box & image).empty())
            continue;
        in.startup_visible_classes |= bit;
        in.startup_scene_us = std::max(in.startup_scene_us, d.timestamp_us);
        ++in.startup_visible_count;
        if (d.track_id < 0 || d.track_id == rejected_id ||
            (ordinary_only && d.label != "ordinary_supply"))
            continue;
        if (preferred_id >= 0 && d.track_id != preferred_id) {
            if (can_rebind && d.label == reference.label) {
                const double overlap = (d.box & reference.box).area();
                const double total = double(d.box.area()) + reference.box.area() - overlap;
                const cv::Point2f delta(d.box.x + .5f * d.box.width -
                                           reference.box.x - .5f * reference.box.width,
                                       d.box.y + .5f * d.box.height -
                                           reference.box.y - .5f * reference.box.height);
                // The logged ID changes resize or clip a box at the image edge.
                // Accept a unique same-class overlap, bounded by image and box scale.
                const float box_diagonal = std::max(std::hypot(float(d.box.width), float(d.box.height)),
                    std::hypot(float(reference.box.width), float(reference.box.height)));
                const float center_limit = std::max(.05f * size.width, .25f * box_diagonal);
                if (total > 0 && overlap / total >= .45 && cv::norm(delta) <= center_limit) {
                    rebound = &d;
                    ++matches;
                }
            }
            continue;
        }
        // Image-space nearest proxy: greatest centre Y in the same settled view.
        // Closed guide approach compares all classes; ordinary search/approach filters by class.
        const float y = d.box.y + .5f * d.box.height;
        const float best_y = best ? best->box.y + .5f * best->box.height : -1.f;
        if (!best || y > best_y || (y == best_y && d.track_id == preferred_id))
            best = &d;
    }
    if (!best && matches == 1) {
        best = rebound;
        in.startup_rebound_from = preferred_id;
    }
    if (!best)
        return;
    in.target_valid = in.target_center_px_valid = true;
    in.target_id = best->track_id;
    in.label = best->label;
    in.target_box = best->box;
    in.target_center_px = {best->box.x + .5f * best->box.width,
                           best->box.y + .5f * best->box.height};
    in.pixel_grasp_sample_us = best->timestamp_us;
    in.pixel_grasp_error_px = .5f * size.width - in.target_center_px.x;
    in.pixel_view_handoff_ready = in.target_center_px.y >= .75f * size.height;
    in.pixel_grasp_target_valid = in.pixel_grasp_view;
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
void applyDeliveryBoxEvidence(PushObservation& in,
                              const std::vector<SegDetection>& detections,
                              const std::string& team, cv::Size size,
                              bool stable, uint64_t now, float threshold) {
    in.delivery_observed = false;
    in.delivery_box_count = 0;
    in.delivery_box_sample_us = 0;
    if (!stable || (team != "blue" && team != "red") || size.width <= 0 || size.height <= 0)
        return;
    const cv::Rect image(0, 0, size.width, size.height);
    const auto usable = [&](const SegDetection& d) {
        return std::isfinite(d.confidence) && d.confidence >= threshold && d.confidence <= 1 &&
               d.timestamp_us && now >= d.timestamp_us && now - d.timestamp_us <= 200000 &&
               !d.box.empty() && (d.box & image) == d.box;
    };
    const std::string own = team + "_safe_zone";
    for (const auto& z : detections)
        if (z.label == own && usable(z))
            in.delivery_box_sample_us = std::max(in.delivery_box_sample_us, z.timestamp_us);
    if (!in.delivery_box_sample_us)
        return;
    for (const auto& d : detections) {
        if (d.label != "ordinary_supply" || !usable(d) ||
            d.timestamp_us != in.delivery_box_sample_us)
            continue;
        for (const auto& z : detections)
            if (z.label == own && usable(z) && z.timestamp_us == d.timestamp_us &&
                (d.box & z.box) == d.box) {
                ++in.delivery_box_count;
                break; // One cargo is counted once even if zone boxes overlap.
            }
    }
    in.delivery_observed = in.delivery_box_count > 0;
}

bool applyZoneDetectionSight(PushObservation& in,
                             const std::vector<SegDetection>& detections,
                             const std::string& team, cv::Size image_size,
                             const cv::Mat& k, int16_t pitch_cdeg,
                             bool pitch_stable, uint64_t now, float threshold) {
    in.zone_sight_valid = in.zone_sight_enemy = in.enemy_zone_seen = false;
    in.zone_sight_bearing = 0;
    in.zone_sight_box = {};
    in.zone_sight_image_size = {};
    in.zone_sight_sample_us = 0;
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
    in.zone_sight_box = best->box;
    in.zone_sight_image_size = image_size;
    in.zone_sight_sample_us = best->timestamp_us;
    return true;
}

void applySubzoneView(PushObservation& in, const ZoneHalves& halves,
                      const ZoneGeometry& geometry, cv::Size size, bool stable,
                      bool own_identity, float supply_x, float injured_x, float min_confidence, uint64_t sample_us) {
    in.supply_view = {}; in.injured_view = {}; in.subzone_view_us = 0;
    if (!stable || !own_identity || size.width<=1 || size.height<=1 ||
        !geometry.confirmed || !std::isfinite(geometry.width_m) ||
        geometry.width_m<=.1f || geometry.width_m>=5.f || !sample_us || sample_us>in.now_us ||
        in.now_us-sample_us>200000) return;
    const auto measure = [&](float target, PushObservation::SubzoneView& out) {
        if (!std::isfinite(target) || std::abs(target)<1e-6f) return;
        const auto& half=halves[target<0 ? 0 : 1];
        if(!half.valid || !std::isfinite(half.score) || half.score<=0) return;
        for(size_t i=0;i<half.keypoints.size();++i) {
            const auto& p=half.keypoints[i];
            if(!std::isfinite(half.confidence[i]) || half.confidence[i]<min_confidence ||
                !std::isfinite(p.x) || !std::isfinite(p.y) ||
                p.x<0 || p.x>=size.width || p.y<0 || p.y>=size.height) return;
        }
        const auto& fl=half.keypoints[0]; const auto& nl=half.keypoints[1];
        const auto& fr=half.keypoints[2]; const auto& nr=half.keypoints[3];
        const float far_width=fr.x-fl.x, near_width=nr.x-nl.x;
        // Connect the observed rectangle, not a fitted ground/PnP pose.
        // Reversed/collapsed corners cannot define an entrance direction.
        if(far_width<2 || near_width<2 || nl.y-fl.y<2 || nr.y-fr.y<2) return;
        const float edge_slope=((fr.y-fl.y)+(nr.y-nl.y))/(far_width+near_width);
        // Each model half already has its own divider-side corners. Do not
        // require the other half just to manufacture merged divider landmarks.
        const float left=(half.keypoints[0].x+half.keypoints[1].x)*.5f;
        const float right=(half.keypoints[2].x+half.keypoints[3].x)*.5f;
        if(right-left<2) return;
        // Interpolate the locked target's x within its observed half, not the whole bbox.
        const float half_width=geometry.width_m*.5f;
        const float local=target<0 ? (target+half_width)/half_width : target/half_width;
        if(local<=0 || local>=1) return;
        const float error=left+local*(right-left)-size.width*.5f;
        out.valid=true; out.target_x=target; out.error_fraction=error/size.width;
        out.edge_slope=edge_slope;
        // This scale is a local steering estimate only; final acceptance is in pixels.
        out.lateral_m=error/(right-left)*half_width;
        out.inside=left<size.width*.5f && right>size.width*.5f;
    };
    measure(supply_x,in.supply_view);measure(injured_x,in.injured_view);
    in.subzone_view_us=sample_us;
}
} // namespace rescue
