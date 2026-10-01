#pragma once
#include "rescue/zone_estimate.hpp"
#include <array>
namespace rescue {
// Fixed six-landmark schema, row-major:
// 0 front-left, 1 front-divider-centre, 2 front-right,
// 3 rear-left, 4 rear-divider-centre, 5 rear-right.
// Zone frame: origin = inner front boundary at the divider centre on the floor,
// x right facing entry, y into the zone, z up. width_m/depth_m are the INNER
// floor (delivery bounds). The landmarks are a separate rectangle
// landmark_width_m x landmark_depth_m at height landmark_height_m, starting at
// y=0 (legacy v1: the inner floor corners themselves, height 0).
struct ZoneGeometry {
    std::string id, label;
    float width_m = 0, depth_m = 0;
    float landmark_width_m = 0, landmark_depth_m = 0, landmark_height_m = 0;
    float divider_exclusion_half_width_m = .02f; // conservative policy, NOT a measured divider width
    bool supply_left = true, confirmed = false;
    bool valid() const;
    // Landmark xy in the zone frame with z=0 relative to the landmark plane
    // (planar for IPPE); the plane itself is at z=landmark_height_m.
    std::array<cv::Point3f,6> objectPoints() const;
    static ZoneGeometry load(const std::string& file, const std::string& label);
};
struct ZoneKeypoint {
    int id = -1;
    cv::Point2f pixel;
    float confidence = 0;
    bool visible = false;
};
struct KeypointFrame {
    std::string zone_label, geometry_id;
    uint64_t frame_id = 0, capture_us = 0;
    cv::Size image_size;
    std::vector<ZoneKeypoint> points;
};
} // namespace rescue
