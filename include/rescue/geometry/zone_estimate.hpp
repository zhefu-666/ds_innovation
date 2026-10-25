#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <limits>
namespace rescue {
float wrapAngle(float angle);
struct ZoneGate {
    int min_points = 2;
    float max_residual_m = .03f, max_position_sigma_m = .05f;
    float max_yaw_sigma_rad = .15f, max_prediction_distance_m = .30f;
    uint64_t max_age_us = 200000, max_prediction_age_us = 350000;
};
struct ZoneEstimate {
    enum class Source { NONE, MULTI_POINT, TWO_POINT, PREDICTED };
    bool valid = false;
    Source source = Source::NONE;
    std::string zone_label, geometry_id, reason = "no_observation";
    uint64_t frame_id = 0, timestamp_us = 0, observed_us = 0;
    cv::Point2f origin_body_m; // zone origin in body ground frame: x right, y forward
    float yaw_body_rad = 0; // p_body = R(yaw) p_zone + origin; CCW positive
    std::vector<int> inlier_ids;
    float residual_m = std::numeric_limits<float>::infinity();
    float position_sigma_m = std::numeric_limits<float>::infinity();
    float yaw_sigma_rad = std::numeric_limits<float>::infinity();
    float predicted_distance_m = 0; // accumulated absolute distance, never net displacement
    bool pnp_checked = false, pnp_consistent = false;
    bool trusted(uint64_t now_us, const ZoneGate& gate = {}) const;
    cv::Point2f bodyToZone(const cv::Point2f& point) const;
    cv::Point2f zoneToBody(const cv::Point2f& point) const;
};
} // namespace rescue
