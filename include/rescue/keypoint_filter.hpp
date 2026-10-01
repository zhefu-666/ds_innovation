#pragma once
#include "rescue/camera_calibration.hpp"
#include "rescue/frame_sensors.hpp"
#include "rescue/zone_geometry.hpp"
namespace rescue {
struct KeypointFilterConfig {
    float min_confidence=.60f, min_baseline_m=.10f;
    float spacing_absolute_m=.025f, spacing_relative=.10f;
    float max_ground_sigma_m=.04f, pixel_sigma=1.5f, max_range_m=3.0f;
    uint64_t max_skew_us=50000, max_frame_age_us=200000;
};
struct GroundKeypoint {
    int id=-1;cv::Point2f pixel, zone_m, body_m;
    float sigma_m=0,confidence=0;
};
struct FilteredKeypoints {
    bool valid=false;
    std::string reason="no_observation";
    std::vector<GroundKeypoint> points;
};
class KeypointFilter {
public:
    explicit KeypointFilter(KeypointFilterConfig config={}) : config_(config) {}
    std::string frameRejection(const GeometryFrame&, const CameraCalibration&) const;
    FilteredKeypoints filter(const KeypointFrame&, const GeometryFrame&,
                             const ZoneGeometry&, const CameraCalibration&) const;
private:
    KeypointFilterConfig config_;
};
} // namespace rescue
