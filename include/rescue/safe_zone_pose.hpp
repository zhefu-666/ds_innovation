#pragma once

#include "rescue/types.hpp"

#include <opencv2/calib3d.hpp>

#include <vector>

namespace rescue {

struct SafeZoneObservation {
    std::string label;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;
    bool has_divider = false; // legacy diagnostic, never a validity gate
    cv::Mat reference_rvec, reference_tvec; // optional independent ground-fit cross-check reference
};

class SafeZonePoseEstimator {
public:
    SafeZonePoseEstimator(cv::Mat camera_matrix, cv::Mat dist_coeffs,
                          float max_reprojection_error_px = 4.0f,
                          float max_pose_jump_m = 0.35f);

    SafeZonePose estimate(const SafeZoneObservation &observation, uint64_t timestamp_us = 0);
    void reset();

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    float max_error_px_;
    float max_jump_m_;
    SafeZonePose previous_, pending_;
    uint64_t previous_us_ = 0, last_us_ = 0, pending_us_ = 0;
    int pending_count_ = 0;
};

} // namespace rescue
