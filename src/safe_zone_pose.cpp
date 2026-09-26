#include "rescue/safe_zone_pose.hpp"

#include <cmath>

namespace rescue {

SafeZonePoseEstimator::SafeZonePoseEstimator(cv::Mat camera_matrix, cv::Mat dist_coeffs,
                                             float max_reprojection_error_px,
                                             float max_pose_jump_m)
    : camera_matrix_(std::move(camera_matrix)), dist_coeffs_(std::move(dist_coeffs)),
      max_error_px_(max_reprojection_error_px), max_jump_m_(max_pose_jump_m) {}

void SafeZonePoseEstimator::reset() { previous_ = SafeZonePose{}; }

SafeZonePose SafeZonePoseEstimator::estimate(const SafeZoneObservation &observation) {
    SafeZonePose result;
    result.label = observation.label;
    if (camera_matrix_.empty() || observation.image_points.size() < 4 ||
        observation.image_points.size() != observation.object_points.size()) {
        result.view = SafeZonePose::PARTIAL;
        return result;
    }

    if (camera_matrix_.type() != CV_64F) camera_matrix_.convertTo(camera_matrix_, CV_64F);
    if (!dist_coeffs_.empty() && dist_coeffs_.type() != CV_64F) dist_coeffs_.convertTo(dist_coeffs_, CV_64F);
    cv::Mat rvec, tvec, inliers;
    if (!cv::solvePnPRansac(observation.object_points, observation.image_points,
                            camera_matrix_, dist_coeffs_, rvec, tvec, false,
                            100, max_error_px_, 0.99, inliers, cv::SOLVEPNP_ITERATIVE) ||
        inliers.rows < 4) {
        result.view = SafeZonePose::PARTIAL;
        return result;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(observation.object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
    double error_sum = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        error_sum += cv::norm(projected[i] - observation.image_points[i]);
    }
    result.reprojection_error_px = static_cast<float>(error_sum / projected.size());
    result.distance_m = static_cast<float>(cv::norm(tvec));
    result.heading_error_deg = static_cast<float>(std::atan2(tvec.at<double>(0), tvec.at<double>(2)) * 180.0 / CV_PI);
    result.view = std::abs(result.heading_error_deg) < 15.0f ? SafeZonePose::FRONT :
                  std::abs(result.heading_error_deg) < 55.0f ? SafeZonePose::OBLIQUE : SafeZonePose::SIDE;
    if (!observation.has_divider) result.view = SafeZonePose::PARTIAL;
    if (!std::isfinite(result.reprojection_error_px) || result.reprojection_error_px > max_error_px_ ||
        (previous_.valid && cv::norm(tvec - previous_.tvec) > max_jump_m_)) {
        result.view = SafeZonePose::UNKNOWN;
        return result;
    }
    result.rvec = rvec;
    result.tvec = tvec;
    // A side/oblique pose is useful for discovery, but the state machine only
    // permits delivery after it has obtained a FRONT observation.
    result.valid = result.view != SafeZonePose::UNKNOWN && result.view != SafeZonePose::PARTIAL;
    previous_ = result;
    return result;
}

} // namespace rescue
