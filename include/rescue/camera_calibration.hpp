#pragma once

#include "rescue/types.hpp"

#include <opencv2/calib3d.hpp>

#include <string>

namespace rescue {

class CameraCalibration {
public:
    bool load(const std::string &path);
    void setIntrinsics(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);
    void setGroundHomography(const cv::Mat &homography);

    bool valid() const;
    bool pixelToGround(const cv::Point2f &pixel, cv::Point2f &ground_m,
                       const SensorState *sensor = nullptr) const;
    bool detectionToBody(SegDetection &detection, const SensorState &sensor) const;
    bool tiltTrusted(const SensorState &sensor) const;

    const cv::Mat &cameraMatrix() const { return camera_matrix_; }
    const cv::Mat &distCoeffs() const { return dist_coeffs_; }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat ground_homography_;
    bool undistorted_ground_ = false;
    float tilt_limit_deg_ = 12.0f;
};

} // namespace rescue
