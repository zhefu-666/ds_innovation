#include "rescue/camera_calibration.hpp"

#include <cmath>
#include <vector>

namespace rescue {

bool CameraCalibration::load(const std::string &path) {
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) return false;
    file["camera_matrix"] >> camera_matrix_;
    file["dist_coeffs"] >> dist_coeffs_;
    file["ground_homography"] >> ground_homography_;
    std::string domain;
    if (!file["ground_pixel_domain"].empty()) file["ground_pixel_domain"] >> domain;
    if (!domain.empty() && domain != "undistorted_pixels") return false;
    undistorted_ground_ = domain == "undistorted_pixels";
    double limit = tilt_limit_deg_;
    if (!file["imu_tilt_limit_deg"].empty()) file["imu_tilt_limit_deg"] >> limit;
    tilt_limit_deg_ = static_cast<float>(limit);
    return valid();
}

void CameraCalibration::setIntrinsics(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) {
    camera_matrix.copyTo(camera_matrix_);
    dist_coeffs.copyTo(dist_coeffs_);
}

void CameraCalibration::setGroundHomography(const cv::Mat &homography) {
    homography.copyTo(ground_homography_);
    undistorted_ground_ = false; // Legacy setter accepts a raw-pixel mapping.
}

bool CameraCalibration::valid() const {
    return !camera_matrix_.empty() && camera_matrix_.rows == 3 && camera_matrix_.cols == 3 &&
           !ground_homography_.empty() && ground_homography_.rows == 3 &&
           ground_homography_.cols == 3;
}

bool CameraCalibration::tiltTrusted(const SensorState &sensor) const {
    const float limit_rad = tilt_limit_deg_ * (3.14159265358979323846f / 180.0f);
    return sensor.imu_valid && std::abs(sensor.pitch_rad) <= limit_rad &&
           std::abs(sensor.roll_rad) <= limit_rad;
}

bool CameraCalibration::pixelToGround(const cv::Point2f &pixel, cv::Point2f &ground_m,
                                      const SensorState *sensor) const {
    if (!valid() || (sensor != nullptr && !tiltTrusted(*sensor))) return false;
    std::vector<cv::Point2f> source{pixel};
    if (undistorted_ground_) {
        if (dist_coeffs_.empty()) return false;
        cv::undistortPoints(source, source, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
    }
    std::vector<cv::Point2f> target;
    cv::perspectiveTransform(source, target, ground_homography_);
    if (target.empty() || !std::isfinite(target[0].x) || !std::isfinite(target[0].y)) return false;
    ground_m = target[0];
    return true;
}

bool CameraCalibration::detectionToBody(SegDetection &detection, const SensorState &sensor) const {
    const cv::Point2f pixel = detection.ground_point_px == cv::Point2f() ?
        cv::Point2f(detection.box.x + detection.box.width * 0.5f, detection.box.y + detection.box.height) :
        detection.ground_point_px;
    cv::Point2f ground;
    if (!pixelToGround(pixel, ground, &sensor)) return false;
    detection.ground_point_px = pixel;
    detection.body_xy_m = ground;
    return true;
}

} // namespace rescue
