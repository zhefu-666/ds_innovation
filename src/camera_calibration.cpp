#include "rescue/camera_calibration.hpp"

#include <cmath>
#include <vector>

namespace rescue {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr float kFixedPitchTiltLimitDeg = 2.0f;  // 无补偿：只容许地面起伏级别的倾斜
constexpr float kPitchModelTiltLimitDeg = 8.0f;  // 有IMU补偿：小角度近似和车体高度不变假设的适用范围

cv::Matx33d rotX(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {1, 0, 0, 0, c, -s, 0, s, c};
}
cv::Matx33d rotY(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, 0, s, 0, 1, 0, -s, 0, c};
}
bool finiteMat(const cv::Mat &m) { return !m.empty() && cv::checkRange(m); }
bool readInt16(const cv::FileNode &node, int16_t &out, int limit = 9000) {
    if (node.empty() || !node.isInt()) return false;
    const int v = static_cast<int>(node);
    if (v < -limit || v > limit) return false;
    out = static_cast<int16_t>(v);
    return true;
}
} // namespace

bool CameraCalibration::load(const std::string &path, bool allow_mechanical_assumption) {
    *this = CameraCalibration{};
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) return false;
    file["camera_matrix"] >> camera_matrix_;
    file["dist_coeffs"] >> dist_coeffs_;
    if (!file["image_width"].empty() && !file["image_height"].empty())
        image_size_ = {static_cast<int>(file["image_width"]), static_cast<int>(file["image_height"])};
    std::string domain;
    if (!file["ground_pixel_domain"].empty()) file["ground_pixel_domain"] >> domain;
    if (!domain.empty() && domain != "undistorted_pixels") return false;
    undistorted_ground_ = domain == "undistorted_pixels";
    double roll = 0, pitch = 0;
    if (!file["imu_reference_roll_rad"].empty()) file["imu_reference_roll_rad"] >> roll;
    if (!file["imu_reference_pitch_rad"].empty()) file["imu_reference_pitch_rad"] >> pitch;
    if (!std::isfinite(roll) || !std::isfinite(pitch)) return false;
    setImuReference(static_cast<float>(roll), static_cast<float>(pitch));
    // 旧文件的imu_tilt_limit_deg(12°)是停车阈值，不能当作地面映射可信范围，故不读取。
    if (!file["ground_tilt_limit_deg"].empty()) {
        double limit = 0;
        file["ground_tilt_limit_deg"] >> limit;
        if (!std::isfinite(limit) || limit <= 0) return false;
        tilt_limit_deg_ = static_cast<float>(limit);
    }

    // 优先：已验收外参 + 标定时pitch → 按读回pitch实时计算H。
    cv::Mat extrinsic;
    int validated = 0;
    if (!file["extrinsics_validated"].empty()) file["extrinsics_validated"] >> validated;
    int mechanical = 0;
    if (!file["mechanical_assumption"].empty()) file["mechanical_assumption"] >> mechanical;
    if (mechanical && !allow_mechanical_assumption) return false;
    file["T_camera_from_robot"] >> extrinsic;
    int16_t reference = 0, low = 0, high = 0;
    if ((validated == 1 || (mechanical == 1 && allow_mechanical_assumption)) && !extrinsic.empty() && readInt16(file["pitch_model_reference_cdeg"], reference)) {
        low = high = reference;
        if (!file["pitch_model_min_cdeg"].empty() && !readInt16(file["pitch_model_min_cdeg"], low)) return false;
        if (!file["pitch_model_max_cdeg"].empty() && !readInt16(file["pitch_model_max_cdeg"], high)) return false;
        cv::Mat pivot;
        file["pitch_pivot_camera_m"] >> pivot;
        cv::Vec3d pivot_m{0,0,0};
        if (!pivot.empty()) {
            if (pivot.total() != 3 || !finiteMat(pivot)) return false;
            pivot.convertTo(pivot, CV_64F);
            pivot_m = {pivot.at<double>(0), pivot.at<double>(1), pivot.at<double>(2)};
        }
        if (!undistorted_ground_ && !domain.empty()) return false;
        undistorted_ground_ = true; // 模型H作用于去畸变像素
        setPitchModel(extrinsic, reference, low, high, pivot_m);
        return valid();
    }

    // 其次：固定单应，必须记录标定时的相机pitch，否则无法判断当前是否可用。
    cv::Mat homography;
    file["ground_homography"] >> homography;
    int16_t fixed = 0, tolerance = 50;
    if (homography.empty() || !readInt16(file["ground_camera_pitch_cdeg"], fixed)) return false;
    if (!file["ground_pitch_tolerance_cdeg"].empty() &&
        (!readInt16(file["ground_pitch_tolerance_cdeg"], tolerance, 1000) || tolerance < 0)) return false;
    const bool undistorted = undistorted_ground_;
    setGroundHomography(homography, fixed, undistorted, tolerance);
    return valid();
}

void CameraCalibration::setIntrinsics(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) {
    camera_matrix.copyTo(camera_matrix_);
    dist_coeffs.copyTo(dist_coeffs_);
}

void CameraCalibration::setGroundHomography(const cv::Mat &homography, int16_t camera_pitch_cdeg,
                                            bool undistorted_pixels, int16_t tolerance_cdeg) {
    mode_ = GroundMode::NONE;
    if (homography.rows!=3 || homography.cols!=3 || homography.channels()!=1 || !finiteMat(homography) ||
        camera_pitch_cdeg==kCameraPitchInvalid || tolerance_cdeg<0 || tolerance_cdeg>1000) return;
    homography.convertTo(ground_homography_, CV_64F);
    cv::Mat inverse;
    if (!cv::invert(ground_homography_,inverse,cv::DECOMP_LU)) return;
    fixed_pitch_cdeg_ = camera_pitch_cdeg;
    pitch_tolerance_cdeg_ = tolerance_cdeg;
    undistorted_ground_ = undistorted_pixels;
    mode_ = GroundMode::FIXED_PITCH;
}

void CameraCalibration::setPitchModel(const cv::Mat &T_camera_from_robot, int16_t reference_pitch_cdeg,
                                      int16_t min_pitch_cdeg, int16_t max_pitch_cdeg,
                                      const cv::Vec3d &pivot_camera_m) {
    mode_ = GroundMode::NONE;
    if (T_camera_from_robot.rows != 4 || T_camera_from_robot.cols != 4 || T_camera_from_robot.channels()!=1 || !finiteMat(T_camera_from_robot) ||
        min_pitch_cdeg > reference_pitch_cdeg || reference_pitch_cdeg > max_pitch_cdeg) return;
    cv::Mat t;
    T_camera_from_robot.convertTo(t, CV_64F);
    const cv::Mat rotation=t(cv::Rect(0,0,3,3));
    if(cv::norm(rotation.t()*rotation-cv::Mat::eye(3,3,CV_64F))>1e-3 ||
       std::abs(cv::determinant(rotation)-1)>1e-3 ||
       std::abs(t.at<double>(3,0))+std::abs(t.at<double>(3,1))+std::abs(t.at<double>(3,2))>1e-6 ||
       std::abs(t.at<double>(3,3)-1)>1e-6 || reference_pitch_cdeg==kCameraPitchInvalid ||
       !std::isfinite(pivot_camera_m[0]) || !std::isfinite(pivot_camera_m[1]) || !std::isfinite(pivot_camera_m[2])) return;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) camera_from_robot_ref_(r, c) = t.at<double>(r, c);
    reference_pitch_cdeg_ = reference_pitch_cdeg;
    min_pitch_cdeg_ = min_pitch_cdeg;
    max_pitch_cdeg_ = max_pitch_cdeg;
    pivot_camera_m_ = pivot_camera_m;
    undistorted_ground_ = true;
    mode_ = GroundMode::PITCH_MODEL;
}

void CameraCalibration::setImuReference(float roll_rad, float pitch_rad) {
    imu_ref_roll_rad_ = roll_rad;
    imu_ref_pitch_rad_ = pitch_rad;
}

bool CameraCalibration::valid() const {
    bool intrinsics = !camera_matrix_.empty() && camera_matrix_.rows == 3 && camera_matrix_.cols == 3 &&
                      camera_matrix_.channels()==1 && finiteMat(camera_matrix_);
    if(intrinsics) {
        cv::Mat k;camera_matrix_.convertTo(k,CV_64F);
        intrinsics=k.at<double>(0,0)>0 && k.at<double>(1,1)>0 &&
            std::abs(k.at<double>(2,0))+std::abs(k.at<double>(2,1))<1e-9 && std::abs(k.at<double>(2,2)-1)<1e-9;
    }
    if (mode_ == GroundMode::FIXED_PITCH)
        return intrinsics && ground_homography_.rows == 3 && ground_homography_.cols == 3 &&
               finiteMat(ground_homography_);
    return intrinsics && mode_ == GroundMode::PITCH_MODEL;
}

bool CameraCalibration::pitchUsable(int16_t cdeg) const {
    if (cdeg == kCameraPitchInvalid) return false;
    if (mode_ == GroundMode::FIXED_PITCH)
        return std::abs(static_cast<int>(cdeg) - fixed_pitch_cdeg_) <= pitch_tolerance_cdeg_;
    if (mode_ == GroundMode::PITCH_MODEL) return cdeg >= min_pitch_cdeg_ && cdeg <= max_pitch_cdeg_;
    return false;
}

float CameraCalibration::tiltLimitDeg() const {
    if (tilt_limit_deg_ > 0) return tilt_limit_deg_;
    return mode_ == GroundMode::PITCH_MODEL ? kPitchModelTiltLimitDeg : kFixedPitchTiltLimitDeg;
}

bool CameraCalibration::tiltTrusted(const SensorState &sensor) const {
    const float limit_rad = tiltLimitDeg() * static_cast<float>(kPi / 180.0);
    return sensor.imu_valid && std::isfinite(sensor.pitch_rad) && std::isfinite(sensor.roll_rad) &&
           std::abs(sensor.pitch_rad - imu_ref_pitch_rad_) <= limit_rad &&
           std::abs(sensor.roll_rad - imu_ref_roll_rad_) <= limit_rad;
}

bool CameraCalibration::cameraFromGroundAt(int16_t cdeg, const SensorState *sensor, cv::Matx34d &T) const {
    // 舵机绕相机x轴（图像右）转动，正值向下：新相机C'相对参考相机C有 p_C' = Rx(Δ)(p_C - pivot) + pivot。
    const double delta = (static_cast<int>(cdeg) - reference_pitch_cdeg_) * kPi / 18000.0;
    const cv::Matx33d servo = rotX(delta);
    const cv::Vec3d servo_t = pivot_camera_m_ - servo * pivot_camera_m_;
    // IMU相对标定姿态的倾斜：车体(机器人坐标x右y前z上)相对地面 R_ground_from_body。
    // REP-103车头下俯pitch为正 → 绕x右轴转-Δpitch；右侧下沉roll为正 → 绕y前轴转+Δroll。
    cv::Matx33d body_from_ground = cv::Matx33d::eye();
    if (sensor) {
        const double dp = sensor->pitch_rad - imu_ref_pitch_rad_, dr = sensor->roll_rad - imu_ref_roll_rad_;
        body_from_ground = (rotX(-dp) * rotY(dr)).t();
    }
    const cv::Matx33d R_ref = camera_from_robot_ref_.get_minor<3, 3>(0, 0);
    const cv::Vec3d t_ref(camera_from_robot_ref_(0, 3), camera_from_robot_ref_(1, 3), camera_from_robot_ref_(2, 3));
    const cv::Matx33d R = servo * R_ref * body_from_ground;
    const cv::Vec3d t = servo * t_ref + servo_t;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T(r, c) = R(r, c);
        T(r, 3) = t[r];
    }
    return true;
}

bool CameraCalibration::cameraFromGround(int16_t pitch, const SensorState& sensor, cv::Matx34d& transform) const {
    if (mode_ != GroundMode::PITCH_MODEL || !valid() || !pitchUsable(pitch) || !tiltTrusted(sensor)) return false;
    return cameraFromGroundAt(pitch, &sensor, transform);
}

bool CameraCalibration::groundHomographyAt(int16_t cdeg, const SensorState *sensor,
                                           cv::Matx33d &homography) const {
    return planeHomographyAt(cdeg, sensor, 0.0, homography);
}

bool CameraCalibration::planeHomographyAt(int16_t cdeg, const SensorState *sensor, double height_m,
                                          cv::Matx33d &homography) const {
    if (!valid() || !pitchUsable(cdeg) || (sensor != nullptr && !tiltTrusted(*sensor))) return false;
    if (!std::isfinite(height_m)) return false;
    if (mode_ == GroundMode::FIXED_PITCH) {
        if (height_m != 0.0) return false;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) homography(r, c) = ground_homography_.at<double>(r, c);
        return true;
    }
    cv::Matx34d T;
    if (!cameraFromGroundAt(cdeg, sensor, T)) return false;
    cv::Matx33d K;
    cv::Mat k64;
    camera_matrix_.convertTo(k64, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) K(r, c) = k64.at<double>(r, c);
    // 平面z=h：像素 ~ K [r1 r2 r3·h+t] (x, y, 1)；h=0即地面。
    cv::Matx33d plane;
    for (int r = 0; r < 3; ++r) {
        plane(r, 0) = T(r, 0); plane(r, 1) = T(r, 1); plane(r, 2) = T(r, 2) * height_m + T(r, 3);
    }
    const cv::Matx33d plane_to_pixel = K * plane;
    if (std::abs(cv::determinant(plane_to_pixel)) < 1e-12) return false;
    homography = plane_to_pixel.inv();
    return true;
}

bool CameraCalibration::pixelToGround(const cv::Point2f &pixel, int16_t cdeg, cv::Point2f &ground_m,
                                      const SensorState *sensor) const {
    return pixelToPlane(pixel, cdeg, 0.0f, ground_m, sensor);
}

bool CameraCalibration::pixelToPlane(const cv::Point2f &pixel, int16_t cdeg, float height_m, cv::Point2f &xy_m,
                                     const SensorState *sensor) const {
    cv::Matx33d H;
    if (!planeHomographyAt(cdeg, sensor, height_m, H)) return false;
    std::vector<cv::Point2f> source{pixel};
    if (undistorted_ground_) {
        if (dist_coeffs_.empty()) return false;
        cv::undistortPoints(source, source, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
    }
    const cv::Vec3d g = H * cv::Vec3d(source[0].x, source[0].y, 1.0);
    if (!std::isfinite(g[2]) || std::abs(g[2]) < 1e-12) return false;
    const cv::Point2f ground(static_cast<float>(g[0] / g[2]), static_cast<float>(g[1] / g[2]));
    if (!std::isfinite(ground.x) || !std::isfinite(ground.y)) return false;
    if (mode_ == GroundMode::PITCH_MODEL) {
        // 地平线以上的像素反投影到相机后方，必须拒绝。
        cv::Matx34d T;
        cameraFromGroundAt(cdeg, sensor, T);
        const cv::Vec3d camera_point = T * cv::Vec4d(ground.x, ground.y, height_m, 1.0);
        if (camera_point[2] <= 0.0) return false;
    }
    xy_m = ground;
    return true;
}

bool CameraCalibration::detectionToBody(SegDetection &detection, int16_t cdeg, const SensorState &sensor) const {
    detection.ground_position_valid = false;
    const cv::Point2f pixel = detection.ground_point_px == cv::Point2f() ?
        cv::Point2f(detection.box.x + detection.box.width * 0.5f, detection.box.y + detection.box.height) :
        detection.ground_point_px;
    if(!std::isfinite(pixel.x)||!std::isfinite(pixel.y)||pixel.x<0||pixel.y<0||
       (image_size_.width>0&&(pixel.x>=image_size_.width||pixel.y>=image_size_.height))) return false;
    cv::Point2f ground;
    if (!pixelToGround(pixel, cdeg, ground, &sensor)) return false;
    detection.ground_point_px = pixel;
    detection.body_xy_m = ground;
    detection.ground_position_valid = true;
    return true;
}

} // namespace rescue
