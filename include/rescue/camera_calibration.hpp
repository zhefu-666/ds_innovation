#pragma once

#include "rescue/types.hpp"

#include <opencv2/calib3d.hpp>

#include <string>

namespace rescue {

// 地面映射与相机pitch绑定：相机舵机一动，单应矩阵就失效，因此每次换算都必须给出舵机读回的
// camera_pitch_cdeg（0.01°，正值向下，与A6反馈同口径；kCameraPitchInvalid表示读回无效）。
// 地面坐标沿用标定工具约定：x右、y前、z上，单位m，原点为车体地面参考点。
//
// 两种模式（load()按文件内容自动选择）：
// - FIXED_PITCH：只有ground_homography，记录标定时的ground_camera_pitch_cdeg；
//   读回pitch偏离该值超过容差即拒绝。无法补偿车体倾斜，倾斜限幅默认2°。
// - PITCH_MODEL：有内参+参考pitch下的T_camera_from_robot外参，按读回pitch实时计算H；
//   只在已验证的pitch范围内使用，并用IMU车体roll/pitch补偿倾斜，限幅默认8°。
// 倾斜均相对标定时记录的IMU参考姿态；车体安全停车的12°由SensorFusion单独判断，与此无关。
class CameraCalibration {
public:
    enum class GroundMode { NONE, FIXED_PITCH, PITCH_MODEL };

    bool load(const std::string &path);
    void setIntrinsics(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);
    // H：像素→地面；undistorted_pixels为false时H作用于原图像素。
    void setGroundHomography(const cv::Mat &homography, int16_t camera_pitch_cdeg,
                             bool undistorted_pixels = false, int16_t tolerance_cdeg = 50);
    // T_camera_from_robot：参考pitch下4x4外参（p_camera = T · p_robot）；pivot为舵机转轴在
    // 参考相机坐标系中的位置，m，未测时为0（绕光心转）。min/max为实测验证过的pitch范围。
    void setPitchModel(const cv::Mat &T_camera_from_robot, int16_t reference_pitch_cdeg,
                       int16_t min_pitch_cdeg, int16_t max_pitch_cdeg,
                       const cv::Vec3d &pivot_camera_m = {});
    // 标定时静止车体的IMU姿态（body_rpy，rad），倾斜判断与补偿均以它为零点。
    void setImuReference(float roll_rad, float pitch_rad);
    // <=0恢复为模式默认值。
    void setTiltLimitDeg(float limit_deg) { tilt_limit_deg_ = limit_deg; }

    bool valid() const;
    GroundMode groundMode() const { return mode_; }
    // 读回pitch是否落在本标定可用的范围内；接近阶段应把舵机锁在此范围内。
    bool pitchUsable(int16_t camera_pitch_cdeg) const;
    float tiltLimitDeg() const;
    bool tiltTrusted(const SensorState &sensor) const;
    // 当前pitch（与可选IMU姿态）下的像素→地面单应。sensor为空时不做倾斜检查/补偿，仅供离线工具与测试。
    bool groundHomographyAt(int16_t camera_pitch_cdeg, const SensorState *sensor, cv::Matx33d &homography) const;
    bool pixelToGround(const cv::Point2f &pixel, int16_t camera_pitch_cdeg, cv::Point2f &ground_m,
                       const SensorState *sensor = nullptr) const;
    // Ray intersection with the horizontal plane z=height_m (e.g. fence tops).
    // FIXED_PITCH has only the floor homography, so only height 0 is supported there.
    bool pixelToPlane(const cv::Point2f &pixel, int16_t camera_pitch_cdeg, float height_m, cv::Point2f &xy_m,
                      const SensorState *sensor = nullptr) const;
    bool planeHomographyAt(int16_t camera_pitch_cdeg, const SensorState *sensor, double height_m,
                           cv::Matx33d &homography) const;
    bool detectionToBody(SegDetection &detection, int16_t camera_pitch_cdeg, const SensorState &sensor) const;

    // Available only for an independently validated dynamic extrinsic model.
    bool cameraFromGround(int16_t pitch, const SensorState& sensor, cv::Matx34d& transform) const;
    const cv::Mat &cameraMatrix() const { return camera_matrix_; }
    const cv::Mat &distCoeffs() const { return dist_coeffs_; }
    cv::Size imageSize() const { return image_size_; }

private:
    bool cameraFromGroundAt(int16_t camera_pitch_cdeg, const SensorState *sensor, cv::Matx34d &T) const;

    GroundMode mode_ = GroundMode::NONE;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat ground_homography_;
    bool undistorted_ground_ = false;
    cv::Size image_size_;
    // FIXED_PITCH
    int16_t fixed_pitch_cdeg_ = kCameraPitchInvalid;
    int16_t pitch_tolerance_cdeg_ = 50;
    // PITCH_MODEL
    cv::Matx44d camera_from_robot_ref_ = cv::Matx44d::eye();
    int16_t reference_pitch_cdeg_ = 0, min_pitch_cdeg_ = 0, max_pitch_cdeg_ = 0;
    cv::Vec3d pivot_camera_m_{0,0,0};
    float imu_ref_roll_rad_ = 0.0f, imu_ref_pitch_rad_ = 0.0f;
    float tilt_limit_deg_ = 0.0f; // <=0：FIXED_PITCH/NONE取2°，PITCH_MODEL取8°
};

} // namespace rescue
