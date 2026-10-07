#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <string>
#include <limits>

namespace rescue {

struct SegDetection {
    int track_id = -1;
    std::string label;
    std::string model_label;
    float confidence = 0.0f;
    cv::Rect box;
    cv::Mat mask; // 可选分割掩码
    cv::Point2f ground_point_px; // 地面接触点，像素
    cv::Point2f body_xy_m; // 米；x向右、y向前
    bool ground_contact_valid = false; // explicit measured contact; a box bottom is only an estimate
    bool ground_position_valid = false; // metric projection succeeded; not a path/delivery guarantee
    std::string ground_contact_reason; // empty when ground_contact_valid; otherwise why not
    uint64_t frame_id = 0; // image sequence, not a coordinate-frame name
    int class_id = -1;
    uint64_t timestamp_us = 0; // 上位机单调时钟，微秒

    cv::Point2f center() const;
    float area() const;
};

using Detection = SegDetection;

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Packed UART packets require a little-endian target"
#endif
// 线上包：小端、1字节对齐。
#pragma pack(push, 1)

// 上位机 → 下位机：15字节，小端IEEE754 float32；CRC16/Modbus覆盖字节0..12。
struct MotionPacket {
    static constexpr size_t kSize = 15;
    static constexpr uint8_t kHeader = 0x56;
    uint8_t start_of_frame = kHeader;
    float vx_mps = 0.0f; // m/s，正前进、负后退；下位机对下发速度有限制，buildMotionPacket()硬限幅到±kMaxLinearSpeedMps（1.0m/s）
    float wz_rps = 0.0f; // rad/s
    int8_t gripper_offset = 0; // 方框位置偏移，-20..20度；0中点(90度)，当前0放下、+20抬起
    uint8_t gripper_action_id = 0; // 夹爪动作编号：仅目标变化时+1，1..255循环，0保留为“无动作”
    int16_t camera_pitch_offset = 0; // TX偏移角-40..40；实测TX35->RX75，TX40->RX80
    uint16_t crc16 = 0; // Modbus CRC，覆盖字节0..12，初值FFFF、多项式A001，低字节在前
};

// 下位机 → 上位机：8字节执行器反馈，以换行结尾便于串口助手按行显示；IMU直连，ToF暂未安装。
struct SensorPacket {
    static constexpr size_t kSize = 8;
    static constexpr uint8_t kHeader = 0xA6;
    static constexpr uint8_t kNewline = 0x0A; // 帧尾'\n'，不参与CRC
    uint8_t start_of_frame = kHeader;
    uint8_t gripper_open = 0; // 旧A6二值状态保留：暂映射0放下、1抬起，待电控确认
    uint8_t gripper_action_id = 0; // 该状态对应的夹爪动作编号，0表示上电后尚未收到动作
    int16_t camera_pitch_deg = 40; // 线上整数度0..80；解析时减40转回标定角；-32768无效
    // uint16_t tof_fl_mm = 0; // 左前，毫米
    // uint16_t tof_fr_mm = 0; // 右前，毫米
    // uint16_t tof_rl_mm = 0; // 左后，毫米
    // uint16_t tof_rr_mm = 0; // 右后，毫米
    uint16_t crc16 = 0; // Modbus CRC，覆盖字节0..4，初值FFFF、多项式A001，低字节在前
    uint8_t newline = kNewline; // [7] 帧尾，固定0x0A
};
#pragma pack(pop)

// A6 的开关状态须与本次目标和编号匹配，不是独立到位传感器。
// 相机pitch读回无效标记（int16最小值，即字节00 80）。
constexpr int16_t kCameraPitchInvalid = std::numeric_limits<int16_t>::min();
// 内部沿用旧标定坐标±4000 cdeg；TX仍为标定偏移角-40..40，RX为0..80度；RX减40转换回标定角。
constexpr int16_t kCameraPitchLimitCdeg = 4000; // 旧标定坐标，也是TX偏移角范围
constexpr int16_t kCameraProtocolZeroDeg = 40; // RX = calibration_deg + 40; TX = calibration_deg
inline int16_t cameraPitchToWire(int16_t calibrated_cdeg) {
    return static_cast<int16_t>(std::round(std::clamp<int>(calibrated_cdeg, -4000, 4000) / 100.0));
}
inline int16_t cameraPitchToFeedbackDeg(int16_t calibrated_cdeg) {
    return calibrated_cdeg == kCameraPitchInvalid ? kCameraPitchInvalid
        : static_cast<int16_t>(cameraPitchToWire(calibrated_cdeg) + kCameraProtocolZeroDeg);
}
constexpr int16_t cameraPitchFromWire(int16_t wire_deg) {
    return wire_deg >= 0 && wire_deg <= 80
        ? static_cast<int16_t>((wire_deg - kCameraProtocolZeroDeg) * 100) : kCameraPitchInvalid;
}

// 上位机下发线速度硬限幅，m/s。下位机对速度有限制，buildMotionPacket()把|vx|截断到该值。
constexpr float kMaxLinearSpeedMps = 1.0f;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Motion protocol requires IEEE754 float32");
static_assert(sizeof(MotionPacket) == MotionPacket::kSize && offsetof(MotionPacket, vx_mps) == 1 &&
              offsetof(MotionPacket, wz_rps) == 5 && offsetof(MotionPacket, gripper_offset) == 9 &&
              offsetof(MotionPacket, gripper_action_id) == 10 && offsetof(MotionPacket, camera_pitch_offset) == 11 &&
              offsetof(MotionPacket, crc16) == 13,
              "MotionPacket wire layout mismatch");
static_assert(sizeof(SensorPacket) == SensorPacket::kSize && offsetof(SensorPacket, gripper_action_id) == 2 &&
              offsetof(SensorPacket, camera_pitch_deg) == 3 &&
              offsetof(SensorPacket, crc16) == 5 && offsetof(SensorPacket, newline) == 7,
              "SensorPacket wire layout mismatch");

// 执行器反馈与IMU状态独立；valid只表示帧校验通过且尚未过期。
// 夹爪完成须同时比对action_id与状态，见gripperActionResult()；相机到位见cameraPitchResult()。
struct ActuatorFeedback {
    uint64_t timestamp_us = 0;
    uint8_t gripper_open = 0; // 旧A6二值状态：暂按0放下、1抬起解释，非角度
    uint8_t gripper_action_id = 0;
    int16_t camera_pitch_cdeg = kCameraPitchInvalid; // 转换后的标定角pitch，0.01°，线上角减40度
    bool valid = false;
};

// 独立IMU/可选测距的业务状态；A6反馈解析器不更新本结构。
struct SensorState {
    uint64_t timestamp_us = 0; // 上位机单调时钟，微秒
    // 车体姿态（base_link，FLU），由sensorStateFromImu()取自ImuSample::body_rpy_rad，符号遵循REP-103：
    // 右侧下沉roll为正，车头下俯pitch为正，逆时针yaw增大；yaw仅为相对航向。
    float yaw_rad = 0.0f;
    float pitch_rad = 0.0f;
    float roll_rad = 0.0f;
    float tof_fl_m = 0.0f;
    float tof_fr_m = 0.0f;
    float tof_rl_m = 0.0f;
    float tof_rr_m = 0.0f;
    bool imu_valid = false; // 上位机根据角度范围及接收超时判断
    std::array<bool, 4> tof_valid{{false, false, false, false}}; // 上位机判断，不在线上传输
    bool encoder_valid = false; // 预留，当前协议未上报
};

struct SafeZonePose {
    bool valid = false;
    std::string reason = "no_observation";
    std::string label;
    cv::Mat rvec; // 目标到相机的旋转向量，rad
    cv::Mat tvec; // 目标到相机的平移，m
    float reprojection_error_px = 999.0f;
    float distance_m = 0.0f;
    float heading_error_deg = 0.0f; // 目标方位偏角，度

    enum View { FRONT, OBLIQUE, SIDE, PARTIAL, UNKNOWN } view = UNKNOWN;
};

// 算法业务请求；禁止直接发送sizeof(MotionCommand)，该结构可能包含对齐填充。
// buildMotionPacket()序列化为15字节（含动作编号与CRC），sendMotion()负责串口发送。
struct MotionCommand {
    uint8_t header = 0x56;
    float vx_mps = 0.0f; // m/s；发给下位机的速度有限制，超过±1.0m/s（kMaxLinearSpeedMps）会在打包时被截断
    float wz_rps = 0.0f; // rad/s
    int16_t gripper_offset = 0; // 方框目标偏移度；打包限幅-20..20；0放下、+20抬起
    int16_t camera_pitch_cdeg = 0; // 相机标定角目标，0.01°，正值向下；TX除100发偏移角-40..40，RX减40后入标定；标定范围±4000
};

}
