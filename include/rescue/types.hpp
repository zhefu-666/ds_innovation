#pragma once

#include <opencv2/core.hpp>

#include <array>
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

// 上位机 → 下位机：10字节，小端IEEE754 float32，无CRC/帧尾。
struct MotionPacket {
    uint8_t start_of_frame = 0x56;
    float vx_mps = 0.0f; // m/s，正前进、负后退
    float wz_rps = 0.0f; // rad/s
    uint8_t gripper_closed = 0; // 0张开，1合拢框住；舵机角度由下位机标定
};

// 下位机 → 上位机：4字节执行器反馈；IMU直连，ToF暂未安装。
struct SensorPacket {
    static constexpr size_t kSize = 4;
    static constexpr uint8_t kHeader = 0xA6;
    uint8_t start_of_frame = kHeader;
    uint8_t gripper_done = 0; // 下位机动作完成返回值，保留原始字节
    // uint16_t tof_fl_mm = 0; // 左前，毫米
    // uint16_t tof_fr_mm = 0; // 右前，毫米
    // uint16_t tof_rl_mm = 0; // 左后，毫米
    // uint16_t tof_rr_mm = 0; // 右后，毫米
    uint16_t crc16 = 0; // Modbus CRC，覆盖字节0..1，初值FFFF、多项式A001
};
#pragma pack(pop)
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Motion protocol requires IEEE754 float32");
static_assert(sizeof(MotionPacket) == 10 && offsetof(MotionPacket, vx_mps) == 1 &&
              offsetof(MotionPacket, wz_rps) == 5 && offsetof(MotionPacket, gripper_closed) == 9,
              "MotionPacket wire layout mismatch");
static_assert(sizeof(SensorPacket) == SensorPacket::kSize && offsetof(SensorPacket, crc16) == 2, "SensorPacket wire layout mismatch");

// 执行器反馈与IMU状态独立；valid只表示帧校验通过且尚未过期。
// 没有命令序号，不可仅凭gripper_done确认本次动作。
struct ActuatorFeedback {
    uint64_t timestamp_us = 0;
    uint8_t gripper_done = 0;
    bool valid = false;
};

// 独立IMU/可选测距的业务状态；A6反馈解析器不更新本结构。
struct SensorState {
    uint64_t timestamp_us = 0; // 上位机单调时钟，微秒
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
    std::string label;
    cv::Mat rvec; // 目标到相机的旋转向量，rad
    cv::Mat tvec; // 目标到相机的平移，m
    float reprojection_error_px = 999.0f;
    float distance_m = 0.0f;
    float heading_error_deg = 0.0f; // 目标方位偏角，度

    enum View { FRONT, OBLIQUE, SIDE, PARTIAL, UNKNOWN } view = UNKNOWN;
};

// 算法业务请求；禁止直接发送sizeof(MotionCommand)，该结构可能包含对齐填充。
// buildMotionPacket()序列化为10字节，sendMotion()负责串口发送。
struct MotionCommand {
    uint8_t header = 0x56;
    float vx_mps = 0.0f; // m/s
    float wz_rps = 0.0f; // rad/s
    uint8_t gripper_closed = 0; // 单舵机目标开关状态
};

}
