#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <deque>

namespace rescue {
// HI91解析后的业务数据；不是可直接收发的串口打包结构体。
// 以下“整帧偏移”从5A帧头开始计数；线上整数/float32均为小端。
// 无body_前缀的三轴数据为IMU设备坐标（imu_device），保留原值供诊断；
// body_前缀字段为经安装变换后的车体坐标（base_link，FLU：X前、Y左、Z上）。
struct ImuSample {
    // 上位机生成：主机单调时钟收帧时间，单位us；只在设备时间推进时更新缓存。
    // 与设备时间不是同一时钟域，不能直接相减。
    uint64_t received_us = 0;
    // 上位机生成：CRC通过且类型/长度正确的HI91帧序号，从1开始。
    // 接收层会拒绝重复/倒退时间帧，因此对外快照序号可能跳号。
    uint64_t sequence = 0;
    uint32_t device_time_ms = 0; // 整帧14..17：设备时间，ms，uint32；自然溢出可回绕。
    uint16_t status = 0;        // 整帧7..8：设备状态原值；位含义需与具体型号/固件核对。
    int temperature_c = 0;     // 整帧9：有符号int8温度，℃；解析后提升为int。
    float pressure_pa = 0;     // 整帧10..13：float32气压，Pa。
    // 整帧18..29：x/y/z加速度；线上g乘9.8转为m/s²，仍包含重力相关比力。
    // 整帧30..41：x/y/z角速度；线上°/s乘π/180转为rad/s。
    // 整帧42..53：x/y/z磁场，µT，无单位转换。
    // 整帧54..65：roll/pitch/yaw（横滚/俯仰/偏航）；线上°转为rad。
    std::array<float, 3> acceleration_mps2{}, angular_velocity_rps{}, magnetic_ut{}, rpy_rad{};
    std::array<float, 4> quaternion_wxyz{}; // 整帧66..81：四元数，顺序w/x/y/z，无量纲。
    // 上位机计算，非线上字段：由上面的设备坐标数据经applyImuMounting()得到的车体坐标数据。
    // 单位同设备字段。body_rpy_rad由body_quaternion_wxyz按ZYX分解，符号遵循REP-103：
    // 右侧下沉roll为正，车头下俯pitch为正，逆时针（左转）yaw增大。yaw仅为相对航向。
    std::array<float, 3> body_acceleration_mps2{}, body_angular_velocity_rps{}, body_rpy_rad{};
    std::array<float, 4> body_quaternion_wxyz{};
    // 上位机数值检查：气压、三轴数据、姿态、四元数有限，且四元数模长约为1。
    // 不表示设备状态无告警、姿态已收敛或安装标定有效。
    bool measurements_valid = false;
};
// 上位机生成的接收快照及诊断信息；下列管理字段不占HI91线上字节。
struct ImuSnapshot {
    ImuSample sample; // 最新被接受的测量；过期时仍保留旧值，使用前须检查fresh。
    // connected：串口接收线程尚未报告断开；不等于已经收到有效消息。
    // fresh：已连接且收到过设备时间推进的帧，数据年龄未超过timeout_ms。
    bool connected = false, fresh = false;
    uint64_t sampled_at_us = 0; // 获取快照时的主机单调时钟，us。
    double age_ms = -1;        // (sampled_at_us - sample.received_us) / 1000；无可用时间时为-1。
    uint32_t timeout_ms = 200; // 数据新鲜度期限，ms，默认200。
    // bytes：串口累计读取字节；valid_frames：CRC通过且HI91类型/长度正确的帧数。
    // crc_errors：候选帧CRC失败次数；invalid_frames：非法长度/不支持类型/半包超时等次数。
    uint64_t bytes = 0, valid_frames = 0, crc_errors = 0, invalid_frames = 0;
    // duplicate_times：设备时间重复次数；backward_times：设备时间倒退次数。
    // io_errors：串口poll/read错误或断开次数；以上均为本次接收会话的累计值。
    uint64_t duplicate_times = 0, backward_times = 0, io_errors = 0;
};
uint64_t imuNowUs();
// 安装变换：本车IMU倒装，设备坐标为FRD（X前、Y右、Z下），相对车体FLU绕X轴旋转180°。
// 依据2026-09-27实车转动采样：静止acc_z≈-1g；车头抬高acc_x为正；左转gyro_z为负。
// 根据设备字段填写sample的body_字段，不修改设备字段。
void applyImuMounting(ImuSample& sample);
class Hi91Parser {
public:
    bool consume(uint8_t byte, uint64_t received_us, ImuSample& out);
    uint64_t valid_frames = 0, crc_errors = 0, invalid_frames = 0;
private:
    std::vector<uint8_t> buffer_;
    uint64_t last_byte_us_ = 0;
};
class HipnucImu {
public:
    HipnucImu(const std::string& port, int baud = 115200, uint32_t timeout_ms = 200);
    ~HipnucImu();
    HipnucImu(const HipnucImu&) = delete;
    HipnucImu& operator=(const HipnucImu&) = delete;
    ImuSnapshot snapshot() const;
    ImuSnapshot snapshotAt(uint64_t capture_us, uint32_t max_skew_ms = 50) const;
private:
    void readLoop();
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread reader_;
    mutable std::mutex mutex_;
    ImuSnapshot state_;
    std::deque<ImuSample> history_;
};
}
