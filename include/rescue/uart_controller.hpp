#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rescue/types.hpp"

namespace rescue {

// A6执行器反馈解析，支持拆包、粘包和坏帧恢复。
class SensorPacketParser {
public:
    bool consume(uint8_t byte, uint64_t received_us, ActuatorFeedback &state);
private:
    std::vector<uint8_t> buffer_;
    uint64_t last_byte_us_ = 0;
};

// 上位机当前夹爪动作/相机pitch目标相对下位机反馈的判断结果。
enum class ActionResult {
    Idle,       // 尚未发出任何该类动作
    NoFeedback, // 反馈缺失或超过200ms未更新
    NotDone,    // 夹爪：编号不是最新或标志不为1；相机：读回角超出容差
    Done,       // 夹爪：编号一致且标志为1；相机：读回角在目标容差内
};
using GripperActionResult = ActionResult;

class UARTController {
public:
    UARTController() = default;
    UARTController(const UARTController &) = delete;
    UARTController &operator=(const UARTController &) = delete;
    virtual ~UARTController();

    void initUART(const std::string &port, int baudrate, bool dry_run, bool auto_run);
    void closePort();

    uint8_t getLatestCmd() const;
    void setLatestCmd(uint8_t cmd);

    void setMotorSpeed(int motor_id, int speed);
    void setServoAngle(int servo_id, int angle);
    std::array<int, 2> motorSpeeds() const;
    std::array<int, 2> servoAngles() const;
    ActuatorFeedback latestActuatorFeedback() const;
    // readLoop()仅发布经过CRC校验的完整执行器反馈。
    void publishActuatorFeedback(const ActuatorFeedback &state);

    // 旧双电机/双舵机发送入口已禁用；历史控制器不能发送不兼容协议。
    bool execute();
    // 夹爪目标与上次不同时分配新动作编号；目标不变则沿用原编号。相机pitch每包直接下发。
    bool sendMotion(const MotionCommand &command);
    // 当前动作：id为0表示尚未发送过。
    uint8_t gripperActionId() const;
    ActionResult gripperActionResult() const;
    // 最近一次下发的相机pitch目标（限幅后），0.01°。
    int16_t cameraPitchTarget() const;
    // 读回角与目标之差不超过tolerance_cdeg为Done；读回无效为NoFeedback。
    // 稳定时长由调用方判断；到位前不得使用对应角度的标定参数测距。
    ActionResult cameraPitchResult(int16_t tolerance_cdeg) const;

    static std::array<uint8_t, 13> buildPacket(int speed1, int speed2,
                                               int angle1, int angle2);
    // 按 MotionPacket 组装15字节浮点速度/夹爪/相机pitch包（动作编号+CRC16），速度无单位缩放。
    static std::vector<uint8_t> buildMotionPacket(const MotionCommand &command,
                                                  uint8_t gripper_action_id);
    // 编号在1..255循环，跳过保留值0。
    static uint8_t nextGripperActionId(uint8_t id);
    static uint16_t calculateCRC16(const uint8_t *data, uint8_t start_byte,
                                   uint8_t end_byte);

protected:
    static int clampSpeed(int speed);
    void reopenPort();

private:
    void openPort();
    void readLoop();

    int fd_ = -1;
    std::string port_ = "/dev/ttyACM0";
    int baudrate_ = 115200;
    bool dry_run_ = false;
    std::array<int, 2> motor_speeds_{0, 0};
    std::array<int, 2> servo_angles_{0, 0};
    std::atomic<uint8_t> latest_cmd_{0xAA};
    std::atomic_bool running_{false};
    std::thread reader_;
    mutable std::mutex io_mutex_;
    ActuatorFeedback actuator_feedback_;
    // 以下受io_mutex_保护
    uint8_t gripper_action_id_ = 0;
    uint8_t gripper_target_ = 0; // 0关闭，1张开
    bool camera_sent_ = false;
    int16_t camera_pitch_target_ = 0; // 0.01°，已限幅
};

} // namespace rescue
