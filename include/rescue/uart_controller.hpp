#pragma once

#include <array>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rescue/types.hpp"
#include "rescue/frame_sensors.hpp"

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
    // Opens O_RDONLY and blocks sendMotion even if accidentally called. No TX bytes.
    void initFeedbackOnly(const std::string &port, int baudrate);
    FrameSensors feedbackAt(uint64_t capture_us) const;
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
    // 首包前接管下位机当前编号和夹爪状态：主程序重启时下位机不重启，仍停在上次的编号。
    // 首包保持该状态则沿用该编号（0表示不触发动作），之后每次换状态用编号+1，与下位机计数一致。
    // 等待有效A6反馈至多timeout，超时返回false且不改编号。
    bool syncGripperActionId(std::chrono::milliseconds timeout);
    // 夹爪确认：同一把锁内读取结果与对应目标，避免发送线程在两次读取之间换目标。
    // target为-1表示尚未发送或角度不能由二值A6确认，0放下，1抬起；result为Done时target即已完成的目标状态。
    struct GripperAck { ActionResult result = ActionResult::Idle; int target = -1; };
    GripperAck gripperAck() const;
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
    bool feedback_only_ = false;
    PitchHistory pitch_history_;
    std::array<int, 2> motor_speeds_{0, 0};
    std::array<int, 2> servo_angles_{0, 0};
    std::atomic<uint8_t> latest_cmd_{0xAA};
    std::atomic_bool running_{false};
    std::thread reader_;
    mutable std::mutex io_mutex_;
    ActuatorFeedback actuator_feedback_;
    // 以下受io_mutex_保护
    uint8_t gripper_action_id_ = 0; // 已发送时为当前编号；同步后未发送时为下位机编号
    bool gripper_sent_ = false;
    bool gripper_synced_ = false; // 已接管下位机编号/状态，尚未发送时gripper_target_为下位机状态
    int16_t gripper_target_ = 0; // 已限幅方框偏移角，0放下、20抬起
    bool camera_sent_ = false;
    int16_t camera_pitch_target_ = 0; // 0.01°，已限幅
};

} // namespace rescue
