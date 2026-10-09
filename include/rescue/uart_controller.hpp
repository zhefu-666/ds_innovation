#pragma once

#include <array>
#include <stdexcept>
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
    void configureFrameFeedback(int open_state, int close_state) {
        if(open_state<0||open_state>1||close_state<0||close_state>1||open_state==close_state)
            throw std::invalid_argument("Unverified A6 mapping");
        feedback_open_=open_state;feedback_close_=close_state;done_flag_=false;
    }
    // TEMP_ASSUMPTION（2026-10-08现场）：A6第2字节是“动作完成”位，开(0°)和关(20°)完成后都回1，不是开闭位置。
    // 完成=编号一致+反馈新鲜+该位为1；开闭状态只取本程序最近下发并已完成的目标，不能从A6恢复。
    // 启动时必须由操作者提供本轮已知的方框角度与下位机当前编号，二者与A6不符则拒绝接管。
    // 实测新编号约10ms即回1，早于舵机实际转动；故另加最短settle_ms，从首次发出新编号起计。
    void configureDoneFlagFeedback(int known_angle, int known_id, int settle_ms) {
        if((known_angle!=0&&known_angle!=20)||known_id<1||known_id>255||settle_ms<100||settle_ms>3000)
            throw std::invalid_argument("Done-flag A6 requires known frame angle 0/20, action id 1..255, settle 100..3000ms");
        done_flag_=true;feedback_open_=feedback_close_=-1;known_angle_=known_angle;known_id_=known_id;
        done_settle_us_=uint64_t(settle_ms)*1000;
    }
    bool doneFlagFeedback() const { return done_flag_; }
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
    // 当前可确认的方框逻辑状态：1开、0关、-1未知/动作未完成。
    // 映射模式取反馈原始位；完成位模式取已完成的发送目标（或启动时接管的已知角度）。
    int confirmedFrameOpen(const ActuatorFeedback& fb) const;
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

    int feedback_open_=-1, feedback_close_=-1;
    bool done_flag_=false; int known_angle_=-1, known_id_=0; uint64_t done_settle_us_=0;
    uint64_t transaction_=0, action_started_us_=0;
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
    int16_t gripper_target_ = 0; // 已限幅方框偏移角，0开、20关
    bool camera_sent_ = false;
    int16_t camera_pitch_target_ = 0; // 0.01°，已限幅
};

} // namespace rescue
