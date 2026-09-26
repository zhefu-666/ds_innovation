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

    // 旧13字节发送入口已禁用；历史控制器不能发送不兼容协议。
    bool execute();
    bool sendMotion(const MotionCommand &command);

    static std::array<uint8_t, 13> buildPacket(int speed1, int speed2,
                                               int angle1, int angle2);
    // 按 MotionPacket 组装10字节浮点速度/舵机包，无单位缩放。
    static std::vector<uint8_t> buildMotionPacket(const MotionCommand &command);
    static uint16_t calculateCRC16(const uint8_t *data, uint8_t start_byte,
                                   uint8_t end_byte);

protected:
    static int clampSpeed(int speed);
    void reopenPort();

private:
    void openPort();
    void readLoop();

    int fd_ = -1;
    std::string port_ = "/dev/ttyUSB0";
    int baudrate_ = 115200;
    bool dry_run_ = false;
    std::array<int, 2> motor_speeds_{0, 0};
    std::array<int, 2> servo_angles_{0, 0};
    std::atomic<uint8_t> latest_cmd_{0xAA};
    std::atomic_bool running_{false};
    std::thread reader_;
    mutable std::mutex io_mutex_;
    ActuatorFeedback actuator_feedback_;
};

} // namespace rescue
