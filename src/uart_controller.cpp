#include "rescue/uart_controller.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rescue/utils.hpp"

namespace rescue {

namespace {

speed_t baudToTermios(int baudrate) {
    switch (baudrate) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
    }
}

} // namespace

bool SensorPacketParser::consume(uint8_t byte, uint64_t received_us, ActuatorFeedback &state) {
    // 字节间隔超过100ms，丢弃残帧。
    if (last_byte_us_ && (received_us < last_byte_us_ || received_us - last_byte_us_ > 100000))
        buffer_.clear();
    last_byte_us_ = received_us;
    buffer_.push_back(byte);
    while (!buffer_.empty()) {
        if (buffer_[0] != SensorPacket::kHeader) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        if (buffer_.size() < SensorPacket::kSize) return false;
        SensorPacket packet;
        std::memcpy(&packet, buffer_.data(), sizeof(packet));
        if (UARTController::calculateCRC16(buffer_.data(), 0,
                static_cast<uint8_t>(offsetof(SensorPacket, crc16) - 1)) != packet.crc16 ||
            packet.newline != SensorPacket::kNewline) {
            // CRC或帧尾不符：丢弃1字节重新找帧头。
            buffer_.erase(buffer_.begin());
            continue;
        }
        ActuatorFeedback decoded;
        decoded.timestamp_us = received_us;
        decoded.gripper_done = packet.gripper_done;
        decoded.gripper_action_id = packet.gripper_action_id;
        decoded.camera_pitch_cdeg = packet.camera_pitch_cdeg;
        decoded.valid = received_us != 0;
        // ToF尚未安装；IMU独立接入，不能用执行器反馈刷新姿态数据。
        state = decoded;
        buffer_.clear();
        return true;
    }
    return false;
}

UARTController::~UARTController() {
    closePort();
}

void UARTController::initUART(const std::string &port, int baudrate,
                              bool dry_run, bool auto_run) {
    port_ = port;
    baudrate_ = baudrate;
    dry_run_ = dry_run;
    latest_cmd_.store(auto_run ? 0xBB : 0xAA);
    {
        std::lock_guard<std::mutex> guard(io_mutex_);
        actuator_feedback_ = {};
    }

    if (dry_run_) {
        std::cout << "UART dry-run enabled, no serial port will be opened\n";
        return;
    }

    openPort();
    running_.store(true);
    reader_ = std::thread(&UARTController::readLoop, this);
    std::cout << "UART connected: " << port_ << "@" << baudrate_ << "bps\n";
}

void UARTController::closePort() {
    running_.store(false);
    if (reader_.joinable()) {
        reader_.join();
    }

    std::lock_guard<std::mutex> guard(io_mutex_);
    actuator_feedback_.valid = false;
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "UART closed\n";
    }
}

uint8_t UARTController::getLatestCmd() const {
    return latest_cmd_.load();
}

void UARTController::setLatestCmd(uint8_t cmd) {
    latest_cmd_.store(cmd);
}

void UARTController::setMotorSpeed(int motor_id, int speed) {
    if (motor_id < 1 || motor_id > 2) {
        throw std::out_of_range("motor_id must be 1 or 2");
    }
    motor_speeds_[static_cast<size_t>(motor_id - 1)] = clampSpeed(speed);
}

void UARTController::setServoAngle(int servo_id, int angle) {
    if (servo_id < 1 || servo_id > 2) {
        throw std::out_of_range("servo_id must be 1 or 2");
    }
    servo_angles_[static_cast<size_t>(servo_id - 1)] = std::clamp(angle, 0, 360);
}

std::array<int, 2> UARTController::motorSpeeds() const {
    return motor_speeds_;
}

std::array<int, 2> UARTController::servoAngles() const {
    return servo_angles_;
}

ActuatorFeedback UARTController::latestActuatorFeedback() const {
    std::lock_guard<std::mutex> guard(io_mutex_);
    ActuatorFeedback state = actuator_feedback_;
    const auto now_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count());
    // 超过200ms无有效帧，返回无效快照；保留原始值但不将其当成新动作确认。
    if (state.timestamp_us == 0 || now_us < state.timestamp_us ||
        now_us - state.timestamp_us > 200000) state.valid = false;
    return state;
}

void UARTController::publishActuatorFeedback(const ActuatorFeedback &state) {
    std::lock_guard<std::mutex> guard(io_mutex_);
    if (state.valid && state.timestamp_us != 0 &&
        state.timestamp_us >= actuator_feedback_.timestamp_us) actuator_feedback_ = state;
}

bool UARTController::execute() {
    std::cerr << "[ERR] Legacy motor/servo protocol disabled; use sendMotion(MotionCommand)\n";
    return false;
}

uint8_t UARTController::nextGripperActionId(uint8_t id) {
    return id == 255 ? 1 : static_cast<uint8_t>(id + 1);
}

uint8_t UARTController::gripperActionId() const {
    std::lock_guard<std::mutex> guard(io_mutex_);
    return gripper_action_id_;
}

int16_t UARTController::cameraPitchTarget() const {
    std::lock_guard<std::mutex> guard(io_mutex_);
    return camera_pitch_target_;
}

namespace {
// 编号不一致说明完成标志属于上一次动作，不能采信。
ActionResult judgeAction(uint8_t id, bool feedback_valid, uint8_t feedback_id, uint8_t done) {
    if (id == 0) return ActionResult::Idle;
    if (!feedback_valid) return ActionResult::NoFeedback;
    return feedback_id == id && done == kActionDone ? ActionResult::Done : ActionResult::NotDone;
}

int16_t clampPitch(int16_t cdeg) {
    return std::clamp<int16_t>(cdeg, -kCameraPitchLimitCdeg, kCameraPitchLimitCdeg);
}
} // namespace

ActionResult UARTController::gripperActionResult() const {
    const uint8_t id = gripperActionId();
    const auto fb = latestActuatorFeedback();
    return judgeAction(id, fb.valid, fb.gripper_action_id, fb.gripper_done);
}

ActionResult UARTController::cameraPitchResult(int16_t tolerance_cdeg) const {
    bool sent;
    int16_t target;
    {
        std::lock_guard<std::mutex> guard(io_mutex_);
        sent = camera_sent_;
        target = camera_pitch_target_;
    }
    if (!sent) return ActionResult::Idle;
    const auto fb = latestActuatorFeedback();
    if (!fb.valid || fb.camera_pitch_cdeg == kCameraPitchInvalid) return ActionResult::NoFeedback;
    const int error = std::abs(static_cast<int>(fb.camera_pitch_cdeg) - target);
    return error <= std::abs(static_cast<int>(tolerance_cdeg)) ? ActionResult::Done : ActionResult::NotDone;
}

bool UARTController::sendMotion(const MotionCommand &command) {
    if (command.header != MotionPacket::kHeader) throw std::invalid_argument("Motion header must be 0x56");
    std::lock_guard<std::mutex> guard(io_mutex_);
    // 夹爪首次发送或目标变化才算新动作；写失败后重发沿用同一编号。
    const uint8_t gripper = command.gripper_open ? 1 : 0;
    if (gripper_action_id_ == 0 || gripper != gripper_target_) {
        gripper_action_id_ = nextGripperActionId(gripper_action_id_);
        gripper_target_ = gripper;
    }
    camera_pitch_target_ = clampPitch(command.camera_pitch_cdeg);
    camera_sent_ = true;
    const auto packet = buildMotionPacket(command, gripper_action_id_);

    if (dry_run_) {
        return true;
    }

    if (fd_ < 0) {
        std::cerr << "[ERR] UART is not ready\n";
        return false;
    }

    size_t written = 0;
    while (written < packet.size()) {
        const ssize_t n = ::write(fd_, packet.data() + written, packet.size() - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        std::cerr << "[ERR] UART write failed: " << std::strerror(errno) << "\n";
        return false;
    }
    int drained;
    do { drained = ::tcdrain(fd_); } while (drained < 0 && errno == EINTR);
    return drained == 0;
}

std::array<uint8_t, 13> UARTController::buildPacket(int speed1, int speed2,
                                                    int angle1, int angle2) {
    std::array<uint8_t, 13> pack{};
    pack[0] = 0x55;
    pack[1] = 13;

    const int speed1_val = clampSpeed(speed1) + 10000;
    pack[2] = static_cast<uint8_t>((speed1_val >> 8) & 0xFF);
    pack[3] = static_cast<uint8_t>(speed1_val & 0xFF);

    const int speed2_val = clampSpeed(speed2) + 10000;
    pack[4] = static_cast<uint8_t>((speed2_val >> 8) & 0xFF);
    pack[5] = static_cast<uint8_t>(speed2_val & 0xFF);

    angle1 = std::clamp(angle1, 0, 360);
    angle2 = std::clamp(angle2, 0, 360);
    pack[6] = static_cast<uint8_t>((angle1 >> 8) & 0xFF);
    pack[7] = static_cast<uint8_t>(angle1 & 0xFF);
    pack[8] = static_cast<uint8_t>((angle2 >> 8) & 0xFF);
    pack[9] = static_cast<uint8_t>(angle2 & 0xFF);

    const uint16_t crc16 = calculateCRC16(pack.data(), 2, 9);
    pack[10] = static_cast<uint8_t>(crc16 & 0xFF);
    pack[11] = static_cast<uint8_t>((crc16 >> 8) & 0xFF);
    pack[12] = 0xAA;
    return pack;
}

std::vector<uint8_t> UARTController::buildMotionPacket(const MotionCommand &command,
                                                      uint8_t gripper_action_id) {
    MotionPacket packet;
    if (command.header != MotionPacket::kHeader) throw std::invalid_argument("Motion header must be 0x56");
    const bool valid = std::isfinite(command.vx_mps) && std::isfinite(command.wz_rps);
    packet.vx_mps = valid ? command.vx_mps : 0.0f;
    packet.wz_rps = valid ? command.wz_rps : 0.0f;
    packet.gripper_open = command.gripper_open ? 1 : 0;
    packet.gripper_action_id = gripper_action_id;
    packet.camera_pitch_cdeg = clampPitch(command.camera_pitch_cdeg);
    std::vector<uint8_t> bytes(sizeof(packet));
    std::memcpy(bytes.data(), &packet, sizeof(packet));
    // CRC覆盖帧头到相机pitch，与A6反馈同一算法；按字节写入，低字节在前。
    const uint16_t crc = calculateCRC16(bytes.data(), 0,
        static_cast<uint8_t>(offsetof(MotionPacket, crc16) - 1));
    bytes[offsetof(MotionPacket, crc16)] = static_cast<uint8_t>(crc & 0xFF);
    bytes[offsetof(MotionPacket, crc16) + 1] = static_cast<uint8_t>(crc >> 8);
    return bytes;
}

uint16_t UARTController::calculateCRC16(const uint8_t *data, uint8_t start_byte,
                                        uint8_t end_byte) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = start_byte; i <= end_byte; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

int UARTController::clampSpeed(int speed) {
    return std::clamp(speed, -9999, 9999);
}

void UARTController::reopenPort() {
    closePort();
    if (!dry_run_) {
        openPort();
        running_.store(true);
        reader_ = std::thread(&UARTController::readLoop, this);
    }
}

void UARTController::openPort() {
    std::lock_guard<std::mutex> guard(io_mutex_);
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open " + port_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
        const std::string msg = "tcgetattr failed: " + std::string(std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(msg);
    }

    ::cfmakeraw(&tty);
    const speed_t speed = baudToTermios(baudrate_);
    ::cfsetospeed(&tty, speed);
    ::cfsetispeed(&tty, speed);

    tty.c_cflag = static_cast<tcflag_t>((tty.c_cflag & ~CSIZE) | CS8);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const std::string msg = "tcsetattr failed: " + std::string(std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(msg);
    }
}

void UARTController::readLoop() {
    SensorPacketParser parser;
    while (running_.load()) {
        int local_fd = -1;
        {
            std::lock_guard<std::mutex> guard(io_mutex_);
            local_fd = fd_;
        }

        if (local_fd < 0) {
            std::this_thread::sleep_for(Ms(20));
            continue;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(local_fd, &read_fds);
        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        const int ready = ::select(local_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[ERR] UART select failed: " << std::strerror(errno) << "\n";
            break;
        }
        if (ready == 0 || !FD_ISSET(local_fd, &read_fds)) {
            continue;
        }

        uint8_t bytes[256];
        const ssize_t n = ::read(local_fd, bytes, sizeof(bytes));
        if (n > 0) {
            const auto received_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    Clock::now().time_since_epoch()).count());
            for (ssize_t i = 0; i < n; ++i) {
                ActuatorFeedback state;
                if (parser.consume(bytes[i], received_us, state)) publishActuatorFeedback(state);
            }
        } else if (n < 0 && errno != EINTR && errno != EAGAIN) {
            std::cerr << "[ERR] UART read failed: " << std::strerror(errno) << "\n";
            break;
        }
    }
    std::lock_guard<std::mutex> guard(io_mutex_);
    actuator_feedback_.valid = false;
}

} // namespace rescue
