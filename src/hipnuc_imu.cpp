#include "rescue/hipnuc_imu.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace rescue {
namespace {
// Layout and CRC match hipnuc/products commit 2676ffb1214fc5eb80e1410c5b217adda1569035.
uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t u32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
float f32(const uint8_t* p) { uint32_t n = u32(p); float v; std::memcpy(&v, &n, 4); return v; }
uint16_t crc16(uint16_t crc, const uint8_t* p, size_t size) {
    for (size_t j = 0; j < size; ++j) {
        crc ^= uint16_t(p[j]) << 8;
        for (int i = 0; i < 8; ++i) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
    return crc;
}
speed_t speed(int baud) {
    switch (baud) {
    case 9600: return B9600; case 115200: return B115200;
    case 230400: return B230400; case 460800: return B460800; case 921600: return B921600;
    default: throw std::runtime_error("Unsupported IMU baudrate");
    }
}
}
uint64_t imuNowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void applyImuMounting(ImuSample& s) {
    // 绕X轴旋转180°：R = diag(1, -1, -1)，v_body = R · v_device。
    const auto rotate = [](const std::array<float, 3>& v) { return std::array<float, 3>{v[0], -v[1], -v[2]}; };
    s.body_acceleration_mps2 = rotate(s.acceleration_mps2);
    s.body_angular_velocity_rps = rotate(s.angular_velocity_rps);
    // 设备四元数为device→导航系；q_body = q_device ⊗ q_mount，q_mount = (w0, x1, y0, z0)。
    const float w = s.quaternion_wxyz[0], x = s.quaternion_wxyz[1], y = s.quaternion_wxyz[2], z = s.quaternion_wxyz[3];
    s.body_quaternion_wxyz = {-x, w, z, -y};
    // ZYX欧拉分解（roll绕X、pitch绕Y、yaw绕Z）；不使用设备的312欧拉角，避免倒装时roll停在±180°附近跳变。
    const float bw = s.body_quaternion_wxyz[0], bx = s.body_quaternion_wxyz[1];
    const float by = s.body_quaternion_wxyz[2], bz = s.body_quaternion_wxyz[3];
    s.body_rpy_rad[0] = std::atan2(2 * (bw * bx + by * bz), 1 - 2 * (bx * bx + by * by));
    s.body_rpy_rad[1] = std::asin(std::clamp(2 * (bw * by - bz * bx), -1.0f, 1.0f));
    s.body_rpy_rad[2] = std::atan2(2 * (bw * bz + bx * by), 1 - 2 * (by * by + bz * bz));
}
// 整帧82字节：5A A5(2) + 载荷长度(2) + CRC(2) + HI91载荷(76)。
// 字节下标从0开始；CRC16/XMODEM初值0、多项式0x1021，跳过CRC字段自身。
bool Hi91Parser::consume(uint8_t byte, uint64_t received_us, ImuSample& out) {
    if (last_byte_us_ && (received_us < last_byte_us_ || received_us - last_byte_us_ > 100000)) {
        if (!buffer_.empty()) ++invalid_frames;
        buffer_.clear();
    }
    last_byte_us_ = received_us;
    buffer_.push_back(byte);
    while (!buffer_.empty()) {
        // 整帧0..1：固定同步头5A A5。
        if (buffer_[0] != 0x5a || (buffer_.size() >= 2 && buffer_[1] != 0xa5)) {
            buffer_.erase(buffer_.begin()); continue;
        }
        if (buffer_.size() < 6) return false;
        const size_t length = u16(buffer_.data() + 2); // 整帧2..3：载荷长度；HI91为76。
        if (length == 0 || length > 506) { ++invalid_frames; buffer_.erase(buffer_.begin()); continue; }
        if (buffer_.size() < length + 6) return false;
        auto crc = crc16(0, buffer_.data(), 4); // 先计算整帧0..3。
        crc = crc16(crc, buffer_.data() + 6, length); // 再计算载荷6..81。
        // 整帧4..5：设备发送的CRC，低字节在前；错误帧不刷新测量。
        if (crc != u16(buffer_.data() + 4)) { ++crc_errors; buffer_.erase(buffer_.begin()); continue; }
        if (length != 76 || buffer_[6] != 0x91) { // 整帧6：HI91类型标识0x91。
            ++invalid_frames; buffer_.erase(buffer_.begin(), buffer_.begin() + length + 6); continue;
        }
        // p指向载荷起点（整帧6），所以p+n对应整帧偏移6+n。
        const auto* p = buffer_.data() + 6;
        ImuSample sample;
        sample.received_us = received_us; // 主机收帧时刻，非IMU线上字段。
        sample.sequence = ++valid_frames; // 主机帧序号，非IMU线上字段。
        sample.status = u16(p + 1); // 载荷1..2 → 整帧7..8：状态uint16，保留原值。
        sample.temperature_c = static_cast<int8_t>(p[3]); // 整帧9：int8温度，℃。
        sample.pressure_pa = f32(p + 4); // 载荷4..7 → 整帧10..13：气压，Pa。
        sample.device_time_ms = u32(p + 8); // 载荷8..11 → 整帧14..17：设备时间，ms。
        constexpr float deg_to_rad = 0.017453292519943295f; // π/180：角度转弧度。
        bool finite = std::isfinite(sample.pressure_pa);
        for (int i = 0; i < 3; ++i) {
            // 载荷12..23 → 整帧18..29：x/y/z，每轴float32；g→m/s²（未去重力）。
            sample.acceleration_mps2[i] = f32(p + 12 + 4*i) * 9.8f;
            // 载荷24..35 → 整帧30..41：x/y/z，每轴float32；°/s→rad/s。
            sample.angular_velocity_rps[i] = f32(p + 24 + 4*i) * deg_to_rad;
            // 载荷36..47 → 整帧42..53：x/y/z磁场，µT。
            sample.magnetic_ut[i] = f32(p + 36 + 4*i);
            // 载荷48..59 → 整帧54..65：i=0横滚、1俯仰、2偏航；°→rad。
            sample.rpy_rad[i] = f32(p + 48 + 4*i) * deg_to_rad;
            finite = finite && std::isfinite(sample.acceleration_mps2[i]) && std::isfinite(sample.angular_velocity_rps[i])
                && std::isfinite(sample.magnetic_ut[i]) && std::isfinite(sample.rpy_rad[i]);
        }
        float norm2 = 0;
        for (int i = 0; i < 4; ++i) {
            // 载荷60..75 → 整帧66..81：i=0为w，1/2/3为x/y/z，无量纲。
            sample.quaternion_wxyz[i] = f32(p + 60 + 4*i);
            finite = finite && std::isfinite(sample.quaternion_wxyz[i]);
            norm2 += sample.quaternion_wxyz[i] * sample.quaternion_wxyz[i];
        }
        // 上位机数值有效性标记；不代替设备状态位、姿态收敛和安装方向检查。
        sample.measurements_valid = finite && norm2 > 0.81f && norm2 < 1.21f;
        applyImuMounting(sample);
        out = sample;
        buffer_.erase(buffer_.begin(), buffer_.begin() + length + 6);
        return true;
    }
    return false;
}
HipnucImu::HipnucImu(const std::string& port, int baud, uint32_t timeout_ms) {
    const auto baud_code = speed(baud);
    if (!timeout_ms || timeout_ms > 10000) throw std::runtime_error("IMU timeout must be 1..10000 ms");
    state_.timeout_ms = timeout_ms;
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("Cannot open IMU " + port + ": " + std::strerror(errno));
    try {
        if (::flock(fd_, LOCK_EX | LOCK_NB) < 0 || ::ioctl(fd_, TIOCEXCL) < 0)
            throw std::runtime_error("IMU port is already owned or cannot be locked");
        termios tty{};
        if (::tcgetattr(fd_, &tty) < 0) throw std::runtime_error("Cannot read IMU serial settings");
        ::cfmakeraw(&tty);
        ::cfsetispeed(&tty, baud_code); ::cfsetospeed(&tty, baud_code);
        tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
        if (::tcsetattr(fd_, TCSANOW, &tty) < 0) throw std::runtime_error("Cannot configure IMU serial port");
        ::tcflush(fd_, TCIFLUSH);
        state_.connected = true;
        running_ = true;
        reader_ = std::thread(&HipnucImu::readLoop, this);
    } catch (...) {
        ::ioctl(fd_, TIOCNXCL); ::close(fd_); fd_ = -1; throw;
    }
}
HipnucImu::~HipnucImu() {
    running_ = false;
    if (reader_.joinable()) reader_.join();
    if (fd_ >= 0) { ::ioctl(fd_, TIOCNXCL); ::close(fd_); }
}
ImuSnapshot HipnucImu::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = state_;
    result.sampled_at_us = imuNowUs();
    if (result.sample.received_us && result.sampled_at_us >= result.sample.received_us)
        result.age_ms = (result.sampled_at_us - result.sample.received_us) / 1000.0;
    result.fresh = result.connected && result.age_ms >= 0 && result.age_ms <= result.timeout_ms;
    return result;
}
void HipnucImu::readLoop() {
    Hi91Parser parser;
    uint8_t bytes[4096];
    while (running_) {
        pollfd pfd{fd_, POLLIN, 0};
        int ready = ::poll(&pfd, 1, 50);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            std::lock_guard<std::mutex> lock(mutex_); ++state_.io_errors; break;
        }
        if (!ready) continue;
        const auto n = ::read(fd_, bytes, sizeof(bytes));
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        if (n <= 0) { std::lock_guard<std::mutex> lock(mutex_); ++state_.io_errors; break; }
        const auto now = imuNowUs();
        std::lock_guard<std::mutex> lock(mutex_);
        state_.bytes += n;
        for (ssize_t i = 0; i < n; ++i) {
            ImuSample sample;
            if (!parser.consume(bytes[i], now, sample)) continue;
            if (state_.sample.received_us) {
                const uint32_t delta = sample.device_time_ms - state_.sample.device_time_ms;
                if (delta == 0) { ++state_.duplicate_times; continue; }
                if (delta >= 0x80000000U) { ++state_.backward_times; continue; }
            }
            // Bad numeric data replaces the last sample, so healthy cached values never mask a fault.
            state_.sample = sample;
        }
        state_.valid_frames = parser.valid_frames;
        state_.crc_errors = parser.crc_errors;
        state_.invalid_frames = parser.invalid_frames;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
}
}
