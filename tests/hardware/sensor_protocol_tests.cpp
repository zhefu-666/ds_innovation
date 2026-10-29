#include "rescue/hardware/uart_controller.hpp"
#include "rescue/hardware/motion_link.hpp"
#include <cstring>
#include <vector>
#include "rescue/core/utils.hpp"
#include <cassert>
#include <fcntl.h>
#include <cstdlib>
#include <unistd.h>
using namespace rescue;

// 模拟下位机打包8字节A6反馈（pitch为int16小端），CRC覆盖0..4，末尾0x0A。
static void packFeedback(uint8_t* out,
                         uint8_t gripper_done,
                         uint8_t gripper_action_id,
                         int16_t camera_pitch_cdeg) {
    const auto pitch = static_cast<uint16_t>(camera_pitch_cdeg == kCameraPitchInvalid
                                                 ? kCameraPitchInvalid
                                                 : camera_pitch_cdeg / 100 + 120);
    out[0] = SensorPacket::kHeader;
    out[1] = gripper_done;
    out[2] = gripper_action_id;
    out[3] = static_cast<uint8_t>(pitch & 0xFF);
    out[4] = static_cast<uint8_t>(pitch >> 8);
    const uint16_t crc = UARTController::calculateCRC16(out, 0, 4);
    out[5] = static_cast<uint8_t>(crc & 0xFF);
    out[6] = static_cast<uint8_t>(crc >> 8);
    out[7] = SensorPacket::kNewline;
}

int main() {
    {
        MotionLinkConfig c;
        MotionVelocityRamp ramp(c);
        MotionCommand target;
        target.vx_mps = .16f;
        target.wz_rps = .2f;
        target.gripper_offset = 20;
        target.camera_pitch_cdeg = 1200;
        uint64_t at = 1000000;
        auto sent = ramp.next(target, at);
        assert(sent.vx_mps == 0 && sent.wz_rps == 0); // start from rest
        assert(sent.gripper_offset == 20 && sent.camera_pitch_cdeg == 1200);
        ramp.accept(sent, at);
        for (int i = 0; i < 30; ++i) {
            at += 20000;
            const auto next = ramp.next(target, at);
            assert(next.vx_mps >= sent.vx_mps && next.vx_mps <= target.vx_mps);
            assert(next.wz_rps >= sent.wz_rps && next.wz_rps <= target.wz_rps);
            assert(next.vx_mps - sent.vx_mps <= c.linear_accel_mps2 * .020f + 1e-6f);
            assert(next.wz_rps - sent.wz_rps <= c.angular_accel_rps2 * .020f + 1e-6f);
            ramp.accept(next, at);
            sent = next;
        }
        assert(std::abs(sent.vx_mps - .16f) < 1e-6f && std::abs(sent.wz_rps - .2f) < 1e-6f);
        // Reversal must cross zero, including rapid changes of steering sign.
        target.vx_mps = -.16f;
        target.wz_rps = -.2f;
        bool linear_zero = false, angular_zero = false;
        for (int i = 0; i < 65; ++i) {
            at += 20000;
            auto next = ramp.next(target, at);
            if (sent.vx_mps > 0)
                assert(next.vx_mps >= 0);
            if (sent.wz_rps > 0)
                assert(next.wz_rps >= 0);
            if (next.vx_mps < 0)
                assert(linear_zero);
            if (next.wz_rps < 0)
                assert(angular_zero);
            linear_zero = linear_zero || next.vx_mps == 0;
            angular_zero = angular_zero || next.wz_rps == 0;
            assert(std::abs(next.vx_mps - sent.vx_mps) <= c.linear_decel_mps2 * .02f + 1e-6f);
            assert(std::abs(next.wz_rps - sent.wz_rps) <= c.angular_decel_rps2 * .02f + 1e-6f);
            ramp.accept(next, at);
            sent = next;
        }
        assert(linear_zero && angular_zero && std::abs(sent.vx_mps + .16f) < 1e-6f);
        // Normal zero requests decelerate; explicit safety stops reset immediately.
        target.vx_mps = target.wz_rps = 0;
        at += 20000;
        sent = ramp.next(target, at);
        assert(sent.vx_mps < 0 && sent.wz_rps < 0);
        ramp.accept(sent, at);
        at += 1;
        sent = ramp.next(target, at, true);
        assert(sent.vx_mps == 0 && sent.wz_rps == 0);
        ramp.accept(sent, at);
        target.vx_mps = .16f;
        target.wz_rps = .2f;
        // A scheduler pause cannot accumulate an arbitrarily large acceleration step.
        sent = ramp.next(target, at + 1000000);
        assert(sent.vx_mps <= c.linear_accel_mps2 * .04f + 1e-6f);
        assert(sent.wz_rps <= c.angular_accel_rps2 * .04f + 1e-6f);
    }

    // 金向量由独立Python CRC计算，校验打包与CRC实现。
    using Frame = std::array<uint8_t, 8>;
    Frame frame{};
    packFeedback(frame.data(), 1, 1, 3000);
    assert((frame == Frame{0xA6, 0x01, 0x01, 0x96, 0x00, 0x13, 0x85, 0x0A}));
    Frame moving{};
    packFeedback(moving.data(), 0, 1, 0);
    assert((moving == Frame{0xA6, 0x00, 0x01, 0x78, 0x00, 0x5F, 0xD9, 0x0A}));
    Frame up{};
    packFeedback(up.data(), 1, 2, -1500);
    assert((up == Frame{0xA6, 0x01, 0x02, 0x69, 0x00, 0xA2, 0x75, 0x0A}));
    SensorPacketParser parser;
    ActuatorFeedback state;
    auto feed = [&](const auto& bytes, uint64_t time) {
        int count = 0;
        for (auto byte : bytes)
            if (parser.consume(byte, time, state))
                ++count;
        return count;
    };
    assert(feed(std::array<uint8_t, 2>{0x01, 0x55}, 1000) == 0);
    for (size_t i = 0; i < 7; ++i)
        assert(!parser.consume(frame[i], 2000, state));
    assert(state.timestamp_us == 0 && !state.valid);
    assert(parser.consume(frame[7], 2001, state));
    assert(state.timestamp_us == 2001 && state.gripper_open == 1 && state.gripper_action_id == 1 &&
           state.camera_pitch_cdeg == 3000 && state.valid);
    assert(feed(moving, 3000) == 1 && state.gripper_open == 0);
    assert(feed(frame, 3000) == 1 && state.gripper_open == 1); // concatenated frames
    auto corrupt = frame;
    corrupt[4] ^= 1; // pitch高字节损坏
    assert(feed(corrupt, 4000) == 0 && state.timestamp_us == 3000);
    assert(feed(frame, 5000) == 1);
    auto no_newline = frame;
    no_newline[7] = 0x0D; // CRC正确但帧尾不是0x0A
    assert(feed(no_newline, 5500) == 0 && state.timestamp_us == 5000);
    assert(feed(frame, 5600) == 1);
    auto dropped = std::vector<uint8_t>(frame.begin(), frame.end());
    dropped.erase(dropped.begin() + 1);
    assert(feed(dropped, 6000) == 0 && feed(frame, 7000) == 1);
    for (int i = 0; i < 2; ++i)
        assert(!parser.consume(frame[i], 8000, state));
    assert(feed(frame, 200000) == 1); // partial frame timeout
    assert(!parser.consume(0xA6, 201000, state));
    assert(feed(frame, 1000) == 1); // time moves backwards: discard partial bytes
    assert(feed(std::array<uint8_t, 5>{0xA6, 0xA6, 0xA6, 0xA6, 0xA6}, 2000) == 0);
    assert(feed(frame, 3000) == 1); // repeated header resynchronization
    Frame raw{};
    raw = Frame{0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0x15, 0x68, 0x0A};
    assert((raw == Frame{0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0x15, 0x68, 0x0A}));
    assert(feed(raw, 4000) == 1 && state.gripper_open == 0xA6 &&
           state.camera_pitch_cdeg == kCameraPitchInvalid); // raw payload retained

    {
        for (int wire : {0, 40, 89, 90, 115, 120, 125, 150, 151, -32768}) {
            Frame f{0xA6,
                    0,
                    0,
                    static_cast<uint8_t>(wire & 255),
                    static_cast<uint8_t>((wire >> 8) & 255),
                    0,
                    0,
                    10};
            auto crc = UARTController::calculateCRC16(f.data(), 0, 4);
            f[5] = crc & 255;
            f[6] = crc >> 8;
            assert(feed(f, 6000) == 1);
            const int expected =
                wire >= 90 && wire <= 150 ? (wire - 120) * 100 : kCameraPitchInvalid;
            assert(state.camera_pitch_cdeg == expected);
            if (expected != kCameraPitchInvalid) {
                assert(cameraPitchToFeedbackDeg(expected) == wire);
                MotionCommand c;
                c.camera_pitch_cdeg = expected;
                const auto b = UARTController::buildMotionPacket(c, 0);
                const uint16_t packed = uint16_t(b[11]) | (uint16_t(b[12]) << 8);
                assert(static_cast<int16_t>(packed) == std::clamp(wire - 120, -30, 30));
            }
        }
    }

    {
        // dry-run也按目标变化分配编号，不打开串口。
        UARTController dry;
        dry.configureFrameFeedback(1, 0);
        dry.initUART("/dev/null", 115200, true, false);
        assert(dry.gripperActionId() == 0 &&
               dry.gripperActionResult() == GripperActionResult::Idle);
        assert(dry.cameraPitchResult(300) == ActionResult::Idle);
        MotionCommand c;
        c.gripper_offset = 0;
        assert(dry.sendMotion(c) && dry.gripperActionId() == 1);
        assert(dry.sendMotion(c) && dry.gripperActionId() == 1); // 重复关闭不换编号
        c.vx_mps = 0.2f;
        assert(dry.sendMotion(c) && dry.gripperActionId() == 1); // 只改速度不换编号
        c.gripper_offset = 20;
        assert(dry.sendMotion(c) && dry.gripperActionId() == 2);
        c.header = 0x55;
        bool rejected = false;
        try {
            dry.sendMotion(c);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected && dry.gripperActionId() == 2); // 非法包不消耗编号
        c.header = 0x56;
        assert(dry.cameraPitchTarget() == 0 &&
               dry.cameraPitchResult(300) == ActionResult::NoFeedback);
        c.camera_pitch_cdeg = 4500;
        assert(dry.sendMotion(c) && dry.cameraPitchTarget() == 3000 &&
               dry.gripperActionId() == 2); // 改pitch不换夹爪编号
        c.camera_pitch_cdeg = -20000;
        assert(dry.sendMotion(c) && dry.cameraPitchTarget() == -3000); // 限幅
    }

    {
        // Only the two accepted logical angles are writable. Retries keep the ID because MCU counts
        // offset changes only.
        UARTController dry;
        dry.configureFrameFeedback(1, 0);
        dry.initUART("/dev/null", 115200, true, false);
        MotionCommand c;
        c.gripper_offset = 20;
        assert(dry.sendMotion(c));
        c.gripper_offset = 21;
        assert(!dry.sendMotion(c) && dry.gripperActionId() == 1);
        c.gripper_offset = -20;
        assert(!dry.sendMotion(c) && dry.gripperActionId() == 1);
        c.gripper_offset = 20;
        c.frame_transaction = 1;
        assert(dry.sendMotion(c) && dry.gripperActionId() == 1);
        assert(dry.sendMotion(c) && dry.gripperActionId() == 1);
        c.frame_transaction = 2;
        assert(dry.sendMotion(c) && dry.gripperActionId() == 1);
    }

    // A virtual terminal exercises real read/write code without opening hardware.
    int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    UARTController uart;
    uart.configureFrameFeedback(1, 0);
    uart.initUART(ptsname(master), 115200, false, false);
    assert(write(master, frame.data(), 2) == 2);
    std::this_thread::sleep_for(Ms(20));
    assert(uart.latestActuatorFeedback().timestamp_us == 0);
    assert(write(master, frame.data() + 2, 6) == 6);
    auto deadline = Clock::now() + Ms(1000);
    while (uart.latestActuatorFeedback().timestamp_us == 0 && Clock::now() < deadline)
        std::this_thread::sleep_for(Ms(5));
    auto received = uart.latestActuatorFeedback();
    assert(received.timestamp_us > 0 && received.gripper_open == 1 &&
           received.gripper_action_id == 1 && received.valid);
    ActuatorFeedback older{received.timestamp_us - 1, 0, 0, 0, true};
    uart.publishActuatorFeedback(older);
    assert(uart.latestActuatorFeedback().gripper_open == 1);
    assert(write(master, corrupt.data(), corrupt.size()) == 8);
    std::this_thread::sleep_for(Ms(30));
    assert(uart.latestActuatorFeedback().timestamp_us == received.timestamp_us);
    assert(uart.getLatestCmd() == 0xAA);
    std::this_thread::sleep_for(Ms(220));
    const auto stale = uart.latestActuatorFeedback();
    assert(stale.timestamp_us == received.timestamp_us && !stale.valid);

    // 下位机写一帧反馈，等待上位机收到后返回。
    int16_t cam_pitch = 0; // 夹爪场景中相机读回保持平视
    auto mcuReport = [&](uint8_t done, uint8_t id) {
        Frame f{};
        packFeedback(f.data(), done, id, cam_pitch);
        const auto before = uart.latestActuatorFeedback().timestamp_us;
        assert(write(master, f.data(), f.size()) == 8);
        auto until = Clock::now() + Ms(1000);
        while (uart.latestActuatorFeedback().timestamp_us == before && Clock::now() < until)
            std::this_thread::sleep_for(Ms(5));
        const auto fb = uart.latestActuatorFeedback();
        assert(fb.valid && fb.gripper_open == done && fb.gripper_action_id == id &&
               fb.camera_pitch_cdeg == cam_pitch);
    };
    using Out = std::array<uint8_t, MotionPacket::kSize>;
    auto readFrame = [&](Out& out) {
        size_t count = 0;
        auto until = Clock::now() + Ms(1000);
        while (count < out.size() && Clock::now() < until) {
            auto n = read(master, out.data() + count, out.size() - count);
            if (n > 0)
                count += static_cast<size_t>(n);
            else
                std::this_thread::sleep_for(Ms(5));
        }
        return count;
    };
    using R = GripperActionResult;
    // 上电：下位机报“已完成、编号0”，上位机尚未发动作。
    mcuReport(1, 0);
    assert(uart.gripperActionResult() == R::Idle);

    MotionCommand command;
    command.vx_mps = 0.1f;
    command.wz_rps = -0.25f;
    command.gripper_offset = 20;
    Out outgoing{};
    // 第一次关闭：编号1
    assert(uart.sendMotion(command) && uart.gripperActionId() == 1);
    assert(readFrame(outgoing) == MotionPacket::kSize);
    assert((outgoing == Out{0x56,
                            0xCD,
                            0xCC,
                            0xCC,
                            0x3D,
                            0x00,
                            0x00,
                            0x80,
                            0xBE,
                            0xFB,
                            0x01,
                            0x00,
                            0x00,
                            0x79,
                            0x11}));
    assert(uart.gripperActionResult() == R::NotDone); // 反馈编号仍为0
    // 关闭过程中重复发送“关闭”：编号不变，字节完全相同
    assert(uart.sendMotion(command) && uart.gripperActionId() == 1);
    assert(readFrame(outgoing) == MotionPacket::kSize);
    assert((outgoing == Out{0x56,
                            0xCD,
                            0xCC,
                            0xCC,
                            0x3D,
                            0x00,
                            0x00,
                            0x80,
                            0xBE,
                            0xFB,
                            0x01,
                            0x00,
                            0x00,
                            0x79,
                            0x11}));
    mcuReport(1, 1);
    assert(uart.gripperActionResult() == R::NotDone);
    mcuReport(0, 1);
    assert(uart.gripperActionResult() == R::Done); // 两次“关闭”共用一次完成

    // 张开：编号2；下位机仍报旧的“已完成、编号1”，不能误判
    command.gripper_offset = 0;
    assert(uart.sendMotion(command) && uart.gripperActionId() == 2);
    assert(readFrame(outgoing) == MotionPacket::kSize);
    assert((outgoing == Out{0x56,
                            0xCD,
                            0xCC,
                            0xCC,
                            0x3D,
                            0x00,
                            0x00,
                            0x80,
                            0xBE,
                            0x05,
                            0x02,
                            0x00,
                            0x00,
                            0xB8,
                            0xF9}));
    assert(uart.gripperActionResult() == R::NotDone);
    mcuReport(1, 1); // 编号不符：即使标志为完成也不采信
    assert(uart.gripperActionResult() == R::NotDone);
    mcuReport(1, 2);
    assert(uart.gripperActionResult() == R::Done);

    // 再关闭：编号3；下位机报旧的“已完成、编号1”也不采信
    command.gripper_offset = 20;
    assert(uart.sendMotion(command) && uart.gripperActionId() == 3);
    assert(readFrame(outgoing) == MotionPacket::kSize && outgoing[9] == 0xFB && outgoing[10] == 3 &&
           outgoing[11] == 0 && outgoing[12] == 0);
    const uint16_t crc = UARTController::calculateCRC16(outgoing.data(), 0, 12);
    assert(outgoing[13] == (crc & 0xFF) && outgoing[14] == (crc >> 8));
    mcuReport(1, 1);
    assert(uart.gripperActionResult() == R::NotDone);
    mcuReport(0x7F, 3); // 非1的值不算完成
    assert(uart.gripperActionResult() == R::NotDone);
    mcuReport(0, 3);
    assert(uart.gripperActionResult() == R::Done);
    std::this_thread::sleep_for(Ms(220));
    assert(uart.gripperActionResult() == R::NoFeedback); // 反馈过期

    // 相机pitch：目标0、读回0 → 到位；改为向下30°后读回未跟上 → 未到位
    assert(uart.cameraPitchResult(300) == R::NoFeedback); // 反馈过期
    mcuReport(0, 3);
    assert(uart.cameraPitchTarget() == 0 && uart.cameraPitchResult(300) == R::Done);
    command.camera_pitch_cdeg = 3000;
    assert(uart.sendMotion(command) && uart.cameraPitchTarget() == 3000 &&
           uart.gripperActionId() == 3);
    assert(readFrame(outgoing) == MotionPacket::kSize && outgoing[10] == 3 && outgoing[11] == 30 &&
           outgoing[12] == 0);
    assert(uart.cameraPitchResult(300) == R::NotDone);
    cam_pitch = 1500;
    mcuReport(0, 3);
    assert(uart.cameraPitchResult(300) == R::NotDone);
    cam_pitch = 2800;
    mcuReport(0, 3); // 误差2°，在3°容差内
    assert(uart.cameraPitchResult(300) == R::Done && uart.cameraPitchResult(100) == R::NotDone);
    assert(uart.gripperActionResult() == R::Done); // 相机与夹爪互不影响
    cam_pitch = kCameraPitchInvalid;
    mcuReport(0, 3);
    assert(uart.cameraPitchResult(300) == R::NoFeedback && uart.gripperActionResult() == R::Done);

    assert(!uart.execute()); // 旧版电机/舵机协议已停用，不得发送任何字节
    uint8_t extra = 0;
    assert(read(master, &extra, 1) < 0);
    uart.closePort();
    assert(!uart.latestActuatorFeedback().valid);
    assert(!uart.sendMotion(command));
    close(master);

    {
        // MotionLink on a fresh PTY: id sync with a restarted host, resend, stall -> zero, stop.
        int mcu = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        assert(mcu >= 0 && grantpt(mcu) == 0 && unlockpt(mcu) == 0);
        const std::string path = ptsname(mcu);
        UARTController link_uart;
        link_uart.configureFrameFeedback(1, 0);
        link_uart.initUART(path, 115200, false, false);
        {
            // 独占：第二个进程/对象不能同时打开同一串口。
            UARTController second;
            second.configureFrameFeedback(1, 0);
            bool owned = false;
            try {
                second.initUART(path, 115200, false, false);
            } catch (const std::runtime_error&) {
                owned = true;
            }
            assert(owned);
        }
        auto report = [&](uint8_t done, uint8_t id, int16_t pitch) {
            Frame f{};
            packFeedback(f.data(), done, id, pitch);
            assert(write(mcu, f.data(), f.size()) == 8);
        };
        // Collect host->MCU packets for a while; returns complete 15-byte frames.
        auto collect = [&](int ms) {
            std::vector<uint8_t> bytes;
            uint8_t buf[256];
            const auto until = Clock::now() + Ms(ms);
            while (Clock::now() < until) {
                const auto n = read(mcu, buf, sizeof(buf));
                if (n > 0)
                    bytes.insert(bytes.end(), buf, buf + n);
                else
                    std::this_thread::sleep_for(Ms(2));
            }
            assert(bytes.size() % MotionPacket::kSize == 0);
            std::vector<Out> frames(bytes.size() / MotionPacket::kSize);
            for (size_t i = 0; i < frames.size(); ++i)
                std::memcpy(
                    frames[i].data(), bytes.data() + MotionPacket::kSize * i, MotionPacket::kSize);
            return frames;
        };
        auto vx = [](const Out& o) {
            float v;
            std::memcpy(&v, o.data() + 1, 4);
            return v;
        };
        auto pitch = [](const Out& o) {
            return int16_t(o[11] | (o[12] << 8));
        };
        // 下位机已停在上次运行的编号7且为关闭：重启后的第一包要张开，必须用8，否则会被当成重复包。
        report(1, 7, 1200);
        assert(link_uart.syncGripperActionId(Ms(500)));
        assert(link_uart.gripperActionId() == 0 && link_uart.gripperAck().result == R::Idle &&
               link_uart.gripperAck().target == -1);
        MotionLink link(link_uart, {40, 150});
        assert(collect(120).empty()); // nothing is sent before the first command
        assert(!link.healthy());
        MotionCommand drive;
        drive.vx_mps = 0.1f;
        drive.wz_rps = 0.2f;
        drive.gripper_offset = 20;
        drive.camera_pitch_cdeg = 1200;
        link.submit(drive);
        auto frames = collect(100);
        assert(frames.size() >= 2); // immediate write plus periodic resend
        assert(vx(frames.front()) == 0 && vx(frames.back()) > 0 && vx(frames.back()) < .1f);
        float previous_vx = 0;
        for (const auto& f : frames) {
            assert(f[0] == 0x56 && vx(f) >= previous_vx && vx(f) <= .1f &&
                   vx(f) - previous_vx <= .30f * .040f + 1e-6f && f[9] == 0xFB && f[10] == 8 &&
                   pitch(f) == 12);
            previous_vx = vx(f);
            const uint16_t c = UARTController::calculateCRC16(f.data(), 0, 12);
            assert(f[13] == (c & 0xFF) && f[14] == (c >> 8));
        }
        link.submit(drive); // next task frame, same command
        report(1, 7, 1200); // old id: not an acknowledgement
        std::this_thread::sleep_for(Ms(20));
        assert(link.healthy() && link_uart.gripperAck().result == R::NotDone &&
               link_uart.gripperAck().target == 0);
        report(0, 8, 1200);
        std::this_thread::sleep_for(Ms(20));
        assert(link_uart.gripperAck().result == R::Done && link_uart.gripperAck().target == 0);
        // Stalled task loop: after 150 ms the resend becomes zero velocity, gripper/pitch kept.
        frames = collect(250);
        assert(!frames.empty() && vx(frames.back()) == 0.0f && frames.back()[9] == 0xFB &&
               frames.back()[10] == 8 && pitch(frames.back()) == 12);
        assert(link.stats().stale > 0 && link.stats().failed == 0);
        assert(!link.healthy()); // no fresh command within the timeout
        // A new command ramps from rest after a stale stop; gripper changes still take id 9.
        drive.gripper_offset = 0;
        link.submit(drive);
        frames = collect(60);
        assert(!frames.empty() && vx(frames.front()) >= 0 && vx(frames.back()) > 0 &&
               vx(frames.back()) < .1f && frames.front()[9] == 5 && frames.front()[10] == 9);
        // Explicit pixel stop bypasses the ramp; verify both axes on virtual UART.
        MotionCommand pixel_stop = drive;
        pixel_stop.vx_mps = pixel_stop.wz_rps = 0;
        link.submit(pixel_stop, true);
        frames = collect(30);
        assert(!frames.empty() && vx(frames.back()) == 0);
        float stopped_wz;
        std::memcpy(&stopped_wz, frames.back().data() + 5, 4);
        assert(stopped_wz == 0 && frames.back()[9] == 5 && frames.back()[10] == 9 &&
               pitch(frames.back()) == 12);
        link.submit(drive);
        frames = collect(35);
        assert(!frames.empty() && vx(frames.back()) < .1f);
        link.stop();
        frames = collect(120); // a resend may precede the final stop packet
        assert(!frames.empty() && frames.size() <= 2 && vx(frames.back()) == 0.0f &&
               frames.back()[9] == 5 && frames.back()[10] == 9);
        assert(!link.healthy());
        link.submit(drive); // ignored after stop
        assert(collect(80).empty());
        {
            // Invalid sends fail without bytes; valid retries resume fixed 15-byte output.
            MotionLink retry(link_uart, {20, 150});
            MotionCommand invalid;
            invalid.header = 0x55;
            retry.submit(invalid);
            const auto deadline = Clock::now() + Ms(500);
            while (retry.stats().failed == 0 && Clock::now() < deadline)
                std::this_thread::sleep_for(Ms(2));
            assert(retry.stats().failed > 0 && collect(30).empty());
            invalid.header = 0x56;
            retry.submit(invalid);
            const auto resumed = collect(60);
            assert(!resumed.empty());
            for (const auto& f : resumed) {
                const uint16_t crc = UARTController::calculateCRC16(f.data(), 0, 12);
                assert(f.size() == 15 && f[13] == (crc & 255) && f[14] == (crc >> 8));
            }
            retry.stop();
            collect(60);
        }
        {
            // Output authorization is checked in the resend worker, not only in the task loop.
            std::atomic<bool> permit{false};
            MotionLink gated(link_uart, {20, 150}, [&] {
                return permit.load();
            });
            drive.gripper_offset = 20;
            drive.camera_pitch_cdeg = 2200;
            gated.submit(drive);
            assert(collect(50).empty()); // unarmed: no actuator commands at all
            permit = true;
            gated.submit(drive);
            frames = collect(50);
            assert(!frames.empty() && vx(frames.back()) > 0 && frames.back()[9] == 0xFB);
            permit = false;
            drive.gripper_offset = 0;
            drive.camera_pitch_cdeg = 3500;
            gated.submit(drive);
            frames = collect(70);
            assert(!frames.empty());
            const auto& last = frames.back();
            assert(vx(last) == 0 && last[9] == 0xFB &&
                   pitch(last) == 22); // freeze last authorized actuator targets
            float last_wz;
            std::memcpy(&last_wz, last.data() + 5, 4);
            assert(last_wz == 0);
            gated.stop();
            collect(40);
        }
        link_uart.closePort();
        close(mcu);
    }
    {
        // 上位机重启、下位机未重启：先接管下位机编号与夹爪状态。保持当前状态不算新动作（下位机也不计数），
        // 状态变化才用编号+1，之后下位机回报的编号与上位机一致。
        int mcu = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        assert(mcu >= 0 && grantpt(mcu) == 0 && unlockpt(mcu) == 0);
        UARTController sync_uart;
        sync_uart.configureFrameFeedback(1, 0);
        sync_uart.initUART(ptsname(mcu), 115200, false, false);
        auto report = [&](uint8_t open, uint8_t id) {
            Frame f{};
            packFeedback(f.data(), open, id, 500);
            assert(write(mcu, f.data(), f.size()) == 8);
            std::this_thread::sleep_for(Ms(20));
        };
        auto lastId = [&] {
            uint8_t buf[256];
            std::vector<uint8_t> bytes;
            const auto until = Clock::now() + Ms(30);
            while (Clock::now() < until) {
                const auto n = read(mcu, buf, sizeof(buf));
                if (n > 0)
                    bytes.insert(bytes.end(), buf, buf + n);
                else
                    std::this_thread::sleep_for(Ms(2));
            }
            assert(bytes.size() >= MotionPacket::kSize && bytes.size() % MotionPacket::kSize == 0);
            return bytes[bytes.size() - MotionPacket::kSize + 10];
        };
        using R = GripperActionResult;
        report(0, 0); // 下位机：关闭，编号0
        assert(sync_uart.syncGripperActionId(Ms(500)));
        MotionCommand c;
        c.gripper_offset = 20;
        c.camera_pitch_cdeg = 500;
        assert(sync_uart.sendMotion(c) && lastId() == 0); // 保持关闭：沿用0，不触发动作
        assert(sync_uart.gripperAck().result == R::Idle);
        assert(sync_uart.sendMotion(c) && lastId() == 0);
        c.gripper_offset = 0;
        assert(sync_uart.sendMotion(c) && lastId() == 1 && sync_uart.gripperActionId() == 1);
        report(0, 0);
        assert(sync_uart.gripperAck().result == R::NotDone);
        report(1, 1);
        assert(sync_uart.gripperAck().result == R::Done && sync_uart.gripperAck().target == 1);
        c.gripper_offset = 20;
        assert(sync_uart.sendMotion(c) && lastId() == 2);
        report(1, 1);
        assert(sync_uart.gripperAck().result == R::NotDone);
        report(0, 2);
        assert(sync_uart.gripperAck().result == R::Done && sync_uart.gripperAck().target == 0);
        sync_uart.closePort();

        // 下位机停在编号5且已张开：首包保持张开沿用5（状态已一致即完成），关闭用6。
        UARTController open_uart;
        open_uart.configureFrameFeedback(1, 0);
        open_uart.initUART(ptsname(mcu), 115200, false, false);
        report(1, 5);
        assert(open_uart.syncGripperActionId(Ms(500)));
        c.gripper_offset = 0;
        assert(open_uart.sendMotion(c) && lastId() == 5 &&
               open_uart.gripperAck().result == R::Done);
        c.gripper_offset = 20;
        assert(open_uart.sendMotion(c) && lastId() == 6);
        report(1, 5);
        assert(open_uart.gripperAck().result == R::NotDone);
        report(0, 6);
        assert(open_uart.gripperAck().result == R::Done);
        open_uart.closePort();
        close(mcu);
    }
    {
        // TEMP_ASSUMPTION done-flag A6: byte1==1 means "action finished" for both open and close.
        int mcu = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        assert(mcu >= 0 && grantpt(mcu) == 0 && unlockpt(mcu) == 0);
        auto report = [&](uint8_t done, uint8_t id) {
            Frame f{};
            packFeedback(f.data(), done, id, 500);
            assert(write(mcu, f.data(), f.size()) == 8);
            std::this_thread::sleep_for(Ms(20));
        };
        auto drain = [&] {
            uint8_t buf[512];
            while (read(mcu, buf, sizeof(buf)) > 0) {
            }
        };
        auto lastId = [&] {
            uint8_t buf[256];
            std::vector<uint8_t> bytes;
            const auto until = Clock::now() + Ms(30);
            while (Clock::now() < until) {
                const auto n = read(mcu, buf, sizeof(buf));
                if (n > 0)
                    bytes.insert(bytes.end(), buf, buf + n);
                else
                    std::this_thread::sleep_for(Ms(2));
            }
            assert(bytes.size() >= MotionPacket::kSize && bytes.size() % MotionPacket::kSize == 0);
            return bytes[bytes.size() - MotionPacket::kSize + 10];
        };
        using R = GripperActionResult;
        bool rejected = false;
        try {
            UARTController bad;
            bad.configureDoneFlagFeedback(10, 24, 300);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
        rejected = false;
        try {
            UARTController bad;
            bad.configureDoneFlagFeedback(0, 0, 300);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
        // Recorded id differs from the MCU: refuse, never guess the frame position.
        {
            UARTController wrong;
            wrong.configureDoneFlagFeedback(0, 23, 300);
            wrong.initUART(ptsname(mcu), 115200, false, false);
            report(1, 24);
            assert(!wrong.syncGripperActionId(Ms(100)));
            assert(wrong.confirmedFrameOpen(wrong.latestActuatorFeedback()) == -1);
            wrong.closePort();
        }
        // Previous action not finished (flag 0): refuse.
        {
            UARTController busy;
            busy.configureDoneFlagFeedback(0, 24, 300);
            busy.initUART(ptsname(mcu), 115200, false, false);
            report(0, 24);
            assert(!busy.syncGripperActionId(Ms(100)));
            busy.closePort();
        }
        drain();
        UARTController done;
        done.configureDoneFlagFeedback(0, 24, 300);
        assert(done.doneFlagFeedback());
        done.initUART(ptsname(mcu), 115200, false, false);
        report(1, 24);
        assert(done.syncGripperActionId(Ms(500)));
        assert(done.confirmedFrameOpen(done.latestActuatorFeedback()) ==
               1); // known open, not read from A6
        MotionCommand c;
        c.gripper_offset = 0;
        c.camera_pitch_cdeg = 500;
        assert(done.sendMotion(c) && lastId() == 24); // holding open keeps id 24
        report(1, 24);
        assert(done.gripperAck().result == R::Done && done.gripperAck().target == 1);
        c.gripper_offset = 20;
        c.frame_transaction = 1;
        assert(done.sendMotion(c) && lastId() == 25);
        report(1, 24);
        assert(done.gripperAck().result == R::NotDone);                       // old id
        assert(done.confirmedFrameOpen(done.latestActuatorFeedback()) == -1); // in motion: unknown
        report(1, 25);
        assert(done.gripperAck().result == R::NotDone); // id echoed before settle time
        std::this_thread::sleep_for(Ms(320));
        report(1, 25);
        assert(done.gripperAck().result == R::Done && done.gripperAck().target == 0);
        assert(done.confirmedFrameOpen(done.latestActuatorFeedback()) == 0);
        report(0, 25);
        assert(done.gripperAck().result == R::NotDone); // flag 0 is never done
        c.gripper_offset = 0;
        c.frame_transaction = 2;
        assert(done.sendMotion(c) && lastId() == 26);
        std::this_thread::sleep_for(Ms(320));
        report(1, 26);
        assert(done.gripperAck().result == R::Done && done.gripperAck().target == 1);
        assert(done.confirmedFrameOpen(done.latestActuatorFeedback()) == 1);
        done.closePort();
        close(mcu);
    }
    {
        // No A6 feedback: sync fails and leaves the id untouched.
        int mcu = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        assert(mcu >= 0 && grantpt(mcu) == 0 && unlockpt(mcu) == 0);
        UARTController silent;
        silent.configureFrameFeedback(1, 0);
        silent.initUART(ptsname(mcu), 115200, false, false);
        const auto t0 = Clock::now();
        assert(!silent.syncGripperActionId(Ms(100)));
        assert(Clock::now() - t0 >= Ms(100) && silent.gripperActionId() == 0);
        silent.closePort();
        close(mcu);
    }
}
