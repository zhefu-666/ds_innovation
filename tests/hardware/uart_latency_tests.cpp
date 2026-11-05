#include "rescue/control/match_control.hpp"
#include "rescue/core/utils.hpp"
#include "rescue/hardware/uart_controller.hpp"
#include "rescue/hardware/motion_link.hpp"
#include <cstring>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>

namespace {
std::atomic<bool> motion_write_entered{false};
std::atomic<int> write_error{0};

uint64_t nowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               rescue::Clock::now().time_since_epoch())
        .count();
}
} // namespace

extern "C" ssize_t __real_write(int fd, const void* data, size_t size);
extern "C" ssize_t __wrap_write(int fd, const void* data, size_t size) {
    if (size == rescue::MotionPacket::kSize &&
        static_cast<const unsigned char*>(data)[0] == rescue::MotionPacket::kHeader) {
        motion_write_entered = true;
        if (const int error = write_error.load()) {
            errno = error;
            return -1;
        }
    }
    return __real_write(fd, data, size);
}

// Model the historical 462 ms control gap at the real driver-drain boundary.
// No physical device is opened: the UART uses a new pseudo-terminal in every case.
extern "C" int __wrap_tcdrain(int) {
    std::this_thread::sleep_for(std::chrono::milliseconds(462));
    return 0;
}

void heartbeatRemainsResponsive(int error) {
    using namespace rescue;
    const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    UARTController uart;
    uart.initUART(ptsname(master), 115200, false, false);
    ActuatorFeedback feedback;
    feedback.valid = true;
    feedback.timestamp_us = nowUs();
    feedback.camera_pitch_cdeg = -3000;
    uart.publishActuatorFeedback(feedback);
    MatchConfig config;
    config.time_limit_enabled = config.require_measured_progress = false;
    MatchControl match(config);
    match.health(nowUs(), true, "");
    assert(match.command(MatchCommand::START, nowUs()));
    motion_write_entered = false;
    write_error = error;
    MotionCommand command;
    command.gripper_offset = 20;
    bool sent = false;
    uint64_t send_us = 0;
    std::thread sender([&] {
        const auto start = nowUs();
        sent = uart.sendMotion(command);
        send_us = nowUs() - start;
    });
    const auto until = Clock::now() + Ms(1000);
    while (!motion_write_entered && Clock::now() < until)
        std::this_thread::yield();
    assert(motion_write_entered);
    const auto start = nowUs();
    const auto observed = uart.latestActuatorFeedback();
    uart.gripperAck();
    uart.cameraPitchResult(50);
    const auto blocked_us = nowUs() - start;
    match.health(nowUs(), observed.valid, "mcu_feedback_stale");
    const auto status = match.status(nowUs());
    sender.join();
    std::cout << "write_error=" << error << " feedback_read_ms=" << blocked_us / 1000.0
              << " send_ms=" << send_us / 1000.0 << " match=" << status.reason << "\n";
    assert(blocked_us < 100000 && send_us < 100000);
    assert(status.permit && status.reason == "started");
    assert(sent == (error == 0));
    write_error = 0;
    // Timeout retries retain the action ID, and the next healthy write still works.
    const auto id = uart.gripperActionId();
    assert(uart.sendMotion(command) && uart.gripperActionId() == id);
    unsigned char packet[rescue::MotionPacket::kSize];
    assert(read(master, packet, sizeof(packet)) == sizeof(packet));
    assert(packet[0] == rescue::MotionPacket::kHeader && packet[10] == id);
    uart.closePort();
    close(master);
}

void preparedButtonStartsWithoutRelaunch() {
    using namespace rescue;
    const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    UARTController uart;
    uart.configureFrameFeedback(1, 0);
    uart.initUART(ptsname(master), 115200, false, false);
    MatchConfig config;
    config.duration_us = 180000000;
    config.require_measured_progress = false;
    MatchControl match(config);
    MotionLink link(uart, MotionLinkConfig{}, [&] { return match.status(nowUs()).permit; });
    MotionCommand command;
    command.vx_mps = .1f;
    command.gripper_offset = 20;
    command.camera_pitch_cdeg = -3000;
    link.submit(command);
    std::array<uint8_t, 5> start{0xA7, 1, 0, 0, 10};
    auto crc = UARTController::calculateCRC16(start.data(), 0, 1);
    start[2] = crc & 255; start[3] = crc >> 8;
    const auto sendButton = [&] {
        const auto previous = uart.latestStartUs();
        assert(write(master, start.data(), 2) == 2);
        assert(write(master, start.data() + 2, 3) == 3);
        const auto until = Clock::now() + Ms(500);
        while (uart.latestStartUs() == previous && Clock::now() < until)
            std::this_thread::sleep_for(Ms(1));
        assert(uart.latestStartUs() > previous);
        return uart.latestStartUs();
    };
    const auto early = sendButton();
    assert(!match.startFromButton(early, nowUs()));
    match.health(nowUs(), true, "");
    assert(!match.startFromButton(early, nowUs()));
    std::this_thread::sleep_for(Ms(80));
    uint8_t bytes[256];
    assert(read(master, bytes, sizeof(bytes)) < 0 && errno == EAGAIN);
    assert(match.status(nowUs()).ready && !match.status(nowUs()).permit);
    assert(match.status(nowUs()).elapsed_us == 0);
    auto bad = start; bad[2] ^= 1;
    assert(write(master, bad.data(), bad.size()) == ssize_t(bad.size()));
    std::this_thread::sleep_for(Ms(20));
    assert(uart.latestStartUs() == early);
    const auto pressed = sendButton();
    match.health(nowUs(), true, "");
    assert(match.startFromButton(pressed, nowUs()));
    const auto started = match.status(nowUs()).started_us;
    link.submit(command);
    bool moving = false;
    const auto until = Clock::now() + Ms(200);
    std::vector<uint8_t> wire;
    while (!moving && Clock::now() < until) {
        const auto count = read(master, bytes, sizeof(bytes));
        if (count > 0) wire.insert(wire.end(), bytes, bytes + count);
        while (wire.size() >= MotionPacket::kSize) {
            assert(wire[0] == MotionPacket::kHeader);
            float vx = 0;
            std::memcpy(&vx, wire.data() + 1, sizeof(vx));
            moving = moving || vx > 0;
            wire.erase(wire.begin(), wire.begin() + MotionPacket::kSize);
        }
        if (!moving) std::this_thread::sleep_for(Ms(1));
    }
    assert(moving);
    std::cout << "prepared_start pseudo_terminal_rx_to_nonzero_tx_ms="
              << (nowUs() - pressed) / 1000.0 << " no_tx_before_start=1\n";
    const auto duplicate = sendButton();
    assert(!match.startFromButton(duplicate, nowUs()));
    assert(match.status(nowUs()).started_us == started);
    assert(match.command(MatchCommand::STOP, nowUs()));
    const auto repeat = sendButton();
    assert(!match.startFromButton(repeat, nowUs()) && !match.status(nowUs()).permit);
    link.stop();
    uart.closePort();
    close(master);
}

void repeatedButtonRestartsKeepWireStopped() {
    using namespace rescue;
    const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    UARTController uart;
    uart.initUART(ptsname(master), 115200, false, false);
    MatchConfig config;
    config.duration_us = 180000000;
    config.require_measured_progress = false;
    MatchControl match(config);
    match.health(nowUs(), true, "");
    std::unique_ptr<MotionLink> link;
    std::array<uint8_t, 5> start{0xA7, 1, 0, 0, 10};
    const auto crc = UARTController::calculateCRC16(start.data(), 0, 1);
    start[2] = crc & 255; start[3] = crc >> 8;
    MotionCommand command;
    command.vx_mps = .4f;
    command.wz_rps = .3f;
    command.gripper_offset = 20;
    command.camera_pitch_cdeg = -3000;
    const auto receive = [&](bool require_zero) {
        std::vector<uint8_t> bytes;
        uint8_t chunk[4096];
        for (;;) {
            const auto n = read(master, chunk, sizeof(chunk));
            if (n <= 0) break;
            bytes.insert(bytes.end(), chunk, chunk + n);
        }
        assert(!bytes.empty() && bytes.size() % MotionPacket::kSize == 0);
        bool moving = false;
        for (size_t i = 0; i < bytes.size(); i += MotionPacket::kSize) {
            assert(bytes[i] == MotionPacket::kHeader);
            float vx = 0, wz = 0;
            std::memcpy(&vx, bytes.data() + i + 1, sizeof(vx));
            std::memcpy(&wz, bytes.data() + i + 5, sizeof(wz));
            if (require_zero) assert(vx == 0 && wz == 0);
            moving = moving || vx != 0 || wz != 0;
        }
        return moving;
    };
    uint8_t action_id = 0;
    for (int round = 0; round < 3; ++round) {
        match.health(nowUs(), true, "");
        const auto previous = uart.latestStartUs();
        assert(write(master, start.data(), start.size()) == ssize_t(start.size()));
        const auto until = Clock::now() + Ms(300);
        while (uart.latestStartUs() == previous && Clock::now() < until)
            std::this_thread::sleep_for(Ms(1));
        assert(uart.latestStartUs() > previous);
        assert(match.restartFromButton(uart.latestStartUs(), nowUs(), true));
        if (link) {
            link->stop();
            assert(link->stats().last_ok && link->stats().sent_vx_mps == 0 &&
                   link->stats().sent_wz_rps == 0);
            receive(false); // discard packets preceding the completed stop barrier
        }
        link = std::make_unique<MotionLink>(uart, MotionLinkConfig{},
            [&] { return match.status(nowUs()).permit; },
            [&] { return match.status(nowUs()).state == MatchState::RESTARTING; });
        // A nonzero submission still cannot drive during preparation.
        link->submit(command);
        std::this_thread::sleep_for(Ms(60));
        receive(true);
        if (round == 0) action_id = uart.gripperActionId();
        assert(uart.gripperActionId() == action_id);
        match.health(nowUs(), true, "");
        assert(match.completeButtonRestart(nowUs()));
        assert(match.status(nowUs()).run_id == uint64_t(round + 1));
        link->submit(command);
        std::this_thread::sleep_for(Ms(60));
        assert(receive(false));
    }
    match.health(nowUs(), true, "");
    const auto event = nowUs();
    assert(match.restartFromButton(event, nowUs(), true));
    assert(match.command(MatchCommand::STOP, nowUs()));
    match.health(nowUs(), true, "");
    assert(!match.completeButtonRestart(nowUs()));
    link->stop();
    receive(false);
    uart.closePort();
    close(master);
    std::cout << "PASS repeated A7: preparation zero on wire, old sender joined, action ID retained\n";
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--a7-restart-only") {
        repeatedButtonRestartsKeepWireStopped();
        return 0;
    }
    repeatedButtonRestartsKeepWireStopped();
    preparedButtonStartsWithoutRelaunch();
    heartbeatRemainsResponsive(0);
    heartbeatRemainsResponsive(EAGAIN);
    heartbeatRemainsResponsive(EINTR);
    std::cout << "PASS UART driver delay/backpressure cannot starve the health heartbeat\n";
}
