#pragma once
#include "rescue/types.hpp"
#include "rescue/uart_controller.hpp"
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <functional>

namespace rescue {
struct MotionLinkConfig {
    uint32_t period_ms = 40;          // resend rate 25 Hz, well inside the MCU's 200 ms timeout
    uint32_t command_timeout_ms = 150; // older commands are replaced by zero velocity
};
struct MotionLinkStats {
    uint64_t sent = 0, failed = 0, stale = 0; // stale: packets sent as zero velocity for an old command
    uint64_t last_ok_us = 0;                  // host steady clock of the last successful write
    bool last_ok = false;
};
// Periodic MCU command sender. The task loop submits one command per frame; this thread
// writes it immediately and resends it every period, so a healthy loop never trips the MCU
// 200 ms timeout. A command older than command_timeout (stalled loop) is replaced by zero
// velocity. Zero-velocity substitutes and stop() keep the last gripper and pitch targets:
// they stop the wheels and never toggle the gripper. Nothing is sent before the first submit.
class MotionLink {
public:
    explicit MotionLink(UARTController &uart, MotionLinkConfig config = {}, std::function<bool()> permit = {});
    ~MotionLink();
    MotionLink(const MotionLink &) = delete;
    MotionLink &operator=(const MotionLink &) = delete;
    void submit(const MotionCommand &command);
    // Sends a final zero-velocity packet (if anything was sent) and joins; idempotent.
    void stop();
    // A command was submitted and a packet written within command_timeout, and A6 feedback is fresh.
    bool healthy() const;
    MotionLinkStats stats() const;
    static MotionCommand stopped(MotionCommand command) {
        command.vx_mps = 0.0f; command.wz_rps = 0.0f; return command;
    }
private:
    void run();
    void write(const MotionCommand &command, bool stale);
    std::function<bool()> permit_;
    MotionCommand authorized_;
    bool ever_authorized_ = false;
    UARTController &uart_;
    MotionLinkConfig config_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    MotionCommand latest_;
    uint64_t submitted_us_ = 0;
    bool has_command_ = false, pending_ = false, stopping_ = false, stopped_ = false;
    MotionLinkStats stats_;
    std::thread worker_;
};
} // namespace rescue
