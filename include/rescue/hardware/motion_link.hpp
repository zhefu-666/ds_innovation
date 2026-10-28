#pragma once
#include "rescue/core/types.hpp"
#include "rescue/hardware/uart_controller.hpp"
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <functional>

namespace rescue {
struct MotionLinkConfig {
    uint32_t period_ms = 40;           // resend rate 25 Hz, well inside the MCU's 200 ms timeout
    uint32_t command_timeout_ms = 150; // older commands are replaced by zero velocity
    // Initial chassis tuning, not measured mechanical limits. Fault stops bypass these ramps.
    float linear_accel_mps2 = .30f, linear_decel_mps2 = .40f;
    float angular_accel_rps2 = .80f, angular_decel_rps2 = 1.0f;
};

// Rate-limit normal command changes in the sender's time domain, not the camera frame rate.
// Reversals brake to zero before accelerating in the opposite direction.
class MotionVelocityRamp {
  public:
    explicit MotionVelocityRamp(MotionLinkConfig config) : config_(config) {
    }
    MotionCommand next(MotionCommand target, uint64_t now_us, bool immediate_stop = false) const {
        if (immediate_stop) {
            target.vx_mps = target.wz_rps = 0;
            return target;
        }
        const float dt =
            last_us_ && now_us >= last_us_
                ? std::min(now_us - last_us_, uint64_t(config_.period_ms) * 1000) * 1e-6f
                : 0.f;
        target.vx_mps =
            axis(vx_, target.vx_mps, config_.linear_accel_mps2, config_.linear_decel_mps2, dt);
        target.wz_rps =
            axis(wz_, target.wz_rps, config_.angular_accel_rps2, config_.angular_decel_rps2, dt);
        return target;
    }
    void accept(const MotionCommand& sent, uint64_t at_us) {
        vx_ = sent.vx_mps;
        wz_ = sent.wz_rps;
        last_us_ = at_us;
    }

  private:
    static float axis(float current, float target, float accel, float decel, float dt) {
        if (!std::isfinite(target))
            return target; // UART validation must still reject nonfinite requests.
        const bool reversing = (current > 0 && target < 0) || (current < 0 && target > 0);
        const float goal = reversing ? 0.f : target;
        const float limit = std::abs(goal) > std::abs(current) ? accel : decel;
        return current + std::clamp(goal - current, -limit * dt, limit * dt);
    }
    MotionLinkConfig config_;
    float vx_ = 0, wz_ = 0;
    uint64_t last_us_ = 0;
};
struct MotionLinkStats {
    uint64_t sent = 0, failed = 0,
             stale = 0;      // stale: packets sent as zero velocity for an old command
    uint64_t last_ok_us = 0; // host steady clock of the last successful write
    bool last_ok = false;
    float requested_vx_mps = 0, requested_wz_rps = 0;
    float sent_vx_mps = 0, sent_wz_rps = 0;
};
// Periodic MCU command sender. The task loop submits one command per frame; this thread
// ramps velocity on immediate writes and periodic resends, so a healthy loop never trips the MCU
// 200 ms timeout. A command older than command_timeout (stalled loop) is replaced by zero
// velocity. Zero-velocity substitutes and stop() keep the last gripper and pitch targets:
// they stop the wheels and never toggle the gripper. Nothing is sent before the first submit.
class MotionLink {
  public:
    explicit MotionLink(UARTController& uart,
                        MotionLinkConfig config = {},
                        std::function<bool()> permit = {});
    ~MotionLink();
    MotionLink(const MotionLink&) = delete;
    MotionLink& operator=(const MotionLink&) = delete;
    void submit(const MotionCommand& command, bool immediate_stop = false);
    // Sends a final zero-velocity packet (if anything was sent) and joins; idempotent.
    void stop();
    // A command was submitted and a packet written within command_timeout, and A6 feedback is
    // fresh.
    bool healthy() const;
    MotionLinkStats stats() const;
    static MotionCommand stopped(MotionCommand command) {
        command.vx_mps = 0.0f;
        command.wz_rps = 0.0f;
        return command;
    }

  private:
    void run();
    void write(MotionCommand command, bool stale, bool immediate_stop = false);
    std::function<bool()> permit_;
    MotionCommand authorized_;
    bool ever_authorized_ = false;
    UARTController& uart_;
    MotionLinkConfig config_;
    MotionVelocityRamp ramp_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    MotionCommand latest_;
    uint64_t submitted_us_ = 0;
    bool has_command_ = false, pending_ = false, stopping_ = false, stopped_ = false;
    bool immediate_stop_ = false, stop_pending_ = false;
    MotionLinkStats stats_;
    std::thread worker_;
};
} // namespace rescue
