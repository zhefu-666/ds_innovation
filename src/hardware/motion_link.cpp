#include "rescue/hardware/motion_link.hpp"
#include "rescue/core/utils.hpp"
#include <iostream>

namespace rescue {
namespace {
uint64_t nowUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch())
            .count());
}
} // namespace

MotionLink::MotionLink(UARTController& uart, MotionLinkConfig config, std::function<bool()> permit)
    : permit_(std::move(permit)), uart_(uart), config_(config), ramp_(config) {
    if (config_.period_ms == 0 || config_.command_timeout_ms == 0 ||
        config_.command_timeout_ms >= 200)
        throw std::invalid_argument("MotionLink needs period > 0 and command timeout in 1..199 ms");
    for (float rate : {config_.linear_accel_mps2,
                       config_.linear_decel_mps2,
                       config_.angular_accel_rps2,
                       config_.angular_decel_rps2})
        if (!std::isfinite(rate) || rate <= 0)
            throw std::invalid_argument(
                "MotionLink acceleration/deceleration must be finite and positive");
    worker_ = std::thread(&MotionLink::run, this);
}
MotionLink::~MotionLink() {
    stop();
}

void MotionLink::submit(const MotionCommand& command, bool immediate_stop) {
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (stopping_)
            return;
        latest_ = command;
        immediate_stop_ = immediate_stop;
        stop_pending_ = stop_pending_ || immediate_stop;
        stats_.requested_vx_mps = command.vx_mps;
        stats_.requested_wz_rps = command.wz_rps;
        submitted_us_ = nowUs();
        has_command_ = true;
        pending_ = true;
    }
    wake_.notify_one();
}
void MotionLink::write(MotionCommand command, bool stale, bool immediate_stop) {
    MotionCommand guarded = command;
    if (permit_ && !permit_()) {
        if (!ever_authorized_)
            return; // no implicit gripper close or pitch move before START
        guarded = stopped(authorized_);
        immediate_stop = true;
    } else {
        authorized_ = guarded;
        ever_authorized_ = true;
    }
    const auto write_at = nowUs();
    guarded = ramp_.next(guarded, write_at, stale || immediate_stop);
    bool ok = false;
    try {
        ok = uart_.sendMotion(guarded);
    } catch (const std::exception& e) {
        std::cerr << "[MCU] send rejected: " << e.what() << "\n";
    }
    if (ok)
        ramp_.accept(guarded, write_at);
    else
        ramp_.accept(stopped(guarded), write_at);
    std::lock_guard<std::mutex> guard(mutex_);
    stats_.last_ok = ok;
    if (ok) {
        ++stats_.sent;
        stats_.last_ok_us = nowUs();
        stats_.sent_vx_mps = guarded.vx_mps;
        stats_.sent_wz_rps = guarded.wz_rps;
    } else
        ++stats_.failed;
    if (stale)
        ++stats_.stale;
}
void MotionLink::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stopping_) {
        wake_.wait_for(lock, Ms(config_.period_ms), [&] {
            return stopping_ || pending_;
        });
        if (stopping_)
            break;
        pending_ = false;
        if (!has_command_)
            continue;
        MotionCommand command = latest_;
        const bool immediate_stop = immediate_stop_ || stop_pending_;
        stop_pending_ = false;
        const bool stale = nowUs() - submitted_us_ > uint64_t(config_.command_timeout_ms) * 1000;
        if (stale)
            command = stopped(command);
        lock.unlock();
        write(command, stale, immediate_stop);
        lock.lock();
    }
}
void MotionLink::stop() {
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (stopped_)
            return;
        stopping_ = true;
        stopped_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable())
        worker_.join();
    MotionCommand last;
    bool sent;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        last = latest_;
        sent = has_command_;
    }
    if (sent)
        write(stopped(last), false, true);
}
bool MotionLink::healthy() const {
    uint64_t submitted;
    MotionLinkStats s;
    bool stopping;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        submitted = has_command_ ? submitted_us_ : 0;
        s = stats_;
        stopping = stopping_;
    }
    const uint64_t now = nowUs(), limit = uint64_t(config_.command_timeout_ms) * 1000;
    const auto recent = [&](uint64_t t) {
        return t && now >= t && now - t <= limit;
    };
    // Zero-velocity substitutes are successful writes but not a live task loop.
    return !stopping && recent(submitted) && s.last_ok && recent(s.last_ok_us) &&
           uart_.latestActuatorFeedback().valid;
}
MotionLinkStats MotionLink::stats() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return stats_;
}
} // namespace rescue
