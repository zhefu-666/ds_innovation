#include "rescue/motion_link.hpp"
#include "rescue/utils.hpp"
#include <iostream>

namespace rescue {
namespace {
uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count());
}
} // namespace

MotionLink::MotionLink(UARTController &uart, MotionLinkConfig config) : uart_(uart), config_(config) {
    if (config_.period_ms == 0 || config_.command_timeout_ms == 0 || config_.command_timeout_ms >= 200)
        throw std::invalid_argument("MotionLink needs period > 0 and command timeout in 1..199 ms");
    worker_ = std::thread(&MotionLink::run, this);
}
MotionLink::~MotionLink() { stop(); }

void MotionLink::submit(const MotionCommand &command) {
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (stopping_) return;
        latest_ = command; submitted_us_ = nowUs(); has_command_ = true; pending_ = true;
    }
    wake_.notify_one();
}
void MotionLink::write(const MotionCommand &command, bool stale) {
    bool ok = false;
    try { ok = uart_.sendMotion(command); } catch (const std::exception &e) {
        std::cerr << "[MCU] send rejected: " << e.what() << "\n";
    }
    std::lock_guard<std::mutex> guard(mutex_);
    stats_.last_ok = ok;
    if (ok) { ++stats_.sent; stats_.last_ok_us = nowUs(); } else ++stats_.failed;
    if (stale) ++stats_.stale;
}
void MotionLink::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stopping_) {
        wake_.wait_for(lock, Ms(config_.period_ms), [&] { return stopping_ || pending_; });
        if (stopping_) break;
        pending_ = false;
        if (!has_command_) continue;
        MotionCommand command = latest_;
        const bool stale = nowUs() - submitted_us_ > uint64_t(config_.command_timeout_ms) * 1000;
        if (stale) command = stopped(command);
        lock.unlock();
        write(command, stale);
        lock.lock();
    }
}
void MotionLink::stop() {
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (stopped_) return;
        stopping_ = true; stopped_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
    MotionCommand last; bool sent;
    { std::lock_guard<std::mutex> guard(mutex_); last = latest_; sent = has_command_; }
    if (sent) write(stopped(last), false);
}
bool MotionLink::healthy() const {
    uint64_t submitted; MotionLinkStats s;
    { std::lock_guard<std::mutex> guard(mutex_); submitted = has_command_ ? submitted_us_ : 0; s = stats_; }
    const uint64_t now = nowUs(), limit = uint64_t(config_.command_timeout_ms) * 1000;
    const auto recent = [&](uint64_t t) { return t && now >= t && now - t <= limit; };
    // Zero-velocity substitutes are successful writes but not a live task loop.
    return !stopping_ && recent(submitted) && s.last_ok && recent(s.last_ok_us) && uart_.latestActuatorFeedback().valid;
}
MotionLinkStats MotionLink::stats() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return stats_;
}
} // namespace rescue
