#pragma once
#include <atomic>
#include <cerrno>
#include <chrono>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <thread>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

namespace rescue {
// Control thread never waits for a sink or a queue lock. Overload loses logs,
// not control updates. Own the output fd exclusively while this object lives.
class AsyncLog {
public:
    explicit AsyncLog(int fd = STDOUT_FILENO) : fd_(fd) {
        flags_ = fcntl(fd_, F_GETFL);
        if (flags_ < 0 || fcntl(fd_, F_SETFL, flags_ | O_NONBLOCK) < 0)
            throw std::runtime_error("Cannot make log sink nonblocking");
        try { worker_ = std::thread([this] { run(); }); }
        catch (...) { fcntl(fd_, F_SETFL, flags_); throw; }
    }
    ~AsyncLog() {
        stop_ = true;
        worker_.join();
        fcntl(fd_, F_SETFL, flags_);
    }
    void submit(std::string text) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || text.size() > limit_ || bytes_ + text.size() > limit_) {
            ++dropped_; return;
        }
        bytes_ += text.size(); queue_.push_back(std::move(text));
    }
    uint64_t dropped() const { return dropped_; }
private:
    void run() {
        sigset_t signals; sigemptyset(&signals); sigaddset(&signals, SIGPIPE);
        pthread_sigmask(SIG_BLOCK, &signals, nullptr);
        while (!stop_) {
            std::string text;
            { std::lock_guard<std::mutex> lock(mutex_);
              if (!queue_.empty()) { text = std::move(queue_.front()); bytes_ -= text.size(); queue_.pop_front(); } }
            if (text.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
            size_t offset = 0;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
            while (offset < text.size() && !stop_) {
                const auto n = ::write(fd_, text.data() + offset, text.size() - offset);
                if (n > 0) { offset += size_t(n); continue; }
                if (n < 0 && errno == EINTR) continue;
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && std::chrono::steady_clock::now() < deadline) {
                    pollfd p{fd_, POLLOUT, 0}; ::poll(&p, 1, 10); continue;
                }
                break;
            }
            if (offset < text.size()) ++dropped_;
        }
    }
    int fd_, flags_;
    static constexpr size_t limit_ = 256 * 1024;
    size_t bytes_ = 0;
    std::mutex mutex_;
    std::deque<std::string> queue_;
    std::atomic<bool> stop_{false};
    std::atomic<uint64_t> dropped_{0};
    std::thread worker_;
};
}
