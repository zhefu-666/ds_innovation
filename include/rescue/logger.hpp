#pragma once

#include "rescue/types.hpp"

#include <fstream>
#include <mutex>
#include <string>

namespace rescue {

class EventLogger {
public:
    EventLogger() = default;
    explicit EventLogger(const std::string &path) { open(path); }
    ~EventLogger();
    bool open(const std::string &path);
    void close();
    void event(uint64_t timestamp_us, const std::string &state, const std::string &message);
    void sensor(uint64_t timestamp_us, const SensorState &state);
    void command(uint64_t timestamp_us, const MotionCommand &command);

private:
    std::mutex mutex_;
    std::ofstream stream_;
};

} // namespace rescue
