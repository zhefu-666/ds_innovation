#include "rescue/logger.hpp"

namespace rescue {

EventLogger::~EventLogger() { close(); }

bool EventLogger::open(const std::string &path) {
    std::lock_guard<std::mutex> lock(mutex_);
    stream_.open(path, std::ios::out | std::ios::app);
    if (stream_.tellp() == std::streampos(0)) stream_ << "timestamp_us,type,state_or_unused,payload\n";
    return stream_.good();
}

void EventLogger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) stream_.close();
}

void EventLogger::event(uint64_t timestamp_us, const std::string &state, const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_) stream_ << timestamp_us << ",event," << state << "," << message << '\n';
}

void EventLogger::sensor(uint64_t timestamp_us, const SensorState &state) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_) stream_ << timestamp_us << ",sensor_rad," << state.yaw_rad << ';' << state.pitch_rad
                         << ';' << state.roll_rad << ';' << state.tof_fl_m << ';' << state.tof_fr_m
                         << ';' << state.tof_rl_m << ';' << state.tof_rr_m << '\n';
}

void EventLogger::command(uint64_t timestamp_us, const MotionCommand &command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_) stream_ << timestamp_us << ",command,,"
                         << command.vx_mps << ';' << command.wz_rps << '\n';
}

} // namespace rescue
