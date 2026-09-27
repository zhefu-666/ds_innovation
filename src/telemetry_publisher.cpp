#include "rescue/telemetry_publisher.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <sys/file.h>
#include <unistd.h>

namespace rescue {
namespace {
double finite(double value) { return std::isfinite(value) ? value : 0.0; }
void writeAll(int fd, const void* data, size_t size) {
    auto bytes = static_cast<const unsigned char*>(data);
    while (size) {
        const auto n = ::write(fd, bytes, size);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) throw std::runtime_error(std::strerror(errno));
        bytes += n; size -= static_cast<size_t>(n);
    }
}
void writeU32(int fd, uint32_t value) {
    const unsigned char bytes[] = {static_cast<unsigned char>(value),
        static_cast<unsigned char>(value >> 8), static_cast<unsigned char>(value >> 16),
        static_cast<unsigned char>(value >> 24)};
    writeAll(fd, bytes, 4);
}
}
TelemetryPublisher::TelemetryPublisher(const Config& config) : config_(config) {
    const auto lock_path = config_.telemetry_file + ".lock";
    lock_fd_ = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock_fd_ < 0) throw std::runtime_error("Cannot open telemetry lock: " + lock_path);
    if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
        ::close(lock_fd_); lock_fd_ = -1;
        throw std::runtime_error("Another telemetry publisher owns " + config_.telemetry_file);
    }
    try { worker_ = std::thread(&TelemetryPublisher::run, this); }
    catch (...) { ::close(lock_fd_); lock_fd_ = -1; throw; }
}
TelemetryPublisher::~TelemetryPublisher() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
    }
    ready_.notify_one();
    if (worker_.joinable()) worker_.join();
    // Removing the last snapshot makes loss of the data source immediately visible.
    ::unlink(config_.telemetry_file.c_str());
    if (lock_fd_ >= 0) ::close(lock_fd_);
}
void TelemetryPublisher::submit(const cv::Mat& frame,
    const std::vector<SegDetection>& detections, const PushObservation& observation,
    const PushOutput& output, uint64_t epoch_ns, uint64_t sequence, double loop_fps,
    double inference_ms, double capture_ms, const ImuSnapshot& imu) noexcept {
    try {
        const auto now = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) { ++dropped_; return; }
        if (now < next_submit_ || frame.empty()) return;
        next_submit_ = now + std::chrono::microseconds(1000000 / config_.telemetry_fps);
        Snapshot snapshot;
        // Own the pixels: main continues drawing/reusing its buffer independently.
        snapshot.frame = frame.clone();
        snapshot.detections = detections;
        for (auto& d : snapshot.detections) d.mask.release();
        snapshot.imu = imu;
        snapshot.observation = observation; snapshot.output = output;
        snapshot.epoch_ns = epoch_ns; snapshot.sequence = sequence;
        snapshot.loop_fps = loop_fps; snapshot.inference_ms = inference_ms;
        snapshot.capture_ms = capture_ms;
        if (pending_valid_) ++dropped_;
        pending_ = std::move(snapshot); pending_valid_ = true;
        lock.unlock(); ready_.notify_one();
    } catch (...) { ++errors_; }
}
void TelemetryPublisher::run() noexcept {
    for (;;) {
        Snapshot snapshot;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [&] { return stopped_ || pending_valid_; });
            if (stopped_) return;
            snapshot = std::move(pending_); pending_valid_ = false;
        }
        try { writeSnapshot(snapshot); }
        catch (const std::exception& e) {
            if (++errors_ == 1) std::cerr << "[TELEMETRY] " << e.what() << '\n';
        } catch (...) { ++errors_; }
    }
}
void TelemetryPublisher::writeSnapshot(const Snapshot& s) {
    const double scale = std::min(1.0, 640.0 / s.frame.cols);
    cv::Mat preview;
    cv::resize(s.frame, preview,
        cv::Size(std::max(1, static_cast<int>(std::lround(s.frame.cols * scale))),
                 std::max(1, static_cast<int>(std::lround(s.frame.rows * scale)))),
        0, 0, cv::INTER_AREA);
    std::vector<unsigned char> jpeg;
    if (!cv::imencode(".jpg", preview, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70}))
        throw std::runtime_error("JPEG encoding failed");
    cv::FileStorage f(".json", cv::FileStorage::WRITE | cv::FileStorage::MEMORY |
                                   cv::FileStorage::FORMAT_JSON);
    f << "version" << 1 << "sequence" << static_cast<double>(s.sequence)
      << "timestamp" << "{" << "sec" << static_cast<int>(s.epoch_ns / 1000000000)
      << "nsec" << static_cast<int>(s.epoch_ns % 1000000000) << "}"
      << "frame_id" << "camera_optical" << "source_width" << s.frame.cols
      << "source_height" << s.frame.rows << "image_width" << preview.cols
      << "image_height" << preview.rows << "detections" << "[";
    for (const auto& d : s.detections) {
        f << "{" << "track_id" << d.track_id << "class_id" << d.class_id
          << "label" << d.label << "model_label" << d.model_label
          << "confidence" << finite(d.confidence) << "box" << "{"
          << "x" << d.box.x << "y" << d.box.y << "width" << d.box.width
          << "height" << d.box.height << "}" << "}";
    }
    const auto& in = s.observation;
    f << "]" << "state" << "{" << "name" << PushTask::name(s.output.state)
      << "batch_size" << s.output.batch_size << "delivered_total" << s.output.delivered_total
      << "first_ordinary_delivered" << static_cast<int>(s.output.first_ordinary_delivered)
      << "run_requested" << static_cast<int>(in.run)
      << "target_valid" << static_cast<int>(in.target_valid)
      << "target_id" << in.target_id << "target_label" << in.label
      << "geometry_valid" << static_cast<int>(in.geometry_valid)
      << "path_safe" << static_cast<int>(in.path_safe)
      << "safety_ok" << static_cast<int>(in.safety_ok)
      << "zone_valid" << static_cast<int>(in.zone_valid) << "}"
      << "motion" << "{" << "vx_mps" << finite(s.output.motion.vx_mps)
      << "wz_rps" << finite(s.output.motion.wz_rps) << "hardware_output_enabled" << 0 << "}"
      << "health" << "{" << "loop_fps" << finite(s.loop_fps)
      << "inference_ms" << finite(s.inference_ms) << "capture_ms" << finite(s.capture_ms)
      << "publish_fps_limit" << config_.telemetry_fps
      << "dropped_publish_frames" << static_cast<double>(dropped_.load())
      << "publish_errors" << static_cast<double>(errors_.load()) << "}"
      << "config" << "{" << "camera_index" << config_.camera_index
      << "requested_width" << config_.frame_width << "requested_height" << config_.frame_height
      << "requested_fps" << config_.fps << "confidence" << config_.confidence
      << "nms" << config_.nms << "input_size" << config_.input_size
      << "team" << config_.team << "model" << config_.model_path
      << "dry_run" << static_cast<int>(config_.dry_run)
      << "imu_enabled" << static_cast<int>(config_.imu) << "imu_port" << config_.imu_port
      << "imu_baud" << config_.imu_baud << "}";
    const auto& imu = s.imu;
    const auto& p = imu.sample;
    // Carry host monotonic receive time; the local bridge recomputes freshness on every read.
    f << "imu" << "{" << "enabled" << static_cast<int>(config_.imu)
      << "connected" << static_cast<int>(imu.connected) << "fresh" << static_cast<int>(imu.fresh)
      << "measurements_valid" << static_cast<int>(p.measurements_valid)
      << "attitude_valid_for_control" << 0 << "frame_id" << "imu_device"
      << "received_monotonic_us" << static_cast<double>(p.received_us)
      << "timeout_ms" << static_cast<int>(imu.timeout_ms) << "age_ms" << imu.age_ms
      << "sequence" << static_cast<double>(p.sequence) << "device_time_ms" << static_cast<double>(p.device_time_ms)
      << "status_raw" << static_cast<int>(p.status) << "temperature_c" << p.temperature_c
      << "pressure_pa" << finite(p.pressure_pa)
      << "bytes" << static_cast<double>(imu.bytes) << "valid_frames" << static_cast<double>(imu.valid_frames)
      << "crc_errors" << static_cast<double>(imu.crc_errors) << "invalid_frames" << static_cast<double>(imu.invalid_frames)
      << "duplicate_times" << static_cast<double>(imu.duplicate_times) << "backward_times" << static_cast<double>(imu.backward_times)
      << "io_errors" << static_cast<double>(imu.io_errors);
    const auto vector = [&](const char* name, const auto& values) {
        f << name << "[";
        for (float value : values) f << finite(value);
        f << "]";
    };
    vector("acceleration_mps2", p.acceleration_mps2);
    vector("angular_velocity_rps", p.angular_velocity_rps);
    vector("magnetic_ut", p.magnetic_ut);
    vector("rpy_rad", p.rpy_rad);
    vector("quaternion_wxyz", p.quaternion_wxyz);
    // Mounting-corrected copies; raw fields above stay in imu_device for diagnosis.
    f << "body_frame_id" << "base_link";
    vector("body_acceleration_mps2", p.body_acceleration_mps2);
    vector("body_angular_velocity_rps", p.body_angular_velocity_rps);
    vector("body_rpy_rad", p.body_rpy_rad);
    vector("body_quaternion_wxyz", p.body_quaternion_wxyz);
    f << "}";
    const auto metadata = f.releaseAndGetString();
    if (metadata.size() > 1024 * 1024 || jpeg.size() > 8 * 1024 * 1024)
        throw std::runtime_error("Telemetry snapshot exceeds size limit");
    std::string name = config_.telemetry_file + ".XXXXXX";
    std::vector<char> path(name.begin(), name.end()); path.push_back(0);
    int fd = ::mkstemp(path.data());
    if (fd < 0) throw std::runtime_error("Cannot create telemetry snapshot");
    try {
        writeAll(fd, "RSTEL001", 8);
        writeU32(fd, static_cast<uint32_t>(metadata.size()));
        writeU32(fd, static_cast<uint32_t>(jpeg.size()));
        writeAll(fd, metadata.data(), metadata.size()); writeAll(fd, jpeg.data(), jpeg.size());
        if (::close(fd) != 0) { fd = -1; throw std::runtime_error("Snapshot close failed"); }
        fd = -1;
        if (::rename(path.data(), config_.telemetry_file.c_str()) != 0)
            throw std::runtime_error("Snapshot rename failed");
    } catch (...) {
        if (fd >= 0) ::close(fd);
        ::unlink(path.data()); throw;
    }
}
}
