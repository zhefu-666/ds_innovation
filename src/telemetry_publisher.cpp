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
    double inference_ms, double capture_ms, const ImuSnapshot& imu,
    const DecisionTelemetry& decision) noexcept {
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
        snapshot.decision = decision;
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
    // Highlight the state machine's lock, never an unrelated current-frame candidate.
    const bool active=s.output.state!=PushState::WAIT_START && s.output.state!=PushState::SAFE_STOP;
    const int locked=active?s.output.target_id:-1;
    const bool cue=s.output.target_is_search_cue;
    const cv::Scalar color=cue?cv::Scalar(255,220,0):cv::Scalar(0,220,255);
    std::string tracking=active?"TRACK: SELECTING":"TRACK: INACTIVE";
    bool seen=false;
    for(const auto& d:s.detections) if(locked>=0 && d.track_id==locked) {
        cv::Rect box(cvRound(d.box.x*scale),cvRound(d.box.y*scale),
                     cvRound(d.box.width*scale),cvRound(d.box.height*scale));
        box &= cv::Rect(0,0,preview.cols,preview.rows);
        if(box.empty())continue;
        cv::rectangle(preview,box,cv::Scalar(0,0,0),5);
        cv::rectangle(preview,box,color,3);
        const cv::Point centre(box.x+box.width/2,box.y+box.height/2);
        cv::drawMarker(preview,centre,color,cv::MARKER_CROSS,18,2);
        tracking=std::string(cue?"TRACK CUE #":"TRACK CARGO #")+std::to_string(locked)+" "+d.label;
        if(s.output.state==PushState::SCAN)tracking="SELECTING CUE #"+std::to_string(locked)+" "+d.label;
        seen=true;break;
    }
    if(locked>=0&&!seen)tracking="TRACK LOST #"+std::to_string(locked);
    if(active && (s.output.state==PushState::MID_REACQUIRE || s.output.state==PushState::NEAR_REACQUIRE ||
                  s.output.state==PushState::SELECT_CARGO))tracking="TRACK: RESELECTING CARGO";
    const int top=std::min(26,std::max(0,preview.rows-25));
    cv::rectangle(preview,{0,top,preview.cols,std::min(25,preview.rows-top)},cv::Scalar(15,15,15),cv::FILLED);
    cv::putText(preview,tracking,{8,top+17},cv::FONT_HERSHEY_SIMPLEX,.48,color,1,cv::LINE_AA);
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
          << "confidence" << finite(d.confidence)
          << "ground_position_valid" << static_cast<int>(d.ground_position_valid)
          << "ground_contact_valid" << static_cast<int>(d.ground_contact_valid)
          << "ground_contact_reason" << d.ground_contact_reason
          << "body_x_m" << finite(d.body_xy_m.x) << "body_y_m" << finite(d.body_xy_m.y)
          << "image_frame_id" << static_cast<double>(d.frame_id) << "box" << "{"
          << "x" << d.box.x << "y" << d.box.y << "width" << d.box.width
          << "height" << d.box.height << "}" << "}";
    }
    const auto& in = s.observation;
    f << "]" << "state" << "{" << "name" << PushTask::name(s.output.state)
      << "batch_size" << s.output.batch_size << "delivered_total" << s.output.delivered_total
      << "first_ordinary_delivered" << static_cast<int>(s.output.first_ordinary_delivered)
      << "match_state" << s.output.match_state << "match_reason" << s.output.match_reason
      << "match_remaining_ms" << double(s.output.match_remaining_us/1000)
      << "target_region_valid" << int(in.target_region_valid) << "target_in_zone" << int(in.target_in_zone)
      << "zone_identity_verified" << int(in.zone_identity_verified)
      << "retreat_safe" << int(in.retreat_safe) << "opponent_zone_clear" << int(in.opponent_zone_clear)
      << "zone_counts_valid" << int(in.zone_counts_valid)
      << "reason" << s.output.reason << "rule_verdict" << verdictName(s.output.verdict)
      << "frame_offset_cmd_deg" << static_cast<int>(s.output.motion.gripper_offset)
      << "camera_pitch_cmd_cdeg" << static_cast<int>(s.output.motion.camera_pitch_cdeg)
      << "camera_pitch_cdeg" << static_cast<int>(in.camera_pitch_cdeg)
      << "camera_protocol_readback_deg" << (in.camera_pitch_cdeg == kCameraPitchInvalid ? kCameraPitchInvalid : cameraPitchToFeedbackDeg(in.camera_pitch_cdeg))
      << "camera_protocol_cmd_deg" << cameraPitchToWire(s.output.motion.camera_pitch_cdeg)
      << "camera_pitch_stable" << static_cast<int>(in.camera_pitch_stable)
      << "hold_observable" << static_cast<int>(in.hold_observable)
      << "captured" << static_cast<int>(in.captured) << "held_complete" << static_cast<int>(in.held_complete)
      << "held_total" << in.held.total() << "corridor_complete" << static_cast<int>(in.corridor_complete)
      << "corridor_total" << in.corridor.total()
      << "run_requested" << static_cast<int>(in.run)
      << "target_valid" << static_cast<int>(in.target_valid)
      << "target_id" << in.target_id << "target_label" << in.label
      << "geometry_valid" << static_cast<int>(in.geometry_valid)
      << "path_safe" << static_cast<int>(in.path_safe)
      << "safety_ok" << static_cast<int>(in.safety_ok)
      << "zone_valid" << static_cast<int>(in.zone_valid)
      << "zone_quality_valid" << static_cast<int>(in.zone_estimate.trusted(in.now_us))
      << "zone_reason" << in.zone_estimate.reason
      << "zone_inliers" << static_cast<int>(in.zone_estimate.inlier_ids.size())
      << "zone_residual_m" << finite(in.zone_estimate.residual_m)
      << "zone_position_sigma_m" << finite(in.zone_estimate.position_sigma_m)
      << "zone_yaw_sigma_rad" << finite(in.zone_estimate.yaw_sigma_rad)
      << "zone_predicted_distance_m" << finite(in.zone_estimate.predicted_distance_m) << "}"
      << "motion" << "{" << "vx_mps" << finite(s.output.motion.vx_mps)
      << "wz_rps" << finite(s.output.motion.wz_rps) << "hardware_output_enabled" << int(s.output.hardware_output_enabled) << "}"
      << "decision" << "{"
      << "mode" << (config_.dry_run ? "dry_run" : "hardware")
      << "search_cues_enabled" << int(config_.search_cues)
      << "controlled_empty_field" << int(config_.controlled_empty_field)
      << "clearance_source" << (config_.controlled_empty_field ? "operator_cleared_test_area" : "independent_evidence_required")
      << "measured_progress_required" << int(!config_.controlled_empty_field)
      << "startup_forward_commanded_ms" << double(s.output.startup_commanded_us/1000)
      << "clear_forward_commanded_ms" << double(s.output.clear_commanded_us/1000)
      << "clearing_attempts" << s.output.clearing_attempts
      << "cue_contact_allowed" << int(s.output.cue_contact_allowed)
      << "phase" << PushTask::name(s.output.state) << "phase_reason" << s.output.reason
      << "match_state" << s.output.match_state << "match_reason" << s.output.match_reason
      << "match_remaining_ms" << double(s.output.match_remaining_us / 1000)
      << "preflight_ready" << int(s.decision.preflight_reason.empty())
      << "preflight_reason" << s.decision.preflight_reason
      << "run_permitted" << int(in.run) << "safety_ok" << int(in.safety_ok)
      << "target_id" << in.target_id << "target_label" << in.label
      << "target_is_search_cue" << int(in.target_is_search_cue)
      << "gripper_closed_observed" << int(in.gripper_closed_observed)
      << "target_valid" << int(in.target_valid) << "target_geometry_valid" << int(in.geometry_valid)
      << "target_region_known" << int(in.target_region_valid)
      << "target_in_own_zone" << int(in.target_in_zone)
      << "target_distance_m" << finite(in.distance_m)
      << "target_heading_error_rad" << finite(in.heading_error)
      << "pose_model_ran" << int(s.decision.pose_model_ran)
      << "zone_color" << s.decision.zone_color << "zone_color_reason" << s.decision.zone_color_reason
      << "geometry_reason" << s.decision.geometry_reason
      << "zone_identity_verified" << int(in.zone_identity_verified)
      << "zone_valid" << int(in.zone_valid) << "zone_own" << int(in.zone_own)
      << "zone_estimate_reason" << in.zone_estimate.reason
      << "path_safe" << int(in.path_safe) << "retreat_safe" << int(in.retreat_safe)
      << "opponent_zone_clear" << int(in.opponent_zone_clear)
      << "corridor_complete" << int(in.corridor_complete)
      << "corridor_occlusion_free" << int(in.corridor_occlusion_free)
      << "corridor_total" << in.corridor.total()
      << "hold_observable" << int(in.hold_observable)
      << "captured" << int(in.captured) << "held_complete" << int(in.held_complete)
      << "held_total" << in.held.total()
      << "gripper_feedback_confirmed" << int(in.gripper_done)
      << "gripper_feedback_open" << in.gripper_feedback_open
      << "camera_pitch_readback_cdeg" << int(in.camera_pitch_cdeg)
      << "camera_protocol_readback_deg" << (in.camera_pitch_cdeg == kCameraPitchInvalid ? kCameraPitchInvalid : cameraPitchToFeedbackDeg(in.camera_pitch_cdeg))
      << "camera_protocol_cmd_deg" << cameraPitchToWire(s.output.motion.camera_pitch_cdeg)
      << "camera_pitch_stable" << int(in.camera_pitch_stable)
      << "zone_inventory_complete" << int(in.zone_inventory_complete)
      << "zone_counts_valid" << int(in.zone_counts_valid)
      << "zone_supply_count" << in.zone_supply_count
      << "zone_injured_count" << in.zone_injured_count
      << "carry_plan_valid" << int(in.carry_plan_valid)
      << "drop_plan_valid" << int(in.drop_plan_valid)
      << "drop_locked" << int(s.output.drop_locked)
      << "drop_x_zone_m" << finite(s.output.drop_centre_zone.x)
      << "drop_y_zone_m" << finite(s.output.drop_centre_zone.y)
      << "computed_vx_mps" << finite(s.output.motion.vx_mps)
      << "computed_wz_rps" << finite(s.output.motion.wz_rps)
      << "frame_offset_cmd_deg" << int(s.output.motion.gripper_offset)
      << "camera_pitch_cmd_cdeg" << int(s.output.motion.camera_pitch_cdeg)
      << "hardware_output_enabled" << int(s.output.hardware_output_enabled)
      << "missing_evidence" << "[";
    if (!s.decision.preflight_reason.empty()) f << "preflight:" + s.decision.preflight_reason;
    if (!in.run) f << "match_not_running";
    if (!in.target_valid) f << "target_not_selected";
    if (!in.geometry_valid) f << "target_geometry_unavailable";
    if (!in.target_region_valid) f << "target_region_unknown";
    if (in.clearance_checks_disabled) f << "clearance_checks_disabled";
    if (!in.path_safe) f << "path_safety_unavailable";
    if (!in.retreat_safe) f << "retreat_safety_unavailable";
    if (!in.opponent_zone_clear) f << "opponent_zone_clearance_unavailable";
    if (!in.zone_identity_verified) f << "zone_identity_unverified";
    if (!in.zone_inventory_complete) f << "zone_inventory_incomplete";
    if (!in.zone_counts_valid) f << "zone_counts_unavailable";
    f << "]" << "}"
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
      << "search_cues_enabled" << int(config_.search_cues)
      << "controlled_empty_field" << int(config_.controlled_empty_field)
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
