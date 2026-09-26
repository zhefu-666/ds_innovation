#include "rescue/config.hpp"
#include "rescue/detector.hpp"
#include "rescue/push_task.hpp"
#include "rescue/perception_adapter.hpp"
#include "rescue/tracker.hpp"
#include "rescue/utils.hpp"
#include "rescue/vision_logic.hpp"
#include <opencv2/opencv.hpp>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <memory>
#ifdef RESCUE_ENABLE_TELEMETRY
#include "rescue/telemetry_publisher.hpp"
#endif

namespace {
rescue::PushObservation readObservation(const cv::FileNode &n) {
    rescue::PushObservation in;
    double timestamp = 0;
    n["now_us"] >> timestamp;
    if (!std::isfinite(timestamp) || timestamp <= 0 || timestamp > 9007199254740991.0)
        throw std::runtime_error("Replay now_us must be a positive integer <= 2^53-1");
    if (std::floor(timestamp) != timestamp) throw std::runtime_error("Fractional now_us");
    in.now_us = static_cast<uint64_t>(timestamp);
    auto flag = [&](const char *key, bool &value) {
        if (!n[key].empty()) value = static_cast<int>(n[key]) != 0;
    };
    flag("run", in.run); flag("reset", in.reset); flag("safety_ok", in.safety_ok);
    flag("target_valid", in.target_valid); flag("geometry_valid", in.geometry_valid);
    flag("path_safe", in.path_safe); flag("in_push_region", in.in_push_region);
    flag("zone_valid", in.zone_valid); flag("zone_own", in.zone_own);
    flag("zone_aligned", in.zone_aligned); flag("fully_inside", in.fully_inside);
    flag("off_fence", in.off_fence); flag("stable", in.stable);
    flag("separated", in.separated); flag("retreat_safe", in.retreat_safe);
    if (!n["target_id"].empty()) n["target_id"] >> in.target_id;
    if (!n["available_count"].empty()) n["available_count"] >> in.available_count;
    if (!n["delivered_count"].empty()) n["delivered_count"] >> in.delivered_count;
    n["label"] >> in.label; n["zone_class"] >> in.zone_class;
    n["distance_m"] >> in.distance_m; n["heading_error"] >> in.heading_error;
    return in;
}
void report(const rescue::PushOutput &out) {
    std::cout << rescue::PushTask::name(out.state) << " batch=" << out.batch_size
              << " delivered=" << out.delivered_total << " vx=" << out.motion.vx_mps
              << " wz=" << out.motion.wz_rps << " stop="
              << (out.motion.vx_mps == 0.0f && out.motion.wz_rps == 0.0f) << "\n";
}
}
int main(int argc, char **argv) {
    using namespace rescue;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        const Config config = parseArgs(argc, argv);
        PushTask task;
#ifndef RESCUE_ENABLE_TELEMETRY
        if (config.telemetry)
            throw std::runtime_error("Rebuild with -DRESCUE_ENABLE_TELEMETRY=ON to enable telemetry");
#endif
        if (!config.push_replay.empty()) {
            cv::FileStorage replay(config.push_replay, cv::FileStorage::READ);
            if (!replay.isOpened() || !replay["frames"].isSeq())
                throw std::runtime_error("Replay must contain a frames array");
            for (const auto &frame : replay["frames"]) report(task.update(readObservation(frame)));
            return 0;
        }
        if (!config.detect_image.empty()) {
            auto detector = makeDetector(config);
            auto frame = cv::imread(config.detect_image);
            if (frame.empty()) throw std::runtime_error("Cannot read input image");
            auto detections = detector->infer(frame);
            std::cout << "detections=" << detections.size() << "\n";
            for (const auto &d : detections)
                std::cout << "id=" << d.class_id << " raw=" << d.model_label << " task=" << d.label
                          << " confidence=" << d.confidence << " box=" << d.box.x << "," << d.box.y
                          << "," << d.box.width << "," << d.box.height << "\n";
            return 0;
        }
        if (!config.dry_run)
            throw std::runtime_error("Push hardware output is not connected yet. Use --dry-run "
                "for perception or --push-replay PATH for task verification. No serial port was opened.");
        auto detector = makeDetector(config);
        VisionLogic vision(config);
        NearestNeighborTracker tracker;
        // Use the USB camera's MJPEG V4L2 path; automatic GStreamer negotiation
        // fails when applying the requested 720p/60 FPS settings on this board.
        cv::VideoCapture camera(config.camera_index, cv::CAP_V4L2);
        if (!camera.isOpened()) throw std::runtime_error("Cannot open camera");
        camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        camera.set(cv::CAP_PROP_FRAME_WIDTH, config.frame_width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, config.frame_height);
        camera.set(cv::CAP_PROP_FPS, config.fps);
        cv::VideoWriter writer;
        if (config.save_output) {
            writer.open("output_cpp.mp4", cv::VideoWriter::fourcc('m','p','4','v'), config.fps,
                        cv::Size(camera.get(cv::CAP_PROP_FRAME_WIDTH), camera.get(cv::CAP_PROP_FRAME_HEIGHT)));
            if (!writer.isOpened()) throw std::runtime_error("Cannot open recording");
        }
        std::cout << "Ground-pushing preview: waiting for validated geometry, subzone and safety inputs.\n";
#ifdef RESCUE_ENABLE_TELEMETRY
        std::unique_ptr<TelemetryPublisher> telemetry;
        if (config.telemetry) {
            try {
                telemetry = std::make_unique<TelemetryPublisher>(config);
                std::cout << "[TELEMETRY] snapshot=" << config.telemetry_file << "\n";
            } catch (const std::exception& e) {
                std::cerr << "[TELEMETRY] disabled: " << e.what() << "\n";
            }
        }
#endif
        uint64_t frame_sequence = 0, rate_frames = 0;
        auto rate_started = Clock::now();
        double loop_fps = 0;
        PushOutput previous;
        auto last_report = Clock::now();
        while (!g_should_exit.load()) {
            const auto capture_started = Clock::now();
            cv::Mat frame;
            if (!camera.read(frame) || frame.empty()) throw std::runtime_error("Camera frame lost");
            const auto now = Clock::now();
            const auto epoch_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            const double capture_ms = std::chrono::duration<double, std::milli>(now - capture_started).count();
            const auto timestamp = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
            const auto detections = tracker.update(detector->infer(frame), timestamp);
            const double inference_ms = std::chrono::duration<double, std::milli>(Clock::now() - now).count();
            const auto observed_at = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                Clock::now().time_since_epoch()).count());
            auto input = makePushObservation(detections, observed_at,
                previous.first_ordinary_delivered, config.confidence);
            input.run = config.auto_run;
            // Raw boxes do not establish metric distance, route safety or delivery.
            // Keep unconnected evidence invalid; never synthesize successful observations.
            const auto out = task.update(input);
            previous = out;
            ++frame_sequence; ++rate_frames;
            const auto rate_now = Clock::now();
            const double rate_seconds = std::chrono::duration<double>(rate_now - rate_started).count();
            if (rate_seconds >= 1.0) {
                loop_fps = rate_frames / rate_seconds;
                rate_frames = 0; rate_started = rate_now;
            }
            if (now - last_report >= Ms(500)) {
                report(out);
                std::cout << "[PERF] loop_fps=" << loop_fps << " inference_ms=" << inference_ms
                          << " capture_ms=" << capture_ms << " detections=" << detections.size() << "\n";
                last_report = now;
            }
            vision.drawDetections(frame, detections);
            cv::putText(frame, "PUSH PREVIEW - HARDWARE DISABLED", {10,25},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,255}, 2);
#ifdef RESCUE_ENABLE_TELEMETRY
            if (telemetry) telemetry->submit(frame, detections, input, out, epoch_ns,
                                             frame_sequence, loop_fps, inference_ms, capture_ms);
#endif
            if (writer.isOpened()) writer.write(frame);
            if (config.show) {
                cv::imshow("Ground pushing", frame);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') break;
            }
        }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
