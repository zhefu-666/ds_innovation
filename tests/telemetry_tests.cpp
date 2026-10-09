#include "rescue/telemetry_publisher.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <iostream>

int main() {
    char dir[] = "/dev/shm/rescue-telemetry-test-XXXXXX";
    assert(mkdtemp(dir));
    rescue::Config config;
    config.telemetry_file = std::string(dir) + "/snapshot.bin";
    config.telemetry_fps = 30;
    {
        rescue::TelemetryPublisher publisher(config);
        bool duplicate_rejected = false;
        try { rescue::TelemetryPublisher other(config); }
        catch (const std::exception&) { duplicate_rejected = true; }
        assert(duplicate_rejected);
        cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(20, 100, 200));
        rescue::SegDetection detection;
        detection.label = "ordinary_supply"; detection.model_label = "quoted \"label\"";
        detection.track_id = 42; detection.confidence = 0.9f;
        detection.box = {1, 2, 30, 40};
        rescue::PushObservation observation;
        observation.target_valid = true; observation.target_id = 42;
        observation.label = "ordinary_supply";
        rescue::PushOutput output;
        output.reason = "waiting_for_permission";
        rescue::DecisionTelemetry decision;
        decision.preflight_reason = "camera_pitch_feedback_invalid";
        decision.zone_color_reason = "color_unmeasured";
        decision.geometry_reason = "pitch_unusable";
        rescue::ImuSnapshot imu;
        imu.connected = imu.fresh = imu.sample.measurements_valid = true;
        imu.sample.sequence = 100;
        imu.sample.received_us = 123456789;
        imu.sample.status = 0x2323;
        imu.sample.angular_velocity_rps = {0.1f, 0.2f, 0.3f};
        imu.sample.body_angular_velocity_rps = {0.1f, -0.2f, -0.3f};
        publisher.submit(frame, {detection}, observation, output, 1790000000123456789ULL, 5, 25, 20, 5, imu, decision);
        // Changing main's image must not race with or change the published snapshot.
        frame.setTo(cv::Scalar(0, 0, 0));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (access(config.telemetry_file.c_str(), F_OK) != 0 && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::ifstream in(config.telemetry_file, std::ios::binary);
        assert(in.good());
        char magic[8]; in.read(magic, 8);
        assert(std::string(magic, 8) == "RSTEL001");
        auto u32 = [&] { unsigned char b[4]; in.read(reinterpret_cast<char*>(b), 4);
            return uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) | (uint32_t(b[3]) << 24); };
        const auto json_size = u32(), image_size = u32();
        assert(json_size < 1024*1024 && image_size < 8*1024*1024);
        std::string json(json_size, '\0'); in.read(json.data(), json_size);
        std::vector<unsigned char> jpeg(image_size); in.read(reinterpret_cast<char*>(jpeg.data()), image_size);
        assert(in.peek() == EOF);
        cv::FileStorage data(json, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
        assert(static_cast<int>(data["sequence"]) == 5);
        assert(static_cast<int>(data["timestamp"]["nsec"]) == 123456789);
        assert(static_cast<int>(data["detections"][0]["track_id"]) == 42);
        assert(static_cast<std::string>(data["detections"][0]["model_label"]) == detection.model_label);
        assert(static_cast<int>(data["motion"]["hardware_output_enabled"]) == 0);
        assert(static_cast<std::string>(data["decision"]["phase"]) == "WAIT_START");
        assert(static_cast<std::string>(data["decision"]["preflight_reason"]) == decision.preflight_reason);
        assert(static_cast<int>(data["decision"]["target_id"]) == 42);
        assert(static_cast<int>(data["decision"]["path_safe"]) == 0);
        assert(static_cast<std::string>(data["decision"]["missing_evidence"][0]) ==
               "preflight:camera_pitch_feedback_invalid");
        assert(static_cast<int>(data["imu"]["sequence"]) == 100);
        assert(static_cast<int>(data["imu"]["attitude_valid_for_control"]) == 0);
        assert(static_cast<int>(data["imu"]["status_raw"]) == 0x2323);
        assert(static_cast<double>(data["imu"]["received_monotonic_us"]) == 123456789);
        assert(std::abs(static_cast<double>(data["imu"]["angular_velocity_rps"][1]) - 0.2) < 1e-6);
        assert(static_cast<std::string>(data["imu"]["body_frame_id"]) == "base_link");
        assert(std::abs(static_cast<double>(data["imu"]["body_angular_velocity_rps"][2]) + 0.3) < 1e-6);
        auto image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
        assert(image.cols == 640 && image.rows == 360);
        assert(cv::mean(image(cv::Rect(0,60,image.cols,image.rows-60)))[2] > 190); // Owned source pixels, not the black reused frame.
    }
    assert(access(config.telemetry_file.c_str(), F_OK) != 0);
    unlink((config.telemetry_file + ".lock").c_str()); rmdir(dir);
    std::cout << "Telemetry snapshot ownership, wire format, source timestamps and cleanup passed\n";
}
