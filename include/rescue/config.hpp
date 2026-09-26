#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rescue {

struct RectArea {
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;

    bool contains(float x, float y) const;
};

struct Config {
    std::string push_replay;
    std::string detect_image;
    std::string rknn_library = "./benchmark_results/librknnrt.so";
    std::string model_path = "./benchmark_results/best_fp16.rknn";
    std::string uart_port = "/dev/ttyUSB0";
    std::string team = "red";
    int baudrate = 115200;
    int camera_index = 0;
    int frame_width = 1280;
    int frame_height = 720;
    int fps = 60;
    int detect_interval = 1;
    int input_size = 448;
    float confidence = 0.5f;
    float nms = 0.45f;
    bool use_cuda = false;
    bool show = true;
    bool save_output = false;
    bool dry_run = false;
    bool telemetry = false;
    int telemetry_fps = 10;
    std::string telemetry_file = "/dev/shm/rescue-telemetry.bin";
    bool auto_run = false;
    bool require_instance_masks = false;

    // Safety and timing limits used by the non-blocking rescue pipeline.
    uint32_t sensor_timeout_ms = 200;

    uint32_t tracker_max_age_ms = 350;
    float tof_stop_distance_m = 0.18f;
    float imu_tilt_limit_deg = 12.0f;
    float pnp_max_reprojection_error_px = 4.0f;
    float pnp_jump_limit_m = 0.35f;

    RectArea catch_area{165.0f, 430.0f, 485.0f, 460.0f};
    RectArea ready_area{160.0f, 178.0f, 480.0f, 388.0f};
    RectArea holding_area{270.0f, 380.0f, 410.0f, 460.0f};
    RectArea center_region{250.0f, 208.0f, 390.0f, 348.0f};
    RectArea cross_center_region{170.0f, 430.0f, 480.0f, 460.0f};
    std::array<int, 2> catch_angle{90, 90};
    std::array<int, 2> release_angle{0, 0};

    std::vector<std::string> class_names{
        "core", "wounded", "red", "dangerous", "normal", "main", "blue"
    };
};

void printUsage(const char *program);
Config parseArgs(int argc, char **argv);

std::string expectedFirstBallLabel(const std::string &team);
std::string avoidBallLabel(const std::string &team);
std::string avoidAreaLabel(const std::string &team);
std::vector<std::string> makeTargetBalls(const std::string &team);
std::vector<std::string> makeTargetAreas(const std::string &team);
bool isInList(const std::vector<std::string> &values, const std::string &value);

} // namespace rescue
