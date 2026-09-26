#include "rescue/config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace rescue {

bool RectArea::contains(float x, float y) const {
    return x1 <= x && x <= x2 && y1 <= y && y <= y2;
}

bool isInList(const std::vector<std::string> &values, const std::string &value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

std::string expectedFirstBallLabel(const std::string &team) {
    return team == "blue" ? "Blue_Ball" : "Red_Ball";
}

std::string avoidBallLabel(const std::string &team) {
    return team == "blue" ? "Red_Ball" : "Blue_Ball";
}

std::string avoidAreaLabel(const std::string &team) {
    return team == "blue" ? "Red_Placement_Zone" : "Blue_Placement_Zone";
}

std::vector<std::string> makeTargetBalls(const std::string &team) {
    (void)team;
    return {"ordinary_supply", "core_supply", "injured_person"};
}

std::vector<std::string> makeTargetAreas(const std::string &team) {
    return {team == "blue" ? "blue_safe_zone" : "red_safe_zone"};
}

void printUsage(const char *program) {
    std::cout
        << "Usage: " << program << " [options]\n\n"
        << "Ground-pushing task (hardware adapter pending)\n"
        << "Options:\n"
        << "  --detect-image PATH Infer one image without camera or serial\n"
        << "  --rknn-library PATH Runtime library (default benchmark_results/librknnrt.so)\n"
        << "  --push-replay PATH  Replay validated task observations without camera or serial\n"
        << "  --model PATH        Model path, default benchmark_results/best_fp16.rknn\n"
        << "  --port PATH         Serial port, default /dev/ttyUSB0\n"
        << "  --baud N            Baudrate, default 115200\n"
        << "  --team red|blue     Team color, default red\n"
        << "  --camera N          Camera index, default 0\n"
        << "  --width N           Capture width, default 1280\n"
        << "  --height N          Capture height, default 720\n"
        << "  --fps N             Capture FPS, default 60\n"
        << "  --input-size N      YOLO input size, default 448\n"
        << "  --conf X            Confidence threshold, default 0.5\n"
        << "  --nms X             NMS threshold, default 0.45\n"
        << "  --cuda              Use OpenCV DNN CUDA backend\n"
        << "  --no-show           Do not open display window\n"
        << "  --telemetry         Publish observations to the local Foxglove bridge\n"
        << "  --telemetry-fps N   Preview rate limit, default 10 (1..30)\n"
        << "  --telemetry-file P  Snapshot in tmpfs, default /dev/shm/rescue-telemetry.bin\n"
        << "  --save              Save annotated video to output_cpp.mp4\n"
        << "  --dry-run           Skip serial open, useful for vision debugging\n"
        << "  --auto-run          Start in run command state instead of pause\n"
        << "  --require-masks     Reject box-only detections (safety mode)\n"
        << "  --sensor-timeout-ms N  Sensor freshness timeout, default 200\n"
        << "  --tof-stop-m X      ToF emergency-stop distance, default 0.18\n"
        << "  --imu-tilt-deg X    Maximum trusted pitch/roll, default 12\n"
        << "  --classes CSV       Raw model class names in tensor order\n"
        << "  --help              Show this help\n";
}

std::vector<std::string> splitCsv(const std::string &csv) {
    std::vector<std::string> out;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item.erase(item.begin(), std::find_if(item.begin(), item.end(),
                                              [](unsigned char ch) { return !std::isspace(ch); }));
        item.erase(std::find_if(item.rbegin(), item.rend(),
                                [](unsigned char ch) { return !std::isspace(ch); }).base(),
                   item.end());
        if (!item.empty()) {
            out.push_back(item);
        }
    }
    return out;
}

Config parseArgs(int argc, char **argv) {
    Config config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto needValue = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + name);
            }
            return argv[++i];
        };

        if (arg == "--detect-image") {
            config.detect_image = needValue(arg);
        } else if (arg == "--rknn-library") {
            config.rknn_library = needValue(arg);
        } else if (arg == "--push-replay") {
            config.push_replay = needValue(arg);
        } else if (arg == "--model") {
            config.model_path = needValue(arg);
        } else if (arg == "--port") {
            config.uart_port = needValue(arg);
        } else if (arg == "--baud") {
            config.baudrate = std::stoi(needValue(arg));
        } else if (arg == "--team") {
            config.team = needValue(arg);
        } else if (arg == "--camera") {
            config.camera_index = std::stoi(needValue(arg));
        } else if (arg == "--width") {
            config.frame_width = std::stoi(needValue(arg));
        } else if (arg == "--height") {
            config.frame_height = std::stoi(needValue(arg));
        } else if (arg == "--fps") {
            config.fps = std::stoi(needValue(arg));
        } else if (arg == "--input-size") {
            config.input_size = std::stoi(needValue(arg));
        } else if (arg == "--conf") {
            config.confidence = std::stof(needValue(arg));
        } else if (arg == "--nms") {
            config.nms = std::stof(needValue(arg));
        } else if (arg == "--sensor-timeout-ms") {
            config.sensor_timeout_ms = static_cast<uint32_t>(std::stoul(needValue(arg)));
        } else if (arg == "--tof-stop-m") {
            config.tof_stop_distance_m = std::stof(needValue(arg));
        } else if (arg == "--imu-tilt-deg") {
            config.imu_tilt_limit_deg = std::stof(needValue(arg));
        } else if (arg == "--classes") {
            config.class_names = splitCsv(needValue(arg));
        } else if (arg == "--cuda") {
            config.use_cuda = true;
        } else if (arg == "--telemetry") {
            config.telemetry = true;
        } else if (arg == "--telemetry-fps") {
            config.telemetry_fps = std::stoi(needValue(arg));
        } else if (arg == "--telemetry-file") {
            config.telemetry_file = needValue(arg);
        } else if (arg == "--no-show") {
            config.show = false;
        } else if (arg == "--save") {
            config.save_output = true;
        } else if (arg == "--dry-run") {
            config.dry_run = true;
        } else if (arg == "--auto-run") {
            config.auto_run = true;
        } else if (arg == "--require-masks") {
            config.require_instance_masks = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (config.team != "red" && config.team != "blue") {
        throw std::runtime_error("--team must be red or blue");
    }
    if (config.input_size <= 0 || !std::isfinite(config.confidence) || config.confidence < 0 || config.confidence > 1 ||
        !std::isfinite(config.nms) || config.nms < 0 || config.nms > 1)
        throw std::runtime_error("Invalid model size or confidence/NMS thresholds");
    if (config.class_names.size() != 7) {
        throw std::runtime_error("This model profile requires exactly seven classes");
    }
    if (config.sensor_timeout_ms == 0 ||
        config.tof_stop_distance_m <= 0.0f || config.imu_tilt_limit_deg <= 0.0f) {
        throw std::runtime_error("Safety timing and distance limits must be positive");
    }
    if (config.telemetry_fps < 1 || config.telemetry_fps > 30 ||
        config.telemetry_file.rfind("/dev/shm/", 0) != 0 ||
        config.telemetry_file.find("..") != std::string::npos) {
        throw std::runtime_error("Telemetry requires 1..30 FPS and an absolute /dev/shm/ snapshot path");
    }
    return config;
}

} // namespace rescue
