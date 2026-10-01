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
        << "Ground-pushing task\n"
        << "Options:\n"
        << "  --detect-image PATH Infer one image without camera or serial\n"
        << "  --rknn-library PATH Runtime library (default benchmark_results/librknnrt.so)\n"
        << "  --push-replay PATH  Replay validated task observations without camera or serial\n"
        << "  --geometry-replay P Pixel/IMU/pitch geometry replay; no devices or model\n"
        << "  --calibration PATH Validated camera ground calibration, bound to actual pitch\n"
        << "  --zone-geometry P  Fixed landmark schema (default config/zone_geometry.json)\n"
        << "  --pitch-feedback   Open MCU feedback read-only for synchronized ground mapping\n"
        << "  --keypoints-file P Optional atomic same-frame landmark JSON (no box-derived points)\n"
        << "  --model PATH        Detector model, default models/detect_fp.rknn\n"
        << "  --pose-model PATH   Safe-zone YOLOv8-pose RKNN (zone_left/zone_right, 4 kpts); replaces --keypoints-file\n"
        << "  --pose-model-blue P Blue-team pose model override; same thresholds and output schema\n"
        << "  --pose-model-red P  Red-team pose model override; same thresholds and output schema\n"
        << "  --pose-conf X       Safe-zone half score threshold, default 0.25\n"
        << "  --pose-kpt-conf X   Safe-zone keypoint visibility threshold, default 0.5\n"
        << "  --detect-core N     RKNN core mask for detector (0 auto, 1, 2, 4)\n"
        << "  --pose-core N       RKNN core mask for pose (0 auto, 1, 2, 4)\n"
        << "  --parallel-infer    Run detector and pose concurrently\n"
        << "  --port PATH         MCU serial port, default /dev/ttyACM0\n"
        << "  --baud N            Baudrate, default 115200\n"
        << "  --team red|blue     Own zone color, default blue (opponent red)\n"
        << "  --camera N          Camera index, default 0\n"
        << "  --width N           Capture width, default 1280\n"
        << "  --height N          Capture height, default 720\n"
        << "  --fps N             Capture FPS, default 60\n"
        << "  --input-size N      Model input size (both models), default 640\n"
        << "  --conf X            Confidence threshold, default 0.5\n"
        << "  --nms X             NMS threshold, default 0.45\n"
        << "  --cuda              Use OpenCV DNN CUDA backend\n"
        << "  --no-show           Do not open display window\n"
        << "  --imu               Receive HiPNUC HI91 in preview (body axes; stale/over-tilt vetoes safety, no motion output)\n"
        << "  --imu-port PATH     IMU serial port, default /dev/ttyUSB0\n"
        << "  --imu-baud N        IMU baudrate, default 115200\n"
        << "  --imu-timeout-ms N  IMU freshness limit, default 200 (1..10000)\n"
        << "  --telemetry         Publish observations to the local Foxglove bridge\n"
        << "  --telemetry-fps N   Preview rate limit, default 10 (1..30)\n"
        << "  --telemetry-file P  Snapshot in tmpfs, default /dev/shm/rescue-telemetry.bin\n"
        << "  --save              Save annotated video to output_cpp.mp4\n"
        << "  --dry-run           Disable motion output; --imu may open IMU input\n"
        << "  --hardware          Send PushTask motion/gripper/pitch to the MCU (25 Hz, zero velocity on stall/exit);\n"
        << "                      needs --imu, reads gripper and pitch feedback from the same port\n"
        << "  --auto-run          Request START once after live preflight; never auto-resume faults\n"
        << "  --match-seconds N   Competition duration, default 180 s (1..86400)\n"
        << "  --zone-color-calibration P  Validated red/blue zone color thresholds\n"
        << "  --task-calibration P  Measured gripper geometry and NEAR image region JSON\n"
        << "  --match-socket P    Local control socket, default /tmp/rescue-match.sock\n"
        << "  --require-masks     Reject box-only detections (safety mode)\n"
        << "  --sensor-timeout-ms N  Sensor freshness timeout, default 200\n"
        << "  --tof-stop-m X      ToF emergency-stop distance, default 0.18\n"
        << "  --imu-tilt-deg X    Body pitch/roll stop limit, default 12 (ground mapping uses its own tighter limit)\n"
        << "  --pitch-presets F,T,N  Camera FAR,TRACK,NEAR presets in 0.01 deg, positive down, default 0,2500,2500\n"
        << "                      (wire pitch uses integer degrees); needs -4000<=F<=T<=N<=4000\n"
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
        } else if (arg == "--geometry-replay") {
            config.geometry_replay = needValue(arg);
        } else if (arg == "--calibration") {
            config.calibration_file = needValue(arg);
        } else if (arg == "--zone-geometry") {
            config.zone_geometry_file = needValue(arg);
        } else if (arg == "--pitch-feedback") {
            config.pitch_feedback = true;
        } else if (arg == "--keypoints-file") {
            config.keypoints_file = needValue(arg);
        } else if (arg == "--model") {
            config.model_path = needValue(arg);
        } else if (arg == "--pose-model-blue") {
            config.pose_model_blue_path = needValue(arg);
        } else if (arg == "--pose-model-red") {
            config.pose_model_red_path = needValue(arg);
        } else if (arg == "--pose-model") {
            config.pose_model_path = needValue(arg);
        } else if (arg == "--pose-conf") {
            config.pose_confidence = std::stof(needValue(arg));
        } else if (arg == "--pose-kpt-conf") {
            config.pose_keypoint_confidence = std::stof(needValue(arg));
        } else if (arg == "--detect-core") {
            config.detect_core_mask = std::stoi(needValue(arg));
        } else if (arg == "--pose-core") {
            config.pose_core_mask = std::stoi(needValue(arg));
        } else if (arg == "--parallel-infer") {
            config.parallel_inference = true;
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
        } else if (arg == "--pitch-presets") {
            const auto items = splitCsv(needValue(arg));
            if (items.size() != 3) throw std::runtime_error("--pitch-presets needs FAR,TRACK,NEAR");
            for (size_t k = 0; k < 3; ++k) {
                size_t used = 0;
                const int v = std::stoi(items[k], &used);
                if (used != items[k].size() || v < -4000 || v > 4000)
                    throw std::runtime_error("--pitch-presets values must be integers in -4000..4000 (0.01 deg)");
                config.pitch_presets_cdeg[k] = static_cast<int16_t>(v);
            }
            const auto &p = config.pitch_presets_cdeg;
            if (p[0] > p[1] || p[1] > p[2]) throw std::runtime_error("--pitch-presets needs FAR <= TRACK <= NEAR");
        } else if (arg == "--classes") {
            config.class_names = splitCsv(needValue(arg));
        } else if (arg == "--cuda") {
            config.use_cuda = true;
        } else if (arg == "--imu") {
            config.imu = true;
        } else if (arg == "--imu-port") {
            config.imu_port = needValue(arg);
        } else if (arg == "--imu-baud") {
            config.imu_baud = std::stoi(needValue(arg));
        } else if (arg == "--imu-timeout-ms") {
            const int value = std::stoi(needValue(arg));
            if (value < 1 || value > 10000) throw std::runtime_error("IMU timeout must be 1..10000 ms");
            config.imu_timeout_ms = static_cast<uint32_t>(value);
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
        } else if (arg == "--hardware") {
            config.hardware = true;
        } else if (arg == "--match-seconds") {
            const auto text=needValue(arg); size_t used=0; const long v=std::stol(text,&used);
            if(used!=text.size() || v<1 || v>86400) throw std::runtime_error("--match-seconds must be 1..86400");
            config.match_seconds=static_cast<uint32_t>(v);
        } else if (arg == "--zone-color-calibration") {
            config.zone_color_file=needValue(arg);
        } else if (arg == "--task-calibration") {
            config.task_calibration_file=needValue(arg);
        } else if (arg == "--match-socket") {
            config.match_socket=needValue(arg);
            if(config.match_socket.empty() || config.match_socket[0]!='/' || config.match_socket.size()>=108)
                throw std::runtime_error("--match-socket must be an absolute Unix socket path");
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
    // 统一在参数解析完成后选择，避免 --team 与模型参数的先后顺序影响结果。
    // 本方专用路径优先；未配置时沿用 --pose-model 通用模型，绝不借用对方模型。
    // 当前只运行所选的一版；两种颜色共用置信度、NMS、输入尺寸和推理解码。
    const auto &team_pose = config.team == "blue" ? config.pose_model_blue_path : config.pose_model_red_path;
    if (!team_pose.empty()) config.pose_model_path = team_pose;
    if (config.pose_model_path.empty() &&
        (!config.pose_model_blue_path.empty() || !config.pose_model_red_path.empty()))
        throw std::runtime_error("No pose model for selected --team; provide its model or --pose-model fallback");
    if (config.input_size <= 0 || !std::isfinite(config.confidence) || config.confidence < 0 || config.confidence > 1 ||
        !std::isfinite(config.nms) || config.nms < 0 || config.nms > 1)
        throw std::runtime_error("Invalid model size or confidence/NMS thresholds");
    if (config.class_names.empty()) throw std::runtime_error("--classes must name at least one class");
    auto validThreshold = [](float v) { return std::isfinite(v) && v >= 0 && v <= 1; };
    if (!validThreshold(config.pose_confidence) || !validThreshold(config.pose_keypoint_confidence))
        throw std::runtime_error("Pose thresholds must be in [0,1]");
    auto validCore = [](int m) { return m == -1 || m == 0 || m == 1 || m == 2 || m == 4; };
    if (!validCore(config.detect_core_mask) || !validCore(config.pose_core_mask))
        throw std::runtime_error("Core mask must be 0, 1, 2 or 4");
    if (!config.pose_model_path.empty() && !config.keypoints_file.empty())
        throw std::runtime_error("Use either --pose-model or --keypoints-file");
    if (config.parallel_inference && config.pose_model_path.empty())
        throw std::runtime_error("--parallel-infer requires --pose-model");
    if (config.sensor_timeout_ms == 0 ||
        config.tof_stop_distance_m <= 0.0f || config.imu_tilt_limit_deg <= 0.0f) {
        throw std::runtime_error("Safety timing and distance limits must be positive");
    }
    if (config.telemetry_fps < 1 || config.telemetry_fps > 30 ||
        config.telemetry_file.rfind("/dev/shm/", 0) != 0 ||
        config.telemetry_file.find("..") != std::string::npos) {
        throw std::runtime_error("Telemetry requires 1..30 FPS and an absolute /dev/shm/ snapshot path");
    }
    if (config.imu_port.empty() || (config.imu_baud != 9600 && config.imu_baud != 115200 &&
        config.imu_baud != 230400 && config.imu_baud != 460800 && config.imu_baud != 921600))
        throw std::runtime_error("Invalid IMU port or unsupported baudrate");
    const int modes = int(!config.push_replay.empty()) + int(!config.detect_image.empty()) + int(!config.geometry_replay.empty());
    if (modes > 1) throw std::runtime_error("Select only one replay/image mode");
    if ((!config.geometry_replay.empty() || !config.keypoints_file.empty()) && config.calibration_file.empty())
        throw std::runtime_error("Geometry input requires --calibration");
    if (config.hardware && config.dry_run) throw std::runtime_error("Use either --dry-run or --hardware");
    if (config.hardware && modes) throw std::runtime_error("--hardware is live-loop only");
    if (config.hardware && config.pitch_feedback)
        throw std::runtime_error("--hardware already reads MCU feedback; drop --pitch-feedback");
    if (config.hardware && !config.imu) throw std::runtime_error("--hardware requires --imu (tilt/staleness veto)");
    if (config.pitch_feedback && config.calibration_file.empty())
        throw std::runtime_error("Pitch feedback preview requires --calibration");
    if (!config.calibration_file.empty() && (!config.push_replay.empty() || !config.detect_image.empty()))
        throw std::runtime_error("Use --geometry-replay or live preview with --calibration");
    if (config.pitch_feedback && modes) throw std::runtime_error("Pitch receiver is live-preview only");
    if (!config.geometry_replay.empty() && (!config.keypoints_file.empty() || config.imu))
        throw std::runtime_error("Geometry replay must use recorded sensors/keypoints, no live sources");
    if (!config.calibration_file.empty() && modes==0 && (!config.imu || !(config.pitch_feedback || config.hardware)))
        throw std::runtime_error("Live ground mapping requires --imu and --pitch-feedback (or --hardware)");
    if ((config.pitch_feedback || config.hardware) && config.imu && config.uart_port==config.imu_port)
        throw std::runtime_error("MCU feedback and IMU must use different serial ports");
    if (!std::isfinite(config.tof_stop_distance_m) || !std::isfinite(config.imu_tilt_limit_deg))
        throw std::runtime_error("Safety thresholds must be finite");
    if (config.imu && (!config.push_replay.empty() || !config.detect_image.empty()))
        throw std::runtime_error("--imu is supported in live preview only");
    return config;
}

} // namespace rescue
