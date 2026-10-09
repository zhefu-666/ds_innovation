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
    std::string demo_mode="none"; // none, recognize, search, carry_once
    uint32_t demo_seconds=60; // hard wall limit for standalone demonstrations
    std::string push_replay;
    std::string geometry_replay, calibration_file, keypoints_file;
    std::string zone_geometry_file = "./config/zone_geometry.json";
    bool pitch_feedback = false; // O_RDONLY MCU receive; no actuator commands
    std::string detect_image;
    std::string rknn_library = "./benchmark_results/librknnrt.so";
    std::string model_path = "./models/detect_fp.rknn";
    // Optional YOLOv8-pose safe-zone model (zone_left/zone_right, 4 kpts each), own RKNN context.
    std::string pose_model_path; // 通用模型；parseArgs 后为本次实际使用的模型路径。
    // 预留两版安全区模型接口：按己方颜色选择，仅路径不同，不分设置信度。
    // 两版保持相同 zone_left/zone_right 类别、4 个关键点顺序及输出张量。
    // 选择某颜色模型不等于确认画面区域归属，仍需独立颜色观测。
    std::string pose_model_blue_path;
    std::string pose_model_red_path;
    float pose_confidence = 0.25f;     // 两版共用：per-half box score
    float pose_keypoint_confidence = 0.5f; // 两版共用：per-keypoint visibility
    int detect_core_mask = -1;         // -1 leaves the runtime default; 0 auto, 1 core0, 2 core1, 4 core2
    int pose_core_mask = -1;
    bool parallel_inference = false;   // run detect and pose concurrently on separate contexts
    std::string uart_port = "/dev/ttyACM0"; // 下位机MCU（USB CDC）；IMU见imu_port
    std::string team = "blue"; // 暂定蓝色己方、红色对方；比赛时 --team red 可切换。
    int baudrate = 115200;
    int camera_index = 0;
    int frame_width = 1280;
    int frame_height = 720;
    int fps = 60;
    int detect_interval = 1;
    int input_size = 640;
    float confidence = 0.5f;
    float nms = 0.45f;
    bool use_cuda = false;
    bool show = true;
    bool save_output = false;
    bool dry_run = false;
    bool controlled_ignore_clearance = false; // explicit contact-test override, never default
    bool search_cues = false; // explicitly enabled controlled-field pile exploration
    bool controlled_empty_field = false; // explicit operator attestation, never a competition default
    // TEMP_ASSUMPTION（2026-10-08用户要求“默认全都安全”）：所有安全/净空/区域/确认证据视为成立，仅用于跑通决策链。
    bool assume_all_safe = false;
    // TEMP_ASSUMPTION：演示伤员趟——启动即视为已送达首个普通物资，并优先锁定 injured_person。仅与 --assume-all-safe 同用。
    bool assume_injured_trip = false;
    bool check_config = false; // files only; never initializes camera, NPU, sockets or serial
    // Live MCU link: one exclusive RDWR port for motion output and A6 feedback (implies feedback).
    bool hardware = false;
    bool imu = false;
    std::string imu_port = "/dev/ttyUSB0";
    int imu_baud = 115200;
    uint32_t imu_timeout_ms = 200;
    bool telemetry = false;
    int telemetry_fps = 10;
    std::string telemetry_file = "/dev/shm/rescue-telemetry.bin";
    bool auto_run = false;
    bool allow_mechanical_pitch_model = false;
    // TEMP_ASSUMPTION done-flag A6: operator-supplied frame angle/action id from this session's last command.
    int known_frame_angle = -1, known_frame_id = 0;
    bool no_match_time_limit = false; // explicit debugging override
    uint32_t match_seconds = 180; // confirmed competition duration: 3 minutes
    uint32_t startup_advance_ms = 15000; // CLI startup duration: 1.0 m/s for 15 s by default
    float startup_speed_mps = 1.0f, scan_wz_rps = .25f, turn_wz_rps = .3f;
    std::string task_calibration_file, zone_color_file;
    std::string match_socket = "/tmp/rescue-match.sock";
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
    // PushTask camera presets FAR,TRACK,NEAR in 0.01 deg (positive down), overriding TaskTuning.
    // 40度仅用于停稳后的夹持观察；地面测距仍须按实际角度单独验证。
    std::array<int16_t, 3> pitch_presets_cdeg{500, 500, 4000};

    // blocks detector tensor order; taskLabel() maps colours to task semantics.
    std::vector<std::string> class_names{"blue", "orange", "green", "black"};
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
