#include "rescue/camera_calibration.hpp"
#include "rescue/imu_adapter.hpp"
#include "rescue/planner.hpp"
#include "rescue/sensor_fusion.hpp"
#include "rescue/safe_zone_pose.hpp"
#include "rescue/tracker.hpp"
#include "rescue/uart_controller.hpp"
#include "rescue/zone_layout.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <unistd.h>

using namespace rescue;

int main() {
    {
        NearestNeighborTracker tracker({30.0f, 2, 4});
        SegDetection first;
        first.label = "ordinary_supply";
        first.confidence = 0.9f;
        first.box = cv::Rect(10, 10, 20, 20);
        auto a = tracker.update({first}, 1000);
        assert(a.size() == 1 && a[0].track_id > 0);
        const int id = a[0].track_id;
        first.box.x = 15;
        auto b = tracker.update({first}, 2000);
        assert(b.size() == 1 && b[0].track_id == id);
    }

    {
        // 固定单应绑定标定时的相机pitch：读回偏离、读回无效都拒绝。
        CameraCalibration calibration;
        calibration.setGroundHomography(cv::Mat::eye(3, 3, CV_64F), 1500);
        calibration.setIntrinsics(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 5, CV_64F));
        SensorState sensor;
        sensor.imu_valid = true;
        cv::Point2f ground;
        assert(calibration.pixelToGround({0.25f, 0.5f}, 1500, ground, &sensor));
        assert(std::abs(ground.x - 0.25f) < 1e-5f);
        assert(calibration.pixelToGround({0.25f, 0.5f}, 1540, ground, &sensor));   // 容差±0.5°内
        assert(!calibration.pixelToGround({0.25f, 0.5f}, 1600, ground, &sensor));  // 舵机已动
        assert(!calibration.pixelToGround({0.25f, 0.5f}, kCameraPitchInvalid, ground, &sensor));
    }

    {
        LocalPlanner planner({0.35f, 0.1f, 0.1f, 0.05f});
        const std::vector<PlannerObstacle> obstacles{{{0.5f, 0.0f}, 0.05f, true}};
        auto routes = planner.plan({0.0f, 0.0f}, {1.5f, 0.0f}, {2.5f, 0.0f}, obstacles);
        assert(!routes.empty());
        assert(routes.front().kind != "direct");
    }

    {
        const cv::Mat camera = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
        const std::vector<cv::Point3f> object{{-0.5f, -0.4f, 0}, {0.5f, -0.4f, 0},
                                              {0.5f, 0.4f, 0}, {-0.5f, 0.4f, 0}};
        const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 2);
        std::vector<cv::Point2f> image;
        cv::projectPoints(object, rvec, tvec, camera, cv::Mat(), image);
        SafeZonePoseEstimator estimator(camera, cv::Mat());
        SafeZoneObservation observation{"red_safe_zone", image, object, true};
        auto pose = estimator.estimate(observation);
        assert(pose.valid && pose.view == SafeZonePose::FRONT);
    }

    {
        ZoneLayout layout{"red_safe_zone", ZoneLayout::LEFT, ZoneLayout::RIGHT, 0.0f};
        assert(isCorrectSubzone(layout, "ordinary_supply", ZoneLayout::LEFT));
        assert(!isCorrectSubzone(layout, "dangerous_object", ZoneLayout::LEFT));
    }

    {
        using Bytes = std::vector<uint8_t>;
        // 金向量：15字节帧（夹爪动作编号 + int16相机pitch + CRC16/Modbus覆盖0..12，低字节在前）。
        assert((UARTController::buildMotionPacket(MotionCommand{}, 0) ==
                Bytes{0x56,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCA,0x7D}));
        for (const auto requested : {-32768, -21, -20, -1, 0, 1, 20, 21, 32767}) {
            MotionCommand signed_frame;
            signed_frame.gripper_offset = requested;
            if(requested!=0 && requested!=20){bool rejected=false;try{UARTController::buildMotionPacket(signed_frame,7);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);continue;}
            const auto bytes=UARTController::buildMotionPacket(signed_frame,7);
            assert(bytes.size()==15&&bytes[9]==requested&&bytes[10]==7);
        }
        MotionCommand opened;
        opened.gripper_offset = 20;
        assert((UARTController::buildMotionPacket(opened, 1) ==
                Bytes{0x56,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x14,0x01,0x00,0x00,0x9E,0x4D}));
        MotionCommand tilted;
        tilted.camera_pitch_cdeg = 3000; // 向下30°
        assert((UARTController::buildMotionPacket(tilted, 1) ==
                Bytes{0x56,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x1E,0x00,0x92,0x1D}));
        tilted.camera_pitch_cdeg = 12000; // 超过±90°限幅
        assert((UARTController::buildMotionPacket(tilted, 1) ==
                Bytes{0x56,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x28,0x00,0x85,0xBD}));
        tilted.camera_pitch_cdeg = -12000;
        assert((UARTController::buildMotionPacket(tilted, 1) ==
                Bytes{0x56,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xD8,0xFF,0x81,0xFD}));

        MotionCommand command;
        command.vx_mps = 0.1f;
        command.wz_rps = -0.25f;
        auto packet = UARTController::buildMotionPacket(command, 1);
        assert((packet == Bytes{0x56,0xCD,0xCC,0xCC,0x3D,0x00,0x00,0x80,0xBE,0x00,0x01,0x00,0x00,0x48,0x35}));
        command.gripper_offset = 20; // only explicit CLOSE accepted
        command.camera_pitch_cdeg = 4550;
        packet = UARTController::buildMotionPacket(command, 2);
        assert((packet == Bytes{0x56,0xCD,0xCC,0xCC,0x3D,0x00,0x00,0x80,0xBE,0x14,0x02,0x28,0x00,0xA3,0xC5}));
        assert(packet.size() == MotionPacket::kSize);
        const auto ids = UARTController::buildMotionPacket(command, 255);
        assert(ids[10] == 0xFF);
        command.gripper_offset = 20;

        // 编号1..255循环，跳过0
        assert(UARTController::nextGripperActionId(0) == 1);
        assert(UARTController::nextGripperActionId(1) == 2);
        assert(UARTController::nextGripperActionId(255) == 1);

        // CRC自洽，且任意单bit翻转都会被检出
        auto crcOk = [](const Bytes &b) {
            const uint16_t crc = UARTController::calculateCRC16(b.data(), 0, 12);
            return b[13] == (crc & 0xFF) && b[14] == (crc >> 8);
        };
        assert(crcOk(packet));
        for (size_t byte = 0; byte < 13; ++byte) {
            for (int bit = 0; bit < 8; ++bit) {
                Bytes corrupted = packet;
                corrupted[byte] ^= static_cast<uint8_t>(1u << bit);
                assert(!crcOk(corrupted));
            }
        }

        command.vx_mps = std::numeric_limits<float>::infinity();
        packet = UARTController::buildMotionPacket(command, 1);
        for (int i = 1; i <= 8; ++i) assert(packet[i] == 0);
        assert(packet[11] == 40 && packet[12] == 0 && crcOk(packet)); // 速度非法不影响相机pitch
        command.vx_mps = 0.1f;
        command.wz_rps = std::numeric_limits<float>::quiet_NaN();
        packet = UARTController::buildMotionPacket(command, 1);
        for (int i = 1; i <= 8; ++i) assert(packet[i] == 0);
        assert(crcOk(packet));
        // 线速度硬限幅±1.0m/s：-1.25与-1.0打包结果相同。
        command.vx_mps = -1.25f; command.wz_rps = 0; command.gripper_offset = 0; command.camera_pitch_cdeg = 0;
        packet = UARTController::buildMotionPacket(command, 1);
        assert((packet == Bytes{0x56,0x00,0x00,0x80,0xBF,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0xE8,0x62}));
        command.vx_mps = -kMaxLinearSpeedMps;
        assert(UARTController::buildMotionPacket(command, 1) == packet);
        command.vx_mps = 5.0f;
        packet = UARTController::buildMotionPacket(command, 1);
        assert((packet == Bytes{0x56,0x00,0x00,0x80,0x3F,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x89,0xA4}));
        command.vx_mps = 0.15f;
        packet = UARTController::buildMotionPacket(command, 1);
        float unclamped = 0; std::memcpy(&unclamped, packet.data() + 1, 4);
        assert(unclamped == 0.15f);                                   // 限幅内原值透传
        command.vx_mps = 0.1f; command.wz_rps = -0.25f; command.camera_pitch_cdeg = -1500; // 负值向上
        packet = UARTController::buildMotionPacket(command, 3);
        assert((packet == Bytes{0x56,0xCD,0xCC,0xCC,0x3D,0x00,0x00,0x80,0xBE,0x00,0x03,0xF1,0xFF,0xEC,0x25}));
        command.header = 0x55;
        bool rejected = false;
        try { UARTController::buildMotionPacket(command, 1); }
        catch (const std::invalid_argument &) { rejected = true; }
        assert(rejected);
    }

    {
        SensorFusion fusion({200, 0.18f, 12.0f});
        SensorState sensor;
        sensor.timestamp_us = 1000000;
        sensor.imu_valid = true;
        sensor.tof_valid = {true, true, true, true};
        sensor.tof_fl_m = 0.1f;
        fusion.update(sensor);
        MotionCommand requested;

        requested.vx_mps = 0.1f;
        auto protected_command = fusion.protect(requested, 1000100);
        assert(protected_command.vx_mps == 0 && protected_command.wz_rps == 0);
    }

    {
        const auto path = std::string("/tmp/rescue-calibration-") + std::to_string(getpid()) + ".yaml";
        const cv::Mat k = (cv::Mat_<double>(3,3) << 500,0,320,0,500,240,0,0,1);
        const cv::Mat d = (cv::Mat_<double>(1,5) << .2,-.03,0,0,0);
        {
            cv::FileStorage file(path, cv::FileStorage::WRITE);
            file << "camera_matrix" << k << "dist_coeffs" << d;
            file << "ground_homography" << cv::Mat::eye(3,3,CV_64F);
            file << "ground_pixel_domain" << "undistorted_pixels";
        }
        CameraCalibration calibration;
        assert(!calibration.load(path));   // 没记录标定时相机pitch的单应不可用
        {
            cv::FileStorage file(path, cv::FileStorage::APPEND);
            file << "ground_camera_pitch_cdeg" << 2000;
        }
        assert(calibration.load(path));
        assert(calibration.groundMode() == CameraCalibration::GroundMode::FIXED_PITCH);
        assert(calibration.pitchUsable(2000) && !calibration.pitchUsable(1000));
        assert(std::abs(calibration.tiltLimitDeg() - 2.0f) < 1e-6f); // 旧imu_tilt_limit_deg不再放宽映射
        std::remove(path.c_str());
        std::vector<cv::Point2f> projected;
        cv::projectPoints(std::vector<cv::Point3f>{{.4f,.2f,1.f}},
            cv::Mat::zeros(3,1,CV_64F),cv::Mat::zeros(3,1,CV_64F),k,d,projected);
        cv::Point2f point;
        assert(calibration.pixelToGround(projected[0],2000,point));
        assert(cv::norm(point-cv::Point2f(520,340))<.01);
        assert(!calibration.pixelToGround(projected[0],0,point));
    }

    {
        // 输入是弧度。车体停车阈值12°（SensorFusion）与地面映射可信倾斜（固定单应2°）分开判断。
        SensorState sensor;
        sensor.timestamp_us = 1000000;
        sensor.imu_valid = true;
        SensorFusion fusion({200, 0.18f, 12.0f});
        CameraCalibration calibration;
        calibration.setGroundHomography(cv::Mat::eye(3, 3, CV_64F), 0);
        sensor.pitch_rad = 0.02f;
        fusion.update(sensor);
        assert(!fusion.tiltUnsafe() && calibration.tiltTrusted(sensor));
        sensor.pitch_rad = 0.1f;   // 5.7°：车可以走，但无补偿的单应已不可信
        fusion.update(sensor);
        assert(!fusion.tiltUnsafe() && !calibration.tiltTrusted(sensor));
        sensor.pitch_rad = 0.3f;
        fusion.update(sensor);
        assert(fusion.tiltUnsafe() && !calibration.tiltTrusted(sensor));
        sensor.pitch_rad = 0;
        sensor.roll_rad = -0.3f;
        fusion.update(sensor);
        assert(fusion.tiltUnsafe() && !calibration.tiltTrusted(sensor));
        calibration.setImuReference(0.0f, 0.1f);   // 以标定时静止姿态为零点
        sensor.roll_rad = 0; sensor.pitch_rad = 0.11f;
        assert(calibration.tiltTrusted(sensor));
        sensor.imu_valid = false;
        assert(!calibration.tiltTrusted(sensor));
    }

    {
        // 按读回pitch实时计算单应：相机高0.3m、参考pitch向下20°；用独立构造的真值相机检验其他pitch。
        const double pi = 3.14159265358979323846;
        const cv::Matx33d K(500, 0, 320, 0, 500, 240, 0, 0, 1);
        const cv::Mat d = cv::Mat::zeros(1, 5, CV_64F);
        const cv::Matx33d level(1, 0, 0, 0, 0, -1, 0, 1, 0); // 机器人(x右y前z上) → 平视相机(x右y下z前)
        auto rotX = [](double a) { return cv::Matx33d(1,0,0, 0,std::cos(a),-std::sin(a), 0,std::sin(a),std::cos(a)); };
        auto rotY = [](double a) { return cv::Matx33d(std::cos(a),0,std::sin(a), 0,1,0, -std::sin(a),0,std::cos(a)); };
        const cv::Vec3d center(0.0, 0.05, 0.30);
        auto extrinsic = [&](double pitch_deg) {
            const cv::Matx33d R = rotX(pitch_deg * pi / 180.0) * level;
            const cv::Vec3d t = -(R * center);
            return cv::Matx44d(R(0,0),R(0,1),R(0,2),t[0], R(1,0),R(1,1),R(1,2),t[1],
                               R(2,0),R(2,1),R(2,2),t[2], 0,0,0,1);
        };
        auto project = [&](const cv::Matx44d &T, const cv::Matx33d &body_from_ground, const cv::Point2f &g) {
            const cv::Vec3d robot = body_from_ground * cv::Vec3d(g.x, g.y, 0.0);
            const cv::Vec4d cam = T * cv::Vec4d(robot[0], robot[1], robot[2], 1.0);
            const cv::Vec3d px = K * cv::Vec3d(cam[0], cam[1], cam[2]);
            return cv::Point2f(static_cast<float>(px[0] / px[2]), static_cast<float>(px[1] / px[2]));
        };
        CameraCalibration calibration;
        calibration.setIntrinsics(cv::Mat(K), d);
        calibration.setPitchModel(cv::Mat(extrinsic(20.0)), 2000, 1000, 2500);
        assert(calibration.valid() && calibration.groundMode() == CameraCalibration::GroundMode::PITCH_MODEL);
        assert(std::abs(calibration.tiltLimitDeg() - 8.0f) < 1e-6f);
        const cv::Point2f target(0.12f, 0.70f);
        cv::Point2f ground;
        for (const int cdeg : {1000, 1500, 2000, 2500}) {
            const cv::Point2f pixel = project(extrinsic(cdeg / 100.0), cv::Matx33d::eye(), target);
            assert(calibration.pixelToGround(pixel, static_cast<int16_t>(cdeg), ground));
            assert(cv::norm(ground - target) < 1e-4);
        }
        // 舵机已到15°但仍按20°的单应算，误差明显——这正是固定单应不能跨pitch用的原因。
        const cv::Point2f at15 = project(extrinsic(15.0), cv::Matx33d::eye(), target);
        assert(calibration.pixelToGround(at15, 2000, ground) && cv::norm(ground - target) > 0.05);
        assert(!calibration.pixelToGround(at15, 900, ground));                  // 超出验证范围
        assert(!calibration.pixelToGround(at15, kCameraPitchInvalid, ground));
        assert(!calibration.pixelToGround({320.0f, 0.0f}, 2000, ground));       // 地平线以上

        // IMU补偿：车头下俯3°、右倾2°（REP-103均为正）。
        SensorState sensor;
        sensor.imu_valid = true;
        sensor.pitch_rad = static_cast<float>(3.0 * pi / 180.0);
        sensor.roll_rad = static_cast<float>(2.0 * pi / 180.0);
        const cv::Matx33d ground_from_body = rotX(-sensor.pitch_rad) * rotY(sensor.roll_rad);
        const cv::Point2f tilted = project(extrinsic(20.0), ground_from_body.t(), target);
        assert(calibration.pixelToGround(tilted, 2000, ground, &sensor) && cv::norm(ground - target) < 1e-3);
        SensorState level_sensor;
        level_sensor.imu_valid = true;
        assert(calibration.pixelToGround(tilted, 2000, ground, &level_sensor) && cv::norm(ground - target) > 0.03);
        sensor.pitch_rad = static_cast<float>(9.0 * pi / 180.0);                 // 超出8°补偿范围
        assert(!calibration.pixelToGround(tilted, 2000, ground, &sensor));

        // 从标定文件加载：必须是已验收外参并写明参考pitch。
        const auto path = std::string("/tmp/rescue-pitch-model-") + std::to_string(getpid()) + ".yaml";
        {
            cv::FileStorage file(path, cv::FileStorage::WRITE);
            file << "camera_matrix" << cv::Mat(K) << "dist_coeffs" << d;
            file << "ground_pixel_domain" << "undistorted_pixels";
            file << "T_camera_from_robot" << cv::Mat(extrinsic(20.0)) << "extrinsics_validated" << 1;
            file << "pitch_model_reference_cdeg" << 2000 << "pitch_model_min_cdeg" << 1000
                 << "pitch_model_max_cdeg" << 2500;
        }
        CameraCalibration loaded;
        assert(loaded.load(path) && loaded.groundMode() == CameraCalibration::GroundMode::PITCH_MODEL);
        const cv::Point2f pixel = project(extrinsic(12.0), cv::Matx33d::eye(), target);
        assert(loaded.pixelToGround(pixel, 1200, ground) && cv::norm(ground - target) < 1e-4);
        {
            cv::FileStorage file(path, cv::FileStorage::WRITE);
            file << "camera_matrix" << cv::Mat(K) << "dist_coeffs" << d;
            file << "T_camera_from_robot" << cv::Mat(extrinsic(20.0)) << "extrinsics_validated" << 0;
            file << "pitch_model_reference_cdeg" << 2000;
        }
        assert(!loaded.load(path));   // 未验收外参且无固定单应：拒绝
        std::remove(path.c_str());
    }

    {
        // IMU快照 → SensorState：取车体坐标姿态，任一有效条件缺失即imu_valid=false。
        ImuSnapshot imu;
        imu.connected = imu.fresh = true;
        imu.sample.measurements_valid = true;
        imu.sample.received_us = 123456;
        imu.sample.rpy_rad = {3.1f, -0.2f, 0.4f};        // 设备坐标，不得被使用
        imu.sample.body_rpy_rad = {0.01f, 0.02f, 0.4f};
        SensorState state = sensorStateFromImu(imu);
        assert(state.imu_valid && state.timestamp_us == 123456);
        assert(state.roll_rad == 0.01f && state.pitch_rad == 0.02f && state.yaw_rad == 0.4f);
        assert(!state.encoder_valid && !state.tof_valid[0] && !state.tof_valid[3]);
        imu.fresh = false;
        assert(!sensorStateFromImu(imu).imu_valid);
        imu.fresh = true; imu.connected = false;
        assert(!sensorStateFromImu(imu).imu_valid);
        imu.connected = true; imu.sample.measurements_valid = false;
        assert(!sensorStateFromImu(imu).imu_valid);
        imu.sample.measurements_valid = true;
        imu.sample.body_rpy_rad[1] = std::numeric_limits<float>::quiet_NaN();
        state = sensorStateFromImu(imu);
        assert(!state.imu_valid && state.pitch_rad == 0.0f);
        assert(!sensorStateFromImu(ImuSnapshot{}).imu_valid);

        // 接入安全链：新鲜且水平时不停车；倾斜超过12°停车。
        imu.sample.body_rpy_rad = {0.0f, 0.05f, 0.0f};
        SensorFusion fusion({200, 0.18f, 12.0f});
        fusion.update(sensorStateFromImu(imu));
        assert(!fusion.emergencyStop(123456 + 1000));
        assert(fusion.emergencyStop(123456 + 300000));  // 过期
        imu.sample.received_us = 200000;
        imu.sample.body_rpy_rad = {0.0f, 0.3f, 0.0f};
        fusion.update(sensorStateFromImu(imu));
        assert(fusion.emergencyStop(200000 + 1000));
    }

    std::cout << "rescue_core_tests: all checks passed\n";
    return 0;
}
