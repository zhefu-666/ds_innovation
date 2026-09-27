#include "rescue/camera_calibration.hpp"
#include "rescue/planner.hpp"
#include "rescue/rescue_state_machine.hpp"
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
        CameraCalibration calibration;
        calibration.setGroundHomography(cv::Mat::eye(3, 3, CV_64F));
        calibration.setIntrinsics(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 5, CV_64F));
        SensorState sensor;
        sensor.imu_valid = true;
        cv::Point2f ground;
        assert(calibration.pixelToGround({0.25f, 0.5f}, ground, &sensor));
        assert(std::abs(ground.x - 0.25f) < 1e-5f);
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
                Bytes{0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xCA, 0x7D}));
        MotionCommand opened;
        opened.gripper_open = 1;
        assert((UARTController::buildMotionPacket(opened, 1) ==
                Bytes{0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x01, 0x00, 0x00, 0x9A, 0x41}));
        MotionCommand tilted;
        tilted.camera_pitch_cdeg = 3000; // 向下30°
        assert((UARTController::buildMotionPacket(tilted, 1) ==
                Bytes{0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x01, 0xB8, 0x0B, 0xA8, 0x7A}));
        tilted.camera_pitch_cdeg = 12000; // 超过±90°限幅
        assert((UARTController::buildMotionPacket(tilted, 1) ==
                Bytes{0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x01, 0x28, 0x23, 0xC4, 0x64}));
        tilted.camera_pitch_cdeg = -12000;
        assert((UARTController::buildMotionPacket(tilted, 1) ==
                Bytes{0x56, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x01, 0xD8, 0xDC, 0xC0, 0x24}));

        MotionCommand command;
        command.vx_mps = 0.1f;
        command.wz_rps = -0.25f;
        auto packet = UARTController::buildMotionPacket(command, 1);
        assert((packet == Bytes{0x56, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x80, 0xBE,
                                0x00, 0x01, 0x00, 0x00, 0x48, 0x35}));
        command.gripper_open = 7; // 非0值统一归一为1
        command.camera_pitch_cdeg = 4550;
        packet = UARTController::buildMotionPacket(command, 2);
        assert((packet == Bytes{0x56, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x80, 0xBE,
                                0x01, 0x02, 0xC6, 0x11, 0x2A, 0x65}));
        assert(packet.size() == MotionPacket::kSize);
        const auto ids = UARTController::buildMotionPacket(command, 255);
        assert(ids[10] == 0xFF);
        command.gripper_open = 1;

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
        assert(packet[11] == 0xC6 && packet[12] == 0x11 && crcOk(packet)); // 速度非法不影响相机pitch
        command.vx_mps = 0.1f;
        command.wz_rps = std::numeric_limits<float>::quiet_NaN();
        packet = UARTController::buildMotionPacket(command, 1);
        for (int i = 1; i <= 8; ++i) assert(packet[i] == 0);
        assert(crcOk(packet));
        command.vx_mps = -0.25f; command.wz_rps = 0; command.gripper_open = 0; command.camera_pitch_cdeg = 0;
        packet = UARTController::buildMotionPacket(command, 1);
        assert((packet == Bytes{0x56, 0x00, 0x00, 0x80, 0xBE, 0, 0, 0, 0, 0, 0x01, 0x00, 0x00, 0xE5, 0xF2}));
        command.vx_mps = 0.1f; command.wz_rps = -0.25f; command.camera_pitch_cdeg = -1500; // 负值向上
        packet = UARTController::buildMotionPacket(command, 3);
        assert((packet == Bytes{0x56, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x80, 0xBE, 0, 0x03, 0x24, 0xFA, 0x72, 0xB6}));
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
        RescueStateMachine machine({200, 0.18f, 12.0f});
        RescueInputs input;
        input.now_us = 1000000;
        input.robot_disconnected = false;
        input.start_requested = true;
        input.sensors.timestamp_us = input.now_us;
        input.sensors.imu_valid = true;
        machine.update(input);
        assert(machine.state() == RescueState::DEPART);
        input.start_requested = false;
        machine.update(input);
        assert(machine.state() == RescueState::SEARCH_TARGET);
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
        assert(calibration.load(path));
        std::remove(path.c_str());
        std::vector<cv::Point2f> projected;
        cv::projectPoints(std::vector<cv::Point3f>{{.4f,.2f,1.f}},
            cv::Mat::zeros(3,1,CV_64F),cv::Mat::zeros(3,1,CV_64F),k,d,projected);
        cv::Point2f point;
        assert(calibration.pixelToGround(projected[0],point));
        assert(cv::norm(point-cv::Point2f(520,340))<.01);
    }

    {
        // 输入是弧度，原有12度配置必须先转换；0.1rad安全，0.3rad超限。
        SensorState sensor;
        sensor.timestamp_us = 1000000;
        sensor.imu_valid = true;
        SensorFusion fusion({200, 0.18f, 12.0f});
        CameraCalibration calibration;
        sensor.pitch_rad = 0.1f;
        fusion.update(sensor);
        assert(!fusion.tiltUnsafe() && calibration.tiltTrusted(sensor));
        sensor.pitch_rad = 0.3f;
        fusion.update(sensor);
        assert(fusion.tiltUnsafe() && !calibration.tiltTrusted(sensor));
        sensor.pitch_rad = 0;
        sensor.roll_rad = -0.3f;
        fusion.update(sensor);
        assert(fusion.tiltUnsafe() && !calibration.tiltTrusted(sensor));
    }

    std::cout << "rescue_core_tests: all checks passed\n";
    return 0;
}
