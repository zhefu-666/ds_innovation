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
        MotionCommand command;
        command.vx_mps = 0.1f;
        command.wz_rps = -0.25f;
        auto packet = UARTController::buildMotionPacket(command);
        assert((packet == std::vector<uint8_t>{0x56, 0xCD, 0xCC, 0xCC, 0x3D,
                                               0x00, 0x00, 0x80, 0xBE, 0}));
        command.gripper_closed = 1;
        packet = UARTController::buildMotionPacket(command);
        assert(packet.size() == 10 && packet[9] == 1);
        command.vx_mps = std::numeric_limits<float>::infinity();
        packet = UARTController::buildMotionPacket(command);
        for (int i = 1; i <= 8; ++i) assert(packet[i] == 0);
        command.vx_mps = 0.1f;
        command.wz_rps = std::numeric_limits<float>::quiet_NaN();
        packet = UARTController::buildMotionPacket(command);
        for (int i = 1; i <= 8; ++i) assert(packet[i] == 0);
        command.vx_mps = -0.25f; command.wz_rps = 0; command.gripper_closed = 0;
        packet = UARTController::buildMotionPacket(command);
        assert((packet == std::vector<uint8_t>{0x56, 0x00, 0x00, 0x80, 0xBE,
                                               0, 0, 0, 0, 0}));
        packet = UARTController::buildMotionPacket(MotionCommand{});
        assert(packet.size() == 10 && packet[0] == 0x56);
        for (int i = 1; i <= 9; ++i) assert(packet[i] == 0);
        command.header = 0x55;
        bool rejected = false;
        try { UARTController::buildMotionPacket(command); }
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
