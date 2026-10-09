#include "rescue/uart_controller.hpp"
#include "rescue/utils.hpp"
#include <csignal>
#include <iostream>
#include <stdexcept>

// 仅监视8字节A6执行器反馈，不发送运动命令。
int main(int argc, char **argv) {
    using namespace rescue;
    try {
        if (argc > 4) throw std::runtime_error("Usage: rescue_sensor_monitor [port] [baud] [seconds]");
        const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
        const int baud = argc > 2 ? std::stoi(argv[2]) : 115200;
        const int seconds = argc > 3 ? std::stoi(argv[3]) : 30;
        if (seconds <= 0) throw std::runtime_error("seconds must be positive");
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
        std::cout << "A6 actuator feedback monitor (8 bytes); IMU separate, ToF disabled.\n";
        UARTController uart;
        uart.initFeedbackOnly(port, baud);
        const auto end = Clock::now() + std::chrono::seconds(seconds);
        uint64_t last = 0;
        while (!g_should_exit.load() && Clock::now() < end) {
            auto s = uart.latestActuatorFeedback();
            if (s.timestamp_us != 0 && s.timestamp_us != last) {
                last = s.timestamp_us;
                std::cout << "rx_us=" << last << " feedback_valid=" << s.valid
                    << " gripper_open=" << static_cast<unsigned>(s.gripper_open)
                    << " gripper_action_id=" << static_cast<unsigned>(s.gripper_action_id)
                    << " camera_protocol_deg=" << (s.camera_pitch_cdeg == kCameraPitchInvalid ? kCameraPitchInvalid : cameraPitchToFeedbackDeg(s.camera_pitch_cdeg))
                    << " calibration_pitch_cdeg=" << s.camera_pitch_cdeg
                    << std::endl;
            }
            std::this_thread::sleep_for(Ms(10));
        }
        if (!last) { std::cerr << "No valid actuator feedback frames received\n"; return 2; }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
