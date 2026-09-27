#include "rescue/hipnuc_imu.hpp"
#include "rescue/utils.hpp"
#include <csignal>
#include <iostream>
#include <iomanip>
#include <stdexcept>
int main(int argc, char** argv) {
    using namespace rescue;
    try {
        if (argc > 4) throw std::runtime_error("Usage: rescue_imu_monitor [port] [baud] [seconds]");
        const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
        const int baud = argc > 2 ? std::stoi(argv[2]) : 115200;
        const int seconds = argc > 3 ? std::stoi(argv[3]) : 30;
        if (seconds <= 0) throw std::runtime_error("seconds must be positive");
        std::signal(SIGINT, signalHandler); std::signal(SIGTERM, signalHandler);
        HipnucImu imu(port, baud);
        std::cout << "HiPNUC HI91 monitor: " << port << " @ " << baud
                  << "; body_* = base_link FLU after mounting (roll 180 deg), raw_* = device axes; SI units; no serial writes\n";
        auto end = Clock::now() + std::chrono::seconds(seconds);
        while (!g_should_exit.load() && Clock::now() < end) {
            auto s = imu.snapshot(); const auto& p = s.sample;
            std::cout << std::fixed << std::setprecision(4) << "seq=" << p.sequence << " device_ms=" << p.device_time_ms
                << " fresh=" << s.fresh << " numeric_ok=" << p.measurements_valid << " age_ms=" << s.age_ms
                << " status=0x" << std::hex << p.status << std::dec
                << " body_rpy_rad=" << p.body_rpy_rad[0] << ',' << p.body_rpy_rad[1] << ',' << p.body_rpy_rad[2]
                << " body_gyro_rad_s=" << p.body_angular_velocity_rps[0] << ',' << p.body_angular_velocity_rps[1] << ',' << p.body_angular_velocity_rps[2]
                << " body_acc_m_s2=" << p.body_acceleration_mps2[0] << ',' << p.body_acceleration_mps2[1] << ',' << p.body_acceleration_mps2[2]
                << " raw_rpy_rad=" << p.rpy_rad[0] << ',' << p.rpy_rad[1] << ',' << p.rpy_rad[2]
                << " crc_errors=" << s.crc_errors << std::endl;
            std::this_thread::sleep_for(Ms(500));
        }
        const auto s = imu.snapshot();
        std::cout << "frames=" << s.valid_frames << " bytes=" << s.bytes << " crc_errors=" << s.crc_errors
            << " invalid_frames=" << s.invalid_frames << " duplicate_times=" << s.duplicate_times
            << " backward_times=" << s.backward_times << " io_errors=" << s.io_errors << '\n';
        return s.fresh && s.sample.measurements_valid ? 0 : 2;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
