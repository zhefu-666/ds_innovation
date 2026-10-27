#include "rescue/hardware/hipnuc_imu.hpp"
#include <nlohmann/json.hpp>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) {
    stopped = 1;
}
} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: rescue_imu_record [port] [baud] [seconds]\n"
                     "Read-only HI91 JSONL recorder; raw imu_device axes, SI units; "
                     "received_us=CLOCK_MONOTONIC, device_time_ms=IMU clock. No serial writes.\n";
        return 0;
    }
    try {
        if (argc > 4)
            throw std::runtime_error("Too many arguments; use --help");
        const int seconds = argc > 3 ? std::stoi(argv[3]) : 3600;
        if (seconds <= 0)
            throw std::runtime_error("seconds must be positive");
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        rescue::HipnucImu imu(argc > 1 ? argv[1] : "/dev/ttyUSB0",
                              argc > 2 ? std::stoi(argv[2]) : 115200);
        const auto begin = rescue::imuNowUs();
        uint64_t sequence = 0;
        while (!stopped && rescue::imuNowUs() - begin < uint64_t(seconds) * 1000000) {
            for (const auto& s : imu.samplesAfter(sequence)) {
                nlohmann::json row = {{"received_us", s.received_us},
                                      {"device_time_ms", s.device_time_ms},
                                      {"sequence", s.sequence},
                                      {"measurements_valid", s.measurements_valid},
                                      {"status", s.status},
                                      {"gyro_rad_s", s.angular_velocity_rps},
                                      {"accel_m_s2", s.acceleration_mps2},
                                      {"quaternion_wxyz", s.quaternion_wxyz}};
                std::cout << row.dump() << std::endl;
                sequence = s.sequence;
            }
            const auto state = imu.snapshot();
            if (!state.connected || (rescue::imuNowUs() - begin > 2000000 && !state.fresh))
                throw std::runtime_error("IMU disconnected or no fresh HI91 samples");
            if (!std::cout)
                throw std::runtime_error("Recorder output closed");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return sequence ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
