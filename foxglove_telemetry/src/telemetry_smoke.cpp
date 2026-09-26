#include <array>
#include <iostream>
#include <string_view>

int main() {
  constexpr std::array<std::string_view, 9> topics = {
    "/camera/image", "/detections", "/fsm/state", "/fsm/events", "/sensors/imu",
    "/sensors/tof", "/safezone/pose", "/camera/calibration", "/system/health",
  };
  std::cout << "read-only Foxglove telemetry contract\n";
  std::cout << "client publish: disabled\n";
  std::cout << "channels: " << topics.size() << "\n";
  for (const auto topic : topics) {
    std::cout << "  " << topic << "\n";
  }
  return topics.size() == 9 ? 0 : 1;
}
