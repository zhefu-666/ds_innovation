// Based on the Foxglove SDK C++ ws-server and rgb-camera-visualization examples.
// This process is intentionally read-only: no client publish, services, parameters,
// or playback controls are enabled.

#include <foxglove/channel.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::function<void()> g_stop;

foxglove::RawChannel create_json_channel(const std::string& topic, const std::string& schema) {
  foxglove::Schema channel_schema;
  channel_schema.name = "rescue." + topic.substr(1);
  channel_schema.encoding = "jsonschema";
  channel_schema.data = reinterpret_cast<const std::byte*>(schema.data());
  channel_schema.data_len = schema.size();
  auto result = foxglove::RawChannel::create(topic, "json", std::move(channel_schema));
  if (!result.has_value()) {
    throw std::runtime_error("failed to create channel " + topic + ": " + foxglove::strerror(result.error()));
  }
  return std::move(result.value());
}

void publish(foxglove::RawChannel& channel, const std::string& message, uint64_t timestamp) {
  channel.log(reinterpret_cast<const std::byte*>(message.data()), message.size(), timestamp);
}

foxglove::messages::Timestamp split_timestamp(uint64_t timestamp) {
  return foxglove::messages::Timestamp{
    static_cast<uint32_t>(timestamp / 1'000'000'000),
    static_cast<uint32_t>(timestamp % 1'000'000'000),
  };
}
}  // namespace

int main() {
  std::signal(SIGINT, [](int) {
    if (g_stop) g_stop();
  });
  foxglove::setLogLevel(foxglove::LogLevel::Info);

  foxglove::WebSocketServerOptions options = {};
  options.name = "rescue-telemetry";
  options.host = "0.0.0.0";
  options.port = 8765;
  options.capabilities = foxglove::WebSocketServerCapabilities::None;
  options.supported_encodings = {"json"};
  auto server_result = foxglove::WebSocketServer::create(std::move(options));
  if (!server_result.has_value()) {
    std::cerr << "Failed to create Foxglove server: " << foxglove::strerror(server_result.error()) << '\n';
    return 1;
  }
  auto server = std::move(server_result.value());

  const std::string schema = R"({"type":"object","additionalProperties":true})";
  auto image_result = foxglove::messages::CompressedImageChannel::create("/camera/image");
  if (!image_result.has_value()) {
    std::cerr << "Failed to create image channel: " << foxglove::strerror(image_result.error()) << '\n';
    return 1;
  }
  auto image = std::move(image_result.value());
  auto pose_result = foxglove::messages::PoseInFrameChannel::create("/safezone/pose");
  if (!pose_result.has_value()) {
    std::cerr << "Failed to create pose channel: " << foxglove::strerror(pose_result.error()) << '\n';
    return 1;
  }
  auto pose = std::move(pose_result.value());
  auto state = create_json_channel("/fsm/state", schema);
  auto health = create_json_channel("/system/health", schema);
  auto detections = create_json_channel("/detections", schema);

  std::atomic_bool done{false};
  g_stop = [&] {
    done = true;
    server.stop();
  };
  std::cerr << "Foxglove server listening on ws://0.0.0.0:" << server.port() << '\n';

  uint64_t sequence = 0;
  while (!done) {
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto message_timestamp = split_timestamp(static_cast<uint64_t>(timestamp));
    foxglove::messages::CompressedImage image_message;
    image_message.timestamp = message_timestamp;
    image_message.frame_id = "camera";
    image_message.format = "jpeg";
    // A deterministic placeholder keeps this publisher runnable without a camera.
    constexpr std::array<unsigned char, 69> jpeg = {
      0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00,
      0x00, 0x01, 0x00, 0x01, 0x00, 0xff, 0xdb, 0x00, 0x43, 0x00, 0x08, 0x06, 0x06, 0x07,
      0x06, 0x05, 0x08, 0x07, 0x07, 0x07, 0x09, 0x09, 0x08, 0x0a, 0x0c, 0x14, 0x0d, 0x0c,
      0x0b, 0x0b, 0x0c, 0x19, 0x12, 0x13, 0x0f, 0xff, 0xd9, 0xff, 0xd8, 0xff, 0xd9, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    image_message.data.reserve(jpeg.size());
    for (const auto byte : jpeg) image_message.data.push_back(static_cast<std::byte>(byte));
    image.log(image_message, static_cast<uint64_t>(timestamp));
    foxglove::messages::PoseInFrame pose_message;
    pose_message.timestamp = message_timestamp;
    pose_message.frame_id = "map";
    pose_message.pose = foxglove::messages::Pose{
      foxglove::messages::Vector3{0.0, 0.0, 0.0},
      foxglove::messages::Quaternion{0.0, 0.0, 0.0, 1.0},
    };
    pose.log(pose_message, static_cast<uint64_t>(timestamp));
    publish(state, R"({"name":"SEARCH","elapsed_s":0.1})", timestamp);
    publish(health, "{\"sequence\":" + std::to_string(sequence) + ",\"telemetry_hz\":10,\"control_channel\":\"disabled\"}", timestamp);
    publish(detections, R"({"items":[],"frame_id":"camera"})", timestamp);
    ++sequence;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
