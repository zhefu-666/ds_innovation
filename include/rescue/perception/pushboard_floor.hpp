#pragma once
#include "rescue/core/types.hpp"
#include <array>
#include <vector>

namespace rescue {
struct FloorHsvRange {
    cv::Vec3i low, high;
};
struct GraspCenterRange;
struct TransportSurroundingsConfig {
    // Pixels at the UI140 reference resolution; sample every side of each held box.
    int gap_px = 8, band_px = 24;
    float min_own = .50f, min_total_own = .80f;
};
struct PushboardFloorConfig {
    TransportSurroundingsConfig transport_surroundings;
    bool enabled = false;
    cv::Size image_size{1280, 720};
    int16_t pitch_cdeg = 2000;
    std::array<cv::Rect, 2> windows{{{410, 530, 75, 45}, {790, 530, 75, 45}}};
    float min_visible = .60f, min_own = .75f, min_ground = .60f;
    std::vector<FloorHsvRange> blue{{{90, 65, 40}, {125, 255, 255}}};
    std::vector<FloorHsvRange> red{{{0, 80, 40}, {10, 255, 255}},
                                    {{170, 80, 40}, {179, 255, 255}}};
    std::vector<FloorHsvRange> purple{{{126, 55, 35}, {165, 255, 255}}};
    std::vector<FloorHsvRange> white{{{0, 0, 65}, {179, 60, 255}}};
    // Foreground colours veto pixels, never supply floor evidence.
    std::vector<FloorHsvRange> cargo{{{35, 65, 30}, {85, 255, 255}},
                                    {{5, 90, 40}, {25, 255, 255}},
                                    {{0, 0, 0}, {179, 255, 35}}};
};
struct PushboardFloorWindow {
    cv::Rect roi;
    float visible = 0, own = 0, enemy = 0, white = 0, purple = 0;
    bool valid = false;
};
struct TransportFloorEvidence {
    uint64_t sample_us = 0;
    bool valid = false;
    float own_fraction = 0; // Visible-pixel-weighted diagnostic across all cargo; each cargo qualifies separately.
    bool region_valid = false, inside = false;
    int cargo_count = 0;
    // Four windows per cargo: front, rear, left and right in image coordinates.
    std::vector<PushboardFloorWindow> windows;
};
TransportFloorEvidence observeTransportFloor(
    const cv::Mat& frame, const std::vector<SegDetection>& detections,
    const PushboardFloorConfig& config, const std::string& team,
    bool stable_ui140, uint64_t capture_us,
    const std::vector<GraspCenterRange>& grasp_ranges, int expected_cargo_count);

struct PushboardFloorEvidence {
    uint64_t sample_us = 0;
    bool valid = false, inside = false;
    std::array<PushboardFloorWindow, 2> windows;
};
PushboardFloorEvidence observePushboardFloor(
    const cv::Mat& frame, const std::vector<SegDetection>& detections,
    const PushboardFloorConfig& config, const std::string& team,
    bool stable_ui140, uint64_t capture_us);
} // namespace rescue
