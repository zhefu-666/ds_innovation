#pragma once
#include "rescue/core/types.hpp"
#include <array>
#include <vector>

namespace rescue {
struct FloorHsvRange {
    cv::Vec3i low, high;
};
struct TransportRegionConfig {
    float min_width_fraction = .65f, min_height_fraction = .55f, min_bottom_fraction = .95f;
    // Near-view floor colours exclude the purple wall and dark middle divider.
    std::vector<FloorHsvRange> blue{{{90, 65, 65}, {115, 255, 255}}};
    std::vector<FloorHsvRange> red{{{0, 80, 65}, {10, 255, 255}},
                                 {{170, 80, 65}, {179, 255, 255}}};
};
struct PushboardFloorConfig {
    TransportRegionConfig transport_region;
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
    float own_fraction = 0; // Diagnostic only; never the transport release condition.
    bool region_valid = false, inside = false;
    cv::Rect region_box;
    std::vector<cv::Point> region_contour;
    float width_fraction = 0, height_fraction = 0, bottom_fraction = 0;
};
TransportFloorEvidence observeTransportFloor(
    const cv::Mat& frame, const std::vector<SegDetection>& detections,
    const PushboardFloorConfig& config, const std::string& team,
    bool stable_ui120, uint64_t capture_us);

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
