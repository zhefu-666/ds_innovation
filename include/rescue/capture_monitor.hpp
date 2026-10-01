#pragma once
#include "rescue/config.hpp"
#include "rescue/push_task.hpp"
#include <map>
#include <vector>

namespace rescue {
// Image/ground evidence for the gripper frame. Pixel regions are for 1280x720 and are
// placeholders scaled from the old 640x480 values: recalibrate on the robot with the
// gripper closed around 1, 2 and 3 blocks, at every camera pitch listed.
struct HoldingView {
    int16_t pitch_cdeg;   // servo readback this region was calibrated at
    RectArea area;        // inside the closed frame, image px
};
struct CaptureConfig {
    // Only pitches that show the whole closed frame belong here (25 deg did not; NEAR = 35 deg).
    std::vector<HoldingView> holding{{3500, {540.f, 570.f, 820.f, 690.f}}};
    int16_t pitch_tolerance_cdeg = 100;
    float edge_margin_px = 6.f;        // a box crossing the region edge is ambiguous, not held
    float follow_tolerance_px = 12.f;  // held objects move with the robot: near-static in image
    int follow_frames = 3;
    uint64_t max_age_us = 200000;
    float min_confidence = .25f;       // below this, a box counts as an unknown object
    float noise_confidence = .10f;     // ignored entirely below this
    // Rush corridor in body frame: swept width of the open frame plus margin.
    float corridor_half_width_m = .11f, corridor_extra_m = .10f;
    float occlusion_iou = .05f;        // overlapping boxes in the corridor may hide an object
    int image_height_px = 720;
};
// Fills held/captured and corridor fields from tracked detections of one frame.
// Every rule-relevant uncertainty makes the evidence incomplete instead of guessing.
// Reads camera_pitch_cdeg/camera_pitch_stable: a moving camera gives no evidence, and the
// holding region is observable only at a calibrated pitch (hold_observable).
class CaptureMonitor {
public:
    explicit CaptureMonitor(CaptureConfig config = {}) : c_(config) {}
    void update(PushObservation &in, const std::vector<SegDetection> &detections, uint64_t now_us);
    void reset() { history_.clear(); view_ = -1; }
    // Holding region for a stable readback, or nullptr when the frame is out of view.
    const HoldingView *view(int16_t pitch_cdeg, bool stable) const;
private:
    CaptureConfig c_;
    std::map<int, std::vector<cv::Point2f>> history_;
    int view_ = -1; // table entry the follow history was collected at
};
} // namespace rescue
