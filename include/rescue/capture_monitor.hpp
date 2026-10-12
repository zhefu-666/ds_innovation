#pragma once
#include "rescue/config.hpp"
#include "rescue/push_task.hpp"
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace rescue {
// Image/ground evidence for the gripper frame. Pixel regions are for 1280x720 and are
// placeholders scaled from the old 640x480 values: recalibrate on the robot with the
// gripper closed around 1, 2 and 3 blocks, at every camera pitch listed.
struct HoldingView {
    int16_t pitch_cdeg;   // servo readback this region was calibrated at
    RectArea area;        // inside the closed frame, image px
    float min_visible_bottom_y_px = 0; // object must reach this depth, even if clipped by image bottom
    std::vector<cv::Point2f> polygon;
};
struct CaptureConfig {
    // No uncalibrated angle is observable by default.
    std::vector<HoldingView> holding;
    int16_t pitch_tolerance_cdeg = 100;
    float edge_margin_px = 6.f;        // a box crossing the region edge is ambiguous, not held
    float follow_tolerance_px = 12.f;  // held objects move with the robot: near-static in image
    int follow_frames = 3;
    // A box with a measured (untruncated) ground contact this far beyond the mouth lies in front
    // of the jaw, not in it: it is neither held nor ambiguous. Disabled until calibrated.
    float mouth_y_m = std::numeric_limits<float>::infinity(), outside_margin_m = .03f;
    uint64_t max_age_us = 200000;
    float min_confidence = .25f;       // below this, a box counts as an unknown object
    float noise_confidence = .10f;     // ignored entirely below this
    // Rush corridor in body frame: swept width of the open frame plus margin.
    float corridor_half_width_m = .11f, corridor_extra_m = .22f; // field 2026-10-11: a dangerous block 14.6 cm behind the target was enclosed; covers block depth + margin
    // A neighbour counts as in the corridor when any part of its nominal footprint reaches it, not only its centre point
    // (field 2026-10-12: dangerous 9.5 cm and injured 9.8 cm off-axis were missed by 5 mm against a 9 cm half-width).
    // Half-extents are nominal model sizes (40 mm cubes; injured 80x40 lying), not measured; 0 restores centre-point only.
    float footprint_ordinary_m = .02f, footprint_dangerous_m = .02f, footprint_core_m = .03f, footprint_injured_m = .04f, footprint_unknown_m = .03f;
    float occlusion_side_margin_m = .05f; // overlap with a box this far outside the corridor is not occlusion
    float occlusion_iou = .05f;        // overlapping boxes in the corridor may hide an object
    int image_height_px = 720;
};
// Fills held/captured and corridor fields from tracked detections of one frame.
// Every rule-relevant uncertainty makes the evidence incomplete instead of guessing.
// Reads camera_pitch_cdeg/camera_pitch_stable: a moving camera gives no evidence, and the
// holding region is observable only at a calibrated pitch (hold_observable).
// Track ids behind the last holding decision (-1: untracked box), for field logs.
struct HoldDiagnostics {
    std::vector<int> ambiguous, outside, unfollowed;
};
// One line per fresh neighbour of the last corridor evaluation, for event logs (not every frame).
struct CorridorEntry {
    int track_id = -1;
    std::string label;
    float confidence = 0, along_m = 0, across_m = 0, footprint_m = 0;
    uint64_t age_us = 0;
    bool counted = false, footprint_only = false; // footprint_only: centre outside, counted only through its footprint
    const char *reason = ""; // counted / no_ground_position / behind / beyond_reach / outside_lateral
};
class CaptureMonitor {
public:
    explicit CaptureMonitor(CaptureConfig config = {}) : c_(config) {}
    void update(PushObservation &in, const std::vector<SegDetection> &detections, uint64_t now_us);
    void reset() { history_.clear(); view_ = -1; }
    // Holding region for a stable readback, or nullptr when the frame is out of view.
    const HoldingView *view(int16_t pitch_cdeg, bool stable) const;
    const HoldDiagnostics &diagnostics() const { return diag_; }
    const std::vector<CorridorEntry> &corridorEntries() const { return corridor_diag_; }
    float footprintHalf(const std::string &label) const;
private:
    CaptureConfig c_;
    std::map<int, std::vector<cv::Point2f>> history_;
    HoldDiagnostics diag_;
    std::vector<CorridorEntry> corridor_diag_;
    int view_ = -1; // table entry the follow history was collected at
};
} // namespace rescue
