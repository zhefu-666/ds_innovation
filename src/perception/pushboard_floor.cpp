#include "rescue/perception/pushboard_floor.hpp"
#include "rescue/perception/capture_monitor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace rescue {
namespace {
cv::Mat colorMask(const cv::Mat& hsv, const std::vector<FloorHsvRange>& ranges) {
    cv::Mat mask(hsv.size(), CV_8U, cv::Scalar(0));
    for (const auto& r : ranges) {
        cv::Mat part;
        cv::inRange(hsv, r.low, r.high, part);
        cv::bitwise_or(mask, part, mask);
    }
    return mask;
}
}
TransportFloorEvidence observeTransportFloor(
    const cv::Mat& frame, const std::vector<SegDetection>& detections,
    const PushboardFloorConfig& c, const std::string& team,
    bool stable_ui140, uint64_t capture_us,
    const std::vector<GraspCenterRange>& grasp_ranges, int expected_cargo_count) {
    TransportFloorEvidence out;
    if (!stable_ui140 || !capture_us || frame.empty() || frame.type() != CV_8UC3 ||
        (team != "blue" && team != "red"))
        return out;
    out.valid = true;
    out.sample_us = capture_us;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    const auto own = colorMask(hsv, team == "blue" ? c.blue : c.red);
    const auto enemy = colorMask(hsv, team == "blue" ? c.red : c.blue);
    const auto white = colorMask(hsv, c.white), purple = colorMask(hsv, c.purple);
    const cv::Rect bounds(0, 0, frame.cols, frame.rows);
    const float scale_x = float(frame.cols) / c.image_size.width;
    const float scale_y = float(frame.rows) / c.image_size.height;
    const auto& config = c.transport_surroundings;
    const int gap_x = std::max(0, cvRound(config.gap_px * scale_x));
    const int gap_y = std::max(0, cvRound(config.gap_px * scale_y));
    const int band_x = std::max(1, cvRound(config.band_px * scale_x));
    const int band_y = std::max(1, cvRound(config.band_px * scale_y));
    cv::Mat visible(frame.size(), CV_8U, cv::Scalar(255));
    for (const auto& d : detections) {
        if (d.label == "blue_safe_zone" || d.label == "red_safe_zone" ||
            !std::isfinite(d.confidence) || d.confidence < .2f || d.box.empty())
            continue;
        const cv::Rect expanded(d.box.x-gap_x, d.box.y-gap_y,
                                d.box.width+2*gap_x, d.box.height+2*gap_y);
        const auto box = expanded & bounds;
        if (!box.empty()) visible(box).setTo(0);
    }
    bool observable = true, inside = true;
    uint64_t total_own_pixels = 0, total_visible_pixels = 0;
    for (const auto& d : detections) {
        if (!std::isfinite(d.confidence) || d.confidence < .2f || d.box.empty())
            continue;
        // Membership uses the same measured UI140 class centre ranges as capture.
        const bool held = std::any_of(grasp_ranges.begin(), grasp_ranges.end(), [&](const auto& r) {
            if (r.label != d.label || r.pitch_cdeg != c.pitch_cdeg) return false;
            const float x = (d.box.x + .5f*d.box.width) * r.image_width_px / frame.cols;
            const float y = (d.box.y + .5f*d.box.height) * r.image_height_px / frame.rows;
            return x >= r.area.x1 && x <= r.area.x2 && y >= r.area.y1 && y <= r.area.y2;
        });
        if (!held) continue;
        ++out.cargo_count;
        const auto& b = d.box;
        const std::array<cv::Rect,4> windows{{
            {b.x-gap_x-band_x, b.y-gap_y-band_y, b.width+2*(gap_x+band_x), band_y},
            {b.x-gap_x-band_x, b.y+b.height+gap_y, b.width+2*(gap_x+band_x), band_y},
            {b.x-gap_x-band_x, b.y-gap_y, band_x, b.height+2*gap_y},
            {b.x+b.width+gap_x, b.y-gap_y, band_x, b.height+2*gap_y}
        }};
        uint64_t cargo_own_pixels = 0, cargo_visible_pixels = 0;
        for (const auto& roi : windows) {
            PushboardFloorWindow w;
            w.roi = roi;
            if (!roi.empty() && (roi & bounds) == roi) {
                const cv::Mat available = visible(roi);
                const int n = cv::countNonZero(available);
                w.visible = float(n) / roi.area();
                if (n && w.visible >= c.min_visible) {
                    const auto fraction = [&](const cv::Mat& mask) {
                        cv::Mat pixels;
                        cv::bitwise_and(mask(roi), available, pixels);
                        return float(cv::countNonZero(pixels)) / n;
                    };
                    cv::Mat own_pixels;
                    cv::bitwise_and(own(roi), available, own_pixels);
                    const int own_count = cv::countNonZero(own_pixels);
                    cargo_own_pixels += own_count;
                    cargo_visible_pixels += n;
                    w.own = float(own_count) / n; w.enemy = fraction(enemy);
                    w.white = fraction(white); w.purple = fraction(purple);
                    // Divider, white, purple and unknown pixels remain in the denominator.
                    w.valid = true;
                }
            }
            observable &= w.valid;
            inside &= w.valid && w.own >= config.min_own;
            out.windows.push_back(w);
        }
        // A strong neighbour cannot make up for this cargo still crossing the entrance.
        const float cargo_fraction = cargo_visible_pixels
            ? float(cargo_own_pixels) / cargo_visible_pixels : 0.f;
        inside &= cargo_fraction >= config.min_total_own;
        total_own_pixels += cargo_own_pixels;
        total_visible_pixels += cargo_visible_pixels;
    }
    out.region_valid = out.cargo_count >= std::max(1, expected_cargo_count) && observable;
    out.inside = out.region_valid && inside;
    if (total_visible_pixels) out.own_fraction = float(total_own_pixels) / total_visible_pixels;
    return out;
}

PushboardFloorEvidence observePushboardFloor(
    const cv::Mat& frame, const std::vector<SegDetection>& detections,
    const PushboardFloorConfig& c, const std::string& team,
    bool stable_ui140, uint64_t capture_us) {
    PushboardFloorEvidence out;
    if (!c.enabled || !stable_ui140 || !capture_us || frame.empty() ||
        frame.type() != CV_8UC3 || (team != "blue" && team != "red"))
        return out;
    out.sample_us = capture_us;
    const cv::Rect bounds(0, 0, frame.cols, frame.rows);
    for (int i = 0; i < 2; ++i) {
        auto& w = out.windows[i];
        const auto& reference = c.windows[i];
        w.roi = {cvRound(double(reference.x) * frame.cols / c.image_size.width),
                 cvRound(double(reference.y) * frame.rows / c.image_size.height),
                 cvRound(double(reference.width) * frame.cols / c.image_size.width),
                 cvRound(double(reference.height) * frame.rows / c.image_size.height)};
        if (w.roi.empty() || (w.roi & bounds) != w.roi)
            continue;
        cv::Mat hsv;
        cv::cvtColor(frame(w.roi), hsv, cv::COLOR_BGR2HSV);
        cv::Mat visible(hsv.size(), CV_8U, cv::Scalar(255));
        // Mask all object boxes, including cargo with the same colour as the floor.
        // Zone boxes describe background and must not erase the observation windows.
        for (const auto& d : detections) {
            if (d.label == "blue_safe_zone" || d.label == "red_safe_zone")
                continue;
            if (d.box.empty() || !std::isfinite(d.confidence) || d.confidence < .2f)
                continue;
            cv::Rect object(d.box.x - 8, d.box.y - 8, d.box.width + 16, d.box.height + 16);
            auto overlap = object & w.roi;
            if (!overlap.empty()) {
                overlap.x -= w.roi.x; overlap.y -= w.roi.y;
                visible(overlap).setTo(0);
            }
        }
        visible.setTo(0, colorMask(hsv, c.cargo));
        const int n = cv::countNonZero(visible);
        w.visible = float(n) / w.roi.area();
        if (!n || w.visible < c.min_visible)
            continue;
        const auto fraction = [&](const std::vector<FloorHsvRange>& ranges) {
            auto mask = colorMask(hsv, ranges);
            cv::bitwise_and(mask, visible, mask);
            return float(cv::countNonZero(mask)) / n;
        };
        w.own = fraction(team == "blue" ? c.blue : c.red);
        w.enemy = fraction(team == "blue" ? c.red : c.blue);
        w.white = fraction(c.white);
        w.purple = fraction(c.purple);
        // Unknown pixels stay in the denominator. No sparse-colour normalization.
        w.valid = w.own + w.white + w.purple >= c.min_ground && w.enemy <= .10f;
    }
    out.valid = out.windows[0].valid && out.windows[1].valid;
    out.inside = out.valid && out.windows[0].own >= c.min_own &&
                 out.windows[1].own >= c.min_own;
    return out;
}
} // namespace rescue
