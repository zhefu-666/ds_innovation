#include "rescue/perception/pushboard_floor.hpp"
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
    bool stable_ui120, uint64_t capture_us) {
    TransportFloorEvidence out;
    if (!stable_ui120 || !capture_us || frame.empty() || frame.type() != CV_8UC3 ||
        (team != "blue" && team != "red"))
        return out;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    const auto& region = c.transport_region;
    auto own = colorMask(hsv, team == "blue" ? region.blue : region.red);
    own.setTo(0, colorMask(hsv, c.cargo));
    const cv::Rect bounds(0, 0, frame.cols, frame.rows);
    for (const auto& d : detections) {
        if (d.label == "blue_safe_zone" || d.label == "red_safe_zone" ||
            !std::isfinite(d.confidence) || d.confidence < .2f || d.box.empty())
            continue;
        const auto box = cv::Rect(d.box.x - 8, d.box.y - 8,
                                  d.box.width + 16, d.box.height + 16) & bounds;
        if (!box.empty())
            own(box).setTo(0);
    }
    out.valid = true;
    out.sample_us = capture_us;
    out.own_fraction = float(cv::countNonZero(own)) / float(frame.total());
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(own, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = .02 * frame.total();
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area <= best_area)
            continue;
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        const auto box = cv::boundingRect(hull);
        // FAR has already aligned the selected half with the chassis centre.
        // Use that half only; never join equal-colour floor across the divider.
        if (box.x > frame.cols / 2 || box.x + box.width <= frame.cols / 2)
            continue;
        best_area = area;
        out.region_box = box;
        out.region_contour = std::move(hull);
    }
    if (!out.region_contour.empty()) {
        // External contours ignore internal cargo holes. The convex envelope
        // fills the interior without combining disconnected left/right halves.
        out.region_valid = true;
        out.width_fraction = float(out.region_box.width) / frame.cols;
        out.height_fraction = float(out.region_box.height) / frame.rows;
        out.bottom_fraction = float(out.region_box.y + out.region_box.height) / frame.rows;
        out.inside = out.width_fraction >= region.min_width_fraction &&
                     out.height_fraction >= region.min_height_fraction &&
                     out.bottom_fraction >= region.min_bottom_fraction;
    }
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
