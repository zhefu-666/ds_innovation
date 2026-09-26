#include "rescue/vision_logic.hpp"

#include <array>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>

namespace rescue {

namespace {

cv::Rect asCvRect(const RectArea &area) {
    return cv::Rect(cv::Point(static_cast<int>(area.x1), static_cast<int>(area.y1)),
                    cv::Point(static_cast<int>(area.x2), static_cast<int>(area.y2)));
}

} // namespace

VisionLogic::VisionLogic(Config config)
    : config_(std::move(config)),
      target_balls_(makeTargetBalls(config_.team)),
      target_areas_(makeTargetAreas(config_.team)) {
    std::cout << "[VISION] team: " << config_.team << "\n";
    std::cout << "  target balls:";
    for (const auto &label : target_balls_) {
        std::cout << ' ' << label;
    }
    std::cout << "\n  target areas:";
    for (const auto &label : target_areas_) {
        std::cout << ' ' << label;
    }
    std::cout << "\n";
}

std::pair<bool, Detection>
VisionLogic::detectClosestBall(const std::vector<Detection> &detections,
                               bool validity) const {
    std::optional<Detection> area_box;
    if (validity) {
        for (const auto &det : detections) {
            if (det.confidence > config_.confidence && isInList(target_areas_, det.label)) {
                area_box = det;
                break;
            }
        }
    }

    std::optional<Detection> closest;
    float max_area = 0.0f;
    for (const auto &det : detections) {
        if (det.confidence < config_.confidence || !isInList(target_balls_, det.label)) {
            continue;
        }

        if (validity && area_box.has_value()) {
            int corners_inside = 0;
            const auto &a = area_box->box;
            const std::array<cv::Point2f, 4> corners{
                cv::Point2f(det.box.x, det.box.y),
                cv::Point2f(det.box.x + det.box.width, det.box.y),
                cv::Point2f(det.box.x, det.box.y + det.box.height),
                cv::Point2f(det.box.x + det.box.width, det.box.y + det.box.height),
            };
            for (const auto &p : corners) {
                if (a.contains(p)) {
                    ++corners_inside;
                }
            }
            if (corners_inside == 4) {
                continue;
            }
        }

        if (det.area() > max_area) {
            max_area = det.area();
            closest = det;
        }
    }

    if (!closest.has_value()) {
        return {false, Detection{}};
    }
    return {true, *closest};
}

std::pair<bool, Detection>
VisionLogic::detectArea(const std::vector<Detection> &detections) const {
    std::optional<Detection> best;
    for (const auto &det : detections) {
        if (det.confidence < config_.confidence || !isInList(target_areas_, det.label)) {
            continue;
        }
        if (!best.has_value() || det.confidence > best->confidence) {
            best = det;
        }
    }
    if (!best.has_value()) {
        return {false, Detection{}};
    }
    return {true, *best};
}

bool VisionLogic::isReadyToCatch(float x, float y) const {
    return config_.catch_area.contains(x, y);
}

bool VisionLogic::isBallInHoldingArea(const cv::Point2f &center) const {
    return config_.holding_area.contains(center.x, center.y);
}

bool VisionLogic::isAreaCentered(const cv::Point2f &center) const {
    return config_.center_region.contains(center.x, center.y);
}

void VisionLogic::drawGuides(cv::Mat &frame) const {
    cv::rectangle(frame, asCvRect(config_.catch_area), cv::Scalar(255, 255, 255), 2);
    cv::rectangle(frame, asCvRect(config_.center_region), cv::Scalar(0, 255, 255), 2);
    cv::rectangle(frame, asCvRect(config_.holding_area), cv::Scalar(255, 0, 0), 2);
}

void VisionLogic::drawDetections(cv::Mat &frame,
                                 const std::vector<Detection> &detections) const {
    for (const auto &det : detections) {
        cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
        std::ostringstream text;
        text << det.label << ' ' << std::fixed << std::setprecision(2) << det.confidence;
        const int y = std::max(15, static_cast<int>(det.box.y) - 6);
        cv::putText(frame, text.str(), cv::Point(static_cast<int>(det.box.x), y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
    }
}

} // namespace rescue
