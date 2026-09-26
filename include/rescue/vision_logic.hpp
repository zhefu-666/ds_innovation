#pragma once

#include "rescue/config.hpp"
#include "rescue/types.hpp"

#include <opencv2/opencv.hpp>

#include <string>
#include <utility>
#include <vector>

namespace rescue {

class VisionLogic {
public:
    explicit VisionLogic(Config config);

    std::pair<bool, Detection> detectClosestBall(const std::vector<Detection> &detections,
                                                 bool validity = true) const;
    std::pair<bool, Detection> detectArea(const std::vector<Detection> &detections) const;

    bool isReadyToCatch(float x, float y) const;
    bool isBallInHoldingArea(const cv::Point2f &center) const;
    bool isAreaCentered(const cv::Point2f &center) const;

    void drawGuides(cv::Mat &frame) const;
    void drawDetections(cv::Mat &frame, const std::vector<Detection> &detections) const;

private:
    Config config_;
    std::vector<std::string> target_balls_;
    std::vector<std::string> target_areas_;
};

} // namespace rescue
