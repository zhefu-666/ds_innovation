#pragma once
#include "rescue/yolo_detector.hpp"
#include <memory>
namespace rescue {
class YoloRknnDetector : public IDetector {
public:
    explicit YoloRknnDetector(const Config &config);
    ~YoloRknnDetector() override;
    std::vector<SegDetection> infer(const cv::Mat &frame) override;
private:
    struct Runtime;
    std::unique_ptr<Runtime> runtime_;
    Config config_;
};
std::unique_ptr<IDetector> makeDetector(const Config &config);
}
