#pragma once
#include "rescue/yolo_detector.hpp"
#include "rescue/model_io.hpp"
#include <memory>
namespace rescue {
class RknnModel;
class YoloRknnDetector : public IDetector {
public:
    explicit YoloRknnDetector(const Config &config);
    ~YoloRknnDetector() override;
    std::vector<SegDetection> infer(const cv::Mat &frame) override;
    // Shared-letterbox path: the caller prepares one image for detect and pose.
    std::vector<SegDetection> infer(const ModelImage &image,cv::Size original,uint64_t timestamp_us);
private:
    std::unique_ptr<RknnModel> model_;
    Config config_;
};
std::unique_ptr<IDetector> makeDetector(const Config &config);
}
