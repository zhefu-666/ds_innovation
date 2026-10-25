#pragma once
#include "rescue/config/config.hpp"
#include "rescue/core/types.hpp"
#include <opencv2/dnn.hpp>
namespace rescue {
class IDetector {
  public:
    virtual ~IDetector() = default;
    virtual std::vector<SegDetection> infer(const cv::Mat& frame) = 0;
};
class YoloOnnxDetector : public IDetector {
  public:
    explicit YoloOnnxDetector(const Config& config);
    std::vector<SegDetection> infer(const cv::Mat& frame) override;

  private:
    Config config_;
    cv::dnn::Net net_;
};
} // namespace rescue
