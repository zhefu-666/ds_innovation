#pragma once
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>
namespace rescue {
// One RKNN context loaded through dlopen. Not thread-safe: each model is owned by
// exactly one thread. Input is a square NHWC uint8 RGB image (mean/std embedded
// at conversion); the single output is returned as an owned float32 [1,C,N] tensor.
class RknnModel {
public:
    // core_mask: -1 leaves the runtime default; otherwise RKNN_NPU_CORE_* (0 auto, 1, 2, 4).
    RknnModel(const std::string &library,const std::string &model,int core_mask);
    ~RknnModel();
    RknnModel(const RknnModel&)=delete;
    RknnModel &operator=(const RknnModel&)=delete;
    int inputSize() const;
    int outputChannels() const;
    int outputAnchors() const;
    cv::Mat run(const cv::Mat &rgb);
private:
    struct Runtime;
    std::unique_ptr<Runtime> r_;
};
} // namespace rescue
