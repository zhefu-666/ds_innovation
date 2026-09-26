#pragma once
#include "rescue/config.hpp"
#include "rescue/types.hpp"
#include <opencv2/core.hpp>
namespace rescue {
struct ModelImage { cv::Mat rgb; float sx=1, sy=1; int pad_x=0, pad_y=0; };
ModelImage prepareModelImage(const cv::Mat &bgr, int size);
std::string taskLabel(const std::string &model_label);
std::vector<SegDetection> decodeModelOutput(const cv::Mat &output, const ModelImage &image,
    cv::Size original, const Config &config, uint64_t timestamp_us);
}
