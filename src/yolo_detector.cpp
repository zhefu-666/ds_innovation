#include "rescue/yolo_detector.hpp"
#include "rescue/model_io.hpp"
#include <chrono>
#include <stdexcept>
namespace rescue {
YoloOnnxDetector::YoloOnnxDetector(const Config &c):config_(c) {
    if(c.require_instance_masks)throw std::runtime_error("Current model has no masks");
    net_=cv::dnn::readNet(c.model_path);
    if(net_.empty())throw std::runtime_error("Cannot load ONNX model");
    net_.setPreferableBackend(c.use_cuda?cv::dnn::DNN_BACKEND_CUDA:cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(c.use_cuda?cv::dnn::DNN_TARGET_CUDA:cv::dnn::DNN_TARGET_CPU);
}
std::vector<SegDetection> YoloOnnxDetector::infer(const cv::Mat &frame) {
    auto timestamp=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto image=prepareModelImage(frame,config_.input_size);
    // ONNX expects RGB NCHW /255. RKNN embeds normalization and takes RGB uint8.
    net_.setInput(cv::dnn::blobFromImage(image.rgb,1.0/255.0));
    std::vector<cv::Mat> out;net_.forward(out,net_.getUnconnectedOutLayersNames());
    if(out.size()!=1)throw std::runtime_error("Expected one detection tensor");
    return decodeModelOutput(out[0],image,frame.size(),config_,timestamp);
}
}
