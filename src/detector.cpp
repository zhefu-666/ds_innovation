#include "rescue/detector.hpp"
#include "rescue/rknn_model.hpp"
#include <chrono>
#include <stdexcept>
#include <cctype>
#include <algorithm>
namespace rescue {
YoloRknnDetector::~YoloRknnDetector()=default;
YoloRknnDetector::YoloRknnDetector(const Config &c):config_(c) {
    if(c.input_size<=0 || c.class_names.empty() || c.require_instance_masks)
        throw std::runtime_error("RKNN detector requires a positive input size, class names and box-only output");
    model_=std::make_unique<RknnModel>(c.rknn_library,c.model_path,c.detect_core_mask);
    const int attrs=4+int(c.class_names.size());
    // Contract from the model itself; config must agree rather than be trusted.
    if(model_->inputSize()!=c.input_size || model_->outputChannels()!=attrs)
        throw std::runtime_error("RKNN detector contract mismatch: config expects input "+std::to_string(c.input_size)+
            " and [1,"+std::to_string(attrs)+",N]; model has input "+std::to_string(model_->inputSize())+
            " and [1,"+std::to_string(model_->outputChannels())+","+std::to_string(model_->outputAnchors())+"]");
}
std::vector<SegDetection> YoloRknnDetector::infer(const cv::Mat &frame) {
    auto timestamp=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return infer(prepareModelImage(frame,config_.input_size),frame.size(),uint64_t(timestamp));
}
std::vector<SegDetection> YoloRknnDetector::infer(const ModelImage &image,cv::Size original,uint64_t timestamp) {
    return decodeModelOutput(model_->run(image.rgb),image,original,config_,timestamp);
}
std::unique_ptr<IDetector> makeDetector(const Config &config) {
    auto pos=config.model_path.find_last_of('.');auto ext=pos==std::string::npos?"":config.model_path.substr(pos);
    std::transform(ext.begin(),ext.end(),ext.begin(),[](unsigned char ch){return std::tolower(ch);});
    if(ext==".rknn")return std::make_unique<YoloRknnDetector>(config);
    if(ext==".onnx")return std::make_unique<YoloOnnxDetector>(config);
    throw std::runtime_error("Model must be .rknn or .onnx");
}

} // namespace rescue
