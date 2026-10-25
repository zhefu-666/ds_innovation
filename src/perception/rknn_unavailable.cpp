#include "rescue/perception/rknn_model.hpp"
#include <stdexcept>

namespace rescue {
namespace {
[[noreturn]] void unavailable() {
    throw std::runtime_error("RKNN inference is disabled in this build; rebuild with "
                             "-DRESCUE_ENABLE_RKNN=ON and the vendor SDK header");
}
} // namespace

struct RknnModel::Runtime {};

RknnModel::RknnModel(const std::string&, const std::string&, int) {
    unavailable();
}
RknnModel::~RknnModel() = default;
int RknnModel::inputSize() const {
    unavailable();
}
int RknnModel::outputChannels() const {
    unavailable();
}
int RknnModel::outputAnchors() const {
    unavailable();
}
cv::Mat RknnModel::run(const cv::Mat&) {
    unavailable();
}
} // namespace rescue
