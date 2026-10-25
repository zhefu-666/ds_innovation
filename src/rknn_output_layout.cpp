#include "rescue/rknn_output_layout.hpp"
#include <cstring>
#include <limits>
#include <stdexcept>

namespace rescue {
namespace {

size_t elementCount(const std::vector<uint32_t> &dims) {
    size_t count = 1;
    for (const auto dim : dims) {
        if (dim == 0) throw std::runtime_error("Empty RKNN dimension");
        if (count > std::numeric_limits<size_t>::max() / dim)
            throw std::runtime_error("RKNN tensor element count overflow");
        count *= dim;
    }
    return count;
}

void requireDims(const std::string &model, const std::vector<uint32_t> &actual,
                 const std::vector<uint32_t> &expected) {
    if (actual != expected) throw std::runtime_error("RKNN output shape mismatch: " + model);
}

} // namespace

RknnOutputLayout validateRknnOutputLayout(
    const std::string &model, const std::vector<std::vector<uint32_t>> &dims) {
    RknnOutputLayout layout;
    layout.dims = dims;
    if (dims.empty() || (dims.size() != 1 && dims.size() != 2 && dims.size() != 4))
        throw std::runtime_error("Expected one, two or four RKNN outputs: " + model);
    for (const auto &shape : dims) {
        if (shape.empty() || shape[0] != 1)
            throw std::runtime_error("Expected batch-one RKNN output: " + model);
        for (const auto dim : shape) {
            if (dim == 0) throw std::runtime_error("Empty RKNN dimension: " + model);
        }
        (void)elementCount(shape);
    }

    if (dims.size() == 1) {
        if (dims[0].size() != 3)
            throw std::runtime_error("Expected [1,C,N] RKNN output: " + model);
        layout.channels = static_cast<int>(dims[0][1]);
        layout.anchors = static_cast<int>(dims[0][2]);
        return layout;
    }

    if (dims.size() == 2) {
        if (dims[0].size() != 3 || dims[1].size() != 3 ||
            dims[0][1] != 4 || dims[1][1] != 4 || dims[0][2] != dims[1][2])
            throw std::runtime_error(
                "Expected boxes then four-class scores with equal anchors: " + model);
        layout.channels = 8;
        layout.anchors = static_cast<int>(dims[0][2]);
        return layout;
    }

    const uint32_t anchors = dims[0].size() == 3 ? dims[0][2] : 0;
    if (anchors == 0)
        throw std::runtime_error("Expected [1,4,N] pose boxes first: " + model);
    requireDims(model, dims[0], {1, 4, anchors});
    requireDims(model, dims[1], {1, 2, anchors});
    requireDims(model, dims[2], {1, 4, 2, anchors});
    requireDims(model, dims[3], {1, 4, 1, anchors});
    layout.channels = 18;
    layout.anchors = static_cast<int>(anchors);
    return layout;
}

cv::Mat mergeRknnOutputBranches(const std::vector<cv::Mat> &branches) {
    std::vector<std::vector<uint32_t>> dims;
    dims.reserve(branches.size());
    for (const auto &branch : branches) {
        if (branch.type() != CV_32F || !branch.isContinuous())
            throw std::runtime_error("RKNN output branches must be continuous float32");
        std::vector<uint32_t> shape;
        shape.reserve(static_cast<size_t>(branch.dims));
        for (int i = 0; i < branch.dims; ++i) shape.push_back(static_cast<uint32_t>(branch.size[i]));
        if (branch.total() != elementCount(shape))
            throw std::runtime_error("RKNN output element count mismatch");
        dims.push_back(std::move(shape));
    }
    const auto layout = validateRknnOutputLayout("<merged branches>", dims);

    int result_shape[] = {1, layout.channels, layout.anchors};
    cv::Mat result(3, result_shape, CV_32F);
    float *dst = result.ptr<float>();
    const size_t anchors = static_cast<size_t>(layout.anchors);

    if (branches.size() != 4) {
        size_t offset = 0;
        for (const auto &branch : branches) {
            const size_t count = branch.total();
            std::memcpy(dst + offset, branch.ptr<float>(), count * sizeof(float));
            offset += count;
        }
        return result;
    }

    for (int row = 0; row < 6; ++row) {
        const auto &branch = branches[static_cast<size_t>(row / 4)];
        std::memcpy(dst + static_cast<size_t>(row) * anchors,
                    branch.ptr<float>() + static_cast<size_t>(row % 4) * anchors,
                    anchors * sizeof(float));
    }
    const float *xy = branches[2].ptr<float>();
    const float *visibility = branches[3].ptr<float>();
    for (int keypoint = 0; keypoint < 4; ++keypoint) {
        std::memcpy(dst + static_cast<size_t>(6 + 3 * keypoint) * anchors,
                    xy + static_cast<size_t>(2 * keypoint) * anchors, anchors * sizeof(float));
        std::memcpy(dst + static_cast<size_t>(7 + 3 * keypoint) * anchors,
                    xy + static_cast<size_t>(2 * keypoint + 1) * anchors, anchors * sizeof(float));
        std::memcpy(dst + static_cast<size_t>(8 + 3 * keypoint) * anchors,
                    visibility + static_cast<size_t>(keypoint) * anchors, anchors * sizeof(float));
    }
    return result;
}

} // namespace rescue
