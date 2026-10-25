#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace rescue {

struct RknnOutputLayout {
    int channels = 0;
    int anchors = 0;
    std::vector<std::vector<uint32_t>> dims;
};

// Validate the supported YOLO/RKNN output contracts:
//   1 branch:  [1,C,N]
//   2 branches: [1,4,N] boxes, [1,4,N] class scores (detector split)
//   4 branches: [1,4,N], [1,2,N], [1,4,2,N], [1,4,1,N] (pose split)
// The 4-branch contract is normalized to [1,18,N].
RknnOutputLayout validateRknnOutputLayout(
    const std::string &model, const std::vector<std::vector<uint32_t>> &dims);

// Merge already-validated float32 branches into one owned [1,C,N] matrix.
// Two detector branches are concatenated boxes-then-scores. Four pose branches
// are reordered so each keypoint's x,y,visibility rows are adjacent.
cv::Mat mergeRknnOutputBranches(const std::vector<cv::Mat> &branches);

} // namespace rescue
