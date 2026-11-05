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
//   2 branches: [1,4,N] boxes, [1,4,N] or [1,6,N] class scores (detector split)
//   4 branches: [1,4,N], [1,2,N], [1,4,2,N], [1,4,1,N] (pose split)
//   dual expert: [1,4,8400], [1,2,8400], [1,16,8400], [1,4,8400]
//   raw XY: [1,4,8400], [1,2,8400], [1,8,8400], [1,8,8400], [1,4,8400]
// Raw XY uses CPU grid/stride decoding before the same dual-expert layout.
// Legacy pose is normalized to [1,18,N]; dual expert remains [1,26,8400]
// in boxes/scores/baseXY/farXY/visibility order for scale-aware decoding.
RknnOutputLayout validateRknnOutputLayout(const std::string& model,
                                          const std::vector<std::vector<uint32_t>>& dims);

// Merge already-validated float32 branches into one owned [1,C,N] matrix.
// Two detector branches are concatenated boxes-then-scores. Four pose branches
// are reordered so each keypoint's x,y,visibility rows are adjacent. Five raw-XY
// branches restore grid/stride coordinates before merging into 26 channels.
cv::Mat mergeRknnOutputBranches(const std::vector<cv::Mat>& branches);

} // namespace rescue
