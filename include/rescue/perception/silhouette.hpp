#pragma once
#include <opencv2/core.hpp>

namespace rescue {

// Height/width of the largest orange blob inside the detection box (original-image pixels).
// The detector emits boxes only, so the silhouette is taken from colour. Returns -1 when the
// blob is too small or the box/image is unusable; callers then keep the box-only verdict.
float orangeSilhouetteAspect(const cv::Mat& bgr, const cv::Rect& box);

} // namespace rescue
