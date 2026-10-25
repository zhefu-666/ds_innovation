#pragma once
#include <opencv2/core.hpp>

namespace rescue {

// Height/width of the largest orange blob inside the detection box (original-image pixels).
// The detector emits boxes only, so the silhouette is taken from colour. Returns -1 when the
// blob is too small or the box/image is unusable; callers then keep the box-only verdict.
float orangeSilhouetteAspect(const cv::Mat& bgr,const cv::Rect& box);

struct SilhouetteRule {
    float lying_below=.9f;    // silhouette clearly wider than tall
    float upright_above=1.15f; // silhouette clearly taller than wide
    float lying_diff=-.11f;   // per-frame pose diff substituted for a clearly lying silhouette
    float upright_diff=.10f;  // per-frame pose diff substituted for a clearly upright silhouette
};

// Per-frame pose diff (lying err - upright err) after the silhouette cue. Between the two
// thresholds, or without a silhouette (aspect <= 0), the box-fit diff is returned unchanged.
float silhouetteAdjustedDiff(float pose_diff,float aspect,const SilhouetteRule& rule={});

} // namespace rescue
