#pragma once
#include "rescue/zone_keypoints.hpp"
#include "rescue/types.hpp"
namespace rescue {
struct ZoneColorConfig {
    bool measured=false;
    int saturation_min=80,value_min=45,min_pixels=200;
    float min_fraction=.60f,max_other_fraction=.10f;
    static ZoneColorConfig load(const std::string& file);
};
struct ZoneColorResult { std::string color,reason="color_unmeasured"; bool verified=false, assumed=false; };
// Blue-only model preview: label the detected zone as a blue assumption without
// turning model selection into independently verified color evidence.
ZoneColorResult previewBlueZoneDefault(ZoneColorResult result, bool dry_run, const std::string& team,
                                       bool blue_model_detected_both_halves);
// Samples only the interiors of BOTH detected halves, excludes detected objects.
// A shape label or --team never supplies identity; missing/ambiguous color stays unknown.
class ZoneColorClassifier {
public:
    explicit ZoneColorClassifier(ZoneColorConfig c={}):config_(c){}
    ZoneColorResult classify(const cv::Mat& bgr,const ZoneHalves& halves,
                             const std::vector<SegDetection>& objects,float keypoint_threshold) const;
private:
    ZoneColorConfig config_;
};
}
