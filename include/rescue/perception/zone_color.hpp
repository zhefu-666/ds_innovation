#pragma once
#include "rescue/perception/zone_keypoints.hpp"
#include "rescue/core/types.hpp"
namespace rescue {
struct ZoneColorConfig {
    bool measured=false;
    int saturation_min=80,value_min=30,min_pixels=200;
    // Red and blue are compared only with each other, so white floor, walls and other hues inside a loose polygon do not dilute them.
    float min_fraction=.60f,max_other_fraction=.10f,min_coloured_fraction=.05f;
    // Sampled region = keypoint polygon widened sideways and pushed toward the camera (near edge), never past the far edge.
    float expand_x=.20f,expand_near=.50f;
    static ZoneColorConfig load(const std::string& file);
};
struct ZoneColorResult { std::string color,reason="color_unmeasured"; bool verified=false, assumed=false; int halves_used=0; };
// Blue-only model preview: label the detected zone as a blue assumption without
// turning model selection into independently verified color evidence.
ZoneColorResult previewBlueZoneDefault(ZoneColorResult result, bool dry_run, const std::string& team,
                                       bool blue_model_detected_both_halves);
// Team rule applied to a measured result. Only a measured colour yields identity: a zone whose colour
// differs from the team keeps verified=true with the enemy colour and reason "enemy_zone_color", so it can
// never be labelled as the own zone. With assume_team=false (match mode) an unmeasured colour stays unknown;
// assume_team=true (demo on an attested own-zone-only field) falls back to the team colour.
ZoneColorResult resolveZoneColor(ZoneColorResult measured,const std::string& team,bool assume_team);

// Samples the interior of every usable detected half (at least one), excludes detected objects.
// A half whose corners are out of frame or occluded is skipped; two halves must agree.
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
