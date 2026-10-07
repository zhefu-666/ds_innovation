#pragma once
#include "rescue/config.hpp"
#include "rescue/model_io.hpp"
#include "rescue/zone_geometry.hpp"
#include <array>
#include <memory>
namespace rescue {
// YOLOv8-pose safe-zone contract (Model-Training-Tool zone_pose_v*):
// classes {zone_left, zone_right}, one instance each. Class 0 zone_left is the
// supply half, class 1 zone_right is the injured half in the zone frame
// (facing into its entrance); these are not image-left/right or team colors.
// Each class has 4 keypoints per half in
// order far_left, near_left, far_right, near_right (robot view), output
// [1, 4+2+4*3, N]: cx,cy,w,h | sigmoid class scores | (x,y,sigmoid conf) x4,
// all coordinates in letterboxed model-input pixels.
struct ZoneHalf {
    bool valid=false;
    float score=0;
    cv::Rect2f box;                      // original-image pixels
    std::array<cv::Point2f,4> keypoints; // original-image pixels
    std::array<float,4> confidence{};
};
using ZoneHalves = std::array<ZoneHalf,2>; // [0]=zone_left, [1]=zone_right
constexpr int kZonePoseClasses=2, kZonePoseKeypoints=4;
constexpr int kZonePoseChannels=4+kZonePoseClasses+kZonePoseKeypoints*3;

// Best anchor per half whose argmax class is that half (equivalent to top-1 after
// class-wise NMS). A half below box_confidence stays invalid; nothing is synthesized.
ZoneHalves decodeZonePose(const cv::Mat &output,const ModelImage &image,cv::Size original,float box_confidence);
// Map the two halves to the six ZoneGeometry ids:
// 0 front_left=L.near_left, 1 front_divider=mean(L.near_right,R.near_left), 2 front_right=R.near_right,
// 3 rear_left=L.far_left, 4 rear_divider=mean(L.far_right,R.far_left), 5 rear_right=R.far_right.
// A divider point needs both halves, both keypoints visible and agreeing; otherwise
// it is omitted (the divider width is not dimensioned, one side alone would bias it).
std::vector<ZoneKeypoint> zoneHalvesToKeypoints(const ZoneHalves &halves,float keypoint_confidence);

class RknnModel;
class SafeZoneKeypointRknn {
public:
    explicit SafeZoneKeypointRknn(const Config &config);
    ~SafeZoneKeypointRknn();
    ZoneHalves infer(const ModelImage &image,cv::Size original);
    int inputSize() const;
private:
    std::unique_ptr<RknnModel> model_;
    float box_confidence_;
};
} // namespace rescue
