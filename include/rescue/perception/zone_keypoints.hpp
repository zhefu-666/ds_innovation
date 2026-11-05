#pragma once
#include "rescue/config/config.hpp"
#include "rescue/perception/model_io.hpp"
#include "rescue/geometry/zone_geometry.hpp"
#include <array>
#include <memory>
#include <string>
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
    bool valid = false;
    float score = 0;
    cv::Rect2f box;                       // original-image pixels
    std::array<cv::Point2f, 4> keypoints; // original-image pixels
    std::array<float, 4> confidence{};
};
using ZoneHalves = std::array<ZoneHalf, 2>; // [0]=zone_left, [1]=zone_right
constexpr int kZonePoseClasses = 2, kZonePoseKeypoints = 4;
constexpr int kZonePoseChannels = 4 + kZonePoseClasses + kZonePoseKeypoints * 3;

struct ZonePoseClassMetadata {
    int nms_candidates = 0;
    cv::Rect2f unclipped_box;
};
using ZonePoseMetadata = std::array<ZonePoseClassMetadata, 2>;

// Best anchor per half whose argmax class is that half (equivalent to top-1 after
// class-wise NMS). A half below box_confidence stays invalid; nothing is synthesized.
ZoneHalves decodeZonePose(const cv::Mat& output,
                          const ModelImage& image,
                          cv::Size original,
                          float box_confidence,
                          ZonePoseMetadata* metadata = nullptr);
// Map the two halves to the six ZoneGeometry ids:
// 0 front_left=L.near_left, 1 front_divider=mean(L.near_right,R.near_left), 2
// front_right=R.near_right, 3 rear_left=L.far_left, 4 rear_divider=mean(L.far_right,R.far_left), 5
// rear_right=R.far_right. A divider point needs both halves, both keypoints visible and agreeing;
// otherwise it is omitted (the divider width is not dimensioned, one side alone would bias it).
std::vector<ZoneKeypoint> zoneHalvesToKeypoints(const ZoneHalves& halves,
                                                float keypoint_confidence,
                                                cv::Size image_size = {});

struct ZonePoseRoute {
    bool dual_expert = false;
    float weight_far = 0, max_area_fraction = 0;
    std::array<int, 2> counts{};
    std::string reason = "legacy";
};
// A_INT8 [1,26,8400]: NMS then whole-frame area routing, no extra sigmoid.
ZoneHalves decodeZonePoseExperts(const cv::Mat& output,
                                 const ModelImage& image,
                                 cv::Size original,
                                 ZonePoseMetadata* metadata = nullptr,
                                 ZonePoseRoute* route = nullptr);
class RknnModel;
class SafeZoneKeypointRknn {
  public:
    explicit SafeZoneKeypointRknn(const Config& config);
    ~SafeZoneKeypointRknn();
    ZoneHalves infer(const ModelImage& image,
                     cv::Size original,
                     ZonePoseMetadata* metadata = nullptr,
                     ZonePoseRoute* route = nullptr);
    int inputSize() const;
    bool dualExpert() const;

  private:
    std::unique_ptr<RknnModel> model_;
    float box_confidence_;
};
} // namespace rescue
