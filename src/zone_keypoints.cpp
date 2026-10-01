#include "rescue/zone_keypoints.hpp"
#include "rescue/rknn_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rescue {
ZoneHalves decodeZonePose(const cv::Mat &output,const ModelImage &image,cv::Size original,float box_confidence) {
    if(output.type()!=CV_32F || output.dims!=3 || output.size[0]!=1 || output.size[1]!=kZonePoseChannels || !output.isContinuous())
        throw std::runtime_error("Expected float32 [1,18,N] zone pose output (2 classes, 4 keypoints)");
    if(!std::isfinite(box_confidence)||box_confidence<0||box_confidence>1)throw std::runtime_error("Invalid pose threshold");
    if(image.sx<=0||image.sy<=0)throw std::runtime_error("Invalid letterbox scale");
    const int count=output.size[2];const float *raw=output.ptr<float>();
    const auto at=[&](int row,int i){return raw[row*count+i];};
    std::array<int,2> best{-1,-1};
    for(int i=0;i<count;++i) {
        bool finite=true;for(int k=0;k<kZonePoseChannels;++k)finite=finite&&std::isfinite(at(k,i));
        if(!finite||at(2,i)<=0||at(3,i)<=0)continue;
        const int cls=at(5,i)>at(4,i)?1:0;const float score=at(4+cls,i);
        if(score<box_confidence||score>1)continue;
        if(best[cls]<0||score>at(4+cls,best[cls]))best[cls]=i;
    }
    const auto unmap=[&](float x,float y){return cv::Point2f((x-image.pad_x)/image.sx,(y-image.pad_y)/image.sy);};
    ZoneHalves out;
    for(int cls=0;cls<2;++cls) {
        const int i=best[cls];if(i<0)continue;
        auto &h=out[cls];h.valid=true;h.score=at(4+cls,i);
        const auto tl=unmap(at(0,i)-at(2,i)/2,at(1,i)-at(3,i)/2),br=unmap(at(0,i)+at(2,i)/2,at(1,i)+at(3,i)/2);
        h.box=cv::Rect2f(tl,br)&cv::Rect2f(0,0,float(original.width),float(original.height));
        for(int k=0;k<kZonePoseKeypoints;++k) {
            const int row=4+kZonePoseClasses+3*k;
            h.keypoints[k]=unmap(at(row,i),at(row+1,i));
            h.confidence[k]=std::clamp(at(row+2,i),0.f,1.f);
        }
    }
    return out;
}
std::vector<ZoneKeypoint> zoneHalvesToKeypoints(const ZoneHalves &h,float threshold) {
    enum {FAR_LEFT=0,NEAR_LEFT=1,FAR_RIGHT=2,NEAR_RIGHT=3};
    const auto &L=h[0],&R=h[1];
    std::vector<ZoneKeypoint> out;
    const auto seen=[&](const ZoneHalf &half,int k){return half.valid&&half.confidence[k]>=threshold;};
    const auto single=[&](int id,const ZoneHalf &half,int k) {
        if(seen(half,k))out.push_back({id,half.keypoints[k],half.confidence[k],true});
    };
    // Divider corners from the two halves mark the same physical divider, so they
    // must be close relative to the half width (divider << 330 mm half width).
    const auto divider=[&](int id,int left_k,int left_outer,int right_k,int right_outer) {
        if(!seen(L,left_k)||!seen(R,right_k))return;
        const auto a=L.keypoints[left_k],b=R.keypoints[right_k];
        float span=0;
        if(seen(L,left_outer))span=std::max(span,float(cv::norm(a-L.keypoints[left_outer])));
        if(seen(R,right_outer))span=std::max(span,float(cv::norm(b-R.keypoints[right_outer])));
        if(span>0&&cv::norm(a-b)>.25f*span)return;
        out.push_back({id,(a+b)*.5f,std::min(L.confidence[left_k],R.confidence[right_k]),true});
    };
    single(0,L,NEAR_LEFT);
    divider(1,NEAR_RIGHT,NEAR_LEFT,NEAR_LEFT,NEAR_RIGHT);
    single(2,R,NEAR_RIGHT);
    single(3,L,FAR_LEFT);
    divider(4,FAR_RIGHT,FAR_LEFT,FAR_LEFT,FAR_RIGHT);
    single(5,R,FAR_RIGHT);
    return out;
}
SafeZoneKeypointRknn::SafeZoneKeypointRknn(const Config &c):box_confidence_(c.pose_confidence) {
    model_=std::make_unique<RknnModel>(c.rknn_library,c.pose_model_path,c.pose_core_mask);
    if(model_->inputSize()!=c.input_size || model_->outputChannels()!=kZonePoseChannels)
        throw std::runtime_error("Pose model contract mismatch: expected input "+std::to_string(c.input_size)+
            " (shared letterbox with detector) and [1,"+std::to_string(kZonePoseChannels)+",N]; model has input "+
            std::to_string(model_->inputSize())+" and [1,"+std::to_string(model_->outputChannels())+","+
            std::to_string(model_->outputAnchors())+"]");
}
SafeZoneKeypointRknn::~SafeZoneKeypointRknn()=default;
int SafeZoneKeypointRknn::inputSize() const {return model_->inputSize();}
ZoneHalves SafeZoneKeypointRknn::infer(const ModelImage &image,cv::Size original) {
    return decodeZonePose(model_->run(image.rgb),image,original,box_confidence_);
}
} // namespace rescue
