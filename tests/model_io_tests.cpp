#include "rescue/model_io.hpp"
#include "rescue/perception_adapter.hpp"
#include "rescue/zone_keypoints.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <map>
#include <iostream>
using namespace rescue;
namespace {
// 640x480 into 640: scale 1, pad_y 80. 1280x720 into 640: scale .5, pad_y 140.
void detectorTests() {
    Config c;c.confidence=0.25f;
    assert(c.input_size==640&&c.class_names==std::vector<std::string>({"blue","orange","green","black"}));
    cv::Mat bgr(480,640,CV_8UC3,cv::Scalar(10,20,30));auto image=prepareModelImage(bgr,640);
    assert(image.pad_y==80&&image.pad_x==0&&image.sx==1&&image.sy==1);
    assert(image.rgb.at<cv::Vec3b>(100,100)==cv::Vec3b(30,20,10));
    assert(image.rgb.at<cv::Vec3b>(0,0)==cv::Vec3b(114,114,114));
    int dims[]={1,8,4};cv::Mat raw(3,dims,CV_32F,cv::Scalar(0));float *p=raw.ptr<float>();
    for(int i=0;i<4;++i){p[i]=320;p[4+i]=320;p[8+i]=200;p[12+i]=100;}
    p[6*4]=.9f;   // green index 2
    p[6*4+1]=.8f; // overlapping green suppressed
    p[4*4+2]=.85f;// blue (danger) index 0, overlapping but different class: retained
    p[7*4+3]=.7f; // black index 3
    auto out=decodeModelOutput(raw,image,bgr.size(),c,1000000);
    assert(out.size()==3&&out[0].label=="ordinary_supply"&&out[0].class_id==2&&out[0].model_label=="green");
    assert(out[0].box==cv::Rect(220,190,200,100));
    assert(out[1].label=="dangerous_object"&&out[1].model_label=="blue"&&out[2].label=="core_supply"&&out[0].mask.empty());
    out[0].track_id=1;out[1].track_id=2;out[2].track_id=3;
    auto in=makePushObservation(out,1050000,false,.25f);
    assert(in.target_valid&&in.label=="ordinary_supply"&&!in.geometry_valid&&!in.safety_ok&&!in.zone_valid&&!in.path_safe&&!in.captured&&!in.zone_counts_valid);
    assert(!makePushObservation(out,1300000,false,.25f).target_valid);
    // A different nearer candidate never replaces a locked ID; rejected IDs are skipped.
    assert(makePushObservation(out,1050000,true,.25f,3).target_id==3);
    assert(!makePushObservation(out,1050000,true,.25f,99).target_valid);
    assert(makePushObservation(out,1050000,true,.25f,-1,{1}).target_id==3);
    assert(!makePushObservation(out,1050000,false,.25f,-1,{1}).target_valid);
    // The danger object is never a push target, even after the first ordinary delivery.
    std::vector<SegDetection> danger{out[1]};assert(!makePushObservation(danger,1050000,true,.25f).target_valid);
    { // All known colours are search cues, with metric distance and stable locks.
        auto objects=out;
        for(auto& d:objects){d.ground_position_valid=d.ground_contact_valid=true;d.body_xy_m={0,.7f};}
        objects[1].body_xy_m={0,.4f}; // blue nearest, green still visible
        auto picked=makePushObservation(objects,1050000,false,.25f,-1,{},true,false,true);
        assert(picked.target_id==1 && picked.target_is_search_cue);
        picked=makePushObservation(objects,1050000,false,.25f,2,{},true,true,true);
        assert(picked.target_id==1); // valid green replaces a non-green search cue
        objects[0].ground_contact_valid=false;
        assert(makePushObservation(objects,1050000,false,.25f,2,{},true,true,true).target_id==2);
        objects[0].ground_contact_valid=true;
        assert(makePushObservation(objects,1050000,false,.25f,2,{1},true,true,true).target_id==2);
        assert(makePushObservation(objects,1050000,false,.25f,1,{},true,true,true).target_id==1);
        picked=makePushObservation(objects,1050000,false,.25f,-1,{},false,false,true);
        assert(picked.target_id==1 && !picked.target_is_search_cue);
        picked=makePushObservation(objects,1050000,true,.25f,-1,{},false,false,true);
        assert(picked.target_id==3); // black wins despite farther distance
        objects[2].ground_contact_valid=false;
        assert(makePushObservation(objects,1050000,true,.25f,-1,{},false,false,true).target_id==1);
        assert(!makePushObservation(objects,1300000,false,.25f,-1,{},true,false,true).target_valid);
        objects[1].label="unknown";objects[0].ground_contact_valid=false;
        assert(!makePushObservation(objects,1050000,false,.25f,-1,{},true,false,true).target_valid);
    }
    assert(taskLabel("blue")=="dangerous_object"&&taskLabel("orange")=="injured_person");
    assert(taskLabel("green")=="ordinary_supply"&&taskLabel("black")=="core_supply");
    assert(taskLabel("red")=="unmapped_red"&&taskLabel("core")=="unmapped_core");
    p[0]=std::numeric_limits<float>::quiet_NaN();out=decodeModelOutput(raw,image,bgr.size(),c,1000000);
    assert(out.size()==3&&out[0].label=="dangerous_object");
    bool failed=false;try{decodeModelOutput(cv::Mat(8,4,CV_32F),image,bgr.size(),c,1);}catch(...){failed=true;}assert(failed);
    int old[]={1,11,4};failed=false;try{decodeModelOutput(cv::Mat(3,old,CV_32F,cv::Scalar(0)),image,bgr.size(),c,1);}catch(...){failed=true;}
    assert(failed); // old 7-class tensor no longer accepted
    c.require_instance_masks=true;failed=false;try{decodeModelOutput(raw,image,bgr.size(),c,1);}catch(...){failed=true;}assert(failed);
}
// Fill one anchor of a [1,18,N] pose tensor; kpts in model pixels.
void setAnchor(cv::Mat &t,int i,int cls,float score,const std::array<cv::Point2f,4> &k,const std::array<float,4> &v) {
    const int n=t.size[2];float *p=t.ptr<float>();
    p[0*n+i]=320;p[1*n+i]=320;p[2*n+i]=200;p[3*n+i]=100;p[(4+cls)*n+i]=score;
    for(int j=0;j<4;++j){p[(6+3*j)*n+i]=k[j].x;p[(7+3*j)*n+i]=k[j].y;p[(8+3*j)*n+i]=v[j];}
}
void poseTests() {
    cv::Mat bgr(720,1280,CV_8UC3,cv::Scalar(0));auto image=prepareModelImage(bgr,640);
    assert(image.pad_y==140&&std::abs(image.sx-.5f)<1e-6);
    int dims[]={1,kZonePoseChannels,6};
    assert(kZonePoseChannels==18);
    cv::Mat raw(3,dims,CV_32F,cv::Scalar(0));
    // Model pixel (x,y) -> original ((x-0)/.5, (y-140)/.5).
    // Left half: far_left, near_left, far_right(divider), near_right(divider).
    setAnchor(raw,0,0,.80f,{{{100,240},{60,340},{300,240},{298,340}}},{.9f,.9f,.9f,.9f});
    setAnchor(raw,1,0,.60f,{{{0,0},{0,0},{0,0},{0,0}}},{.9f,.9f,.9f,.9f}); // weaker duplicate left
    setAnchor(raw,2,1,.85f,{{{302,240},{302,340},{520,240},{580,340}}},{.9f,.9f,.2f,.9f}); // right far_right hidden
    setAnchor(raw,3,1,.10f,{{{0,0},{0,0},{0,0},{0,0}}},{1,1,1,1}); // below threshold
    auto halves=decodeZonePose(raw,image,bgr.size(),.25f);
    assert(halves[0].valid&&halves[1].valid&&std::abs(halves[0].score-.8f)<1e-6&&std::abs(halves[1].score-.85f)<1e-6);
    assert(cv::norm(halves[0].keypoints[1]-cv::Point2f(120,400))<1e-3);
    assert(cv::norm(halves[0].box.tl()-cv::Point2f(440,260))<1e-3);
    auto points=zoneHalvesToKeypoints(halves,.5f);
    std::map<int,ZoneKeypoint> by;for(auto &k:points)by[k.id]=k;
    assert(points.size()==5&&!by.count(5)); // hidden rear_right is omitted, never synthesized
    assert(cv::norm(by[0].pixel-cv::Point2f(120,400))<1e-3);   // L.near_left
    assert(cv::norm(by[1].pixel-cv::Point2f(600,400))<1e-3);   // mean(L.near_right 596, R.near_left 604)
    assert(cv::norm(by[2].pixel-cv::Point2f(1160,400))<1e-3);  // R.near_right
    assert(cv::norm(by[3].pixel-cv::Point2f(200,200))<1e-3);   // L.far_left
    assert(cv::norm(by[4].pixel-cv::Point2f(602,200))<1e-3);   // mean(L.far_right 600, R.far_left 604)
    for(auto &k:points)assert(k.visible&&k.confidence>=.5f);
    // Missing right half: no divider and no right corners.
    auto only_left=halves;only_left[1].valid=false;points=zoneHalvesToKeypoints(only_left,.5f);
    assert(points.size()==2&&points[0].id==0&&points[1].id==3);
    // Disagreeing divider corners (> 25% of half width apart) are dropped, not averaged.
    auto split=halves;split[1].keypoints[1]+=cv::Point2f(200,0);points=zoneHalvesToKeypoints(split,.5f);
    for(auto &k:points)assert(k.id!=1);
    // Nothing above threshold -> empty, still a valid call.
    assert(!decodeZonePose(raw,image,bgr.size(),.95f)[0].valid);
    bool failed=false;int bad[]={1,17,6};
    try{decodeZonePose(cv::Mat(3,bad,CV_32F,cv::Scalar(0)),image,bgr.size(),.25f);}catch(...){failed=true;}assert(failed);
    raw.ptr<float>()[0]=std::numeric_limits<float>::quiet_NaN(); // anchor 0 non-finite -> next-best left
    halves=decodeZonePose(raw,image,bgr.size(),.25f);assert(halves[0].valid&&std::abs(halves[0].score-.6f)<1e-6);
}
}
int main(){
    detectorTests();poseTests();
    std::cout<<"Vision tensor, RGB, class mapping, NMS, coordinates, pose decode and fail-closed adapter passed\n";
}
