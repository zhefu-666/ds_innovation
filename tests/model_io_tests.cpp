#include "rescue/model_io.hpp"
#include "rescue/perception_adapter.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace rescue;
int main(){
    Config c;c.confidence=0.25f;
    cv::Mat bgr(480,640,CV_8UC3,cv::Scalar(10,20,30));auto image=prepareModelImage(bgr,448);
    assert(image.pad_y==56&&image.pad_x==0);
    assert(image.rgb.at<cv::Vec3b>(100,100)==cv::Vec3b(30,20,10));
    assert(image.rgb.at<cv::Vec3b>(0,0)==cv::Vec3b(114,114,114));
    int dims[]={1,11,4};cv::Mat raw(3,dims,CV_32F,cv::Scalar(0));float *p=raw.ptr<float>();
    for(int i=0;i<4;++i){p[i]=224;p[4+i]=224;p[8+i]=140;p[12+i]=70;}
    p[8*4]=.9f; // normal index 4
    p[8*4+1]=.8f; // overlapping normal suppressed
    p[7*4+2]=.85f; // dangerous index 3, overlapping but retained
    p[9*4+3]=.7f; // unresolved main retained as diagnostic
    auto out=decodeModelOutput(raw,image,bgr.size(),c,1000000);
    assert(out.size()==3&&out[0].label=="ordinary_supply"&&out[0].class_id==4&&out[0].model_label=="normal");
    assert(out[0].box==cv::Rect(220,190,200,100));
    assert(out[1].label=="dangerous_object"&&out[2].label=="unmapped_main"&&out[0].mask.empty());
    out[0].track_id=1;out[1].track_id=2;out[2].track_id=3;
    auto in=makePushObservation(out,1050000,false,.25f);
    assert(in.target_valid&&in.label=="ordinary_supply"&&!in.geometry_valid&&!in.safety_ok&&!in.zone_valid&&!in.fully_inside);
    assert(!makePushObservation(out,1300000,false,.25f).target_valid);
    assert(taskLabel("core")=="core_supply"&&taskLabel("wounded")=="injured_person");
    assert(taskLabel("red")=="red_safe_zone"&&taskLabel("blue")=="blue_safe_zone");
    p[0]=std::numeric_limits<float>::quiet_NaN();out=decodeModelOutput(raw,image,bgr.size(),c,1000000);
    assert(out.size()==3&&out[0].label=="dangerous_object");
    bool failed=false;try{decodeModelOutput(cv::Mat(11,4,CV_32F),image,bgr.size(),c,1);}catch(...){failed=true;}assert(failed);
    c.require_instance_masks=true;failed=false;try{decodeModelOutput(raw,image,bgr.size(),c,1);}catch(...){failed=true;}assert(failed);
    std::cout<<"Vision tensor, RGB, class mapping, NMS, coordinates and fail-closed adapter passed\n";
}
