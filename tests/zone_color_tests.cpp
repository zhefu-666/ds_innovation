#include "rescue/zone_color.hpp"
#include <cassert>
#include <iostream>
using namespace rescue;
int main(){
    ZoneHalves halves;
    for(int i=0;i<2;++i){auto& h=halves[i];h.valid=true;h.score=.9;
        const float x=10+i*100;h.keypoints={cv::Point2f(x,10),{x,100},{x+100,10},{x+100,100}};h.confidence={.9,.9,.9,.9};}
    cv::Mat frame(120,230,CV_8UC3,cv::Scalar(0,0,255));
    assert(!ZoneColorClassifier{}.classify(frame,halves,{},.5).verified);
    ZoneColorConfig c;c.measured=true;ZoneColorClassifier detector(c);
    auto r=detector.classify(frame,halves,{},.5);assert(r.verified&&r.color=="red");
    frame.setTo(cv::Scalar(255,0,0));r=detector.classify(frame,halves,{},.5);assert(r.verified&&r.color=="blue");
    frame(cv::Rect(110,0,120,120)).setTo(cv::Scalar(0,0,255));assert(!detector.classify(frame,halves,{},.5).verified);
    frame.setTo(cv::Scalar(100,100,100));assert(!detector.classify(frame,halves,{},.5).verified);
    frame.setTo(cv::Scalar(0,0,255));SegDetection object;object.box={0,0,230,120};
    assert(!detector.classify(frame,halves,{object},.5).verified);
    halves[1].valid=false;assert(!detector.classify(frame,halves,{},.5).verified);
    std::cout<<"Red/blue identity, incomplete halves, occlusion and ambiguous color passed\n";
}
