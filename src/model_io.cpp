#include "rescue/model_io.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
namespace rescue {
std::string taskLabel(const std::string &s) {
    static const std::map<std::string,std::string> names{
        {"core","core_supply"},{"wounded","injured_person"},{"red","red_safe_zone"},
        {"dangerous","dangerous_object"},{"normal","ordinary_supply"},
        {"main","unmapped_main"},{"blue","blue_safe_zone"}};
    auto it=names.find(s);
    return it==names.end()?"unmapped_"+s:it->second;
}
ModelImage prepareModelImage(const cv::Mat &bgr,int size) {
    if(bgr.empty() || bgr.type()!=CV_8UC3 || size<=0) throw std::runtime_error("Expected nonempty BGR uint8 image");
    ModelImage out;
    float scale=std::min(float(size)/bgr.cols,float(size)/bgr.rows);
    int w=std::max(1,int(std::round(bgr.cols*scale))),h=std::max(1,int(std::round(bgr.rows*scale)));
    out.sx=float(w)/bgr.cols;out.sy=float(h)/bgr.rows;
    out.pad_x=(size-w)/2;out.pad_y=(size-h)/2;
    cv::Mat resized;cv::resize(bgr,resized,{w,h},0,0,cv::INTER_LINEAR);
    cv::Mat canvas(size,size,CV_8UC3,cv::Scalar(114,114,114));
    resized.copyTo(canvas(cv::Rect(out.pad_x,out.pad_y,w,h)));
    cv::cvtColor(canvas,out.rgb,cv::COLOR_BGR2RGB);
    return out;
}
std::vector<SegDetection> decodeModelOutput(const cv::Mat &output,const ModelImage &image,
        cv::Size original,const Config &config,uint64_t timestamp) {
    if(config.require_instance_masks) throw std::runtime_error("Current model is detection-only; instance masks unavailable");
    const int attrs=4+int(config.class_names.size());
    if(output.type()!=CV_32F || output.dims!=3 || output.size[0]!=1 || output.size[1]!=attrs || !output.isContinuous())
        throw std::runtime_error("Expected float32 [1,4+classes,N] output with no objectness channel");
    if(!std::isfinite(config.confidence)||config.confidence<0||config.confidence>1||
       !std::isfinite(config.nms)||config.nms<0||config.nms>1)
        throw std::runtime_error("Invalid detection thresholds");
    const int count=output.size[2];const float *raw=output.ptr<float>();
    std::vector<SegDetection> candidates,results;
    std::map<int,std::vector<int>> groups;
    for(int i=0;i<count;++i) {
        bool finite=true;for(int k=0;k<attrs;++k)finite=finite&&std::isfinite(raw[k*count+i]);
        if(!finite)continue;
        int cls=0;float score=raw[4*count+i];
        for(int c=1;c<attrs-4;++c)if(raw[(4+c)*count+i]>score){score=raw[(4+c)*count+i];cls=c;}
        if(score<config.confidence || score>1)continue;
        float cx=raw[i],cy=raw[count+i],w=raw[2*count+i],h=raw[3*count+i];
        if(w<=0||h<=0)continue;
        // Contract: coordinates are model-input pixels, never guessed from magnitude.
        float l=std::clamp((cx-w/2-image.pad_x)/image.sx,0.f,float(original.width));
        float t=std::clamp((cy-h/2-image.pad_y)/image.sy,0.f,float(original.height));
        float r=std::clamp((cx+w/2-image.pad_x)/image.sx,0.f,float(original.width));
        float b=std::clamp((cy+h/2-image.pad_y)/image.sy,0.f,float(original.height));
        if(r<=l||b<=t)continue;
        SegDetection d;d.class_id=cls;d.model_label=config.class_names[cls];d.label=taskLabel(d.model_label);
        d.confidence=score;d.timestamp_us=timestamp;
        d.box=cv::Rect(cv::Point(int(std::floor(l)),int(std::floor(t))),cv::Point(int(std::ceil(r)),int(std::ceil(b))));
        groups[cls].push_back(int(candidates.size()));candidates.push_back(d);
    }
    for(const auto &group:groups) {
        std::vector<cv::Rect> boxes;std::vector<float> scores;std::vector<int> keep;
        for(int i:group.second){boxes.push_back(candidates[i].box);scores.push_back(candidates[i].confidence);}
        cv::dnn::NMSBoxes(boxes,scores,config.confidence,config.nms,keep);
        for(int i:keep)results.push_back(candidates[group.second[i]]);
    }
    std::sort(results.begin(),results.end(),[](const auto &a,const auto &b){return a.confidence>b.confidence;});
    return results;
}
}
