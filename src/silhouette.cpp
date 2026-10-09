#include "rescue/silhouette.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace rescue {

float orangeSilhouetteAspect(const cv::Mat& bgr,const cv::Rect& box) {
    if(bgr.empty()||bgr.type()!=CV_8UC3)return -1.f;
    const cv::Rect roi=box&cv::Rect(0,0,bgr.cols,bgr.rows);
    if(roi.width<8||roi.height<8)return -1.f;
    cv::Mat hsv,mask;
    cv::cvtColor(bgr(roi),hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(3,120,90),cv::Scalar(16,255,255),mask);
    const int k=std::min(roi.width,roi.height)<40?3:5;
    cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,{k,k}));
    cv::Mat labels,stats,centroids;
    const int n=cv::connectedComponentsWithStats(mask,labels,stats,centroids);
    int best=-1,best_area=0;
    for(int i=1;i<n;++i) {
        const int a=stats.at<int>(i,cv::CC_STAT_AREA);
        if(a>best_area){best_area=a;best=i;}
    }
    if(best<0||best_area<30||best_area<roi.area()*.15)return -1.f;
    const int w=stats.at<int>(best,cv::CC_STAT_WIDTH),h=stats.at<int>(best,cv::CC_STAT_HEIGHT);
    return w>0?float(h)/float(w):-1.f;
}

float silhouetteAdjustedDiff(float pose_diff,float aspect,const SilhouetteRule& rule) {
    if(!(aspect>0)||!std::isfinite(aspect))return pose_diff;
    if(aspect<rule.lying_below)return std::min(pose_diff,rule.lying_diff);
    if(aspect>rule.upright_above)return std::max(pose_diff,rule.upright_diff);
    return pose_diff;
}

} // namespace rescue
