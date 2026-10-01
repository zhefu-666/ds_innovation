#include "rescue/safe_zone_pose.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
namespace rescue {
namespace {
bool finitePose(const cv::Mat& m){return !m.empty()&&m.total()==3&&cv::checkRange(m);}
double rotationDistance(const cv::Mat& a,const cv::Mat& b) {
    cv::Mat A,B;cv::Rodrigues(a,A);cv::Rodrigues(b,B);
    return std::acos(std::clamp((cv::trace(A.t()*B)[0]-1)/2,-1.0,1.0));
}
}
SafeZonePoseEstimator::SafeZonePoseEstimator(cv::Mat k,cv::Mat d,float error,float jump)
    :camera_matrix_(std::move(k)),dist_coeffs_(std::move(d)),max_error_px_(error),max_jump_m_(jump){}
void SafeZonePoseEstimator::reset(){previous_={};pending_={};previous_us_=last_us_=pending_us_=0;pending_count_=0;}
SafeZonePose SafeZonePoseEstimator::estimate(const SafeZoneObservation& o,uint64_t now) {
    SafeZonePose result;result.label=o.label;
    const auto fail=[&](const char* why,bool keep_pending=false){
        if(!keep_pending) pending_count_=0;
        result.valid=false;result.reason=why;result.view=SafeZonePose::UNKNOWN;return result;
    };
    if(!now)now=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if(last_us_&&now<=last_us_)return fail("non_monotonic_frame");
    last_us_=now;
    if(previous_.label!=o.label){previous_={};pending_count_=0;}
    if(camera_matrix_.rows!=3||camera_matrix_.cols!=3||!cv::checkRange(camera_matrix_)||
       o.image_points.size()<4||o.image_points.size()!=o.object_points.size())return fail("insufficient_points");
    for(size_t i=0;i<o.object_points.size();++i) {
        const auto& a=o.object_points[i];const auto& b=o.image_points[i];
        if(!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||std::abs(a.z)>1e-5||
           !std::isfinite(b.x)||!std::isfinite(b.y))return fail("invalid_planar_points");
        for(size_t j=0;j<i;++j)if(cv::norm(a-o.object_points[j])<1e-5||cv::norm(b-o.image_points[j])<1e-3)
            return fail("duplicate_points");
    }
    double area=0;
    for(size_t i=1;i<o.object_points.size();++i)for(size_t j=i+1;j<o.object_points.size();++j) {
        auto a=o.object_points[i]-o.object_points[0],b=o.object_points[j]-o.object_points[0];
        area=std::max(area,std::abs(double(a.x*b.y-a.y*b.x)));
    }
    if(area<1e-4)return fail("collinear_points");
    camera_matrix_.convertTo(camera_matrix_,CV_64F);
    if(!dist_coeffs_.empty())dist_coeffs_.convertTo(dist_coeffs_,CV_64F);
    std::vector<cv::Mat> rv,tv;
    try {if(!cv::solvePnPGeneric(o.object_points,o.image_points,camera_matrix_,dist_coeffs_,rv,tv,false,cv::SOLVEPNP_IPPE))
        return fail("ippe_failed");}
    catch(const cv::Exception&){return fail("ippe_failed");}
    const bool reference=finitePose(o.reference_rvec)&&finitePose(o.reference_tvec);
    const bool recent=previous_.valid&&now>=previous_us_&&now-previous_us_<=350000;
    struct Candidate {int index;double rms,score;};std::vector<Candidate> candidates;
    for(size_t i=0;i<rv.size();++i) {
        if(!finitePose(rv[i])||!finitePose(tv[i]))continue;
        cv::Mat R;cv::Rodrigues(rv[i],R);bool positive=true;
        for(const auto& p:o.object_points)if(R.at<double>(2,0)*p.x+R.at<double>(2,1)*p.y+tv[i].at<double>(2)<=0)positive=false;
        if(!positive)continue;
        std::vector<cv::Point2f> projected;cv::projectPoints(o.object_points,rv[i],tv[i],camera_matrix_,dist_coeffs_,projected);
        double sum=0;for(size_t j=0;j<projected.size();++j){double e=cv::norm(projected[j]-o.image_points[j]);sum+=e*e;}
        const double rms=std::sqrt(sum/projected.size());
        if(!std::isfinite(rms)||rms>max_error_px_)continue;
        double score=rms;
        if(reference)score+=5*cv::norm(tv[i]-o.reference_tvec)+2*rotationDistance(rv[i],o.reference_rvec);
        else if(recent)score+=cv::norm(tv[i]-previous_.tvec)+rotationDistance(rv[i],previous_.rvec);
        candidates.push_back({int(i),rms,score});
    }
    if(candidates.empty())return fail("ippe_reprojection_rejected");
    std::sort(candidates.begin(),candidates.end(),[](const auto&a,const auto&b){return a.score<b.score;});
    if(!reference&&!recent&&candidates.size()>1&&std::abs(candidates[0].rms-candidates[1].rms)<.1&&
       rotationDistance(rv[candidates[0].index],rv[candidates[1].index])>.15)return fail("ippe_ambiguous");
    const auto best=candidates.front();result.rvec=rv[best.index];result.tvec=tv[best.index];
    result.reprojection_error_px=best.rms;result.distance_m=cv::norm(result.tvec);
    result.heading_error_deg=std::atan2(result.tvec.at<double>(0),result.tvec.at<double>(2))*180/CV_PI;
    cv::Mat R;cv::Rodrigues(result.rvec,R);
    const double normal_angle=std::acos(std::clamp(std::abs(R.at<double>(2,2)),0.0,1.0))*180/CV_PI;
    result.view=normal_angle<15?SafeZonePose::FRONT:normal_angle<55?SafeZonePose::OBLIQUE:SafeZonePose::SIDE;
    // view and divider visibility are diagnostics only. Planar IPPE is a cross-check,
    // not the authority granting motion or choosing a subzone.
    if(previous_.valid && (!recent || cv::norm(result.tvec-previous_.tvec)>max_jump_m_ ||
                           rotationDistance(result.rvec,previous_.rvec)>.35)) {
        const bool consistent=pending_count_&&now>pending_us_&&now-pending_us_<=350000&&
            cv::norm(result.tvec-pending_.tvec)<.04&&rotationDistance(result.rvec,pending_.rvec)<.08;
        pending_count_=consistent?pending_count_+1:1;pending_=result;pending_us_=now;
        if(pending_count_<3)return fail("ippe_reacquiring",true);
    }
    result.valid=true;result.reason="ok";previous_=result;previous_us_=now;pending_count_=0;
    return result;
}
} // namespace rescue
