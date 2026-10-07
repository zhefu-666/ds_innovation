#include "rescue/controlled_field.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
namespace rescue {
void ControlledField::invalidate(const std::string& why) {
    reason_=why;stable_frames_=0;stable_since_=last_frame_=0;last_objects_.clear();
    last_supply_=last_injured_=-1;
}
void ControlledField::update(PushObservation& in,const std::vector<SegDetection>& detections,
        const ZoneHalves& halves,cv::Size size,const PushOutput& previous,bool mapping,bool ignore_clearance) {
    in.clearance_checks_disabled=ignore_clearance;
    // Provenance: the user's explicit controlled test-area attestation. These are
    // not inferred from empty detection output and do not apply outside this mode.
    in.opponent_zone_clear=true;
    in.path_safe=in.retreat_safe=mapping && std::isfinite(body_radius_) && body_radius_>0;
    in.directional_clearance_valid=envelope_[0]>0;
    in.turn_safe=in.arc_safe=in.jaw_open_safe=in.path_safe;
    // Forward checks the full straight approach to the target, using its current bearing depth.
    const float forward=in.geometry_valid?std::max(0.f,in.distance_m*std::cos(in.heading_error)-envelope_[5]):0.f;
    const auto intersects=[&](const cv::Point2f& q,float radius,float front,float rear,float left,float right) {
        const float dx=std::max({-left-q.x,0.f,q.x-right});
        const float dy=std::max({-rear-q.y,0.f,q.y-front});
        return std::hypot(dx,dy)<=radius;
    };
    for(const auto& d:detections) if(d.label=="dangerous_object" || targetKind(d.label)==TargetKind::UNKNOWN) {
        if(!d.ground_position_valid || !std::isfinite(d.body_xy_m.x)||!std::isfinite(d.body_xy_m.y)||
           targetKind(d.label)==TargetKind::UNKNOWN) {
            in.path_safe=in.retreat_safe=in.turn_safe=in.arc_safe=in.jaw_open_safe=false;continue;
        }
        if(!in.directional_clearance_valid) {
            if(cv::norm(d.body_xy_m)<=body_radius_)in.path_safe=in.retreat_safe=false;
            continue;
        }
        // Blue is a 4cm cube. The bottom projection is a footprint edge, not its centre;
        // a 4cm diagonal about that edge encloses the footprint (not a protection margin).
        const float radius=std::sqrt(2.f)*.04f;
        const float front=std::max(envelope_[0],envelope_[5]);
        const float left=std::max(envelope_[2],envelope_[4]/2),right=std::max(envelope_[3],envelope_[4]/2);
        if(intersects(d.body_xy_m,radius,front+forward,envelope_[1],left,right))in.path_safe=false;
        if(intersects(d.body_xy_m,radius,front,envelope_[1]+.2f,left,right))in.retreat_safe=false;
        if(intersects(d.body_xy_m,radius,front,envelope_[1],left,right))in.jaw_open_safe=false;
        if(cv::norm(d.body_xy_m)<=body_radius_+radius)in.turn_safe=false;
        // A curved centre path may leave the straight corridor. Bound all headings
        // conservatively by the full planned travel length about the current centre.
        if(cv::norm(d.body_xy_m)<=body_radius_+radius+forward)in.arc_safe=false;
    }
    // Explicit operator contact-test permission; never claim this is sensed clearance.
    if(ignore_clearance)
        in.path_safe=in.retreat_safe=in.turn_safe=in.arc_safe=in.jaw_open_safe=true;
    // Short pushing is a separate, explicitly authorized contact operation.
    // Require an observed own-zone boundary, never infer it from an empty image.
    in.clear_push_safe=mapping && in.directional_clearance_valid && in.zone_own &&
        in.zone_identity_verified && in.zone_estimate.trusted(in.now_us) &&
        in.zone_estimate.source!=ZoneEstimate::Source::PREDICTED;
    if(in.clear_push_safe) {
        const auto& z=in.zone_estimate;
        float xmin=1e9f,xmax=-1e9f,ymin=1e9f,ymax=-1e9f;
        for(float x:{-envelope_[2]-.10f,envelope_[3]+.10f})
            for(float y:{-envelope_[1]-.10f,envelope_[0]+.22f+.10f}) {
                const auto q=z.bodyToZone({x,y});xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);
                ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);
            }
        if(xmax>=-geometry_.width_m/2 && xmin<=geometry_.width_m/2 && ymax>=0 && ymin<=geometry_.depth_m)
            in.clear_push_safe=false;
        bool contact=false;
        for(const auto& d:detections) {
            if(targetKind(d.label)==TargetKind::UNKNOWN || !d.ground_position_valid || !d.ground_contact_valid ||
               !d.timestamp_us || in.now_us<d.timestamp_us || in.now_us-d.timestamp_us>200000) {in.clear_push_safe=false;break;}
            const auto q=d.body_xy_m;
            const bool frontal=q.y>=envelope_[0]-.02f && q.y<=envelope_[0]+.05f &&
                std::abs(q.x)<=envelope_[4]/2;
            if(frontal)contact=true;
            if(d.label=="dangerous_object" && !frontal &&
               intersects(q,std::sqrt(2.f)*.04f,envelope_[0]+.22f,envelope_[1],envelope_[2],envelope_[3]))
                in.clear_push_safe=false;
        }
        in.clear_push_safe=in.clear_push_safe&&contact;
    }
    if (!previous.first_ordinary_delivered && !in.target_region_valid && in.geometry_valid) {
        // Initial objects are explicitly placed outside the only zone. Once a trip
        // was delivered, unseen-zone candidates stay unknown to prevent re-picking.
        in.target_region_valid=true;in.target_in_zone=false;
    }
    in.zone_counts_valid=in.zone_inventory_complete=false;in.zone_occupied.clear();
    if (!mapping || !in.zone_own || !in.zone_identity_verified ||
        !in.zone_estimate.trusted(in.now_us) ||
        in.zone_estimate.source==ZoneEstimate::Source::PREDICTED) {
        invalidate("zone_not_observed");return;
    }
    std::vector<std::vector<cv::Point2f>> polygons;
    for (const auto& half:halves) {
        if (!half.valid) {invalidate("both_halves_required");return;}
        std::vector<cv::Point2f> polygon;
        for(int i:{0,2,3,1}) {
            const auto p=half.keypoints[i];
            if(half.confidence[i]<.6f || !std::isfinite(p.x)||!std::isfinite(p.y)||
               p.x<8||p.y<8||p.x>=size.width-8||p.y>=size.height-8) {
                invalidate("whole_zone_not_visible");return;
            }
            polygon.push_back(p);
        }
        if(!cv::isContourConvex(polygon)||std::abs(cv::contourArea(polygon))<400) {
            invalidate("zone_too_small_or_invalid");return;
        }
        polygons.push_back(polygon);
    }
    int supplies=0,injured=0;
    std::vector<PlannerObstacle> objects;
    std::vector<cv::Rect> boxes;
    for(const auto& d:detections) {
        bool overlaps=false;
        const cv::Rect2f box(d.box);
        std::vector<cv::Point2f> rectangle{box.tl(),{box.x+box.width,box.y},box.br(),{box.x,box.y+box.height}};
        for(const auto& polygon:polygons) {
            std::vector<cv::Point2f> intersection;
            if(cv::intersectConvexConvex(rectangle,polygon,intersection)>0) overlaps=true;
        }
        if (!overlaps) continue;
        if (!d.ground_contact_valid || !d.ground_position_valid || d.frame_id!=in.zone_estimate.frame_id ||
            !d.timestamp_us || in.now_us<d.timestamp_us || in.now_us-d.timestamp_us>200000) {
            invalidate("occluded_or_unmapped_zone_object");return;
        }
        const auto q=in.zone_estimate.bodyToZone(d.body_xy_m);
        const bool supply=d.label=="ordinary_supply"||d.label=="core_supply";
        const bool person=d.label=="injured_person";
        if (!supply&&!person) {invalidate("unexpected_zone_object");return;}
        // Ground projection estimates the near footprint edge. Its conservative
        // radius includes the offset to the actual centre and full class footprint.
        const float radius=person?.065f:.05f;
        const float margin=radius+2*in.zone_estimate.position_sigma_m+
            2*cv::norm(q)*in.zone_estimate.yaw_sigma_rad;
        const bool fully_inside=std::abs(q.x)<geometry_.width_m/2-margin &&
            q.y>margin && q.y<geometry_.depth_m-margin;
        if (!fully_inside) {invalidate("zone_boundary_object");return;}
        const bool left=supply?geometry_.supply_left:!geometry_.supply_left;
        if (!(left?q.x < -geometry_.divider_exclusion_half_width_m-margin:
                    q.x > geometry_.divider_exclusion_half_width_m+margin)) {
            invalidate("wrong_half_or_divider");return;
        }
        for(const auto& prior:boxes) if((prior&d.box).area()>0) {
            invalidate("overlapping_zone_objects");return;
        }
        boxes.push_back(d.box);objects.push_back({q,radius,false});
        if(supply)++supplies;else ++injured;
    }
    std::sort(objects.begin(),objects.end(),[](const auto&a,const auto&b){
        return a.center_m.x==b.center_m.x?a.center_m.y<b.center_m.y:a.center_m.x<b.center_m.x;
    });
    bool unchanged=supplies==last_supply_&&injured==last_injured_&&objects.size()==last_objects_.size();
    if(unchanged)for(size_t i=0;i<objects.size();++i)
        if(cv::norm(objects[i].center_m-last_objects_[i].center_m)>.015f)unchanged=false;
    const auto frame=in.zone_estimate.frame_id;
    if(frame==last_frame_)return;
    last_frame_=frame;
    if(!unchanged) {stable_frames_=0;stable_since_=in.now_us;last_objects_=objects;}
    if(!stable_since_)stable_since_=in.now_us;
    last_supply_=supplies;last_injured_=injured;++stable_frames_;
    if(stable_frames_<3 || in.now_us-stable_since_<200000) {reason_="inventory_settling";return;}
    in.zone_counts_valid=in.zone_inventory_complete=true;
    in.zone_supply_count=supplies;in.zone_injured_count=injured;in.zone_occupied=objects;
    reason_="controlled_visible_zone";
}
}
