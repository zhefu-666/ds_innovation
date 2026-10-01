#include "rescue/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace rescue {

LocalPlanner::LocalPlanner(PlannerConfig config) : config_(config) {}

cv::Point2f LocalPlanner::approachPoint(const cv::Point2f &target, const cv::Point2f &goal) const {
    const cv::Point2f delta = goal - target;
    const float distance = std::sqrt(delta.dot(delta));
    if (distance < 1e-4f) return target;
    return target - delta * (config_.approach_distance_m / distance);
}

bool LocalPlanner::segmentSafe(const cv::Point2f &a, const cv::Point2f &b,
                               const std::vector<PlannerObstacle> &obstacles) const {
    const cv::Point2f segment = b - a;
    const float length_sq = segment.dot(segment);
    for (const auto &obstacle : obstacles) {
        const float radius = obstacle.radius_m + config_.robot_radius_m +
                             (obstacle.dangerous ? config_.danger_inflation_m : 0.0f);
        const float t = length_sq > 1e-6f ?
            std::clamp((obstacle.center_m - a).dot(segment) / length_sq, 0.0f, 1.0f) : 0.0f;
        const cv::Point2f closest = a + segment * t;
        if (cv::norm(closest - obstacle.center_m) <= radius) return false;
    }
    return true;
}

std::vector<PlannedRoute> LocalPlanner::plan(const cv::Point2f &robot, const cv::Point2f &target,
                                             const cv::Point2f &goal,
                                             const std::vector<PlannerObstacle> &obstacles) const {
    std::vector<PlannedRoute> routes;
    const cv::Point2f approach = approachPoint(target, goal);
    const std::array<std::pair<std::string, std::vector<cv::Point2f>>, 3> candidates{{
        {"direct", {robot, approach}},
        {"left_detour", {robot, robot + cv::Point2f(0.0f, 1.0f), approach}},
        {"right_detour", {robot, robot + cv::Point2f(0.0f, -1.0f), approach}},
    }};
    for (const auto &candidate : candidates) {
        bool safe = true;
        float cost = 0.0f;
        for (size_t i = 1; i < candidate.second.size(); ++i) {
            if (!segmentSafe(candidate.second[i - 1], candidate.second[i], obstacles)) {
                safe = false;
                break;
            }
            cost += cv::norm(candidate.second[i] - candidate.second[i - 1]);
        }
        if (safe) routes.push_back(PlannedRoute{true, candidate.first, candidate.second, cost});
    }
    std::sort(routes.begin(), routes.end(), [](const PlannedRoute &a, const PlannedRoute &b) {
        return a.cost < b.cost;
    });
    return routes;
}

} // namespace rescue

// Carry planning is independent of image detection. Its caller supplies validated,
// timestamped metric occupancy and measured swept radius, or receives no route.
namespace rescue {
namespace {
bool finitePoint(const cv::Point2f& p){return std::isfinite(p.x)&&std::isfinite(p.y);}
bool usableScene(const NavigationScene& s,uint64_t now) {
    if(!s.complete || !s.opponent_region_known || !s.timestamp_us || now<s.timestamp_us || now-s.timestamp_us>200000 ||
       !std::isfinite(s.swept_radius_m) || s.swept_radius_m<=0 || s.known_region.size()<3 || s.known_region.size()>32 || s.obstacles.size()>64)return false;
    for(const auto& p:s.known_region)if(!finitePoint(p))return false;
    if(!cv::isContourConvex(s.known_region) || std::abs(cv::contourArea(s.known_region))<1e-4)return false;
    for(const auto& o:s.obstacles)if(!finitePoint(o.center_m)||!std::isfinite(o.radius_m)||o.radius_m<0)return false;
    return true;
}
float pointSegmentDistance(cv::Point2f p,cv::Point2f a,cv::Point2f b){
    const auto d=b-a;const float l=d.dot(d);
    return cv::norm(p-(a+d*(l>0?std::clamp((p-a).dot(d)/l,0.f,1.f):0.f)));
}
}
bool LocalPlanner::sweptSegmentSafe(const cv::Point2f& a,const cv::Point2f& b,
                                   const NavigationScene& s,uint64_t now) const {
    if(!usableScene(s,now)||!finitePoint(a)||!finitePoint(b))return false;
    // In a convex polygon, the segment between two contained discs is contained too.
    const float radius=s.swept_radius_m+.01f;
    if(cv::pointPolygonTest(s.known_region,a,true)<=radius || cv::pointPolygonTest(s.known_region,b,true)<=radius)return false;
    for(const auto& o:s.obstacles)
        if(pointSegmentDistance(o.center_m,a,b)<=radius+o.radius_m+(o.dangerous?config_.danger_inflation_m:0.f))return false;
    return true;
}
PlannedRoute LocalPlanner::planCarry(const cv::Point2f& goal,const NavigationScene& s,uint64_t now) const {
    PlannedRoute out;
    if(!usableScene(s,now)||!finitePoint(goal))return out;
    if(!sweptSegmentSafe({0,0},{0,0},s,now)||!sweptSegmentSafe(goal,goal,s,now))return out;
    if(sweptSegmentSafe({0,0},goal,s,now))return {true,"carry_direct",{{0,0},goal},float(cv::norm(goal))};
    if(s.obstacles.size()>16)return out; // bounded graph; dense scene requires another route producer
    std::vector<cv::Point2f> points{{0,0},goal};
    // Circumscribed rings create candidates around metric obstacles, independent of
    // chassis orientation. Every chosen edge is checked again; graph failure stops.
    for(const auto& o:s.obstacles) {
        const float r=(o.radius_m+s.swept_radius_m+.025f+(o.dangerous?config_.danger_inflation_m:0.f))/std::cos(float(CV_PI)/12);
        for(int i=0;i<12;++i) {
            const float angle=i*float(CV_PI)/6;
            const auto p=o.center_m+cv::Point2f(std::cos(angle)*r,std::sin(angle)*r);
            if(sweptSegmentSafe(p,p,s,now))points.push_back(p);
        }
    }
    const size_t n=points.size();std::vector<float> distance(n,std::numeric_limits<float>::infinity());
    std::vector<int> parent(n,-1);std::vector<bool> used(n,false);distance[0]=0;
    for(size_t iteration=0;iteration<n;++iteration) {
        int at=-1;for(size_t i=0;i<n;++i)if(!used[i] && (at<0||distance[i]<distance[size_t(at)]))at=int(i);
        if(at<0||!std::isfinite(distance[size_t(at)]))break;
        if(at==1)break;
        used[size_t(at)]=true;
        for(size_t i=0;i<n;++i)if(!used[i]) {
            const float candidate=distance[size_t(at)]+cv::norm(points[i]-points[size_t(at)]);
            if(candidate<distance[i] && sweptSegmentSafe(points[size_t(at)],points[i],s,now)) {
                distance[i]=candidate;parent[i]=at;
            }
        }
    }
    if(!std::isfinite(distance[1]))return out;
    for(int at=1;at>=0;at=parent[size_t(at)])out.waypoints_m.push_back(points[size_t(at)]);
    std::reverse(out.waypoints_m.begin(),out.waypoints_m.end());
    out.valid=true;out.kind=out.waypoints_m.size()==2?"carry_direct":"carry_detour";out.cost=distance[1];return out;
}
DropPlan DropPlanner::plan(const ZoneGeometry& g,const ZoneEstimate& z,bool identity,bool complete,
                           const std::vector<PlannerObstacle>& occupied,bool injured,float radius,uint64_t now) const {
    DropPlan out;
    if(!identity||!complete||!g.valid()||!z.trusted(now)||z.zone_label!=g.label||z.geometry_id!=g.id||
       !std::isfinite(radius)||radius<=0)return out;
    for(const auto& o:occupied)if(!finitePoint(o.center_m)||!std::isfinite(o.radius_m)||o.radius_m<0)return out;
    const bool left=injured?!g.supply_left:g.supply_left;
    const float sign=left?-1.f:1.f;
    float best=std::numeric_limits<float>::infinity();
    for(float y=.02f;y<g.depth_m;y+=.02f)for(float x=.02f;x<g.width_m/2;x+=.02f) {
        const cv::Point2f candidate{sign*x,y};
        const float margin=radius+.01f+2*z.position_sigma_m+2*cv::norm(candidate)*z.yaw_sigma_rad;
        if(x<=g.divider_exclusion_half_width_m+margin||x>=g.width_m/2-margin||y<=margin||y>=g.depth_m-margin)continue;
        bool clear=true;for(const auto& o:occupied)if(cv::norm(candidate-o.center_m)<=margin+o.radius_m) {clear=false;break;}
        if(!clear)continue;
        // Fill the rear first to preserve the entry; prefer each half's centreline.
        const float cost=(g.depth_m-y)+.2f*std::abs(x-g.width_m/4);
        if(cost<best){best=cost;out.valid=true;out.centre_zone_m=candidate;out.radius_m=radius;out.reason="ok";}
    }
    if(!out.valid)out.reason="no_clear_drop_position";
    return out;
}
} // namespace rescue
