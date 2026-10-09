#include "rescue/geometry_pipeline.hpp"
#include "rescue/uart_controller.hpp"
#include "rescue/imu_adapter.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <limits>
#include <thread>
#include <chrono>
#include <iostream>
using namespace rescue;
struct Scene {
    CameraCalibration camera;
    ZoneGeometry geometry;
    GeometryFrame frame;
    cv::Mat K,D,R,t;
    explicit Scene(const char* geometry_path=ZONE_GEOMETRY_PATH) {
        geometry=ZoneGeometry::load(geometry_path,"red_safe_zone");
        K=(cv::Mat_<double>(3,3)<<800,0,640,0,800,360,0,0,1);D=cv::Mat::zeros(1,5,CV_64F);
        const double a=20*CV_PI/180;
        R=(cv::Mat_<double>(3,3)<<1,0,0,0,-std::sin(a),-std::cos(a),0,std::cos(a),-std::sin(a));
        const cv::Mat centre=(cv::Mat_<double>(3,1)<<0,.05,.30);t=-R*centre;
        cv::Mat T=cv::Mat::eye(4,4,CV_64F);R.copyTo(T(cv::Rect(0,0,3,3)));t.copyTo(T(cv::Rect(3,0,1,3)));
        const auto path=std::string("/tmp/stage01-camera-")+std::to_string(getpid())+".json";
        {cv::FileStorage f(path,cv::FileStorage::WRITE);f<<"camera_matrix"<<K<<"dist_coeffs"<<D
            <<"image_width"<<1280<<"image_height"<<720<<"ground_pixel_domain"<<"undistorted_pixels"
            <<"T_camera_from_robot"<<T<<"extrinsics_validated"<<1<<"pitch_model_reference_cdeg"<<2000
            <<"pitch_model_min_cdeg"<<1000<<"pitch_model_max_cdeg"<<2500;}
        assert(camera.load(path));std::remove(path.c_str());
        frame.frame_id=1;frame.capture_us=1000000;frame.now_us=1010000;frame.image_size={1280,720};
        frame.sensors.imu.imu_valid=true;frame.sensors.imu.timestamp_us=990000;
        frame.sensors.actuator.valid=true;frame.sensors.actuator.timestamp_us=990000;
        frame.sensors.actuator.camera_pitch_cdeg=2000;frame.sensors.pitch_stable=true;
    }
    void advance(uint64_t delta=50000) {
        ++frame.frame_id;frame.capture_us+=delta;frame.now_us=frame.capture_us+10000;
        frame.sensors.imu.timestamp_us=frame.sensors.actuator.timestamp_us=frame.capture_us-10000;
    }
    // Bounding box of a cuboid at body (x,y) centre, lifted off the floor and yawed (rad).
    cv::Rect cubeBox(cv::Point2f c,float size=.04f,float length=0,float lift=0,float yaw=0) const {
        if(length<=0)length=size;
        std::vector<cv::Point3f> body;const float cy=std::cos(yaw),sy=std::sin(yaw);
        for(float dx:{-length/2,length/2})for(float dy:{-size/2,size/2})for(float z:{0.f,size})
            body.emplace_back(c.x+cy*dx-sy*dy,c.y+sy*dx+cy*dy,z+lift);
        cv::Mat rv;cv::Rodrigues(R,rv);std::vector<cv::Point2f> px;cv::projectPoints(body,rv,t,K,D,px);
        return cv::boundingRect(px);
    }
    KeypointFrame points(float yaw=.3f,cv::Point2f origin={.05f,.85f},std::vector<int> ids={0,1,2,3,4,5}) const {
        KeypointFrame out;out.zone_label=geometry.label;out.geometry_id=geometry.id;
        out.frame_id=frame.frame_id;out.capture_us=frame.capture_us;out.image_size=frame.image_size;
        const auto objects=geometry.objectPoints();std::vector<cv::Point3f> body;
        for(int id:ids){auto q=objects[id];body.emplace_back(origin.x+std::cos(yaw)*q.x-std::sin(yaw)*q.y,
            origin.y+std::sin(yaw)*q.x+std::cos(yaw)*q.y,geometry.landmark_height_m);}
        cv::Mat rv;cv::Rodrigues(R,rv);std::vector<cv::Point2f> pixels;cv::projectPoints(body,rv,t,K,D,pixels);
        for(size_t i=0;i<ids.size();++i)out.points.push_back({ids[i],pixels[i],.95f,true});
        return out;
    }
};
int main() {
    { // Mechanical files require explicit opt-in; the measured 5-degree model is untouched.
        std::string root=ZONE_GEOMETRY_PATH;root=root.substr(0,root.find_last_of('/'));
        CameraCalibration assumed,measured;
        assert(!assumed.load(root+"/camera.mechanical_5_40.yaml"));
        assert(assumed.load(root+"/camera.mechanical_5_40.yaml",true));
        assert(measured.load(root+"/camera.yaml"));
        assert(assumed.pitchUsable(500)&&assumed.pitchUsable(4000)&&!assumed.pitchUsable(4001));
    }

    {
        auto red=ZoneGeometry::load(ZONE_GEOMETRY_PATH,"red_safe_zone");
        auto blue=ZoneGeometry::load(ZONE_GEOMETRY_PATH,"blue_safe_zone");
        assert(red.valid()&&blue.valid()&&red.supply_left&&blue.supply_left);
        // v2 landmarks: fence tops 660x330 at z=0.02; delivery bounds stay the inner 600x300 floor.
        const auto p=red.objectPoints();assert(p[0]==cv::Point3f(-.33f,0,0)&&p[5]==cv::Point3f(.33f,.33f,0));
        assert(red.width_m==.6f&&red.depth_m==.3f&&red.landmark_height_m==.02f&&red.id=="rescue2027-fence-top-v2-red");
        auto legacy=ZoneGeometry::load(LEGACY_ZONE_GEOMETRY_PATH,"red_safe_zone");
        const auto q=legacy.objectPoints();
        assert(q[0]==cv::Point3f(-.3f,0,0)&&q[5]==cv::Point3f(.3f,.3f,0)&&legacy.landmark_height_m==0);
    }
    {
        // Ray/plane intersection at fence-top height round-trips exactly; the floor plane would not.
        Scene s;cv::Mat rv;cv::Rodrigues(s.R,rv);std::vector<cv::Point2f> px;
        cv::projectPoints(std::vector<cv::Point3f>{{.2f,.9f,.02f}},rv,s.t,s.K,s.D,px);
        cv::Point2f top,floor;
        assert(s.camera.pixelToPlane(px[0],2000,.02f,top)&&cv::norm(top-cv::Point2f(.2f,.9f))<1e-4);
        assert(s.camera.pixelToGround(px[0],2000,floor)&&cv::norm(floor-cv::Point2f(.2f,.9f))>.02);
        CameraCalibration fixed;fixed.setIntrinsics(s.K,s.D);fixed.setGroundHomography(cv::Mat::eye(3,3,CV_64F),2000);
        assert(fixed.pixelToPlane({1,1},2000,0,top)&&!fixed.pixelToPlane({1,1},2000,.02f,top));
    }
    {
        // Legacy floor-corner schema still runs end to end.
        Scene s(LEGACY_ZONE_GEOMETRY_PATH);GeometryPipeline pipeline(s.camera,s.geometry);
        auto out=pipeline.process(s.frame,s.points(),{});
        assert(out.zone.trusted(s.frame.now_us)&&cv::norm(out.zone.origin_body_m-cv::Point2f(.05f,.85f))<1e-4);
        assert(out.zone.pnp_checked&&out.zone.pnp_consistent);
    }
    {
        Scene s;GeometryPipeline pipeline(s.camera,s.geometry);
        auto out=pipeline.process(s.frame,s.points(),{});
        assert(out.zone.trusted(s.frame.now_us)&&out.zone.inlier_ids.size()==6);
        assert(cv::norm(out.zone.origin_body_m-cv::Point2f(.05f,.85f))<1e-4);
        assert(std::abs(out.zone.yaw_body_rad-.3f)<1e-4);
        assert(out.zone.pnp_checked&&out.zone.pnp_consistent);
        // A different heading is not rejected just for being oblique or sideways.
        for(float yaw:{-1.1f,1.1f,2.2f}) {
            GeometryPipeline independent(s.camera,s.geometry);
            auto rotated=independent.process(s.frame,s.points(yaw,{0,.9f}),{});
            assert(rotated.zone.trusted(s.frame.now_us));
            const auto left=rotated.zone.zoneToBody({-.15f,.15f});
            auto decision=classifyExpectedStop(s.geometry,rotated.zone,left,.02f,s.frame.now_us);
            assert(decision.valid&&decision.zone_class=="supply");
            auto right=classifyExpectedStop(s.geometry,rotated.zone,rotated.zone.zoneToBody({.15f,.15f}),.02f,s.frame.now_us);
            assert(right.valid&&right.zone_class=="injured");
            assert(!classifyExpectedStop(s.geometry,rotated.zone,rotated.zone.zoneToBody({0,.15f}),.02f,s.frame.now_us).valid);
            assert(!classifyExpectedStop(s.geometry,rotated.zone,rotated.zone.zoneToBody({-.29f,.15f}),.02f,s.frame.now_us).valid);
        }
    }
    {
        Scene s;GeometryPipeline pipeline(s.camera,s.geometry);
        auto corrupted=s.points();corrupted.points[2].pixel+=cv::Point2f(180,80);
        auto out=pipeline.process(s.frame,corrupted,{});
        assert(out.zone.trusted(s.frame.now_us)&&out.zone.inlier_ids.size()>=4);
        assert(std::find(out.zone.inlier_ids.begin(),out.zone.inlier_ids.end(),2)==out.zone.inlier_ids.end());
        assert(cv::norm(out.zone.origin_body_m-cv::Point2f(.05f,.85f))<.005);
        // Real pixel perturbations, not perfect self-inverse data only.
        s.advance();auto noisy=s.points();
        for(size_t i=0;i<noisy.points.size();++i)noisy.points[i].pixel+=cv::Point2f(i%2?.5f:-.5f,i%3?.3f:-.3f);
        auto noise=pipeline.process(s.frame,noisy,{});assert(noise.zone.trusted(s.frame.now_us));
        assert(cv::norm(noise.zone.origin_body_m-cv::Point2f(.05f,.85f))<.01);
    }
    {
        Scene s;GeometryPipeline pipeline(s.camera,s.geometry);
        auto two=s.points(.3f,{.05f,.85f},{0,2});
        assert(!pipeline.process(s.frame,two,{}).zone.valid); // no prior
        s.advance();assert(pipeline.process(s.frame,s.points(),{}).zone.valid);
        s.advance();auto held=pipeline.process(s.frame,s.points(.3f,{.05f,.85f},{0,2}),{});
        assert(held.zone.trusted(s.frame.now_us)&&held.zone.source==ZoneEstimate::Source::TWO_POINT);
        // Left body rotation decreases zone yaw in body coordinates.
        s.advance();s.frame.sensors.imu.yaw_rad=.08f;
        const float x=std::cos(.08f)*.05f+std::sin(.08f)*.85f;
        const float y=-std::sin(.08f)*.05f+std::cos(.08f)*.85f;
        auto turned=pipeline.process(s.frame,s.points(.22f,{x,y},{0,2}),{});
        assert(turned.zone.valid);
        s.advance();auto wrong=pipeline.process(s.frame,s.points(.8f,{x,y},{0,2}),{});assert(!wrong.zone.valid);
        s.advance(400000);auto expired=pipeline.process(s.frame,s.points(.22f,{x,y},{0,2}),{});assert(!expired.zone.valid);
    }
    {
        Scene s;GeometryPipeline pipeline(s.camera,s.geometry);
        assert(pipeline.process(s.frame,s.points(),{}).zone.valid);
        s.advance();assert(!pipeline.process(s.frame,s.points(.3f,{.05f,1.35f}),{}).zone.valid);
        s.advance();assert(!pipeline.process(s.frame,s.points(.3f,{.05f,1.35f}),{}).zone.valid);
        s.advance();assert(pipeline.process(s.frame,s.points(.3f,{.05f,1.35f}),{}).zone.valid);
    }
    {
        Scene s;KeypointFilter filter;
        auto expectBad=[&](GeometryFrame frame,KeypointFrame points){assert(!filter.filter(points,frame,s.geometry,s.camera).valid);};
        auto base=s.points();auto f=s.frame;f.sensors.imu.timestamp_us=f.capture_us+1;expectBad(f,base);
        f=s.frame;f.sensors.imu.timestamp_us=f.capture_us-60000;expectBad(f,base);
        f=s.frame;f.sensors.actuator.timestamp_us=f.capture_us+1;expectBad(f,base);
        f=s.frame;f.sensors.pitch_stable=false;expectBad(f,base);
        f=s.frame;f.sensors.imu.pitch_rad=.2;expectBad(f,base);
        f=s.frame;f.sensors.imu.yaw_rad=std::numeric_limits<float>::quiet_NaN();expectBad(f,base);
        f=s.frame;f.sensors.actuator.camera_pitch_cdeg=3000;expectBad(f,base);
        f=s.frame;f.now_us=f.capture_us+300000;expectBad(f,base);
        f=s.frame;f.image_size={640,360};expectBad(f,base);
        auto bad=base;bad.frame_id++;expectBad(s.frame,bad);
        bad=base;bad.geometry_id="other";expectBad(s.frame,bad);
        bad=base;bad.points[1].id=bad.points[0].id;expectBad(s.frame,bad);
        bad=base;for(auto& k:bad.points)k.confidence=.1;expectBad(s.frame,bad);
        bad=base;for(auto& k:bad.points)k.pixel.y=70;expectBad(s.frame,bad); // above horizon
    }
    {
        Scene s;CameraCalibration c;
        c.setIntrinsics(s.K,s.D);
        c.setGroundHomography(cv::Mat::zeros(3,3,CV_64F),2000);assert(!c.valid());
        cv::Mat reflected=cv::Mat::eye(4,4,CV_64F);reflected.at<double>(0,0)=-1;
        c.setPitchModel(reflected,2000,1000,2500);assert(!c.valid());
        cv::Mat nonrigid=cv::Mat::eye(4,4,CV_64F);nonrigid.at<double>(0,0)=2;
        c.setPitchModel(nonrigid,2000,1000,2500);assert(!c.valid());
        // Geometry application projects targets but does not invent a safe route or delivery.
        GeometryPipeline pipeline(s.camera,s.geometry);auto points=s.points();points.identity_verified=true;
        SegDetection d;d.track_id=7;d.label="ordinary_supply";d.confidence=.9f;
        d.frame_id=s.frame.frame_id;d.timestamp_us=s.frame.capture_us;
        d.box=s.cubeBox({0,.82f});d.ground_contact_valid=false;
        auto result=pipeline.process(s.frame,points,{d});
        assert(result.detections[0].ground_position_valid&&result.detections[0].ground_contact_valid);
        assert(result.detections[0].ground_contact_reason.empty());
        PushObservation in;in.target_valid=true;in.target_id=7;in.label=d.label;
        ExpectedStop stop;stop.valid=true;stop.target_id=7;stop.frame_id=s.frame.frame_id;stop.capture_us=s.frame.capture_us;
        stop.body_m=result.zone.zoneToBody({-.15f,.15f});stop.radius_m=.02f;
        pipeline.apply(in,result,stop,"red",s.frame.now_us);
        assert(in.geometry_valid&&in.zone_valid&&in.zone_own&&in.zone_class=="supply");
        assert(!in.path_safe&&!in.safety_ok&&!in.retreat_safe&&!in.opponent_zone_clear&&!in.zone_counts_valid&&!in.captured);
        result.identity_verified=false;
        pipeline.apply(in,result,stop,"red",s.frame.now_us);
        assert(!in.zone_own&&!in.zone_identity_verified&&!in.target_region_valid&&in.zone_class.empty());
        result.identity_verified=true;
        result.detections[0].ground_contact_valid=false;
        pipeline.apply(in,result,stop,"red",s.frame.now_us);assert(!in.geometry_valid);
        stop.target_id=8;pipeline.apply(in,result,stop,"red",s.frame.now_us);assert(in.zone_class.empty());
        stop.target_id=7;pipeline.apply(in,result,stop,"blue",s.frame.now_us);assert(!in.zone_own);
    }
    {
        // Ground contact is produced by the pipeline from measured evidence only.
        Scene s;GeometryPipeline pipeline(s.camera,s.geometry);
        auto make=[&](const char* label,cv::Rect box,float conf=.9f){SegDetection d;d.track_id=1;d.label=label;
            d.confidence=conf;d.box=box;d.frame_id=s.frame.frame_id;d.timestamp_us=s.frame.capture_us;return d;};
        auto run=[&](SegDetection d,GeometryFrame f){return pipeline.process(f,s.points(),{d}).detections[0];};
        auto ok=run(make("ordinary_supply",s.cubeBox({.1f,.6f})),s.frame);
        assert(ok.ground_contact_valid&&std::abs(ok.body_xy_m.x-.1f)<.01f&&std::abs(ok.body_xy_m.y-.58f)<.01f);
        assert(run(make("core_supply",s.cubeBox({-.1f,1.f},.0346f,.04f)),s.frame).ground_contact_valid);
        assert(run(make("injured_person",s.cubeBox({0,.9f},.04f,.08f)),s.frame).ground_contact_valid);
        {   // Sideways-only injured grab: on-end (upright) is rejected, lying is kept at any yaw.
            GroundContactConfig lc;lc.injured_lying_only=true;GeometryPipeline lying_only(s.camera,s.geometry,lc);
            auto runL=[&](SegDetection d){return lying_only.process(s.frame,s.points(),{d}).detections[0];};
            int up_total=0,up_rej=0,lie_total=0,lie_rej=0;
            for(float y=.4f;y<=1.6f;y+=.1f)for(float x=-.6f;x<=.6f;x+=.2f) {
                const auto upright=s.cubeBox({x,y},.04f)|s.cubeBox({x,y},.04f,0,.04f);
                if(upright.x>=6&&upright.y>=6&&upright.br().x<=1274&&upright.br().y<=714) {
                    auto r=runL(make("injured_person",upright));
                    if(r.ground_contact_reason=="out_of_range")continue;
                    ++up_total;if(r.injured_upright_rejected){++up_rej;assert(r.ground_contact_reason=="injured_upright"&&!r.ground_contact_valid);}
                }
                for(float yaw=0;yaw<3.1f;yaw+=.4f) {
                    const auto lie=s.cubeBox({x,y},.04f,.08f,0,yaw);
                    if(lie.x<6||lie.y<6||lie.br().x>1274||lie.br().y>714)continue;
                    for(float jw:{1.f,.94f,1.06f})for(float jh:{1.f,.94f,1.06f}) { // loose detector boxes: +-6 % per axis
                        cv::Rect jb(lie.x+int(lie.width*(1-jw)/2),lie.y+int(lie.height*(1-jh)/2),int(lie.width*jw),int(lie.height*jh));
                        auto r=runL(make("injured_person",jb));
                        if(r.ground_contact_reason=="out_of_range")continue;
                        ++lie_total;if(r.injured_upright_rejected){++lie_rej;std::cerr<<"lying rejected "<<x<<","<<y<<" yaw "<<yaw<<" jw "<<jw<<" jh "<<jh<<" e "<<r.pose_err_lying<<"/"<<r.pose_err_upright<<"\n";}
                    }
                }
            }
            std::cerr<<"upright rejected "<<up_rej<<"/"<<up_total<<" lying rejected "<<lie_rej<<"/"<<lie_total<<"\n";
            assert(up_total>20&&lie_total>1000);
            assert(lie_rej==0);              // never drop a lying block
            assert(up_rej*4>=up_total*3);    // most on-end views are caught (single-box cue, not a guarantee)
            {   // Positive lying evidence (real-robot setting): everything the margin rule rejects stays rejected, ambiguous blocks are added.
                GroundContactConfig cc=lc;cc.lying_confirm_diff=-.06f;GeometryPipeline confirm(s.camera,s.geometry,cc);
                int up2=0,lie2=0,lie_n=0,not_lying=0;
                for(float y=.4f;y<=1.6f;y+=.1f)for(float x=-.6f;x<=.6f;x+=.2f) {
                    const auto upright=s.cubeBox({x,y},.04f)|s.cubeBox({x,y},.04f,0,.04f);
                    if(upright.x>=6&&upright.y>=6&&upright.br().x<=1274&&upright.br().y<=714) {
                        const auto r=confirm.process(s.frame,s.points(),{make("injured_person",upright)}).detections[0];
                        if(r.ground_contact_reason=="out_of_range")continue;
                        up2+=r.injured_upright_rejected;not_lying+=r.ground_contact_reason=="injured_not_lying";
                    }
                    for(float yaw=0;yaw<3.1f;yaw+=.4f) {
                        const auto lie=s.cubeBox({x,y},.04f,.08f,0,yaw);
                        if(lie.x<6||lie.y<6||lie.br().x>1274||lie.br().y>714)continue;
                        const auto r=confirm.process(s.frame,s.points(),{make("injured_person",lie)}).detections[0];
                        if(r.ground_contact_reason=="out_of_range")continue;
                        ++lie_n;lie2+=r.injured_upright_rejected;
                    }
                }
                std::cerr<<"confirm rule: upright rejected "<<up2<<"/"<<up_total<<" (not_lying "<<not_lying<<") lying rejected "<<lie2<<"/"<<lie_n<<"\n";
                assert(up2>=up_rej&&lie_n>100);
            }
            {   // Silhouette cue: a clearly wide orange blob confirms lying, a clearly tall one rejects, the grey zone keeps the box fit.
                GroundContactConfig sc=lc;sc.lying_confirm_diff=-.06f;sc.use_silhouette=true;
                GeometryPipeline sil(s.camera,s.geometry,sc),plain(s.camera,s.geometry,[&]{auto c=sc;c.use_silhouette=false;return c;}());
                int flipped_ok=0,flipped_rej=0,grey_same=0,n=0;
                for(float y=.4f;y<=1.6f;y+=.1f)for(float x=-.6f;x<=.6f;x+=.2f) {
                    const auto upright=s.cubeBox({x,y},.04f)|s.cubeBox({x,y},.04f,0,.04f);
                    const auto lie=s.cubeBox({x,y},.04f,.08f,0,0);
                    for(const auto& b:{upright,lie}) {
                        if(b.x<6||b.y<6||b.br().x>1274||b.br().y>714)continue;
                        auto d=make("injured_person",b);
                        const auto base=plain.process(s.frame,s.points(),{d}).detections[0];
                        if(base.ground_contact_reason=="out_of_range"||base.pose_err_lying<0)continue;
                        ++n;
                        d.silhouette_hw=.7f;auto wide=sil.process(s.frame,s.points(),{d}).detections[0];
                        d.silhouette_hw=1.5f;auto tall=sil.process(s.frame,s.points(),{d}).detections[0];
                        d.silhouette_hw=1.0f;auto grey=sil.process(s.frame,s.points(),{d}).detections[0];
                        d.silhouette_hw=-1.f;auto none=sil.process(s.frame,s.points(),{d}).detections[0];
                        assert(!wide.injured_upright_rejected);                 // wide silhouette: always a lying candidate
                        assert(tall.injured_upright_rejected&&tall.ground_contact_reason=="injured_upright");
                        assert(grey.injured_upright_rejected==base.injured_upright_rejected&&none.injured_upright_rejected==base.injured_upright_rejected);
                        flipped_ok+=base.injured_upright_rejected&&!wide.injured_upright_rejected;
                        flipped_rej+=!base.injured_upright_rejected&&tall.injured_upright_rejected;
                        ++grey_same;
                    }
                }
                std::cerr<<"silhouette: n "<<n<<" rescued "<<flipped_ok<<" newly rejected "<<flipped_rej<<"\n";
                assert(n>100&&grey_same==n&&flipped_ok>0);
            }
            // Default config keeps the old behaviour: no rejection.
            auto base=run(make("injured_person",s.cubeBox({0,.9f},.04f)|s.cubeBox({0,.9f},.04f,0,.04f)),s.frame);
            assert(!base.injured_upright_rejected);
            {   // Cross-frame vote: a missed on-end frame inside a track is still rejected, one outlier does not reject a lying block.
                auto uprightBox=[&](float x,float y){return s.cubeBox({x,y},.04f)|s.cubeBox({x,y},.04f,0,.04f);};
                auto inImage=[](cv::Rect b){return b.x>=6&&b.y>=6&&b.br().x<=1274&&b.br().y<=714;};
                auto single=[&](cv::Rect b){GeometryPipeline p(s.camera,s.geometry,lc);
                    return p.process(s.frame,s.points(),{make("injured_person",b)}).detections[0];};
                auto step=[&](GeometryPipeline& p,int id,int k,cv::Rect b){
                    auto f=s.frame;const uint64_t dt=uint64_t(k)*50000;f.capture_us+=dt;f.now_us+=dt;f.frame_id+=k;f.sensors.imu.timestamp_us+=dt;f.sensors.actuator.timestamp_us+=dt;
                    auto d=make("injured_person",b);d.track_id=id;d.timestamp_us=f.capture_us;d.frame_id=f.frame_id;
                    return p.process(f,s.points(),{d}).detections[0];};
                std::vector<cv::Point2f> miss,hit;
                for(float y=.4f;y<=1.6f;y+=.05f)for(float x=-.6f;x<=.6f;x+=.1f) {
                    const auto b=uprightBox(x,y);if(!inImage(b))continue;
                    const auto r=single(b);if(r.pose_err_lying<0)continue;
                    (r.injured_upright_rejected?hit:miss).emplace_back(x,y);
                }
                bool paired=false;
                for(const auto& m:miss)for(const auto& h:hit) {
                    if(paired||cv::norm(m-h)>.2f)continue;
                    paired=true;
                    const auto bm=uprightBox(m.x,m.y),bh=uprightBox(h.x,h.y);
                    assert(!single(bm).injured_upright_rejected&&single(bh).injured_upright_rejected);
                    GeometryPipeline vp(s.camera,s.geometry,lc);
                    assert(step(vp,5,0,bh).injured_upright_rejected&&step(vp,5,1,bh).injured_upright_rejected);
                    const auto third=step(vp,5,2,bm); // single frame would let it through
                    assert(third.pose_vote_n==3&&third.injured_upright_rejected&&third.ground_contact_reason=="injured_upright");
                    const auto lie=s.cubeBox({h.x,h.y},.04f,.08f,0,0);
                    assert(inImage(lie)&&!single(lie).injured_upright_rejected);
                    GeometryPipeline lp(s.camera,s.geometry,lc);
                    assert(!step(lp,6,0,lie).injured_upright_rejected&&!step(lp,6,1,lie).injured_upright_rejected);
                    const auto outlier=step(lp,6,2,bh); // single frame would reject it
                    assert(outlier.pose_vote_n==3&&!outlier.injured_upright_rejected);
                    // A track that jumps to another place starts a new vote.
                    const auto far=step(lp,6,3,uprightBox(h.x+.5f,h.y));
                    assert(far.pose_vote_n<=1);
                }
                {   // Approach sequences (0.05 m steps, 50 ms apart): voting must never reject a lying block and must catch more on-end frames.
                    int track=100,up_frames=0,up_single=0,up_voted=0,lie_frames=0,lie_voted=0;
                    for(float x=-.4f;x<=.41f;x+=.2f) {
                        GeometryPipeline vu(s.camera,s.geometry,lc);++track;int k=0;
                        for(float y=1.5f;y>=.55f;y-=.05f,++k) {
                            const auto b=uprightBox(x,y);if(!inImage(b))continue;
                            const auto r=step(vu,track,k,b);if(r.pose_err_lying<0)continue;
                            ++up_frames;up_single+=single(b).injured_upright_rejected;up_voted+=r.injured_upright_rejected;
                        }
                        for(float yaw=0;yaw<3.1f;yaw+=.8f) {
                            GeometryPipeline vl(s.camera,s.geometry,lc);++track;k=0;
                            for(float y=1.5f;y>=.55f;y-=.05f,++k) {
                                const auto lb=s.cubeBox({x,y},.04f,.08f,0,yaw);if(!inImage(lb))continue;
                                const float jw=1+.06f*float((k%3)-1),jh=1+.06f*float(((k+1)%3)-1);
                                cv::Rect jb(lb.x+int(lb.width*(1-jw)/2),lb.y+int(lb.height*(1-jh)/2),int(lb.width*jw),int(lb.height*jh));
                                const auto r=step(vl,track,k,jb);if(r.pose_err_lying<0)continue;
                                ++lie_frames;lie_voted+=r.injured_upright_rejected;
                            }
                        }
                    }
                    std::cerr<<"approach upright frames "<<up_frames<<" single_rej "<<up_single<<" voted_rej "<<up_voted<<" | lying frames "<<lie_frames<<" voted_rej "<<lie_voted<<"\n";
                    assert(up_frames>30&&lie_frames>100&&lie_voted==0&&up_voted>=up_single);
                }
                std::cerr<<"vote pairs miss="<<miss.size()<<" hit="<<hit.size()<<" paired="<<paired<<"\n";
                assert(paired);
            }
        }
        // A box claiming contact on input is ignored; the producer decides.
        auto claimed=make("ordinary_supply",s.cubeBox({.1f,.6f}),.2f);claimed.ground_contact_valid=true;
        auto low=run(claimed,s.frame);assert(!low.ground_contact_valid&&low.ground_contact_reason=="low_confidence");
        assert(low.ground_position_valid); // a box-bottom estimate is still reported, without contact
        // Dangerous cubes get measured contact too (for avoidance); they are never selected as targets.
        assert(run(make("dangerous_object",s.cubeBox({.1f,.6f})),s.frame).ground_contact_valid);
        assert(run(make("mystery",s.cubeBox({.1f,.6f})),s.frame).ground_contact_reason=="unknown_label");
        // Two adjacent cubes merged into one box: footprint twice as wide.
        auto merged=s.cubeBox({.08f,.6f})|s.cubeBox({.16f,.6f});
        assert(run(make("ordinary_supply",merged),s.frame).ground_contact_reason=="size_mismatch");
        // A cube stacked on a cube: twice as tall as a floor cube at the projected range.
        auto stacked=s.cubeBox({.1f,.6f})|s.cubeBox({.1f,.6f},.04f,0,.04f);
        assert(run(make("ordinary_supply",stacked),s.frame).ground_contact_reason=="size_mismatch");
        // Off-axis cubes keep contact despite perspective widening of the box.
        assert(run(make("ordinary_supply",s.cubeBox({-.6f,1.1f},.04f,0,0,.8f)),s.frame).ground_contact_valid);
        {
            // Sweep the field of view: every floor block fully inside the image keeps contact,
            // at any yaw (not just the producer's 10 degree grid) and pose.
            const float e=.04f,r=e/std::sqrt(3.f),hz=e*std::sqrt(2.f/3.f);
            const std::vector<cv::Point3f> tetra{{-e/2,-r/2,0},{e/2,-r/2,0},{0,r,0},{0,0,hz}};
            cv::Mat rv;cv::Rodrigues(s.R,rv);int checked=0,rejected=0;
            for(float y=.35f;y<=2.5f;y+=.15f)for(float x=-1.f;x<=1.f;x+=.1f)for(float yaw=.05f;yaw<3.2f;yaw+=.37f) {
                std::vector<std::pair<const char*,cv::Rect>> cases{{"ordinary_supply",s.cubeBox({x,y},.04f,0,0,yaw)},
                    {"injured_person",s.cubeBox({x,y},.04f,.08f,0,yaw)}};
                std::vector<cv::Point3f> body;
                for(const auto& v:tetra)body.emplace_back(x+std::cos(yaw)*v.x-std::sin(yaw)*v.y,y+std::sin(yaw)*v.x+std::cos(yaw)*v.y,v.z);
                std::vector<cv::Point2f> px;cv::projectPoints(body,rv,s.t,s.K,s.D,px);
                cases.push_back({"core_supply",cv::boundingRect(px)});
                for(const auto& c:cases) {
                    if(c.second.x<6||c.second.y<6||c.second.br().x>1274||c.second.br().y>714)continue;
                    auto r=run(make(c.first,c.second),s.frame);
                    if(r.ground_contact_reason=="out_of_range")continue;
                    ++checked;
                    if(!r.ground_contact_valid){++rejected;std::cerr<<c.first<<" "<<x<<","<<y<<" yaw "<<yaw<<": "<<r.ground_contact_reason<<"\n";}
                }
            }
            assert(checked>1000&&rejected==0);
        }
        auto edge=s.cubeBox({.1f,.6f});edge.x=1;
        assert(run(make("ordinary_supply",edge),s.frame).ground_contact_reason=="box_at_image_edge");
        auto f=s.frame;f.sensors.actuator.camera_pitch_cdeg=3000;
        auto pitch=run(make("ordinary_supply",s.cubeBox({.1f,.6f})),f);
        assert(!pitch.ground_contact_valid&&!pitch.ground_position_valid&&pitch.ground_contact_reason=="pitch_not_calibrated");
        f=s.frame;f.sensors.pitch_stable=false;
        assert(run(make("ordinary_supply",s.cubeBox({.1f,.6f})),f).ground_contact_reason=="pitch_moving");
        f=s.frame;f.sensors.imu.imu_valid=false;
        assert(run(make("ordinary_supply",s.cubeBox({.1f,.6f})),f).ground_contact_reason=="imu_not_synchronized");
        auto stale=make("ordinary_supply",s.cubeBox({.1f,.6f}));stale.frame_id--;
        assert(run(stale,s.frame).ground_contact_reason=="frame_mismatch");
        assert(run(make("ordinary_supply",s.cubeBox({0,3.2f})),s.frame).ground_contact_reason=="out_of_range");
        std::vector<std::vector<cv::Point3f>> solids;
        assert(!classSolids("unknown",solids)&&classSolids("injured_person",solids)&&solids.size()==2);
        // FIXED_PITCH files have no extrinsic model, so no solid can be projected: no contact.
        CameraCalibration fixed;fixed.setIntrinsics(s.K,s.D);
        cv::Matx33d H;assert(s.camera.groundHomographyAt(2000,nullptr,H));
        fixed.setGroundHomography(cv::Mat(H),2000,true);
        {GeometryPipeline fp(fixed,s.geometry);auto fr=s.frame;auto fd=make("ordinary_supply",s.cubeBox({.1f,.6f}));
         auto r=fp.process(fr,s.points(),{fd}).detections[0];
         assert(!r.ground_contact_valid&&(r.ground_contact_reason=="no_extrinsic_model"||r.ground_contact_reason=="calibration_image_size_mismatch"));}
        GroundContactConfig bad;bad.size_tolerance=1;bool threw=false;
        try{GeometryPipeline p(s.camera,s.geometry,bad);}catch(const std::runtime_error&){threw=true;}
        assert(threw);
    }
    {
        PitchHistory h;
        for(uint64_t t=800000;t<=1000000;t+=25000){ActuatorFeedback a;a.timestamp_us=t;a.valid=true;a.camera_pitch_cdeg=2000;h.add(a);}
        assert(h.at(1010000).pitch_stable);
        ActuatorFeedback future;future.timestamp_us=1100000;future.valid=true;future.camera_pitch_cdeg=1000;h.add(future);
        assert(h.at(1010000).actuator.camera_pitch_cdeg==2000);
        assert(!h.at(1100000).pitch_stable);
        assert(!h.at(1200000).actuator.valid);
        assert(!h.at(700000).actuator.valid);
    }
    {
        Scene s;GeometryPipeline pipeline(s.camera,s.geometry);auto z=pipeline.process(s.frame,s.points(),{}).zone;
        z.residual_m=.1;assert(!z.trusted(s.frame.now_us));z.residual_m=.001;
        z.source=ZoneEstimate::Source::PREDICTED;z.predicted_distance_m=.31;assert(!z.trusted(s.frame.now_us));
        z.predicted_distance_m=.1;assert(z.trusted(s.frame.now_us));
        z.timestamp_us+=500000;assert(!z.trusted(s.frame.now_us));
        z.timestamp_us=s.frame.capture_us;z.inlier_ids={99,1};assert(!z.trusted(s.frame.now_us));
    }
    {
        // A historical bool alone must never bypass the zone-quality gate: a verified load
        // with an untrusted zone estimate only searches in place, it never drives to a gate.
        TaskTuning tuning;tuning.startup_advance_us=0;PushTask task(tuning);PushObservation in;
        in.target_region_valid=in.zone_identity_verified=true;
        in.run=in.safety_ok=in.target_valid=in.geometry_valid=in.path_safe=in.opponent_zone_clear=true;
        in.target_id=1;in.label="ordinary_supply";in.distance_m=.2;
        in.zone_valid=in.zone_own=true;in.zone_class="supply";
        in.corridor.add("ordinary_supply");in.corridor_complete=in.corridor_occlusion_free=true;
        PushOutput out;
        // Ideal servo: readback is the last command, settled; the frame is in view only at NEAR.
        auto step=[&]{in.now_us+=50000;in.gripper_feedback_open=out.motion.gripper_offset==0?1:0;
            in.camera_pitch_cdeg=out.motion.camera_pitch_cdeg;in.camera_pitch_stable=true;
            in.hold_observable=in.camera_pitch_cdeg==TaskTuning{}.near_pitch_cdeg;out=task.update(in);};
        in.gripper_done=true;
        for(int i=0;i<12&&out.state!=PushState::RUSH;++i)step();
        assert(out.state==PushState::RUSH);
        in.captured=in.held_complete=true;in.held.add("ordinary_supply");
        in.multi_view_finished=true;in.multi_view_verdict=1;in.multi_view_inventory=in.held;
        for(int i=0;i<20&&out.state!=PushState::CARRY;++i)step();
        assert(out.state==PushState::CARRY);
        for(int i=0;i<6;++i){step();assert(out.state==PushState::CARRY&&out.motion.vx_mps==0);}
    }
    {
        const cv::Mat K=(cv::Mat_<double>(3,3)<<500,0,320,0,500,240,0,0,1);
        SafeZonePoseEstimator estimator(K,{});SafeZoneObservation o;o.label="red_safe_zone";
        o.object_points={{-.5,-.4,0},{.5,-.4,0},{.5,.4,0},{-.5,.4,0},{0,-.4,0},{0,.4,0}};
        auto observe=[&](double x,uint64_t stamp){const cv::Mat t=(cv::Mat_<double>(3,1)<<x,0,2);
            cv::projectPoints(o.object_points,cv::Mat::zeros(3,1,CV_64F),t,K,cv::Mat(),o.image_points);
            return estimator.estimate(o,stamp);};
        assert(observe(0,1000000).valid); // no visible divider is not a veto
        assert(!observe(.6,1050000).valid);assert(!observe(.6,1100000).valid);
        assert(observe(.6,1150000).valid);assert(observe(.6,1200000).valid);
        // A missing frame interrupts the consecutive reacquisition window.
        assert(!observe(0,1250000).valid);
        auto missing=o;missing.image_points.clear();assert(!estimator.estimate(missing,1300000).valid);
        assert(!observe(0,1350000).valid);assert(!observe(0,1400000).valid);assert(observe(0,1450000).valid);
    }
    {
        // Actual RX-only serial path through a pseudo-terminal; no real hardware.
        int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);assert(master>=0&&grantpt(master)==0&&unlockpt(master)==0);
        UARTController uart;uart.initFeedbackOnly(ptsname(master),115200);
        assert(!uart.sendMotion(MotionCommand{}));assert(uart.gripperActionId()==0);
        uint8_t packet[8]={0xA6,1,1,0x3C,0x00,0,0,0x0A};
        auto crc=UARTController::calculateCRC16(packet,0,4);packet[5]=crc&255;packet[6]=crc>>8;
        assert(write(master,packet,8)==8);
        for(int i=0;i<100&&!uart.latestActuatorFeedback().valid;++i)std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto feedback=uart.latestActuatorFeedback();assert(feedback.valid&&feedback.camera_pitch_cdeg==2000);
        assert(uart.feedbackAt(feedback.timestamp_us).actuator.valid);
        uint8_t output;assert(read(master,&output,1)<0);
        uart.closePort();close(master);
    }
    std::cout<<"Stage 0/1 geometry, timing, consensus, recovery, side and RX-only checks passed\n";
}
