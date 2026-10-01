#include "rescue/geometry_pipeline.hpp"
#include <iostream>
#include <stdexcept>
namespace rescue {
int runGeometryReplay(const std::string& path,const std::string& calibration_path,
                      const std::string& geometry_path,const std::string& team) {
    CameraCalibration calibration;
    if(!calibration.load(calibration_path))throw std::runtime_error("Cannot load validated ground calibration");
    auto geometry=ZoneGeometry::load(geometry_path,team+"_safe_zone");
    GeometryPipeline pipeline(calibration,geometry);
    cv::FileStorage file(path,cv::FileStorage::READ);
    if(!file.isOpened()||!file["frames"].isSeq())throw std::runtime_error("Invalid geometry replay");
    PitchHistory pitches;size_t valid=0,total=0;uint64_t last_time=0,last_id=0;
    for(const auto& n:file["frames"]) {
        auto keypoints=readKeypointFrame(n);GeometryFrame f;
        f.frame_id=keypoints.frame_id;f.capture_us=keypoints.capture_us;f.now_us=readExactTime(n["now_us"]);f.image_size=keypoints.image_size;
        if(f.capture_us<=last_time||f.frame_id<=last_id)throw std::runtime_error("Replay frames must increase");
        last_time=f.capture_us;last_id=f.frame_id;
        if(!n["pitch_samples"].isSeq())throw std::runtime_error("Replay requires recorded pitch samples");
        for(const auto& p:n["pitch_samples"]) {
            const int pitch=int(p["pitch_cdeg"]);
            if(!p["pitch_cdeg"].isInt()||pitch<-32768||pitch>32767||p["valid"].empty())throw std::runtime_error("Invalid pitch record");
            ActuatorFeedback a;a.timestamp_us=readExactTime(p["received_us"]);a.camera_pitch_cdeg=pitch;a.valid=int(p["valid"])==1;pitches.add(a);
        }
        f.sensors=pitches.at(f.capture_us);
        const auto imu=n["imu"];const auto rpy=imu["body_rpy_rad"];
        if(!rpy.isSeq()||rpy.size()!=3||imu["valid"].empty())throw std::runtime_error("Replay requires recorded IMU");
        f.sensors.imu.timestamp_us=readExactTime(imu["received_us"]);f.sensors.imu.imu_valid=int(imu["valid"])==1;
        f.sensors.imu.roll_rad=float(rpy[0]);f.sensors.imu.pitch_rad=float(rpy[1]);f.sensors.imu.yaw_rad=float(rpy[2]);
        const auto result=pipeline.process(f,keypoints,{});++total;valid+=result.zone.trusted(f.now_us);
        auto stop=readExpectedStop(n,keypoints);std::string side="unavailable";
        if(stop.valid) {const auto d=classifyExpectedStop(geometry,result.zone,stop.body_m,stop.radius_m,f.now_us);
            if(d.valid)side=d.zone_class;}
        std::cout<<"frame="<<f.frame_id<<" zone_valid="<<result.zone.trusted(f.now_us)<<" points="<<result.zone.inlier_ids.size()
                 <<" residual_m="<<result.zone.residual_m<<" reason="<<result.reason<<" subzone="<<side<<'\n';
        if(!n["expect_valid"].empty()&&result.zone.trusted(f.now_us)!=(int(n["expect_valid"])==1))
            throw std::runtime_error("Replay validity expectation failed at frame "+std::to_string(f.frame_id));
        if(!n["expect_subzone"].empty()&&side!=std::string(n["expect_subzone"]))
            throw std::runtime_error("Replay subzone expectation failed");
    }
    if(!total)throw std::runtime_error("Empty geometry replay");
    std::cout<<"Geometry replay frames="<<total<<" accepted="<<valid<<" hardware_output=disabled\n";return 0;
}
} // namespace rescue
