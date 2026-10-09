#include "rescue/task_calibration.hpp"
#include <cassert>
#include <fstream>
#include <unistd.h>
#include <iostream>
using namespace rescue;
int main(){
    assert(!loadFitsHalf(.32f,.6f,.3f,.02f));
    assert(loadFitsHalf(.04f,.6f,.3f,.02f));
    assert(!loadFitsHalf(.14f,.6f,.3f,.02f));
    assert(!loadFitsHalf(0,.6f,.3f,.02f));
    char file[]="/tmp/rescue-calibration-test-XXXXXX";int fd=mkstemp(file);assert(fd>=0);close(fd);
    const auto write=[&](int measured,int pitch,float mouth,const char* area){
        std::ofstream f(file);f<<"{\"schema_version\":1,\"measured\":"<<measured<<",\"image_width\":1280,\"image_height\":720,"
          "\"hold_center_y_m\":0.12,\"mouth_y_m\":"<<mouth<<",\"corridor_half_width_m\":0.11,\"load_radius_m\":0.04,"
          "\"holding_views\":[{\"pitch_cdeg\":"<<pitch<<",\"area\":"<<area<<"}]}";
    };
    TaskTuning legacy;legacy.near_pitch_cdeg=3500;
    const auto rejected=[&](int width=1280){try{loadTaskCalibration(file,legacy,width,720);}catch(const std::exception&){return true;}return false;};
    write(0,3500,.2,"[540,570,820,690]");assert(rejected());
    write(1,2500,.2,"[540,570,820,690]");assert(rejected()); // no relabelling an unmeasured pitch
    write(1,3500,.1,"[540,570,820,690]");assert(rejected());
    write(1,3500,.2,"[540,570,820,900]");assert(rejected());
    write(1,3500,.2,"[540,570,820,690]");assert(rejected(640));
    auto c=loadTaskCalibration(file,legacy,1280,720);
    assert(c.capture.holding.size()==1 && c.capture.holding[0].pitch_cdeg==3500 && c.task.mouth_y_m==.2f);
    {
        std::ofstream f(file);f<<"{\"schema_version\":1,\"measured\":1,\"image_width\":1280,\"image_height\":720,"
          "\"grasp_trigger_y_m\":0.20,\"hold_center_y_m\":0.12,\"injured_hold_center_y_m\":0.14,"
          "\"mouth_y_m\":0.21,\"corridor_half_width_m\":0.11,\"load_radius_m\":0.10,"
          "\"holding_views\":[{\"pitch_cdeg\":4000,\"area\":[440,210,930,720],\"min_visible_bottom_y_px\":440}]}";
    }
    c=loadTaskCalibration(file,TaskTuning{},1280,720);
    assert(c.capture.holding[0].pitch_cdeg==4000 && c.capture.holding[0].min_visible_bottom_y_px==440);
    assert(c.task.injured_hold_center_y_m==.14f && c.task.grasp_trigger_y_m==.20f);
    // Synthetic schema 2 acceptance exists only in a temporary test file.
    const std::string synthetic=R"JSON({"schema_version":2,"version":"frame-20261007-pending","status":"pending_field_calibration","dimensions_source":"user_confirmed_not_field_measured","image_width":1280,"image_height":720,"coordinate_transform_verified":1,"action_mapping_verified":1,"capture_stop_verified":1,"body_envelope_verified":1,"visual_acceptance_passed":1,"box_area_accepted":0,"a6_open_state":1,"a6_close_state":0,"forward_sign":1,"rotation_origin_x_m":0,"rotation_origin_y_m":0,"outer_width_m":0.165,"outer_depth_m":0.09515,"inner_width_m":0.15,"inner_depth_m":0.08008,"entrance_width_m":0.15,"edge_height_m":0.01,"near_inner_m":0.09092,"far_inner_m":0.171,"body_left_m":0.145,"body_right_m":0.145,"body_rear_m":0.1,"body_front_m":0.18107,"body_front_source":"derived_0.28107_minus_0.100_pending_endpoint_verification","grasp_trigger_y_m":0.13096,"maximum_block_depth_m":0.04,"stopping_margin_m":0.005,"open_width_m":0.29,"open_front_m":0.19,"robot_swept_radius_m":0.25,"frame_views":[{"pitch_cdeg":500,"calibrated":1,"fully_observable":1,"polygon":[[10,10],[300,10],[300,300],[10,300]]},{"pitch_cdeg":2000,"calibrated":1,"fully_observable":1,"polygon":[[50,50],[350,50],[350,350],[50,350]]}],"pending":["coordinate origin and sign","0 open 20 close physical and A6 mapping","stop position and block dimensions","two or three view polygons and independent samples","body and moving mechanism envelope","box-area mode acceptance or original-image instance masks"],"maximum_block_width_m":0.04})JSON";
    const auto write2=[&](std::string data){std::ofstream f(file);f<<data;};
    write2(synthetic);c=loadTaskCalibration(file,TaskTuning{},1280,720);
    assert(std::abs(c.task.mouth_y_m-.171f)<1e-6f);
    assert(std::abs(c.task.hold_center_y_m-.13096f)<1e-6f);
    assert(std::abs(c.load_radius_m-.08501883f)<1e-6f);
    assert(c.feedback_open==1&&c.feedback_close==0&&c.frame_views.size()==2);
    assert(c.task.require_multi_view&&c.task.max_speed==.1f&&c.task.attempt_budget_us==60000000);
    const auto invalid2=[&](const std::string& old,const std::string& replacement){auto data=synthetic;auto at=data.find(old);assert(at!=std::string::npos);data.replace(at,old.size(),replacement);write2(data);assert(rejected());};
    invalid2("\"action_mapping_verified\":1","\"action_mapping_verified\":0");
    invalid2("\"coordinate_transform_verified\":1","\"coordinate_transform_verified\":0");
    invalid2("\"visual_acceptance_passed\":1","\"visual_acceptance_passed\":0");
    invalid2("\"a6_close_state\":0","\"a6_close_state\":1");
    invalid2("\"grasp_trigger_y_m\":0.13096","\"grasp_trigger_y_m\":0.2");
    invalid2("\"pitch_cdeg\":2000","\"pitch_cdeg\":500");
    invalid2("\"maximum_block_width_m\":0.04","\"maximum_block_width_m\":0.2");
    {   // TEMP_ASSUMPTION done_flag semantics + image_polygon view matching.
        auto d=synthetic;auto rep=[&](const std::string& o,const std::string& v){auto at=d.find(o);assert(at!=std::string::npos);d.replace(at,o.size(),v);};
        rep("\"a6_open_state\":1,\"a6_close_state\":0","\"a6_semantics\":\"done_flag\",\"a6_done_settle_ms\":900,\"frame_view_matching\":\"image_polygon\",\"a6_open_state\":-1,\"a6_close_state\":-1");
        write2(d);c=loadTaskCalibration(file,TaskTuning{},1280,720);
        assert(c.a6_done_flag&&c.a6_done_settle_ms==900&&c.feedback_open==-1&&c.frame_view_image_polygon);
        auto bad=[&](const std::string& o,const std::string& v){auto x=d;auto at=x.find(o);assert(at!=std::string::npos);x.replace(at,o.size(),v);write2(x);assert(rejected());};
        bad("\"a6_done_settle_ms\":900","\"a6_done_settle_ms\":50");
        bad("\"a6_done_settle_ms\":900","\"a6_done_settle_ms\":900.5");
        bad("\"done_flag\"","\"guess\"");
        bad("\"image_polygon\"","\"pixels\"");
        bad("\"action_mapping_verified\":1","\"action_mapping_verified\":0");
        write2(synthetic);c=loadTaskCalibration(file,TaskTuning{},1280,720);assert(!c.a6_done_flag&&!c.frame_view_image_polygon);
    }
    // Search requires accepted body/action geometry but does not pretend capture calibration exists.
    auto search_data=synthetic;
    auto replace_one=[&](const std::string& old,const std::string& value){auto at=search_data.find(old);assert(at!=std::string::npos);search_data.replace(at,old.size(),value);};
    replace_one("\"visual_acceptance_passed\":1","\"visual_acceptance_passed\":0");
    replace_one("\"capture_stop_verified\":1","\"capture_stop_verified\":0");
    replace_one("\"grasp_trigger_y_m\":0.13096","\"grasp_trigger_y_m\":0");
    write2(search_data);assert(rejected());
    c=loadTaskCalibration(file,TaskTuning{},1280,720,true);
    assert(c.frame_views.empty()&&c.feedback_open==1&&c.robot_swept_radius_m>0);
    replace_one("\"body_envelope_verified\":1","\"body_envelope_verified\":0");write2(search_data);
    bool search_rejected=false;try{loadTaskCalibration(file,TaskTuning{},1280,720,true);}catch(const std::exception&){search_rejected=true;}assert(search_rejected);
    unlink(file);std::cout<<"Measured gripper calibration validation passed\n";
}
