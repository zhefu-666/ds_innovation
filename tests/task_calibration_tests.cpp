#include "rescue/task_calibration.hpp"
#include <cassert>
#include <fstream>
#include <unistd.h>
#include <iostream>
using namespace rescue;
int main(){
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
    unlink(file);std::cout<<"Measured gripper calibration validation passed\n";
}
