#include "rescue/demo_policy.hpp"
#include <cassert>
#include <iostream>
using namespace rescue;
Config parse(std::initializer_list<const char*> values){
    std::vector<char*> args;for(auto v:values)args.push_back(const_cast<char*>(v));return parseArgs(args.size(),args.data());
}
int main(){
    auto c=parse({"demo","--demo-mode","search","--startup-advance-ms","60000","--scan-wz","3"});
    assert(c.startup_advance_ms==0&&c.scan_wz_rps==.25f&&c.match_seconds==60);
    TaskTuning t;configureDemo(c,t);assert(t.demo_search_only&&!t.demo_carry_once&&t.startup_advance_us==0&&t.max_speed==.1f);
    MotionCommand m;m.vx_mps=1;m.wz_rps=3;m.gripper_offset=20;m.camera_pitch_cdeg=4000;m.frame_transaction=9;
    guardDemoMotion("search",m,0,500);assert(m.vx_mps==0&&m.wz_rps==.25f&&m.gripper_offset==0&&m.camera_pitch_cdeg==500&&m.frame_transaction==0);
    guardDemoMotion("search",m,-1,kCameraPitchInvalid);assert(m.vx_mps==0&&m.wz_rps==0);
    c=parse({"demo","--demo-mode","recognize"});assert(c.dry_run&&!c.hardware&&!c.imu&&!c.pitch_feedback);
    for(auto extra:{"--hardware","--imu","--pitch-feedback","--auto-run","--no-match-time-limit","--allow-mechanical-pitch-model"}){
        bool rejected=false;try{parse({"demo","--demo-mode","recognize",extra});}catch(const std::exception&){rejected=true;}assert(rejected);
    }
    bool rejected=false;try{parse({"demo","--demo-mode","search","--demo-seconds","61"});}catch(const std::exception&){rejected=true;}assert(rejected);
    c=parse({"demo","--demo-mode","carry_once"});configureDemo(c,t);assert(!t.demo_search_only&&t.demo_carry_once);
    c=parse({"demo","--demo-mode","carry_once","--controlled-empty-field","--assume-all-safe"});
    assert(c.assume_all_safe&&c.controlled_ignore_clearance);
    c=parse({"demo","--demo-mode","carry_once","--controlled-empty-field","--assume-all-safe","--allow-mechanical-pitch-model","--pitch-presets","500,4000,4000"});
    assert(c.allow_mechanical_pitch_model&&c.pitch_presets_cdeg[1]==4000);
    rejected=false;try{parse({"demo","--demo-mode","carry_once","--controlled-empty-field","--allow-mechanical-pitch-model"});}catch(const std::exception&){rejected=true;}assert(rejected); // 无assume时仍拒绝
    for(auto mode:{"search","recognize"}){
        rejected=false;try{parse({"demo","--demo-mode",mode,"--controlled-empty-field","--assume-all-safe"});}catch(const std::exception&){rejected=true;}assert(rejected);
    }
    rejected=false;try{parse({"demo","--demo-mode","carry_once","--assume-all-safe"});}catch(const std::exception&){rejected=true;}assert(rejected);
    rejected=false;try{parse({"demo","--demo-mode","carry_once","--controlled-empty-field","--controlled-ignore-clearance"});}catch(const std::exception&){rejected=true;}assert(rejected);
    std::cout<<"Standalone demo mode isolation passed\n";
}
