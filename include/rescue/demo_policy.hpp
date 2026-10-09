#pragma once
#include "rescue/config.hpp"
#include "rescue/push_task.hpp"
namespace rescue {
inline void configureDemo(const Config& c,TaskTuning& t){
    if(c.demo_mode=="none")return;
    t.startup_advance_us=0;t.enable_search_cues=false;t.enable_short_push=false;
    t.max_speed=.10f;t.approach_speed=.10f;t.rush_speed=.05f;
    t.scan_wz=std::min(t.scan_wz,.25f);t.turn_wz=std::min(t.turn_wz,.25f);
    t.max_wz=.25f;t.min_turn_wz=std::min(t.min_turn_wz,.25f);
    t.attempt_budget_us=uint64_t(c.demo_seconds)*1000000;t.approach_limit_m=.30f;t.retreat_limit_m=.20f;
    t.demo_search_only=c.demo_mode=="search";t.demo_carry_once=c.demo_mode=="carry_once";
}
// Independent actuator guard, applied even if a future task-state edit regresses isolation.
inline void guardDemoMotion(const std::string& mode,MotionCommand& m,int held_angle,int16_t held_pitch){
    if(mode=="recognize"){m.vx_mps=m.wz_rps=0;return;}
    if(mode=="search"){
        m.vx_mps=0;m.wz_rps=std::clamp(m.wz_rps,-.25f,.25f);
        if(held_angle==0||held_angle==20)m.gripper_offset=held_angle;
        else m.wz_rps=0;
        m.frame_transaction=0;
        if(held_pitch!=kCameraPitchInvalid)m.camera_pitch_cdeg=held_pitch;
        else m.wz_rps=0;
    }
}
}
