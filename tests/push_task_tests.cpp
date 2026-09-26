#include "rescue/push_task.hpp"
#include "rescue/config.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace rescue;
bool stopped(const MotionCommand &m) { return m.vx_mps == 0 && m.wz_rps == 0; }
struct Scenario {
    PushTask task;
    PushObservation in;
    Scenario() {
        in.run = in.safety_ok = in.target_valid = in.geometry_valid = in.path_safe = true;
        in.target_id = 7; in.label = "ordinary_supply"; in.distance_m = 0.2f;
        in.in_push_region = true; in.available_count = 5;
        in.zone_valid = in.zone_own = in.zone_aligned = true; in.zone_class = "supply";
    }
    PushOutput tick() { in.now_us += 50000; return task.update(in); }
    PushOutput push() { tick(); tick(); tick(); tick(); return tick(); }
    PushOutput deliver() {
        tick(); // PUSH -> ALIGN
        in.fully_inside = in.off_fence = in.stable = true; in.delivered_count = 1;
        tick(); // ALIGN -> VERIFY
        for(int n=0;n<8;++n) tick(); // VERIFY -> BACK_OUT
        in.separated = true;
        return tick();
    }
};
int main() {
    { Scenario s; auto out=s.push(); assert(out.state==PushState::PUSH_TARGET && out.batch_size==1);
      out=s.deliver(); assert(out.delivered_total==1 && out.first_ordinary_delivered);
      s.in.label="core_supply"; s.in.target_id=8;
      s.tick();s.tick();out=s.tick();assert(out.batch_size==3);
      s.in.run=false;s.tick();s.in.run=true;s.in.label="injured_person";
      s.tick();s.tick();s.tick();out=s.tick();assert(out.batch_size==1 && out.delivered_total==1); }
    for(const auto *label : {"dangerous_object","core_supply","injured_person","unknown"}) {
        Scenario s;s.in.label=label;auto out=s.push();
        assert(out.state==PushState::SEARCH_TARGET && out.batch_size==0 && out.motion.vx_mps==0);
    }
    for(int stage=0;stage<5;++stage) {
        Scenario s;s.push();
        if(stage>=1)s.tick();
        if(stage>=2){s.in.fully_inside=s.in.off_fence=s.in.stable=true;s.in.delivered_count=1;s.tick();}
        if(stage>=3){for(int i=0;i<8;++i)s.tick();}
        if(stage==4)s.in.target_id=99;else s.in.target_valid=false;
        auto out=s.tick(); assert(stopped(out.motion) && out.delivered_total==0);
        assert(out.state==PushState::SEARCH_TARGET);
    }
    { Scenario s;s.push();s.in.zone_own=false;auto out=s.tick();
      assert(stopped(out.motion) && out.delivered_total==0); }
    { Scenario s;s.push();s.in.zone_class="injured";auto out=s.tick();assert(stopped(out.motion)); }
    { Scenario s;s.push();s.tick();s.in.fully_inside=s.in.stable=true;s.in.delivered_count=1;
      PushOutput out;for(int i=0;i<22;++i)out=s.tick();assert(out.delivered_total==0); }
    { Scenario s;s.push();s.tick();s.in.fully_inside=s.in.off_fence=s.in.stable=true;s.in.delivered_count=2;
      PushOutput out;for(int i=0;i<22;++i)out=s.tick();assert(out.delivered_total==0); }
    { Scenario s;s.push();s.in.safety_ok=false;auto out=s.tick();assert(stopped(out.motion)); }
    { Scenario s;s.push();s.in.now_us+=300000;auto out=s.tick();assert(out.state==PushState::WAIT_START && out.motion.vx_mps==0); }
    { Scenario s;s.push();auto out=s.task.update(s.in);assert(out.state==PushState::WAIT_START); }
    { Scenario s;s.push();s.in.distance_m=std::numeric_limits<float>::quiet_NaN();assert(s.tick().motion.vx_mps==0); }
    { Scenario s;s.push();s.deliver();s.in.reset=true;auto out=s.tick();assert(out.delivered_total==0 && !out.first_ordinary_delivered); }
    { Scenario s; s.in.zone_aligned=false;s.push();PushOutput out;
      for(int i=0;i<161;++i)out=s.tick();assert(out.delivered_total==0 && out.motion.vx_mps==0); }
    { Scenario s;s.in.in_push_region=false;s.in.distance_m=0.8f;
      s.tick();s.tick();s.tick();s.tick();assert(s.tick().motion.vx_mps>0);
      s.in.geometry_valid=false;assert(stopped(s.tick().motion)); }
    { Scenario s;s.in.zone_aligned=false;s.push();assert(s.tick().motion.vx_mps>0);
      s.in.path_safe=false;assert(stopped(s.tick().motion)); }
    { Scenario s;s.push();s.tick();assert(s.tick().motion.vx_mps>0);
      s.in.fully_inside=s.in.off_fence=s.in.stable=true;s.in.delivered_count=1;s.tick();
      for(int i=0;i<8;++i)s.tick();
      assert(stopped(s.tick().motion)); // rear clearance absent
      s.in.retreat_safe=true;assert(s.tick().motion.vx_mps<0);
      s.in.separated=true;auto out=s.tick();assert(out.delivered_total==1 && out.motion.vx_mps==0); }
    assert(makeTargetAreas("red")==std::vector<std::string>{"red_safe_zone"});
    assert(makeTargetAreas("blue")==std::vector<std::string>{"blue_safe_zone"});
    assert(makeTargetBalls("red").size()==3);
    std::cout<<"Push rules, delivery, loss, timeout and stop checks passed\n";
}
