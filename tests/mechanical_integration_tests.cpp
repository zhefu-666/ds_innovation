#include "rescue/pixel_selector.hpp"
#include "rescue/multi_view_capture.hpp"
#include "rescue/task_calibration.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace rescue;
SegDetection object(int id,cv::Rect box,std::string label="ordinary_supply") {
    SegDetection d;d.track_id=id;d.box=box;d.label=label;d.confidence=.9;d.timestamp_us=100000;
    d.ground_position_valid=d.ground_contact_valid=true;d.body_xy_m={0,.13f};return d;
}
int main(){
    assert(frameAngle(FrameAction::Open)==0&&frameAngle(FrameAction::Close)==20);
    PixelSelector selector;auto small=object(1,{10,10,10,10}),large=object(2,{30,10,30,30}),illegal=object(3,{0,0,90,90},"dangerous_object");
    PixelSelection s;
    for(int i=0;i<3;++i){small.timestamp_us=large.timestamp_us=100000+i*10000;s=selector.update({small,large,illegal},{100,100},500,true,100000+i*10000,false,.25);}
    assert(s.locked&&s.id==2&&s.mode=="box_area"&&std::abs(s.raw-.09)<1e-6);
    s=selector.update({small,large},{100,100},600,true,140000,false,.25);assert(!s.locked);
    selector.reset();large.mask=cv::Mat::zeros(100,100,CV_8UC1);large.mask(cv::Rect(30,10,2,2)).setTo(255);
    small.mask=cv::Mat::zeros(100,100,CV_8UC1);small.mask(small.box).setTo(255);
    for(int i=0;i<3;++i){small.timestamp_us=large.timestamp_us=150000+i*10000;s=selector.update({small,large},{100,100},500,true,150000+i*10000,false,.25);}
    assert(s.id==1&&s.mode=="mask_pixels");
    s=selector.update({small,illegal},{0,0},500,true,190000,false,.25);assert(!s.locked);
    { // prefer_label restricts the lock to the preferred class when it is visible, else falls back.
        PixelSelector pref;auto big=object(5,{10,10,60,60}),inj=object(6,{80,10,10,10},"injured_person");
        PixelSelection p;
        for(int i=0;i<3;++i){big.timestamp_us=inj.timestamp_us=500000+i*10000;p=pref.update({big,inj},{100,100},500,true,500000+i*10000,true,.25,{},"injured_person");}
        assert(p.locked&&p.id==6);
        PixelSelector fallback;
        for(int i=0;i<3;++i){big.timestamp_us=600000+i*10000;p=fallback.update({big},{100,100},500,true,600000+i*10000,true,.25,{},"injured_person");}
        assert(p.locked&&p.id==5);
    }
    // A challenger must exceed the hysteresis threshold for three consecutive new frames.
    PixelSelector hysteresis;small.mask.release();large.mask.release();
    small.box={10,10,20,20};large.box={40,10,10,10};
    auto rank=[&](uint64_t t){small.timestamp_us=large.timestamp_us=t;return hysteresis.update({small,large},{100,100},500,true,t,false,.25);};
    rank(300000);rank(320000);assert(rank(340000).id==1);
    large.box={40,10,30,30};assert(rank(360000).id==1);assert(rank(380000).id==1);assert(rank(400000).id==2);
    assert(!hysteresis.update({small,large},{100,100},500,true,410000,false,.25).locked); // repeated captured frame
    assert(!rank(800000).locked); // gap breaks continuity
    FrameView a{500,{{5,5},{95,5},{95,95},{5,95}},true,true},b=a;b.pitch_cdeg=2000;
    MultiViewCapture multi({a,b},{100,100});uint64_t now=1000000;
    auto d=object(1,{25,25,10,10});
    auto sample=[&](bool has=true){now+=20000;d.timestamp_us=now;multi.update(now,now,7,true,true,multi.desiredPitch(),true,has?std::vector<SegDetection>{d}:std::vector<SegDetection>{});};
    multi.begin(now,7,1);for(int i=0;i<4;++i)sample();assert(!multi.result().finished);
    sample();assert(multi.result().view==1);d.track_id=99; // ids need not survive view change
    for(int i=0;i<5;++i){sample();}assert(multi.result().verdict==CaptureVerdict::Enclosed&&multi.result().inventory.total()==1);
    multi.begin(now,7,2);for(int i=0;i<5;++i)sample();d.label="core_supply";
    for(int i=0;i<5;++i){sample();}assert(multi.result().finished&&multi.result().verdict==CaptureVerdict::Uncertain);
    multi.begin(now,7,3);for(int i=0;i<10;++i)sample(false);assert(multi.result().verdict==CaptureVerdict::Empty);
    multi.begin(now,7,4);multi.update(now+1,now+1,8,true,true,500,true,{});assert(multi.result().finished&&multi.result().verdict==CaptureVerdict::Uncertain);
    multi.begin(now,7,5);d=object(4,{90,30,10,10});for(int i=0;i<10;++i)sample();assert(multi.result().verdict==CaptureVerdict::Uncertain);
    multi.update(now+8000001,now+8000001,7,true,true,500,true,{});assert(multi.result().finished);
    multi.begin(now,7,6);multi.update(now+1,now+1,7,true,false,500,true,{});assert(multi.result().finished);
    // A stale/duplicate frame cannot satisfy the minimum five independent frames.
    multi.begin(now,7,7);for(int i=0;i<10;++i)multi.update(now+1,now+1,7,true,true,500,true,{});assert(multi.result().frames==1);
    {   // image_polygon: no ground ranging; per-view pixel continuity, cross-view class counts only.
        MultiViewCapture img({a,b},{100,100},true);uint64_t t=2000000;
        auto g=object(1,{25,25,10,10});g.ground_position_valid=g.ground_contact_valid=false;g.body_xy_m={NAN,NAN};
        auto feed=[&](std::vector<SegDetection> ds){t+=20000;for(auto& x:ds)x.timestamp_us=t;img.update(t,t,9,true,true,img.desiredPitch(),true,ds);};
        img.begin(t,9,1);for(int i=0;i<5;++i)feed({g});assert(img.result().view==1);
        g.box={50,60,12,12}; // different pixel position at the second pitch is expected
        for(int i=0;i<5;++i)feed({g});assert(img.result().verdict==CaptureVerdict::Enclosed&&img.result().inventory.total()==1);
        img.begin(t,9,2);for(int i=0;i<5;++i)feed({g});auto h=g;h.label="core_supply";
        for(int i=0;i<5;++i)feed({h});assert(img.result().finished&&img.result().verdict==CaptureVerdict::Uncertain);
        img.begin(t,9,3);for(int i=0;i<10;++i)feed({});assert(img.result().verdict==CaptureVerdict::Empty);
        // Jumping pixel position within one view restarts the five-frame count.
        img.begin(t,9,4);auto j=g;for(int i=0;i<4;++i){j.box.x=i%2?20:60;feed({j});}assert(img.result().view==0&&img.result().frames==1);
        // Boundary-crossing box stays uncertain even without ground requirements.
        img.begin(t,9,5);auto e=object(2,{90,30,10,10});for(int i=0;i<10;++i)feed({e});assert(img.result().verdict==CaptureVerdict::Uncertain);
        // Body-ground mode still rejects detections lacking ground validity.
        MultiViewCapture body({a,b},{100,100});body.begin(t,9,6);
        for(int i=0;i<10;++i){t+=20000;g.timestamp_us=t;body.update(t,t,9,true,true,body.desiredPitch(),true,{g});}
        assert(body.result().verdict!=CaptureVerdict::Enclosed);
    }
    std::cout<<"mechanical integration checks passed\n";
}
