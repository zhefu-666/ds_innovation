#include "rescue/perception/pushboard_floor.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <iostream>
using namespace rescue;
namespace {
cv::Scalar bgr(int h, int s, int v) {
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v)), rgb;
    cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
    auto p = rgb.at<cv::Vec3b>(0, 0);
    return {double(p[0]), double(p[1]), double(p[2])};
}
}
int main(int argc, char** argv) {
    PushboardFloorConfig c; c.enabled = true;
    cv::Mat frame(720,1280,CV_8UC3,cv::Scalar(150,150,150));
    auto see = [&](const std::string& team = "blue", std::vector<SegDetection> objects = {},
                   bool stable = true) {
        return observePushboardFloor(frame, objects, c, team, stable, 1000000);
    };
    auto f = see(); assert(f.valid && !f.inside && f.windows[0].white > .99f);
    // Purple only at the sides has no influence.
    frame(cv::Rect(0,0,100,720)).setTo(bgr(145,180,150));
    assert(!see().inside);
    frame(c.windows[0]).setTo(bgr(145,180,150));
    frame(c.windows[1]).setTo(bgr(145,180,150));
    f = see(); assert(f.valid && !f.inside && f.windows[0].purple > .99f);
    frame(c.windows[0]).setTo(bgr(100,150,160));
    assert(!see().inside); // one-sided crossing
    frame(c.windows[1]).setTo(bgr(100,150,160));
    assert(see().inside); // no purple crossing history required
    assert(!see("red").valid && !see("red").inside);
    assert(!see("blue", {}, false).valid);
    for (int hue : {0, 179}) {
        frame.setTo(bgr(hue,180,160));
        assert(see("red").inside && !see("blue").inside);
    }
    for (auto color : {bgr(55,180,160), bgr(15,180,160), bgr(0,0,20)}) {
        frame.setTo(color);
        assert(!see().valid && !see().inside); // ordinary, injured, core
    }
    // Even a blue object cannot become blue-floor evidence.
    frame.setTo(bgr(100,150,160));
    for (const char* label : {"ordinary_supply","injured_person","core_supply","dangerous_object"}) {
        SegDetection d; d.box = c.windows[0]; d.label = label; d.confidence = .9f;
        assert(!see("blue", {d}).valid);
    }
    SegDetection zone; zone.box = {0,0,1280,720}; zone.label = "blue_safe_zone"; zone.confidence = .9f;
    assert(see("blue",{zone}).inside);
    // Small dirt survives; unknown majority must not be normalized away.
    for (const auto& roi : c.windows)
        frame(cv::Rect(roi.x,roi.y,10,roi.height)).setTo(bgr(80,150,150));
    assert(see().inside);
    for (const auto& roi : c.windows)
        frame(cv::Rect(roi.x,roi.y,50,roi.height)).setTo(bgr(80,150,150));
    assert(!see().inside);
    // Scaling keeps the physical reference windows at the same normalized location.
    frame.setTo(bgr(100,150,160));
    cv::Mat small; cv::resize(frame, small, {640,360});
    assert(observePushboardFloor(small,{},c,"blue",true,1000000).inside);
    if (argc == 3 && std::string(argv[1])!="--ui120-reference") {
        auto inside = cv::imread(argv[1]), outside = cv::imread(argv[2]);
        auto a = observePushboardFloor(inside,{},c,"blue",true,1000000);
        auto b = observePushboardFloor(outside,{},c,"blue",true,1000000);
        assert(a.valid && a.inside && b.valid && !b.inside);
        std::cout << "screenshots inside=" << a.windows[0].own << "," << a.windows[1].own
                  << " outside-white=" << b.windows[0].white << "," << b.windows[1].white << "\n";
    }
    // UI120 uses one aligned half's outer extent, independent of interior cargo.
    for (const std::string team : {"blue", "red"}) {
        const auto color = team=="blue" ? bgr(100,180,160) : bgr(179,180,160);
        frame.setTo(cv::Scalar(150,150,150));
        const std::vector<cv::Point> half{{170,240},{800,240},{1040,719},{60,719}};
        cv::fillConvexPoly(frame,half,color);
        auto base=observeTransportFloor(frame,{},c,team,true,1000000);
        assert(base.valid && base.region_valid && base.inside);
        assert(base.width_fraction>.75f && base.height_fraction>.66f);
        // More than half the floor can be cargo without changing its visible boundary.
        std::vector<SegDetection> cargo;
        for (int i=0;i<3;++i) {
            SegDetection d;d.label=i==0?"ordinary_supply":i==1?"injured_person":"core_supply";
            d.confidence=.9f;d.box={250+i*160,300,140,340};
            frame(d.box).setTo(i==0?bgr(55,180,160):i==1?bgr(15,180,160):bgr(0,0,20));
            cargo.push_back(d);
        }
        auto loaded=observeTransportFloor(frame,cargo,c,team,true,1000000);
        assert(loaded.inside && loaded.region_box==base.region_box);
        assert(loaded.own_fraction<.40f); // Old full-image 70% would reject this case.
        assert(!observeTransportFloor(frame,cargo,c,team,false,1000000).valid);
        auto enemy=observeTransportFloor(frame,cargo,c,team=="blue"?"red":"blue",true,1000000);
        assert(!enemy.region_valid && !enemy.inside);
        cv::Mat small;cv::resize(frame,small,{640,360},0,0,cv::INTER_NEAREST);
        auto scaled=observeTransportFloor(small,{},c,team,true,1000000);
        assert(scaled.inside && std::abs(scaled.width_fraction-loaded.width_fraction)<.01f);
        // A wide but shallow half must not release.
        frame.setTo(cv::Scalar(150,150,150));
        frame(cv::Rect(0,500,1100,220)).setTo(color);
        assert(!observeTransportFloor(frame,{},c,team,true,1000000).inside);
        // Neither two separate halves nor a purple connecting wall may be merged.
        frame.setTo(cv::Scalar(150,150,150));
        frame(cv::Rect(450,240,320,480)).setTo(color);
        frame(cv::Rect(850,240,430,480)).setTo(color);
        frame(cv::Rect(450,210,830,30)).setTo(bgr(121,180,120));
        auto split=observeTransportFloor(frame,{},c,team,true,1000000);
        assert(split.region_valid && !split.inside && split.region_box.width==320);
        // Width and height without sufficient depth in the image still reject.
        frame.setTo(cv::Scalar(150,150,150));
        frame(cv::Rect(100,100,1000,450)).setTo(color);
        assert(!observeTransportFloor(frame,{},c,team,true,1000000).inside);
        // A same-colour object alone is never a floor region.
        frame.setTo(cv::Scalar(150,150,150));
        SegDetection object;object.label="core_supply";object.confidence=.9f;
        object.box={100,240,1000,480};frame(object.box).setTo(color);
        assert(!observeTransportFloor(frame,{object},c,team,true,1000000).region_valid);
    }
    if ((argc==3 || argc==4) && std::string(argv[1])=="--ui120-reference") {
        auto image=cv::imread(argv[2]);assert(!image.empty());
        std::vector<SegDetection> cargo(2);
        cargo[0].label=cargo[1].label="ordinary_supply";
        cargo[0].confidence=cargo[1].confidence=.9f;
        cargo[0].box={672,539,244,173};cargo[1].box={627,317,143,183};
        const auto f=observeTransportFloor(image,cargo,c,"blue",true,1000000);
        assert(f.region_valid && f.inside && f.region_box.x+f.region_box.width<image.cols);
        if (argc==4) {
            cv::polylines(image,std::vector<std::vector<cv::Point>>{f.region_contour},true,{0,255,0},3);
            const std::string label="UI120 HALF W="+std::to_string(int(f.width_fraction*100))+
                "% H="+std::to_string(int(f.height_fraction*100))+
                "% B="+std::to_string(int(f.bottom_fraction*100))+"% - STOP/CONFIRM";
            cv::putText(image,label,{25,40},cv::FONT_HERSHEY_SIMPLEX,.75,{0,255,0},2);
            assert(cv::imwrite(argv[3],image));
        }
        std::cout<<"REFERENCE width="<<f.width_fraction<<" height="<<f.height_fraction
                 <<" bottom="<<f.bottom_fraction<<" blue="<<f.own_fraction<<" box="<<f.region_box<<"\n";
    }
    std::cout << "PASS floor colours, UI120 half contour, mixed cargo, divider and depth thresholds\n";
}
