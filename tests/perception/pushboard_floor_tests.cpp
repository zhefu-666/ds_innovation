#include "rescue/perception/pushboard_floor.hpp"
#include "rescue/perception/capture_monitor.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <cmath>
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
void cargoSurroundings() {
    PushboardFloorConfig c;
    const std::vector<GraspCenterRange> ranges{
        {"ordinary_supply",2000,1280,720,{209,226,1071,489},{647.5f,337},true},
        {"core_supply",2000,1280,720,{268,236,1017,555},{647.5f,337},true},
        {"injured_person",2000,1280,720,{268,236,1017,555},{647.5f,337},true}
    };
    cv::Mat frame(720,1280,CV_8UC3);
    SegDetection cargo;cargo.label="ordinary_supply";cargo.confidence=.95f;
    cargo.box={550,280,160,160};
    const auto see = [&](const std::string& team, std::vector<SegDetection> objects, int expected=1,
                         bool stable=true) {
        return observeTransportFloor(frame,objects,c,team,stable,1000000,ranges,expected);
    };
    for (const std::string team : {"red","blue"}) {
        const auto own=team=="red" ? bgr(179,180,160) : bgr(100,180,160);
        const auto enemy=team=="red" ? bgr(100,180,160) : bgr(179,180,160);
        frame.setTo(own);frame(cargo.box).setTo(bgr(55,180,160));
        auto full=see(team,{cargo});
        assert(full.valid && full.region_valid && full.inside && full.cargo_count==1);
        assert(full.windows.size()==4);
        assert(!see(team,{cargo},1,false).valid);
        // No cargo, missing batch member, or a box outside the measured jaw is not arrival.
        assert(!see(team,{}).region_valid);
        assert(!see(team,{cargo},2).inside);
        auto outside=cargo;outside.box={30,50,100,100};
        assert(!see(team,{outside}).region_valid);
        // Only the immediate floor is required. A narrow half and a centre divider elsewhere
        // must not recreate the obsolete full-image width/height endpoint.
        frame.setTo(cv::Scalar(150,150,150));
        for(const auto& w:full.windows) frame(w.roi).setTo(own);
        frame(cargo.box).setTo(bgr(55,180,160));
        assert(see(team,{cargo}).inside);
        for(size_t side=0;side<4;++side) {
            for(const auto& other : {cv::Scalar(150,150,150),bgr(145,180,150),
                                    bgr(0,0,20),enemy,bgr(80,180,150)}) {
                frame(full.windows[side].roi).setTo(other);
                const auto partial=see(team,{cargo});
                assert(partial.region_valid && !partial.inside);
                frame(full.windows[side].roi).setTo(own);
            }
        }
        // A same-colour load does not count as surrounding floor on white ground.
        frame.setTo(cv::Scalar(150,150,150));frame(cargo.box).setTo(own);
        assert(!see(team,{cargo}).inside);
        // Neighbour occlusion must leave enough visible ground on every side.
        frame.setTo(own);
        auto occluder=cargo;occluder.label="dangerous_object";
        occluder.box=full.windows[0].roi;
        assert(!see(team,{cargo,occluder}).region_valid);
        // All held cargo, with different foreground colours, must independently qualify.
        std::vector<SegDetection> load(3,cargo);
        for(int i=0;i<3;++i) {
            load[i].label=i==0?"ordinary_supply":i==1?"core_supply":"injured_person";
            load[i].box={280+i*280,300,120,120};
            frame(load[i].box).setTo(i==0?bgr(55,180,160):i==1?bgr(0,0,20):bgr(15,180,160));
        }
        auto all=see(team,load,3);
        assert(all.inside && all.cargo_count==3 && all.windows.size()==12);
        frame(all.windows[9].roi).setTo(cv::Scalar(150,150,150));
        assert(!see(team,load,3).inside);
        frame.setTo(own);
        auto edge=cargo;edge.box={600,50,100,600};
        // Use a test range reaching the image edge; clipping cannot satisfy a missing side.
        auto edge_ranges=ranges;edge_ranges[0].area={0,0,1279,719};
        edge.box={600,0,100,200};
        assert(!observeTransportFloor(frame,{edge},c,team,true,1000000,edge_ranges,1).region_valid);
        frame(cargo.box).setTo(bgr(55,180,160));
        cv::Mat small;cv::resize(frame,small,{640,360},0,0,cv::INTER_NEAREST);
        auto scaled_cargo=cargo;scaled_cargo.box={275,140,80,80};
        auto scaled=observeTransportFloor(small,{scaled_cargo},c,team,true,1000000,ranges,1);
        assert(scaled.inside && scaled.cargo_count==1);
    }
    std::cout << "PASS red/blue, four sides, white/purple/divider/enemy, all cargo, occlusion and scaling\n";
}
void cargoSurroundingsTolerateInternalMarkings() {
    PushboardFloorConfig c;
    const std::vector<GraspCenterRange> ranges{
        {"ordinary_supply",2000,1280,720,{209,226,1071,500},{647.5f,337},true}
    };
    cv::Mat frame(720,1280,CV_8UC3);
    SegDetection cargo;cargo.label="ordinary_supply";cargo.confidence=.95f;
    for(const std::string team : {"red","blue"}) {
        const auto own=team=="red" ? bgr(179,180,160) : bgr(100,180,160);
        const auto see=[&](const std::vector<SegDetection>& load,int expected=1) {
            return observeTransportFloor(frame,load,c,team,true,1000000,ranges,expected);
        };
        const auto paint=[&](const cv::Rect& roi,int count) {
            frame(roi).setTo(cv::Scalar(150,150,150));
            for(int i=0;i<count;++i)
                frame.at<cv::Vec3b>(roi.y+i/roi.width,roi.x+i%roi.width)=
                    cv::Vec3b(own[0],own[1],own[2]);
        };
        const auto scene=[&](cv::Rect box,std::array<float,4> fractions) {
            cargo.box=box;frame.setTo(own);
            const auto reference=see({cargo});
            for(size_t i=0;i<4;++i)
                paint(reference.windows[i].roi,cvRound(fractions[i]*reference.windows[i].roi.area()));
            return see({cargo});
        };
        // 05:14 trace: top ~55%, rear ~100%, left ~84%, right 100%.
        auto observed=scene({390,165,274,285},{.55f,1.f,.84f,1.f});
        assert(observed.region_valid && observed.inside);
        assert(observed.own_fraction>.84f && observed.own_fraction<.85f);
        assert(!scene({390,165,274,285},{0,0,0,0}).inside); // outside on white
        assert(!scene({390,165,274,285},{1,0,.5f,.5f}).inside); // crossing entrance
        assert(!scene({390,165,274,285},{.49f,1,1,1}).inside); // side gate despite high total
        // Long boxes distinguish pixel weighting from averaging four percentages.
        assert(!scene({400,300,400,80},{1,.5f,1,1}).inside); // mean 87.5%, pixels 79.3%
        assert(scene({400,300,400,80},{1,1,.5f,.5f}).inside); // mean 75%, pixels 91.4%
        // Exact inclusive thresholds: 50% on top and 80% over all 19200 pixels.
        cargo.box={550,280,160,160};frame.setTo(own);
        const auto reference=see({cargo});
        const int own_pixels[]={2688,5376,3648,3648};
        for(size_t i=0;i<4;++i)paint(reference.windows[i].roi,own_pixels[i]);
        auto boundary=see({cargo});
        assert(boundary.inside && boundary.windows[0].own==.5f);
        assert(std::abs(boundary.own_fraction-.8f)<1e-6f);
        paint(reference.windows[1].roi,5375);
        assert(!see({cargo}).inside); // one pixel below 80%
        // Strong cargo cannot compensate for another cargo below its combined gate.
        std::vector<SegDetection> load(3,cargo);
        for(int i=0;i<3;++i)load[i].box={280+i*280,300,120,120};
        frame.setTo(own);
        const auto all=see(load,3);
        assert(all.inside);
        for(size_t i=4;i<8;++i)paint(all.windows[i].roi,cvRound(.7f*all.windows[i].roi.area()));
        const auto partial=see(load,3);
        assert(partial.region_valid && partial.own_fraction>.8f && !partial.inside);
    }
    std::cout<<"PASS logged red/blue entry, outside/crossing, weighted 50%/80% boundaries and each cargo independently\n";
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
    cargoSurroundingsTolerateInternalMarkings();
    cargoSurroundings();
    std::cout << "PASS UI140 fixed windows and per-cargo surrounding floor\n";
}
