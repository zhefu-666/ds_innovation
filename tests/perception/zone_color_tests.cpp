#include "rescue/perception/zone_color.hpp"
#include <cassert>
#include <iostream>
using namespace rescue;
int main() {
    ZoneHalves halves;
    for (int i = 0; i < 2; ++i) {
        auto& h = halves[i];
        h.valid = true;
        h.score = .9;
        const float x = 10 + i * 100;
        h.keypoints = {cv::Point2f(x, 10), {x, 100}, {x + 100, 10}, {x + 100, 100}};
        h.confidence = {.9, .9, .9, .9};
    }
    cv::Mat frame(120, 230, CV_8UC3, cv::Scalar(0, 0, 255));
    assert(!ZoneColorClassifier{}.classify(frame, halves, {}, .5).verified);
    auto assumed = previewBlueZoneDefault(
        ZoneColorClassifier{}.classify(frame, halves, {}, .5), true, "blue", true);
    assert(assumed.color == "blue" && assumed.assumed && !assumed.verified &&
           assumed.reason == "blue_pose_model_assumption_preview_only");
    assert(!previewBlueZoneDefault({}, false, "blue", true).assumed);
    assert(!previewBlueZoneDefault({}, true, "red", true).assumed);
    assert(!previewBlueZoneDefault({}, true, "blue", false).assumed);
    ZoneColorConfig c;
    c.measured = true;
    ZoneColorClassifier detector(c);
    auto r = detector.classify(frame, halves, {}, .5);
    assert(r.verified && r.color == "red");
    frame.setTo(cv::Scalar(255, 0, 0));
    r = detector.classify(frame, halves, {}, .5);
    assert(r.verified && r.color == "blue");
    frame(cv::Rect(110, 0, 120, 120)).setTo(cv::Scalar(0, 0, 255));
    assert(!detector.classify(frame, halves, {}, .5).verified);
    frame.setTo(cv::Scalar(100, 100, 100));
    assert(!detector.classify(frame, halves, {}, .5).verified);
    frame.setTo(cv::Scalar(0, 0, 255));
    SegDetection object;
    object.box = {0, 0, 230, 120};
    assert(!detector.classify(frame, halves, {object}, .5).verified);
    { // One usable half is enough; no half at all is not.
        auto one = halves;
        one[1].valid = false;
        r = detector.classify(frame, one, {}, .5);
        assert(r.verified && r.color == "red" && r.halves_used == 1);
        one = halves;
        one[0].keypoints[1].x = -5; // half 0 leaves the frame
        r = detector.classify(frame, one, {}, .5);
        assert(r.verified && r.halves_used == 1);
        one[1].valid = false;
        assert(!detector.classify(frame, one, {}, .5).verified);
        ZoneHalves none;
        assert(!detector.classify(frame, none, {}, .5).verified);
        // Disagreeing halves and one genuinely ambiguous half stay unknown.
        auto mixed = frame.clone();
        mixed(cv::Rect(110, 0, 120, 120)).setTo(cv::Scalar(255, 0, 0));
        assert(!detector.classify(mixed, halves, {}, .5).verified);
    }
    { // Loose polygon: a thin coloured mat on white floor still decides; shares are taken among
      // red/blue pixels only.
        cv::Mat white(120, 230, CV_8UC3, cv::Scalar(235, 235, 235));
        auto strip = white.clone();
        strip(cv::Rect(0, 55, 230, 10)).setTo(cv::Scalar(40, 8, 8)); // dim navy, V=40
        r = detector.classify(strip, halves, {}, .5);
        assert(r.verified && r.color == "blue");
        auto red_strip = white.clone();
        red_strip(cv::Rect(0, 55, 230, 10)).setTo(cv::Scalar(20, 20, 150));
        r = detector.classify(red_strip, halves, {}, .5);
        assert(r.verified && r.color == "red");
        auto clutter = strip.clone();
        clutter(cv::Rect(0, 20, 230, 10))
            .setTo(cv::Scalar(0, 160, 0)); // green is not evidence for either colour
        r = detector.classify(clutter, halves, {}, .5);
        assert(r.verified && r.color == "blue");
        auto mixed_strip = strip.clone();
        mixed_strip(cv::Rect(0, 70, 230, 10)).setTo(cv::Scalar(20, 20, 150));
        assert(!detector.classify(mixed_strip, halves, {}, .5)
                    .verified); // comparable red and blue stays unknown
        auto speck = white.clone();
        speck(cv::Rect(40, 50, 5, 5)).setTo(cv::Scalar(255, 0, 0));
        assert(!detector.classify(speck, halves, {}, .5).verified); // too few coloured pixels
        assert(!detector.classify(white, halves, {}, .5).verified);
    }
    { // Team rule: an enemy-colour zone is verified as enemy, never own; unknown stays unknown in
      // match mode.
        ZoneColorResult red;
        red.color = "red";
        red.verified = true;
        red.reason = "ok";
        auto own = resolveZoneColor(red, "red", false);
        assert(own.verified && own.color == "red" && own.reason == "ok");
        auto enemy = resolveZoneColor(red, "blue", false);
        assert(enemy.verified && enemy.color == "red" && enemy.color != "blue" &&
               enemy.reason == "enemy_zone_color");
        auto unknown = resolveZoneColor({}, "blue", false);
        assert(!unknown.verified && unknown.color.empty());
        auto assumed = resolveZoneColor({}, "blue", true);
        assert(assumed.verified && assumed.assumed && assumed.color == "blue");
        assert(resolveZoneColor(red, "blue", true).color ==
               "red"); // measurement wins over the fallback
    }
    { // Model color belongs to the pose half enclosed by its zone box.
        auto gray = frame.clone();
        gray.setTo(cv::Scalar(100, 100, 100));
        SegDetection blue, red;
        blue.label = "blue_safe_zone";
        red.label = "red_safe_zone";
        blue.confidence = red.confidence = .9f;
        blue.box = red.box = {0, 0, 230, 120};
        auto detected = ZoneColorClassifier{}.classify(gray, halves, {blue}, .5f);
        assert(detected.verified && detected.color == "blue" && detected.halves_used == 2 &&
               detected.reason == "zone_detection");
        assert(resolveZoneColor(detected, "red", false).reason == "enemy_zone_color");
        detected = detector.classify(gray, halves, {red}, .5f);
        assert(detected.verified && detected.color == "red");
        detected = detector.classify(frame, halves, {blue, red}, .5f);
        assert(!detected.verified && detected.color.empty() &&
               detected.reason == "zone_detection_ambiguous");
        assert(!ZoneColorClassifier{}.classify(gray, {}, {blue}, .5f).verified);
        blue.box = {215, 0, 15, 120};
        assert(!ZoneColorClassifier{}.classify(gray, halves, {blue}, .5f).verified);
        auto separate = halves;
        for (auto& point : separate[1].keypoints)
            point.x += 5;
        blue.box = {0, 0, 113, 120};
        red.box = {113, 0, 117, 120};
        detected = detector.classify(gray, separate, {blue, red}, .5f);
        assert(!detected.verified && detected.reason == "zone_detection_halves_disagree");
        // Too few visible corners for model association: zone boxes must not mask HSV.
        auto sparse = halves;
        for (auto& half : sparse) {
            half.confidence[2] = .1f;
            half.confidence[3] = .1f;
        }
        assert(!ZoneColorClassifier{}.classify(gray, sparse, {blue}, .5f).verified);
        blue.box = {0, 0, 230, 120};
        blue.confidence = 0;
        detected = detector.classify(frame, halves, {blue}, .5f);
        assert(detected.verified && detected.color == "red" && detected.reason == "ok");
    }
    std::cout << "Red/blue identity, loose polygons, incomplete halves, occlusion and ambiguous "
                 "color passed\n";
}
