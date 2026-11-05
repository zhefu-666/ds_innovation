#include "rescue/perception/pose_temporal_smoother.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
using namespace rescue;
namespace {
ZoneHalves sample(float x = 100) {
    ZoneHalves h{};
    for (int c = 0; c < 2; ++c) {
        h[c].valid = true;
        h[c].score = .9f;
        h[c].box = {50, 50, 400, 300};
        for (int k = 0; k < 4; ++k) {
            h[c].keypoints[k] = {x + 100 * c + 20 * k, 200.f + 10 * k};
            h[c].confidence[k] = .9f;
        }
    }
    return h;
}
ZonePoseMetadata metadata() {
    ZonePoseMetadata m{};
    for (auto& c : m) {
        c.nms_candidates = 1;
        c.unclipped_box = {-10, -10, 500, 400};
    }
    return m;
}
bool same(const ZoneHalves& a, const ZoneHalves& b) {
    for (int c = 0; c < 2; ++c) {
        if (a[c].valid != b[c].valid || a[c].score != b[c].score || a[c].box != b[c].box ||
            a[c].keypoints != b[c].keypoints || a[c].confidence != b[c].confidence)
            return false;
    }
    return true;
}
void decoder() {
    ModelImage image;
    image.sx = image.sy = .5f;
    image.pad_y = 140;
    int dims[] = {1, 18, 5};
    cv::Mat tensor(3, dims, CV_32F, cv::Scalar(0));
    auto put = [&](int row, int i, float v) {
        tensor.ptr<float>()[row * 5 + i] = v;
    };
    for (int i = 0; i < 5; ++i) {
        put(0, i, i == 2 ? 550 : 60);
        put(1, i, 150);
        put(2, i, 200);
        put(3, i, 200);
        put(4, i, i == 4 ? 1.1f : .8f);
        for (int k = 0; k < 4; ++k) {
            put(6 + 3 * k, i, 60 + i);
            put(7 + 3 * k, i, 150);
            put(8 + 3 * k, i, .7f);
        }
    }
    put(0, 3, std::numeric_limits<float>::quiet_NaN());
    const auto old = decodeZonePose(tensor, image, {1280, 720}, .5f);
    ZonePoseMetadata m;
    const auto now = decodeZonePose(tensor, image, {1280, 720}, .5f, &m);
    assert(same(old, now));
    assert(now[0].keypoints[0].x == 120); // first tied anchor
    assert(m[0].nms_candidates == 2 && m[1].nms_candidates == 0);
    assert(m[0].unclipped_box.x == -80 && now[0].box.x == 0);
}
} // namespace
int main(int argc, char** argv) {
    ModelImage image;
    image.sx = image.sy = .5f;
    image.pad_y = 140;
    auto m = metadata();
    auto h = sample();
    const auto untouched = h;
    PoseTemporalSmoother s;
    auto update = [&](uint64_t t, PoseFrameIdentity id = PoseFrameIdentity{}) {
        return s.update(h, m, image, {1280, 720}, t, id);
    };
    // Deterministic sequence exported for independent Python parity.
    if (argc == 2 && std::string(argv[1]) == "--parity") {
        std::cout << std::setprecision(10);
        for (int i = 0; i < 120; ++i) {
            h = sample(float(100 + 20 * std::sin(i * .2) + (i >= 60 ? 80 : 0)));
            const auto r = update(1000000 + 33333 * i);
            assert(r.ok);
            std::cout << i << " " << h[0].keypoints[0].x << " " << r.halves[0].keypoints[0].x
                      << "\n";
        }
        return 0;
    }
    auto a = update(1000000);
    assert(a.ok && same(a.halves, h) && same(h, untouched));
    for (int i = 1; i < 5; ++i)
        assert(same(update(1000000 + 33333 * i).halves, h));
    h = sample(200);
    auto b = update(1200000);
    assert(b.halves[0].keypoints[0].x > 100 && b.halves[0].keypoints[0].x < 200);
    assert(b.halves[1].keypoints[3].x > 260 && b.halves[1].keypoints[3].x < 360);
    assert(same(update(1200000).halves, b.halves)); // duplicate cannot update twice
    h[0].keypoints[0].x += 1;
    assert(!update(1200000).ok);
    assert(same(update(1233333).halves, h)); // error cleared state
    // Per-slot and per-class reset; original values/confidences stay available.
    h[0].confidence[0] = .49f;
    assert(!update(1266666).visible[0][0]);
    h[0].confidence[0] = .5f;
    h[0].keypoints[0].x = 300;
    assert(update(1299999).halves[0].keypoints[0].x == 300);
    h[0].keypoints[0].x = -1;
    auto outside = update(1333332);
    assert(!outside.visible[0][0] && outside.halves[0].confidence[0] == .5f);
    h[0].keypoints[0].x = 400;
    assert(update(1366665).halves[0].keypoints[0].x == 400);
    h[0].valid = false;
    update(1399998);
    h[0].valid = true;
    assert(update(1433331).halves[0].keypoints[0].x == 400);
    m[0].nms_candidates = 2;
    h[0].keypoints[0].x = 500;
    assert(update(1466664).halves[0].keypoints[0].x == 500);
    m[0].nms_candidates = 1;
    h[0].keypoints[0].x = 600;
    assert(update(1499997).halves[0].keypoints[0].x == 600);
    m[0].unclipped_box = {1000, 0, 100, 100};
    h[0].keypoints[0].x = 700;
    assert(update(1533330).reset_reason[0] == "box_discontinuity_raw_reset");
    assert(same(update(1800000).halves, h)); // long gap
    assert(same(update(1700000).halves, h)); // clock reversal
    PoseFrameIdentity id;
    id.frame_id = 9;
    id.source = "video1";
    id.model = "B";
    assert(same(update(1733333, id).halves, h));
    id.frame_id++;
    id.pitch_valid = true;
    id.pitch_cdeg = 100;
    assert(update(1766666, id).reset_reason[0] == "context_changed");
    id.frame_id++;
    id.pitch_cdeg = 200;
    assert(update(1799999, id).reset_reason[0] == "context_changed");
    id.frame_id++;
    id.source = "video2";
    assert(update(1833332, id).reset_reason[0] == "context_changed");
    id.frame_id++;
    id.model = "other";
    assert(update(1866665, id).reset_reason[0] == "context_changed");
    h[0].keypoints[0].x = std::numeric_limits<float>::quiet_NaN();
    assert(!update(1899998, id).ok);
    h = sample();
    h[0].score = .49f;
    assert(!update(1933331).visible[0][0]);
    h[0].score = .5f;
    assert(update(1966664).visible[0][0]);
    // Unequal axes represent rounded odd-size letterboxing, never a hard-coded .5.
    s.reset();
    image.sx = 640.f / 1279;
    image.sy = 359.f / 719;
    image.pad_y = 140;
    const auto r = s.update(h, m, image, {1279, 719}, 1, {});
    assert(same(r.halves, h));
    s.reset();
    h = sample();
    m = metadata();
    id = {};
    id.frame_id = 1;
    s.update(h, m, image, {1279, 719}, 1000000, id);
    h[0].keypoints[0].x += 100;
    id.frame_id = 2;
    const auto odd = s.update(h, m, image, {1279, 719}, 1033333, id);
    const double dt = .033333;
    const double speed = (100 * double(image.sx) / dt) * (-std::expm1(-2 * CV_PI * dt));
    const double cutoff = std::min(8., 1.2 + .016 * speed);
    const double expected = 100 + 100 * (-std::expm1(-2 * CV_PI * cutoff * dt));
    assert(std::abs(odd.halves[0].keypoints[0].x - expected) < 1e-3);
    for (int c = 0; c < 2; ++c)
        for (int k = 0; k < 4; ++k)
            if (c != 0 || k != 0)
                assert(odd.halves[c].keypoints[k] == h[c].keypoints[k]);
    id.frame_id = 3;
    assert(s.update(h, m, image, {1279, 719}, 1033333, id).reset_reason[0] == "time_discontinuity");
    id.frame_id = 4;
    assert(s.update(h, m, image, {1278, 719}, 1066666, id).reset_reason[0] == "context_changed");
    decoder();
    std::cout << "pose smoothing resets, visibility, NMS, raw preservation passed\n";
}
