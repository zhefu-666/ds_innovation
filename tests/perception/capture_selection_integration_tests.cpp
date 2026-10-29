#include "rescue/perception/pixel_selector.hpp"
#include "rescue/perception/multi_view_capture.hpp"
#include "rescue/config/task_calibration.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace rescue;
SegDetection object(int id, cv::Rect box, std::string label = "ordinary_supply") {
    SegDetection d;
    d.track_id = id;
    d.box = box;
    d.label = label;
    d.confidence = .9;
    d.timestamp_us = 100000;
    d.ground_position_valid = d.ground_contact_valid = true;
    d.body_xy_m = {0, .13f};
    return d;
}
void splitRetreatBoxEvidence() {
    constexpr uint64_t now = 1000000;
    auto original = object(7, {300, 80, 60, 60});
    auto centre = object(8, {610, 0, 60, 60});
    original.timestamp_us = centre.timestamp_us = now;
    original.ground_position_valid = original.ground_contact_valid = false;
    centre.ground_position_valid = centre.ground_contact_valid = false;
    PushObservation in;
    applySplitRetreatBox(in, {original, centre}, {1280, 720}, now, .25f, 7);
    assert(in.split_ordinary_box == original.box && in.split_ordinary_box_us == now);
    assert(in.split_ordinary_box_id == 7 && in.split_image_width_px == 1280);
    assert(!in.target_valid && !in.geometry_valid);
    applySplitRetreatBox(in, {centre}, {1280, 720}, now, .25f, 7);
    assert(in.split_ordinary_box.y == 0 && in.split_ordinary_box_us == now);
    for (int invalid = 0; invalid < 6; ++invalid) {
        auto bad = centre;
        if (invalid == 0)
            bad.label = "core_supply";
        if (invalid == 1)
            bad.timestamp_us = now - 200001;
        if (invalid == 2)
            bad.timestamp_us = now + 1;
        if (invalid == 3)
            bad.confidence = .24f;
        if (invalid == 4)
            bad.box.width = 0;
        if (invalid == 5)
            bad.box.y = -100; // entirely outside, not a top-edge observation
        applySplitRetreatBox(in, {bad}, {1280, 720}, now, .25f, 7);
        assert(in.split_ordinary_box.empty() && in.split_ordinary_box_us == 0);
    }
    centre.box.y = -1; // detector coordinates crossing the image top are clipped to Y=0
    applySplitRetreatBox(in, {centre}, {1280, 720}, now, .25f, 7);
    assert(in.split_ordinary_box.y == 0 && in.split_ordinary_box.height == 59);
}
void nearOwnZoneExclusion() {
    const uint64_t now = 1000000;
    for (const std::string team : {"blue", "red"}) {
        auto zone = object(-1, {200, 200, 300, 300}, team + "_safe_zone");
        zone.timestamp_us = now;
        zone.ground_position_valid = zone.ground_contact_valid = false;
        for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"}) {
            auto cargo = object(11, {140, 250, 30, 30}, label);
            cargo.timestamp_us = now;
            const auto excluded = [&](cv::Rect box) {
                cargo.box = box;
                return targetsNearOwnZone({cargo, zone}, team, {1000, 1000}, now, .25f);
            };
            assert(excluded({140, 250, 30, 30}) == std::vector<int>{11}); // 30 px boundary
            assert(excluded({139, 250, 30, 30}).empty());                 // 31 px away
            assert(excluded({250, 140, 30, 30}) == std::vector<int>{11});
            assert(excluded({250, 531, 30, 30}).empty());
            assert(excluded({150, 150, 30, 30}) == std::vector<int>{11}); // diagonal < 30
            assert(excluded({140, 140, 30, 30}).empty());                 // diagonal > 30
            assert(excluded({180, 250, 30, 30}) == std::vector<int>{11}); // overlap
            assert(excluded({250, 250, 30, 30}) == std::vector<int>{11}); // inside
            cargo.box = {280, 500, 60, 60};
            auto scaled = zone;
            scaled.box = {400, 400, 600, 600};
            assert(targetsNearOwnZone({cargo, scaled}, team, {2000, 2000}, now, .25f) ==
                   std::vector<int>{11});
            cargo.box = {140, 250, 30, 30};
            auto enemy = zone;
            enemy.label = team == "blue" ? "red_safe_zone" : "blue_safe_zone";
            assert(targetsNearOwnZone({cargo, enemy}, team, {1000, 1000}, now, .25f).empty());
            assert(targetsNearOwnZone({cargo, enemy, zone}, team, {1000, 1000}, now, .25f) ==
                   std::vector<int>{11});
            auto bad = zone;
            bad.timestamp_us = now - 200001;
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            bad.timestamp_us = now + 1;
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            bad.timestamp_us = now;
            bad.confidence = .24f;
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            bad = zone;
            bad.box = {1100, 200, 100, 100};
            assert(targetsNearOwnZone({cargo, bad}, team, {1000, 1000}, now, .25f).empty());
            assert(targetsNearOwnZone({cargo, zone}, team, {0, 0}, now, .25f).empty());
            assert(targetsNearOwnZone({cargo, zone}, "unknown", {1000, 1000}, now, .25f).empty());
        }
    }
    // A late zone veto immediately releases a lock, including relaxed and row modes.
    for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"}) {
        PixelSelector selector;
        selector.setRelax(2);
        selector.setRow(true);
        auto near = object(21, {140, 250, 30, 30}, label);
        auto far = object(22, {800, 250, 10, 10});
        auto zone = object(-1, {200, 200, 300, 300}, "blue_safe_zone");
        PixelSelection selected;
        for (int i = 0; i < 3; ++i) {
            near.timestamp_us = far.timestamp_us = now + i * 10000;
            selected = selector.update(
                {near, far}, {1000, 1000}, 500, true, near.timestamp_us, true, .25f);
        }
        assert(selected.locked && selected.id == 21);
        for (int i = 3; i < 7; ++i) {
            near.timestamp_us = far.timestamp_us = zone.timestamp_us = now + i * 10000;
            const auto rejected = targetsNearOwnZone(
                {near, far, zone}, "blue", {1000, 1000}, near.timestamp_us, .25f);
            assert(rejected == std::vector<int>{near.track_id});
            selected = selector.update(
                {near, far}, {1000, 1000}, 500, true, near.timestamp_us, true, .25f, rejected);
            assert(selected.id != near.track_id);
            const auto old = makePushObservation(
                {near, far}, near.timestamp_us, true, .25f, near.track_id, rejected);
            assert(!old.target_valid);
            near.track_id = 23; // tracker re-ID cannot evade the spatial veto
        }
        assert(selected.locked && selected.id == far.track_id);
        near.timestamp_us = zone.timestamp_us = now + 80000;
        const auto rejected =
            targetsNearOwnZone({near, zone}, "blue", {1000, 1000}, near.timestamp_us, .25f);
        assert(
            !makePushObservation({near}, near.timestamp_us, true, .25f, -1, rejected).target_valid);
    }
}
void persistentTargetTests() {
    for (int n : {2, 3, 5}) {
        PixelSelector sel;
        uint64_t now = 1000000;
        std::vector<SegDetection> ds;
        for (int i = 0; i < n; ++i) {
            auto d = object(i + 1, {80 + i * 160, 300, i ? 40 : 70, i ? 40 : 70});
            d.body_xy_m = {-.5f + .25f * i, .8f};
            ds.push_back(d);
        }
        auto step = [&] {
            now += 20000;
            for (auto& d : ds)
                d.timestamp_us = now;
            return sel.update(ds, {1000, 800}, 500, true, now, false, .25f);
        };
        step();
        step();
        auto out = step();
        assert(out.locked && out.id == 1);
        sel.syncTaskTarget(1, "ordinary_supply");
        sel.setRow(true);
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < n; ++j)
                ds[j].box.width = ds[j].box.height = j == 0 ? 35 : (i / 5 % n == j ? 110 : 60);
            out = step();
            assert(out.locked && out.id == 1 && out.event == "target_held");
        }
        ds[0].ground_contact_valid = ds[0].ground_position_valid = false;
        assert(step().id == 1);
        sel.syncTaskTarget(-1, "");
        assert(!step().locked);
    }
    {
        PixelSelector sel;
        uint64_t now = 2000000;
        auto a = object(1, {100, 300, 60, 60});
        auto b = object(2, {700, 300, 50, 50});
        auto step = [&](std::vector<SegDetection> ds, uint64_t dt = 20000) {
            now += dt;
            for (auto& d : ds)
                d.timestamp_us = now;
            return sel.update(ds, {1000, 800}, 500, true, now, false, .25f);
        };
        step({a, b});
        step({a, b});
        assert(step({a, b}).id == 1);
        sel.syncTaskTarget(1, a.label);
        for (int i = 0; i < 14; ++i) {
            auto out = step({b});
            assert(!out.locked && out.held_id == 1 && out.released_id == -1);
        }
        assert(step({a, b}).id == 1);
        auto re = a;
        re.track_id = 9;
        auto other = a;
        other.track_id = 10;
        other.box.x += 20;
        for (int i = 0; i < 3; ++i) {
            const auto out = step({re, other, b});
            assert(!out.locked && out.event == "rebind_ambiguous");
        }
        step({re, b});
        step({re, b});
        auto out = step({re, b});
        assert(out.id == 9 && out.rebound_from == 1);
        sel.syncTaskTarget(1, a.label); // A task guard may delay accepting the rebind.
        out = step({re, b});
        assert(out.id == 9 && out.rebound_from == 1 && out.event == "await_task_rebind");
        sel.syncTaskTarget(9, a.label);
        for (int i = 0; i < 15; ++i)
            assert(!step({b}).locked);
        out = step({b});
        assert(out.released_id == 9 && out.event == "target_lost_timeout");
        sel.syncTaskTarget(9, a.label);
        out = step({b});
        assert(!out.locked && out.released_id == 9 && out.event == "await_task_release");
        sel.syncTaskTarget(-1, "");
        step({b});
        step({b});
        out = step({b});
        assert(out.id == 2 && out.rebound_from == -1);
        b.timestamp_us = now + 20000;
        out = sel.update({b}, {1000, 800}, 500, true, now + 20000, false, .25f, {2});
        assert(!out.locked && out.released_id == 2 && out.event == "target_rejected");
    }
    {
        PixelSelector sel;
        uint64_t now = 4000000;
        auto a = object(1, {100, 300, 60, 60});
        a.body_xy_m = {-.2f, .8f};
        auto b = object(2, {700, 300, 50, 50});
        b.body_xy_m = {.4f, .8f};
        auto step = [&](std::vector<SegDetection> ds, int16_t pitch, bool stable, float yaw) {
            now += 50000;
            for (auto& d : ds)
                d.timestamp_us = now;
            return sel.update(ds, {1000, 800}, pitch, stable, now, false, .25f, {}, "", yaw);
        };
        step({a, b}, 500, true, 0);
        step({a, b}, 500, true, 0);
        assert(step({a, b}, 500, true, 0).id == 1);
        sel.syncTaskTarget(1, a.label);
        for (int i = 0; i < 8; ++i)
            assert(!step({}, 2000, false, 0).locked);
        a.track_id = 9;
        a.box = {500, 100, 70, 70};
        const float angle = std::atan2(.2f, .8f) - .1f, range = std::hypot(.2f, .8f);
        a.body_xy_m = {-range * std::sin(angle), range * std::cos(angle)};
        step({a, b}, 2000, true, .1f);
        step({a, b}, 2000, true, .1f);
        const auto out = step({a, b}, 2000, true, .1f);
        assert(out.id == 9 && out.rebound_from == 1 && out.event == "view_reacquired");
    }
    {
        PixelSelector sel;
        uint64_t now = 6000000;
        auto a = object(1, {100, 300, 60, 60});
        auto step = [&](int16_t pitch, bool stable) {
            now += 50000;
            a.timestamp_us = now;
            return sel.update({a}, {1000, 800}, pitch, stable, now, false, .25f);
        };
        step(500, true);
        step(500, true);
        assert(step(500, true).id == 1);
        step(2000, false);
        PixelSelection out;
        for (int i = 0; i < 7; ++i) {
            out = step(2000, true);
            assert(!out.locked);
        }
        assert(out.released_id == 1 && out.event == "target_lost_timeout");
        step(2000, true);
        step(2000, true);
        assert(step(2000, true).id == 1);
    }
}
void scatterSelectionLifecycle() {
    PixelSelector selector;
    auto d = object(13, {600, 280, 26, 26});
    d.body_xy_m = {0.f, 1.73f};
    PixelSelection selected;
    uint64_t now = 2000000;
    const auto step = [&](PushState state, int target_id) {
        selector.syncTaskState(state, target_id, "ordinary_supply");
        d.timestamp_us = now;
        selected = selector.update({d}, {1280, 720}, -3000, true, now, false, .5f);
        now += 30000;
    };
    step(PushState::SCAN, -1);
    step(PushState::SCAN, -1);
    step(PushState::SCAN, -1);
    assert(selected.locked && selected.id == 13);
    // Entry clears the old episode once; ongoing scatter reaches three fresh frames.
    for (int i = 0; i < 10; ++i) {
        step(PushState::SCATTER, -1);
        assert(selected.locked == (i >= 2));
    }
    step(PushState::SCAN, -1);
    assert(selected.locked && selected.id == 13);
    step(PushState::APPROACH, 13);
    assert(selected.locked && selected.id == 13);
    step(PushState::PILE, -1);
    assert(!selected.locked);
    step(PushState::PILE, -1);
    assert(!selected.locked);
    step(PushState::PILE, -1);
    assert(selected.locked);
    step(PushState::SCATTER, -1);
    assert(!selected.locked);
}

int main() {
    splitRetreatBoxEvidence();
    scatterSelectionLifecycle();
    persistentTargetTests();
    nearOwnZoneExclusion();
    assert(frameAngle(FrameAction::Open) == 0 && frameAngle(FrameAction::Close) == 20);
    PixelSelector selector;
    auto small = object(1, {10, 10, 10, 10}), large = object(2, {30, 10, 30, 30}),
         illegal = object(3, {0, 0, 90, 90}, "dangerous_object");
    PixelSelection s;
    for (int i = 0; i < 3; ++i) {
        small.timestamp_us = large.timestamp_us = 100000 + i * 10000;
        s = selector.update(
            {small, large, illegal}, {100, 100}, 500, true, 100000 + i * 10000, false, .25);
    }
    assert(s.locked && s.id == 2 && s.mode == "box_area" && std::abs(s.raw - .09) < 1e-6);
    s = selector.update({small, large}, {100, 100}, 600, true, 140000, false, .25);
    assert(!s.locked);
    selector.reset();
    large.mask = cv::Mat::zeros(100, 100, CV_8UC1);
    large.mask(cv::Rect(30, 10, 2, 2)).setTo(255);
    small.mask = cv::Mat::zeros(100, 100, CV_8UC1);
    small.mask(small.box).setTo(255);
    for (int i = 0; i < 3; ++i) {
        small.timestamp_us = large.timestamp_us = 150000 + i * 10000;
        s = selector.update({small, large}, {100, 100}, 500, true, 150000 + i * 10000, false, .25);
    }
    assert(s.id == 1 && s.mode == "mask_pixels");
    s = selector.update({small, illegal}, {0, 0}, 500, true, 190000, false, .25);
    assert(!s.locked);
    { // The same physical block re-IDed on every frame still confirms after three frames.
        PixelSelector rid;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            auto b = object(20 + i, {40, 40, 30, 30});
            b.timestamp_us = 900000 + i * 10000;
            p = rid.update({b}, {1000, 1000}, 500, true, 900000 + i * 10000, false, .25);
        }
        assert(p.locked && p.id == 22);
    }
    { // A long disappearance releases the episode; no unbounded nearest-pixel inheritance.
        const auto run = [&](uint64_t window) {
            PixelSelector sel;
            sel.setRelock(window, .10);
            PixelSelection p;
            uint64_t t = 2000000;
            for (int i = 0; i < 4; ++i) {
                auto a = object(40, {100, 300, 40, 40});
                a.timestamp_us = t;
                p = sel.update({a}, {1000, 1000}, 500, true, t, false, .25);
                t += 100000;
            }
            assert(p.locked && p.id == 40);
            for (int i = 0; i < 30; ++i) {
                p = sel.update({}, {1000, 1000}, 500, true, t, false, .25);
                t += 100000;
            }
            for (int i = 0; i < 4; ++i) {
                auto near_ = object(41, {110, 310, 40, 40});
                auto far_ = object(42, {700, 300, 50, 50});
                near_.timestamp_us = far_.timestamp_us = t;
                p = sel.update({near_, far_}, {1000, 1000}, 500, true, t, false, .25);
                t += 100000;
            }
            return p.id;
        };
        assert(run(4000000) == 42 && run(1500000) == 42);
    }
    { // Pile edge: an injured piece on the robot->target line in front of the larger block ranks it
      // behind a clean one; relax levels lift that.
        const auto run = [&](int relax) {
            PixelSelector lane;
            lane.setRelax(relax);
            PixelSelection p;
            for (int i = 0; i < 3; ++i) {
                const uint64_t t = 700000 + i * 10000;
                auto a = object(30, {10, 10, 60, 60});
                a.body_xy_m = {0, .60f};
                auto in_front = object(31, {80, 10, 5, 5}, "injured_person");
                in_front.body_xy_m = {.02f, .40f};
                auto clean = object(32, {10, 80, 40, 40});
                clean.body_xy_m = {.40f, .50f};
                a.timestamp_us = in_front.timestamp_us = clean.timestamp_us = t;
                p = lane.update({a, in_front, clean}, {200, 200}, 500, true, t, true, .25);
            }
            return p.id;
        };
        assert(run(0) == 32 && run(1) == 30 && run(2) == 30);
    }
    { // prefer_label restricts the lock to the preferred class when it is visible, else falls back.
        PixelSelector pref;
        auto big = object(5, {10, 10, 60, 60}), inj = object(6, {80, 10, 10, 10}, "injured_person");
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            big.timestamp_us = inj.timestamp_us = 500000 + i * 10000;
            p = pref.update({big, inj},
                            {100, 100},
                            500,
                            true,
                            500000 + i * 10000,
                            true,
                            .25,
                            {},
                            "injured_person");
        }
        assert(p.locked && p.id == 6);
        PixelSelector fallback;
        for (int i = 0; i < 3; ++i) {
            big.timestamp_us = 600000 + i * 10000;
            p = fallback.update(
                {big}, {100, 100}, 500, true, 600000 + i * 10000, true, .25, {}, "injured_person");
        }
        assert(p.locked && p.id == 5);
    }
    { // Two injured blocks in view: the larger one is on end and rejected, so the lock goes to the
      // lying one.
        PixelSelector two;
        auto up = object(8, {10, 10, 50, 50}, "injured_person"),
             lie = object(9, {70, 10, 20, 10}, "injured_person");
        up.injured_upright_rejected = true;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            up.timestamp_us = lie.timestamp_us = 700000 + i * 10000;
            p = two.update({up, lie},
                           {100, 100},
                           500,
                           true,
                           700000 + i * 10000,
                           true,
                           .25,
                           {},
                           "injured_person");
        }
        assert(p.locked && p.id == 9);
        // Verdict arrives late: a lock on the (still undecided) on-end block is released and moves
        // to the lying one.
        PixelSelector late;
        up.injured_upright_rejected = false;
        for (int i = 0; i < 3; ++i) {
            up.timestamp_us = lie.timestamp_us = 800000 + i * 10000;
            p = late.update({up, lie},
                            {100, 100},
                            500,
                            true,
                            800000 + i * 10000,
                            true,
                            .25,
                            {},
                            "injured_person");
        }
        assert(p.locked && p.id == 8);
        up.injured_upright_rejected = true;
        for (int i = 3; i < 7; ++i) { // release tick, then three fresh confirmations
            up.timestamp_us = lie.timestamp_us = 800000 + i * 10000;
            p = late.update({up, lie},
                            {100, 100},
                            500,
                            true,
                            800000 + i * 10000,
                            true,
                            .25,
                            {},
                            "injured_person");
        }
        assert(p.locked && p.id == 9);
    }
    { // A larger box without ground contact (image-edge clip) never takes the lock from a
      // measurable one.
        PixelSelector geo;
        auto edge = object(12, {10, 10, 60, 60}, "injured_person"),
             ok = object(15, {10, 80, 30, 15}, "injured_person");
        edge.ground_contact_valid = false;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            edge.timestamp_us = ok.timestamp_us = 900000 + i * 10000;
            p = geo.update({edge, ok},
                           {100, 100},
                           2000,
                           true,
                           900000 + i * 10000,
                           true,
                           .25,
                           {},
                           "injured_person");
        }
        assert(p.locked && p.id == 15);
        // No measurable candidate at all: old behaviour (largest box) is kept.
        PixelSelector none;
        ok.ground_contact_valid = false;
        for (int i = 0; i < 3; ++i) {
            edge.timestamp_us = ok.timestamp_us = 950000 + i * 10000;
            p = none.update({edge, ok},
                            {100, 100},
                            2000,
                            true,
                            950000 + i * 10000,
                            true,
                            .25,
                            {},
                            "injured_person");
        }
        assert(p.locked && p.id == 12);
        // The geometry filter never overrides the preferred class.
        PixelSelector cls;
        auto ord = object(20, {10, 10, 30, 30});
        edge.timestamp_us = ord.timestamp_us = 980000;
        for (int i = 0; i < 3; ++i) {
            edge.timestamp_us = ord.timestamp_us = 980000 + i * 10000;
            p = cls.update({edge, ord},
                           {100, 100},
                           2000,
                           true,
                           980000 + i * 10000,
                           true,
                           .25,
                           {},
                           "injured_person");
        }
        assert(p.locked && p.id == 12);
    }
    { // A fresh acquisition prefers the uncrowded block over a larger one with an injured piece
      // inside its footprint.
        PixelSelector iso;
        auto pile = object(30, {10, 10, 50, 50}),
             inj = object(31, {70, 10, 10, 10}, "injured_person"),
             lone = object(32, {10, 70, 30, 30});
        pile.body_xy_m = {0, .8f};
        inj.body_xy_m = {.05f, .82f};
        lone.body_xy_m = {.5f, .8f};
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            pile.timestamp_us = inj.timestamp_us = lone.timestamp_us = 1000000 + i * 10000;
            p = iso.update(
                {pile, inj, lone}, {100, 100}, 500, true, 1000000 + i * 10000, false, .25);
        }
        assert(p.locked && p.id == 32);
        // A tiny but clean block (below 30% of the best value) is not preferred.
        PixelSelector tiny;
        lone.box = {10, 70, 10, 10};
        for (int i = 0; i < 3; ++i) {
            pile.timestamp_us = inj.timestamp_us = lone.timestamp_us = 1100000 + i * 10000;
            p = tiny.update(
                {pile, inj, lone}, {100, 100}, 500, true, 1100000 + i * 10000, false, .25);
        }
        assert(p.locked && p.id == 30);
    }
    { // A fresh acquisition prefers a legal pair (one ordinary neighbour) over a lone block, but
      // not a trio.
        auto a1 = object(40, {10, 10, 30, 30}), a2 = object(41, {50, 10, 30, 30}),
             lone = object(42, {10, 60, 30, 30}), a3 = object(43, {50, 60, 30, 30});
        a1.body_xy_m = {0, .8f};
        a2.body_xy_m = {.05f, .8f};
        lone.body_xy_m = {.5f, .8f};
        a3.body_xy_m = {.025f, .8f};
        PixelSelection p;
        PixelSelector pr;
        for (int i = 0; i < 3; ++i) {
            a1.timestamp_us = a2.timestamp_us = lone.timestamp_us = 1200000 + i * 10000;
            p = pr.update({lone, a1, a2}, {100, 100}, 500, true, 1200000 + i * 10000, true, .25);
        }
        assert(p.locked && (p.id == 40 || p.id == 41));
        PixelSelector trio;
        for (int i = 0; i < 3; ++i) {
            a1.timestamp_us = a2.timestamp_us = a3.timestamp_us = lone.timestamp_us =
                1300000 + i * 10000;
            p = trio.update(
                {a1, a2, a3, lone}, {100, 100}, 500, true, 1300000 + i * 10000, true, .25);
        }
        assert(p.locked && p.id == 42);
    }
    // A larger challenger must never preempt a valid lock.
    PixelSelector hysteresis;
    small.mask.release();
    large.mask.release();
    small.box = {10, 10, 20, 20};
    large.box = {40, 10, 10, 10};
    auto rank = [&](uint64_t t) {
        small.timestamp_us = large.timestamp_us = t;
        return hysteresis.update({small, large}, {100, 100}, 500, true, t, false, .25);
    };
    rank(300000);
    rank(320000);
    assert(rank(340000).id == 1);
    large.box = {40, 10, 30, 30};
    assert(rank(360000).id == 1);
    assert(rank(380000).id == 1);
    assert(rank(400000).id == 1);
    assert(!hysteresis.update({small, large}, {100, 100}, 500, true, 410000, false, .25)
                .locked);         // repeated captured frame
    assert(!rank(800000).locked); // gap breaks continuity
    { // A short tracker dropout keeps the lock; only a re-IDed object at the old place takes over,
      // not a neighbour.
        PixelSelector g;
        auto big = object(1, {10, 10, 24, 24}), nb = object(2, {60, 10, 20, 20});
        auto step = [&](uint64_t t, std::vector<SegDetection> v) {
            for (auto& x : v)
                x.timestamp_us = t;
            return g.update(v, {100, 100}, 500, true, t, false, .25);
        };
        step(1000000, {big, nb});
        step(1020000, {big, nb});
        assert(step(1040000, {big, nb}).id == 1);
        auto r = step(1060000, {nb});
        assert(!r.locked && r.id == -1 && r.reason == "lock_grace_waiting");
        r = step(1080000, {nb});
        r = step(1100000, {nb});
        assert(!r.locked && r.reason == "lock_grace_waiting"); // neighbour never inherits the lock
        assert(step(1120000, {big, nb}).id ==
               1); // original returns within grace: still locked, no re-confirmation
        auto re = big;
        re.track_id = 9;
        step(1140000, {re, nb});
        step(1160000, {re, nb});
        assert(step(1180000, {re, nb}).id == 9); // re-IDed cube relocked by position
    }
    { // Alternating tracker IDs must keep the pending target by image position.
        PixelSelector alternating;
        PixelSelection p;
        for (int i = 0; i < 3; ++i) {
            const uint64_t t = 1200000 + i * 10000;
            auto target = object(10 + i, i == 0 ? cv::Rect{20, 30, 30, 30} : cv::Rect{29, 39, 12, 12});
            auto neighbour = object(20, {70, 30, i == 1 ? 32 : 10, i == 1 ? 32 : 10});
            target.timestamp_us = neighbour.timestamp_us = t;
            p = alternating.update(
                {target, neighbour}, {100, 100}, 500, true, t, false, .25);
        }
        assert(p.locked && p.id == 12);
    }
    FrameView a{500, {{5, 5}, {95, 5}, {95, 95}, {5, 95}}, true, true}, b = a;
    b.pitch_cdeg = 2000;
    MultiViewCapture multi({a, b}, {100, 100});
    uint64_t now = 1000000;
    auto d = object(1, {25, 25, 10, 10});
    auto sample = [&](bool has = true) {
        now += 20000;
        d.timestamp_us = now;
        auto blue_zone = object(100, {0, 0, 100, 100}, "blue_safe_zone");
        auto red_zone = object(101, {20, 20, 30, 30}, "red_safe_zone");
        blue_zone.timestamp_us = red_zone.timestamp_us = now;
        multi.update(now,
                     now,
                     7,
                     true,
                     true,
                     multi.desiredPitch(),
                     true,
                     has ? std::vector<SegDetection>{d, blue_zone, red_zone}
                         : std::vector<SegDetection>{blue_zone, red_zone});
    };
    multi.begin(now, 7, 1);
    for (int i = 0; i < 4; ++i)
        sample();
    assert(!multi.result().finished);
    sample();
    assert(multi.result().view == 1);
    d.track_id = 99; // ids need not survive view change
    for (int i = 0; i < 5; ++i) {
        sample();
    }
    assert(multi.result().verdict == CaptureVerdict::Enclosed &&
           multi.result().inventory.total() == 1);
    multi.begin(now, 7, 2);
    for (int i = 0; i < 5; ++i)
        sample();
    d.label = "core_supply";
    for (int i = 0; i < 5; ++i) {
        sample();
    }
    assert(multi.result().finished && multi.result().verdict == CaptureVerdict::Uncertain);
    multi.begin(now, 7, 3);
    for (int i = 0; i < 10; ++i)
        sample(false);
    assert(multi.result().verdict == CaptureVerdict::Empty);
    multi.begin(now, 7, 4);
    multi.update(now + 1, now + 1, 8, true, true, 500, true, {});
    assert(multi.result().finished && multi.result().verdict == CaptureVerdict::Uncertain);
    multi.begin(now, 7, 5);
    d = object(4, {90, 30, 10, 10});
    for (int i = 0; i < 10; ++i)
        sample();
    assert(multi.result().verdict == CaptureVerdict::Uncertain);
    multi.update(now + 8000001, now + 8000001, 7, true, true, 500, true, {});
    assert(multi.result().finished);
    multi.begin(now, 7, 6);
    multi.update(now + 1, now + 1, 7, true, false, 500, true, {});
    assert(multi.result().finished);
    // A stale/duplicate frame cannot satisfy the minimum five independent frames.
    multi.begin(now, 7, 7);
    for (int i = 0; i < 10; ++i)
        multi.update(now + 1, now + 1, 7, true, true, 500, true, {});
    assert(multi.result().frames == 1);
    { // image_polygon: no ground ranging; per-view pixel continuity, cross-view class counts only.
        MultiViewCapture img({a, b}, {100, 100}, true);
        uint64_t t = 2000000;
        auto g = object(1, {25, 25, 10, 10});
        g.ground_position_valid = g.ground_contact_valid = false;
        g.body_xy_m = {NAN, NAN};
        auto feed = [&](std::vector<SegDetection> ds) {
            t += 20000;
            for (auto& x : ds)
                x.timestamp_us = t;
            img.update(t, t, 9, true, true, img.desiredPitch(), true, ds);
        };
        img.begin(t, 9, 1);
        for (int i = 0; i < 5; ++i)
            feed({g});
        assert(img.result().view == 1);
        g.box = {50, 60, 12, 12}; // different pixel position at the second pitch is expected
        for (int i = 0; i < 5; ++i)
            feed({g});
        assert(img.result().verdict == CaptureVerdict::Enclosed &&
               img.result().inventory.total() == 1);
        img.begin(t, 9, 2);
        for (int i = 0; i < 5; ++i)
            feed({g});
        auto h = g;
        h.label = "core_supply";
        for (int i = 0; i < 5; ++i)
            feed({h});
        assert(img.result().finished && img.result().verdict == CaptureVerdict::Uncertain);
        img.begin(t, 9, 3);
        for (int i = 0; i < 10; ++i)
            feed({});
        assert(img.result().verdict == CaptureVerdict::Empty);
        // Jumping pixel position within one view restarts the five-frame count.
        img.begin(t, 9, 4);
        auto j = g;
        for (int i = 0; i < 4; ++i) {
            j.box.x = i % 2 ? 20 : 60;
            feed({j});
        }
        assert(img.result().view == 0 && img.result().frames == 1);
        // Boundary-crossing box stays uncertain even without ground requirements.
        img.begin(t, 9, 5);
        auto e = object(2, {90, 30, 10, 10});
        for (int i = 0; i < 10; ++i)
            feed({e});
        assert(img.result().verdict == CaptureVerdict::Uncertain);
        // Body-ground mode still rejects detections lacking ground validity.
        MultiViewCapture body({a, b}, {100, 100});
        body.begin(t, 9, 6);
        for (int i = 0; i < 10; ++i) {
            t += 20000;
            g.timestamp_us = t;
            body.update(t, t, 9, true, true, body.desiredPitch(), true, {g});
        }
        assert(body.result().verdict != CaptureVerdict::Enclosed);
    }
    { // Row mode: pick the middle clean block of a left-right row by ground points.
        const auto at = [](int id, float x, float y, const char* label) {
            auto d = object(id, {10 + id * 8, 10, 6, 6}, label);
            d.body_xy_m = {x, y};
            return d;
        };
        std::vector<SegDetection> row = {at(1, -.4f, .5f, "ordinary_supply"),
                                         at(2, -.2f, .5f, "dangerous_object"),
                                         at(3, 0.f, .5f, "ordinary_supply"),
                                         at(4, .2f, .5f, "core_supply"),
                                         at(5, .4f, .5f, "ordinary_supply")};
        auto p = RowSelector::pick(row, 100000, false, .25f);
        assert(p.id == 3 && p.index == 2 && p.seq == "GBGKG");
        auto q = RowSelector::pick(row, 100000, false, .25f, 5);
        assert(q.id == 5 && q.reason == "row_keep_lock");
        // Fatal neighbours: blocks touching a dangerous piece are not skipped; they stay candidates
        // and are listed in split_ids for PILE.
        std::vector<SegDetection> bad = {at(1, -.06f, .5f, "ordinary_supply"),
                                         at(2, 0.f, .5f, "dangerous_object"),
                                         at(3, .06f, .5f, "ordinary_supply"),
                                         at(4, .5f, .5f, "ordinary_supply")};
        auto b = RowSelector::pick(bad, 100000, false, .25f);
        assert(b.id == 3);
        assert(std::count(b.split_ids.begin(), b.split_ids.end(), 1) == 1 &&
               std::count(b.split_ids.begin(), b.split_ids.end(), 3) == 1 &&
               std::count(b.split_ids.begin(), b.split_ids.end(), 4) == 0);
        assert(RowSelector::pick(bad, 100000, false, .25f, 1).id ==
               1); // the lock stays on a flagged block
        {          // PixelSelector reports row_split for the locked flagged block only.
            PixelSelector sel;
            sel.setRow(true);
            PixelSelection r;
            for (int i = 0; i < 4; ++i) {
                auto rw = bad;
                for (auto& d : rw) {
                    d.timestamp_us = 100000 + i * 10000;
                    d.box = cv::Rect(10 + d.track_id * 16, 10, 6, 6);
                }
                r = sel.update(rw, {200, 100}, 500, true, 100000 + i * 10000, false, .25);
            }
            assert(r.locked && r.id == 3 && r.row_split);
        }
        // A radial line is not a lateral row.
        std::vector<SegDetection> radial = {at(1, 0.f, .3f, "ordinary_supply"),
                                            at(2, 0.f, .5f, "ordinary_supply"),
                                            at(3, 0.f, .7f, "ordinary_supply")};
        assert(RowSelector::pick(radial, 100000, false, .25f).reason == "no_row");
        // An off-line outlier is ignored.
        auto out = row;
        out.push_back(at(9, .1f, .9f, "ordinary_supply"));
        assert(RowSelector::pick(out, 100000, false, .25f).id == 3);
        // All one colour: evenly spaced -> the middle one; a solid run (neighbours inside the
        // footprint) -> an end block, never the middle.
        std::vector<SegDetection> green;
        for (int i = 0; i < 5; ++i)
            green.push_back(at(i + 1, -.4f + .2f * i, .5f, "ordinary_supply"));
        assert(RowSelector::pick(green, 100000, false, .25f).id == 3);
        std::vector<SegDetection> solid;
        for (int i = 0; i < 5; ++i)
            solid.push_back(at(i + 1, -.12f + .06f * i, .5f, "ordinary_supply"));
        auto sp = RowSelector::pick(solid, 100000, false, .25f);
        assert((sp.id == 1 || sp.id == 5) && sp.reason == "row_middle");
        // Even when every block is over-crowded, the least crowded end is still chosen instead of
        // dropping the row logic.
        std::vector<SegDetection> packed;
        for (int i = 0; i < 6; ++i)
            packed.push_back(at(i + 1, -.1f + .04f * i, .5f, "ordinary_supply"));
        auto pp = RowSelector::pick(packed, 100000, false, .25f);
        assert((pp.id == 1 || pp.id == 6) && pp.reason == "row_least_crowded");
        assert(RowSelector::pick(packed, 100000, false, .25f, pp.id).id == pp.id);
        // PixelSelector: the end block has the biggest box, row mode still locks the middle.
        for (bool on : {false, true}) {
            PixelSelector sel;
            sel.setRow(on);
            PixelSelection r;
            for (int i = 0; i < 4; ++i) {
                auto rw = row;
                for (auto& d : rw) {
                    d.timestamp_us = 100000 + i * 10000;
                    d.box = cv::Rect(10 + d.track_id * 16,
                                     10,
                                     d.track_id == 1 ? 14 : 6,
                                     d.track_id == 1 ? 14 : 6);
                }
                r = sel.update(rw, {200, 100}, 500, true, 100000 + i * 10000, false, .25);
            }
            assert(r.locked && r.id == (on ? 3 : 1));
        }
    }
    std::cout << "mechanical integration checks passed\n";
}
