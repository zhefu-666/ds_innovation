#include "rescue/task/disperse_cluster_stage.hpp"
#include "rescue/push_task.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

struct Fixture {
    DisperseClusterPlanner planner;
    TaskTuning tuning;
    PushObservation observation;

    Fixture() {
        tuning.blind_scatter_min_y_m = .2f;
        tuning.blind_scatter_max_y_m = 2.f;
        tuning.blind_scatter_far_max_y_m = 4.f;
        tuning.blind_scatter_half_x_m = 1.f;
        tuning.blind_scatter_max_bearing_rad = 1.f;
        tuning.disperse_radius_m = .3f;
        tuning.blind_scatter_far_radius_m = .6f;
        tuning.blind_scatter_min = 2;
        tuning.blind_scatter_far_min = 2;
        tuning.blind_scatter_need_ordinary = false;
    }

    ClusterGroup find(const std::vector<cv::Point2f>& pts, float max_y = 2.f, float radius = .3f) {
        return planner.findBest(tuning, observation, pts, max_y, radius);
    }

    int eval(const std::vector<cv::Point2f>& pts,
             int need_n = 2,
             int far_need = 2,
             float far_y = 2.f,
             int esc = 0,
             float radius = -1.f) {
        return planner.evalGroup(tuning, observation, pts, need_n, far_need, far_y, esc, radius);
    }
};

void findsADenseGroupAndItsCentre() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.1f, 1.05f}, {-.1f, .95f}};
    const auto best = f.find(pts);
    CHECK(best.n == 3);
    CHECK(std::abs(best.d - float(cv::norm(cv::Point2f(0.f, 1.f)))) < 1e-4f);
    CHECK(std::abs(f.planner.bestCentre().y - 1.f) < 1e-4f);
}

void rejectsPointsBehindOrOutsideTheCorridor() {
    Fixture f;
    // Behind the robot, past max_y, sideways and outside the bearing window.
    const std::vector<cv::Point2f> pts{{0.f, -1.f}, {0.f, 3.f}, {2.f, 1.f}, {-1.5f, .5f}};
    CHECK(f.find(pts).n == 0);
}

void rejectsNonFinitePoints() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{std::nanf(""), 1.f}, {0.f, std::nanf("")}};
    CHECK(f.find(pts).n == 0);
}

void requiresAnOrdinaryMemberWhenConfigured() {
    Fixture f;
    f.tuning.blind_scatter_need_ordinary = true;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.1f, 1.f}};
    CHECK(f.find(pts).n == 0); // injured / dangerous only
    f.observation.clutter_ordinary = {{.1f, 1.f}};
    CHECK(f.find(pts).n == 2);
}

void countsAppendedOrdinaryPoints() {
    Fixture f;
    f.tuning.blind_scatter_need_ordinary = true;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.1f, 1.f}};
    CHECK(f.find(pts).n == 0);
    f.planner.addOrdinaryExtra({0.f, 1.f});
    CHECK(f.find(pts).n == 2);
}

void radiusExcludesDistantMembers() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.1f, 1.f}, {0.f, 2.f}};
    CHECK(f.find(pts, 2.f, .3f).n == 2); // the far point is outside the radius
}

void prefersDenseThenTheMostCentred() {
    Fixture f;
    // A 2-point group straight ahead and a 2-point group off to the side: the score prefers
    // density first, then the smaller bearing.
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.05f, 1.f}, {.5f, 1.f}, {.55f, 1.f}};
    const auto best = f.find(pts);
    CHECK(best.n == 2);
    CHECK(std::abs(f.planner.bestCentre().x) < .1f);
}

void tieBreaksTowardTheNearerGroup() {
    Fixture f;
    // Equal count, equal-ish bearing: the nearer centre wins only on an exact score tie.
    const std::vector<cv::Point2f> pts{{0.f, .5f}, {0.f, .55f}, {0.f, 1.5f}, {0.f, 1.55f}};
    const auto best = f.find(pts);
    CHECK(best.n == 2);
    CHECK(best.d < 1.f); // the nearer pair
}

void evalFindsAShortWindowGroup() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.1f, 1.f}};
    CHECK(f.eval(pts) == 1);
}

void evalReportsAFarGroup() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 3.f}, {.1f, 3.f}};
    CHECK(f.eval(pts, 2, 2, 4.f) == 2);
    CHECK(f.eval(pts, 2, 2, 2.f) == 0); // without the long-range window the group is out of reach
}

void evalEscalationRaisesBothThresholds() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.1f, 1.f}};
    CHECK(f.eval(pts, 2, 2, 2.f, 0) == 1);
    CHECK(f.eval(pts, 2, 2, 2.f, 1) == 0); // a need of 2 escalates to 3
}

void evalDoesNotEscalateASingleNeed() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}};
    CHECK(f.eval(pts, 1, 1, 2.f, 5) == 1); // need 1 keeps its threshold
}

void evalClearsTheBestOnAMiss() {
    Fixture f;
    f.eval({{0.f, 1.f}, {.1f, 1.f}});
    CHECK(f.planner.bestCentre().y != 0.f);
    CHECK(f.eval({{0.f, 5.f}}) == 0);
    CHECK(f.planner.bestCentre() == cv::Point2f(0.f, 0.f));
}

void evalHonoursAShortRadiusOverride() {
    Fixture f;
    const std::vector<cv::Point2f> pts{{0.f, 1.f}, {.4f, 1.f}};
    CHECK(f.eval(pts, 2, 2, 2.f, 0, .3f) == 0); // the override splits the pair
    f.tuning.disperse_radius_m = .5f;
    CHECK(f.eval(pts, 2, 2, 2.f, 0, -1.f) == 1); // a negative override falls back to the tuning
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"findsADenseGroupAndItsCentre", findsADenseGroupAndItsCentre},
        {"rejectsPointsBehindOrOutsideTheCorridor", rejectsPointsBehindOrOutsideTheCorridor},
        {"rejectsNonFinitePoints", rejectsNonFinitePoints},
        {"requiresAnOrdinaryMemberWhenConfigured", requiresAnOrdinaryMemberWhenConfigured},
        {"countsAppendedOrdinaryPoints", countsAppendedOrdinaryPoints},
        {"radiusExcludesDistantMembers", radiusExcludesDistantMembers},
        {"prefersDenseThenTheMostCentred", prefersDenseThenTheMostCentred},
        {"tieBreaksTowardTheNearerGroup", tieBreaksTowardTheNearerGroup},
        {"evalFindsAShortWindowGroup", evalFindsAShortWindowGroup},
        {"evalReportsAFarGroup", evalReportsAFarGroup},
        {"evalEscalationRaisesBothThresholds", evalEscalationRaisesBothThresholds},
        {"evalDoesNotEscalateASingleNeed", evalDoesNotEscalateASingleNeed},
        {"evalClearsTheBestOnAMiss", evalClearsTheBestOnAMiss},
        {"evalHonoursAShortRadiusOverride", evalHonoursAShortRadiusOverride},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "disperse cluster planner OK\n";
    return 0;
}
