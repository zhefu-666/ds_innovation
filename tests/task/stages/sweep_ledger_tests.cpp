#include "rescue/task/sweep_ledger.hpp"
#include "rescue/task/push_task.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

void usage(const SweepLedger& ledger, uint64_t now, uint64_t trip, uint64_t match) {
    CHECK(ledger.spentUs(now) == trip);
    CHECK(ledger.matchSpentUs(now) == match);
}

void beginIsIdempotentDuringAnActiveSegment() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_op_budget_us = 50;
    ledger.begin(tuning, 100);
    CHECK(ledger.chainOpen() && ledger.active());
    usage(ledger, 120, 20, 20);
    ledger.begin(tuning, 130);
    usage(ledger, 150, 50, 50);
    CHECK(!ledger.budget(tuning, 150).operation_timeout);
    CHECK(ledger.budget(tuning, 151).operation_timeout);
    ledger.setSwung(true);
    CHECK(ledger.close(tuning, 160));
    CHECK(!ledger.chainOpen() && !ledger.active() && ledger.done() == 1);
    usage(ledger, 1000, 60, 60);
}

void pauseExcludesIdleTimeAndLinksAtTheExactDeadline() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_chain_link_us = 20;
    ledger.begin(tuning, 100);
    ledger.setSwung(true);
    ledger.pause(130);
    CHECK(ledger.chainOpen() && !ledger.active());
    usage(ledger, 1000, 30, 30);
    ledger.pause(149);
    ledger.begin(tuning, 150);
    CHECK(ledger.active() && ledger.done() == 0);
    usage(ledger, 160, 40, 40);
    CHECK(ledger.close(tuning, 170));
    usage(ledger, 1000, 50, 50);
    CHECK(ledger.done() == 1);
}

void settleClosesOnlyAfterTheLinkDeadline() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_chain_link_us = 20;
    ledger.begin(tuning, 100);
    ledger.setSwung(true);
    ledger.pause(120);
    ledger.settle(tuning, 140);
    CHECK(ledger.chainOpen() && !ledger.active() && ledger.done() == 0);
    ledger.settle(tuning, 141);
    CHECK(!ledger.chainOpen() && !ledger.active() && ledger.done() == 1);
    usage(ledger, 1000, 20, 20);
    ledger.settle(tuning, 1000);
    CHECK(ledger.done() == 1);

    SweepLedger late_begin;
    late_begin.begin(tuning, 100);
    late_begin.setSwung(true);
    late_begin.pause(120);
    late_begin.begin(tuning, 141);
    CHECK(late_begin.chainOpen() && late_begin.active() && late_begin.done() == 1);
    usage(late_begin, 151, 30, 30);
    CHECK(!late_begin.close(tuning, 161));
    CHECK(late_begin.done() == 1);
    usage(late_begin, 1000, 40, 40);
}

void closingWithoutSwingStillAccountsTimeAndStartsObservation() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_observation_guard = true;
    tuning.sweep_reobserve_us = 30;
    CHECK(!ledger.close(tuning, 90));
    CHECK(!ledger.waitForObservation(tuning, 90));
    ledger.begin(tuning, 100);
    CHECK(!ledger.close(tuning, 130));
    CHECK(ledger.done() == 0 && !ledger.active() && !ledger.chainOpen());
    usage(ledger, 1000, 30, 30);
    CHECK(ledger.waitForObservation(tuning, 159));
    CHECK(!ledger.close(tuning, 150));
    CHECK(!ledger.waitForObservation(tuning, 160));
    CHECK(!ledger.close(tuning, 170));
    CHECK(!ledger.waitForObservation(tuning, 170));
    usage(ledger, 1000, 30, 30);

    ledger.begin(tuning, 200);
    ledger.setSwung(true);
    CHECK(ledger.close(tuning, 210));
    CHECK(!ledger.close(tuning, 220));
    CHECK(ledger.done() == 1);
    usage(ledger, 1000, 40, 40);
    CHECK(ledger.waitForObservation(tuning, 239));
    CHECK(!ledger.waitForObservation(tuning, 240));
}

void tripAndMatchTimeLimitsAreExclusive() {
    for (const bool match_limited : {false, true}) {
        SweepLedger ledger;
        TaskTuning tuning;
        tuning.sweep_time_budget_us = match_limited ? 100 : 50;
        tuning.sweep_match_budget_us = match_limited ? 50 : 100;
        ledger.begin(tuning, 100);
        const auto before = ledger.budget(tuning, 149);
        CHECK(before.time_ok && before.budget_ok && before.ready && !before.over);
        const auto exhausted = ledger.budget(tuning, 150);
        CHECK(!exhausted.time_ok && !exhausted.budget_ok && !exhausted.ready && exhausted.over);
        ledger.pause(150);
        CHECK(!ledger.budget(tuning, 150).over);
        CHECK(!ledger.close(tuning, 150));
        CHECK(!ledger.budget(tuning, 151).time_ok);
    }

    SweepLedger fresh_trip;
    TaskTuning tuning;
    tuning.sweep_time_budget_us = 50;
    tuning.sweep_match_budget_us = 100;
    fresh_trip.begin(tuning, 100);
    fresh_trip.newTrip(tuning, 150);
    usage(fresh_trip, 200, 0, 50);
    fresh_trip.begin(tuning, 200);
    CHECK(fresh_trip.budget(tuning, 249).time_ok);
    CHECK(!fresh_trip.budget(tuning, 250).time_ok);
    usage(fresh_trip, 250, 50, 100);
}

void operationDeadlineIncludesPausesButOverRequiresActivity() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_op_budget_us = 50;
    tuning.sweep_chain_link_us = 100;
    ledger.begin(tuning, 100);
    ledger.pause(110);
    ledger.begin(tuning, 149);
    const auto exact = ledger.budget(tuning, 150);
    CHECK(!exact.operation_timeout && !exact.over && exact.time_ok);
    const auto expired = ledger.budget(tuning, 151);
    CHECK(expired.operation_timeout && expired.over && expired.time_ok);
    usage(ledger, 151, 12, 12);
    ledger.pause(151);
    const auto paused = ledger.budget(tuning, 200);
    CHECK(paused.operation_timeout && !paused.over && paused.time_ok);

    SweepLedger idle;
    const auto absent_chain = idle.budget(tuning, 51);
    CHECK(absent_chain.operation_timeout && !absent_chain.over && absent_chain.ready);
}

void countLimitAndEscalationDoNotReplaceTimeAccounting() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_total_max = 2;
    tuning.sweep_escalate_max = 1;
    CHECK(ledger.budget(tuning, 100).escalation == 0);
    ledger.begin(tuning, 100);
    ledger.setSwung(true);
    CHECK(ledger.close(tuning, 110));
    CHECK(ledger.done() == 1);
    CHECK(ledger.budget(tuning, 110).budget_ok);
    CHECK(ledger.budget(tuning, 110).escalation == 1);
    ledger.begin(tuning, 120);
    ledger.setSwung(true);
    CHECK(ledger.close(tuning, 130));
    const auto exhausted = ledger.budget(tuning, 130);
    CHECK(exhausted.time_ok && !exhausted.budget_ok && !exhausted.ready);
    CHECK(!exhausted.over && exhausted.escalation == 1 && ledger.done() == 2);
    ledger.begin(tuning, 140);
    const auto active = ledger.budget(tuning, 141);
    CHECK(active.time_ok && !active.budget_ok && !active.over);
    usage(ledger, 141, 21, 21);
}

void assigningAGapMayShortenItWhileExtendingNeverDoes() {
    SweepLedger ledger;
    TaskTuning tuning;
    ledger.setGapUntil(200);
    CHECK(!ledger.budget(tuning, 199).ready);
    CHECK(ledger.budget(tuning, 200).ready);
    ledger.setGapUntil(150);
    CHECK(!ledger.budget(tuning, 149).ready);
    CHECK(ledger.budget(tuning, 150).ready);
    ledger.extendGapUntil(250);
    ledger.extendGapUntil(200);
    CHECK(!ledger.budget(tuning, 249).ready);
    CHECK(ledger.budget(tuning, 250).ready);
    CHECK(ledger.budget(tuning, 249).budget_ok);
    ledger.setGapUntil(0);
    CHECK(ledger.budget(tuning, 1).ready);
}

void newTripRetainsMatchUsageAndObservationWait() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_observation_guard = true;
    tuning.sweep_reobserve_us = 50;
    ledger.begin(tuning, 100);
    ledger.setSwung(true);
    ledger.pause(120);
    ledger.setGapUntil(500);
    ledger.newTrip(tuning, 140);
    CHECK(!ledger.active() && !ledger.chainOpen() && ledger.done() == 0);
    usage(ledger, 140, 0, 20);
    CHECK(ledger.budget(tuning, 140).ready);
    CHECK(ledger.waitForObservation(tuning, 189));
    ledger.newTrip(tuning, 160);
    CHECK(ledger.waitForObservation(tuning, 189));
    CHECK(!ledger.waitForObservation(tuning, 190));
    ledger.begin(tuning, 200);
    ledger.setSwung(true);
    ledger.newTrip(tuning, 230);
    usage(ledger, 230, 0, 50);
    CHECK(ledger.done() == 0 && !ledger.active() && !ledger.chainOpen());
    CHECK(ledger.waitForObservation(tuning, 279));
    CHECK(!ledger.waitForObservation(tuning, 280));
}

void resetDiscardsTheActiveSegmentAndAllPreviousUsage() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_observation_guard = true;
    tuning.sweep_reobserve_us = 500;
    ledger.begin(tuning, 100);
    ledger.setSwung(true);
    CHECK(ledger.close(tuning, 110));
    ledger.setGapUntil(1000);
    ledger.begin(tuning, 150);
    ledger.setSwung(true);
    usage(ledger, 250, 110, 110);
    ledger.reset();
    CHECK(!ledger.active() && !ledger.chainOpen() && ledger.done() == 0);
    usage(ledger, 250, 0, 0);
    CHECK(ledger.budget(tuning, 250).ready);
    CHECK(!ledger.waitForObservation(tuning, 250));
    CHECK(!ledger.close(tuning, 260));
    usage(ledger, 260, 0, 0);
}

void swingMarkBelongsToTheCurrentChain() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_chain_link_us = 20;
    ledger.setSwung(true);
    CHECK(!ledger.close(tuning, 90));
    ledger.begin(tuning, 100);
    CHECK(!ledger.close(tuning, 110));
    CHECK(ledger.done() == 0);
    ledger.begin(tuning, 120);
    ledger.setSwung(true);
    ledger.setSwung(false);
    CHECK(!ledger.close(tuning, 130));
    CHECK(ledger.done() == 0);
    ledger.begin(tuning, 140);
    ledger.setSwung(true);
    ledger.pause(150);
    ledger.begin(tuning, 170);
    CHECK(ledger.close(tuning, 180));
    CHECK(ledger.done() == 1);
    usage(ledger, 180, 40, 40);
}

void observationWaitCanBeDisabledExtendedAndConsumed() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_reobserve_us = 30;
    ledger.extendObservationWait(tuning, 90);
    ledger.begin(tuning, 100);
    CHECK(!ledger.close(tuning, 110));
    tuning.sweep_observation_guard = true;
    CHECK(!ledger.waitForObservation(tuning, 111));
    ledger.extendObservationWait(tuning, 112);
    CHECK(!ledger.waitForObservation(tuning, 113));

    ledger.begin(tuning, 120);
    CHECK(!ledger.close(tuning, 130));
    tuning.sweep_observation_guard = false;
    CHECK(!ledger.waitForObservation(tuning, 140));
    ledger.extendObservationWait(tuning, 145);
    tuning.sweep_observation_guard = true;
    CHECK(ledger.waitForObservation(tuning, 159));
    ledger.extendObservationWait(tuning, 159);
    CHECK(ledger.waitForObservation(tuning, 188));
    CHECK(!ledger.waitForObservation(tuning, 189));
    ledger.extendObservationWait(tuning, 200);
    CHECK(!ledger.waitForObservation(tuning, 201));

    ledger.begin(tuning, 210);
    CHECK(!ledger.close(tuning, 220));
    ledger.extendObservationWait(tuning, 260);
    CHECK(ledger.waitForObservation(tuning, 289));
    CHECK(!ledger.waitForObservation(tuning, 290));
}

void clockRollbackAndZeroSentinelKeepTheirExistingSemantics() {
    SweepLedger ledger;
    TaskTuning tuning;
    tuning.sweep_chain_link_us = 20;
    ledger.begin(tuning, 100);
    usage(ledger, 99, 0, 0);
    CHECK(ledger.budget(tuning, 99).operation_timeout);
    CHECK(ledger.budget(tuning, 99).over);
    ledger.pause(90);
    usage(ledger, 90, 0, 0);
    CHECK(!ledger.active() && ledger.chainOpen());
    ledger.begin(tuning, 100);
    usage(ledger, 110, 10, 10);
    CHECK(!ledger.close(tuning, 110));
    usage(ledger, 200, 10, 10);

    SweepLedger zero;
    zero.begin(tuning, 0);
    CHECK(zero.active() && !zero.chainOpen());
    usage(zero, 100, 0, 0);
    zero.pause(100);
    CHECK(zero.active());
    CHECK(!zero.close(tuning, 100));
    zero.reset();
    CHECK(!zero.active() && !zero.chainOpen());
}
} // namespace

int main() {
    try {
        beginIsIdempotentDuringAnActiveSegment();
        pauseExcludesIdleTimeAndLinksAtTheExactDeadline();
        settleClosesOnlyAfterTheLinkDeadline();
        closingWithoutSwingStillAccountsTimeAndStartsObservation();
        tripAndMatchTimeLimitsAreExclusive();
        operationDeadlineIncludesPausesButOverRequiresActivity();
        countLimitAndEscalationDoNotReplaceTimeAccounting();
        assigningAGapMayShortenItWhileExtendingNeverDoes();
        newTripRetainsMatchUsageAndObservationWait();
        resetDiscardsTheActiveSegmentAndAllPreviousUsage();
        swingMarkBelongsToTheCurrentChain();
        observationWaitCanBeDisabledExtendedAndConsumed();
        clockRollbackAndZeroSentinelKeepTheirExistingSemantics();
        std::cout << "sweep ledger tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "sweep ledger tests failed: " << error.what() << '\n';
        return 1;
    }
}
