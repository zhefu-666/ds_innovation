#include "push_task_cases.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--write-fixture") {
        writeFixture(argv[2]);
        return 0;
    }
    sweepViewTests();
    pileTests();
    precisionMotionTests();
    demoCaptureMotionTests();
    sweepFrameOpenTests();
    defaultOpenFrameTests();
    assumedDeliveryCases();
    navigationReadinessCases();
    cameraStartupCases();
    searchSelectionCases();
    sweepTriggerCases();
    approachCases();
    rulesTests();
    captureTests();
    holding40StaticReviewTests();
    holdingOutsideMouthTests();
    corridorFootprintTests();
    blindRetreatTests();
    strictDeliveryCases();
    captureSafetyCases();
    demoWatchdogCases();
    fieldCarryCases();
    fieldCaptureCases();
    corridorSweepCases();
    std::cout << "Capture, transport rules, carry, delivery, drop and stop checks passed\n";
}
