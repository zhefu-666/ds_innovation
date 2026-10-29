#include "push_task_cases.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--write-fixture") {
        writeFixture(argv[2]);
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--far-smoothing-only") {
        farTargetSmoothingTests();
        std::cout << "Far target smoothing and recovery checks passed\n";
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--capture-only") {
        captureTests();
        std::cout << "Pixel target identity and capture checks passed\n";
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--selection-only") {
        ordinarySelectionRegressionTests();
        std::cout << "Ordinary target selection regression passed\n";
        return 0;
    }
    farTargetSmoothingTests();
    ordinarySelectionRegressionTests();
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
