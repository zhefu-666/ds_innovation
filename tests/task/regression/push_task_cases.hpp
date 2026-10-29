#pragma once

#include <string>

// Entry points run in the original regression order.
void writeFixture(const std::string& path);
void rulesTests();
void captureTests();
void holding40StaticReviewTests();
void holdingOutsideMouthTests();
void corridorFootprintTests();
void blindRetreatTests();
void precisionMotionTests();
void sweepFrameOpenTests();
void defaultOpenFrameTests();
void demoCaptureMotionTests();
void sweepViewTests();
void pileTests();
void assumedDeliveryCases();
void navigationReadinessCases();
void cameraStartupCases();
void searchSelectionCases();
void sweepTriggerCases();
void approachCases();
void farTargetSmoothingTests();
void strictDeliveryCases();
void captureSafetyCases();
void demoWatchdogCases();
void fieldCarryCases();
void fieldCaptureCases();
void corridorSweepCases();

void ordinarySelectionRegressionTests();
