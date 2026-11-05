#include "rescue/app/perception_pipeline.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    assert(argc == 5);
    rescue::Config config;
    config.model_path = argv[1];
    config.pose_model_path = argv[2];
    config.rknn_library = argv[3];
    config.class_names = {"blue", "orange", "green", "black", "zone_blue", "zone_red"};
    config.parallel_inference = true;
    config.detect_core_mask = 4;
    config.pose_core_mask = 3;
    cv::setNumThreads(1);
    const auto frame = cv::imread(argv[4]);
    assert(!frame.empty());
    rescue::ZoneHalves raw_reference{};
    for (int cycle = 0; cycle < 4; ++cycle) {
        config.pose_smoothing_preview = cycle == 0 ? "off" : cycle == 1 ? "raw" :
                                        cycle == 2 ? "smoothed" : "overlay";
        rescue::PerceptionPipeline pipeline(config);
        for (int i = 0; i < 20; ++i) {
            const auto result = pipeline.infer(frame, i + 1);
            assert(result.pose_ran && result.halves[1].valid);
            if (!cycle && !i) raw_reference=result.halves;
            for (int c=0;c<2;++c) {
                const auto& h=result.halves[c]; const auto& expected=raw_reference[c];
                assert(h.valid==expected.valid && h.score==expected.score && h.box==expected.box);
                assert(h.confidence==expected.confidence && h.keypoints==expected.keypoints);
            }
            if (cycle) {
                assert(result.preview_enabled && result.display.ok);
                assert(result.display.measurement_us == uint64_t(i+1));
                for (int c=0;c<2;++c) {
                    assert(result.display.halves[c].valid==result.halves[c].valid);
                    assert(result.display.halves[c].score==result.halves[c].score);
                    assert(result.display.halves[c].confidence==result.halves[c].confidence);
                    assert(result.display.halves[c].box==result.halves[c].box);
                }
            } else assert(!result.preview_enabled);
        }
        // A detector input error must join the pose worker before unwinding.
        auto bad = rescue::prepareModelImage(frame, config.input_size);
        bad.rgb = cv::Mat(config.input_size, config.input_size, CV_8UC1);
        bool caught = false;
        try {
            pipeline.infer(frame, 21, &bad);
        } catch (const std::runtime_error& error) {
            caught = std::string(error.what()).find("RKNN input") != std::string::npos;
        }
        assert(caught);
        assert(pipeline.infer(frame, 22).pose_ran);
    }
    // A pose-worker exception must reach the caller and leave the pipeline usable.
    config.confidence = config.injured_confidence = config.ui140_core_confidence = 1;
    rescue::PerceptionPipeline pipeline(config);
    auto bad_scale = rescue::prepareModelImage(frame, config.input_size);
    bad_scale.sx = 0;
    bool caught = false;
    try {
        pipeline.infer(frame, 23, &bad_scale);
    } catch (const std::runtime_error& error) {
        caught = std::string(error.what()) == "Invalid letterbox scale";
    }
    assert(caught);
    assert(pipeline.infer(frame, 24).pose_ran);
    std::cout << "parallel inference, exception propagation and teardown passed\n";
}
