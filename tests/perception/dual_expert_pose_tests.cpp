#include "rescue/app/perception_pipeline.hpp"
#include "rescue/perception/rknn_model.hpp"
#include "rescue/perception/rknn_output_layout.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
using namespace rescue;
using Json = nlohmann::json;
namespace fs = std::filesystem;
namespace {
Json halvesJson(const ZoneHalves& h) {
    Json out = Json::array();
    for (int c = 0; c < 2; ++c)
        if (h[c].valid) {
            const auto& b = h[c].box;
            Json points = Json::array();
            for (int k = 0; k < 4; ++k)
                points.push_back({h[c].keypoints[k].x, h[c].keypoints[k].y, h[c].confidence[k]});
            out.push_back({{"cls", c},
                           {"score", h[c].score},
                           {"box", {b.x, b.y, b.x + b.width, b.y + b.height}},
                           {"kpt", points}});
        }
    return out;
}
void unitTests() {
    const auto layout = validateRknnOutputLayout(
        "experts", {{1, 4, 8400}, {1, 2, 8400}, {1, 16, 8400}, {1, 4, 8400}});
    assert(layout.channels == 26);
    std::vector<cv::Mat> branches;
    for (int channels : {4, 2, 16, 4}) {
        int s[] = {1, channels, 8400};
        branches.emplace_back(3, s, CV_32F, cv::Scalar(float(branches.size())));
    }
    const auto merged = mergeRknnOutputBranches(branches);
    assert(merged.size[1] == 26 && merged.ptr<float>()[22 * 8400] == 3);
    int shape[] = {1, 26, 8400};
    cv::Mat t(3, shape, CV_32F, cv::Scalar(0));
    auto set = [&](int row, int i, float v) {
        t.ptr<float>()[row * 8400 + i] = v;
    };
    for (int i = 0; i < 2; ++i) {
        set(0, i, 150 + 300 * i);
        set(1, i, 300);
        set(2, i, 60);
        set(3, i, 60);
        set(4 + i, i, .9f);
        for (int k = 0; k < 4; ++k) {
            set(6 + 2 * k, i, 100 + 10 * k);
            set(7 + 2 * k, i, 120 + 10 * k);
            set(14 + 2 * k, i, 200 + 10 * k);
            set(15 + 2 * k, i, 220 + 10 * k);
            set(22 + k, i, .8f);
        }
    }
    ModelImage image;
    ZonePoseMetadata meta;
    ZonePoseRoute route;
    auto decode = [&]() {
        return decodeZonePoseExperts(t, image, {640, 640}, &meta, &route);
    };
    auto h = decode();
    assert(h[0].valid && h[1].valid && route.weight_far == 1);
    assert(h[0].keypoints[0] == cv::Point2f(200, 220));
    set(5, 1, .5f);
    h = decode();
    assert(!h[1].valid && route.weight_far == 0 && h[0].keypoints[0].x == 100);
    set(5, 1, .9f);
    for (int c = 0; c < 26; ++c)
        set(c, 2, t.ptr<float>()[c * 8400]);
    set(4, 2, .8f);
    decode();
    assert(meta[0].nms_candidates == 1); // class NMS
    set(0, 2, 600);
    h = decode();
    assert(meta[0].nms_candidates == 2 && route.weight_far == 0);
    set(4, 2, 0);
    for (int i = 0; i < 2; ++i) {
        set(2, i, 300);
        set(3, i, 300);
    }
    h = decode();
    assert(route.weight_far == 0 && h[0].keypoints[0].x == 100);
    for (int i = 0; i < 2; ++i) {
        set(2, i, 200);
        set(3, i, 200);
    }
    h = decode();
    assert(route.weight_far > 0 && route.weight_far < 1);
    assert(std::abs(h[0].keypoints[0].x - (100 + 100 * route.weight_far)) < 1e-4);
    // Out-of-frame predictions stay measurable rather than becoming border points.
    set(6, 0, -100);
    set(14, 0, -100);
    h = decode();
    assert(h[0].keypoints[0].x == -100);
    set(22, 0, std::numeric_limits<float>::quiet_NaN());
    bool rejected = false;
    try {
        decode();
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    assert(rejected);
    for (auto size : {cv::Size(1280, 720), cv::Size(1279, 721), cv::Size(640, 479)}) {
        const auto p = prepareExpertPoseImage(cv::Mat(size, CV_8UC3, cv::Scalar(1, 2, 3)));
        assert(p.rgb.size() == cv::Size(640, 640) && p.sx == p.sy);
    }
    std::cout << "dual expert layout, merge, routing, threshold, NMS and finite checks passed\n";
}
void fixtures(char** argv) {
    const fs::path model = argv[2], library = argv[3], manifest = argv[4], detector = argv[5],
                   out = argv[6];
    fs::create_directories(out);
    Json fixtures;
    std::ifstream(manifest) >> fixtures;
    RknnModel runtime(library.string(), model.string(), 3);
    Config config;
    config.pose_model_path = model.string();
    config.model_path = detector.string();
    config.rknn_library = library.string();
    config.parallel_inference = true;
    config.detect_core_mask = 4;
    config.pose_core_mask = 3;
    config.pose_smoothing_preview = "overlay";
    PerceptionPipeline pipeline(config);
    Json records = Json::array();
    std::ofstream raw(out / "raw.jsonl"), filtered(out / "filtered.jsonl");
    uint64_t fixture_time = 0;
    for (const auto& fixture : fixtures) {
        const fs::path path =
            manifest.parent_path().parent_path() / fixture.at("image").get<std::string>();
        const auto frame = cv::imread(path.string());
        assert(!frame.empty());
        const auto start = std::chrono::steady_clock::now();
        const auto image = prepareExpertPoseImage(frame);
        const auto prepared = std::chrono::steady_clock::now();
        const auto output = runtime.run(image.rgb);
        const auto inferred = std::chrono::steady_clock::now();
        ZonePoseMetadata metadata;
        ZonePoseRoute route;
        const auto halves = decodeZonePoseExperts(output, image, frame.size(), &metadata, &route);
        const auto decoded = std::chrono::steady_clock::now();
        const auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        const auto name = path.stem().string();
        std::ofstream tensor(out / (name + ".f32"), std::ios::binary);
        tensor.write(reinterpret_cast<const char*>(output.data),
                     std::streamsize(output.total() * sizeof(float)));
        std::ofstream rgb(out / (name + ".rgb"), std::ios::binary);
        rgb.write(reinterpret_cast<const char*>(image.rgb.data),
                  std::streamsize(image.rgb.total() * 3));
        Json stats = Json::array();
        int offset = 0;
        for (int count : {4, 2, 16, 4}) {
            cv::Mat branch(
                1, count * 8400, CV_32F, const_cast<float*>(output.ptr<float>() + offset * 8400));
            double lo, hi;
            cv::minMaxLoc(branch, &lo, &hi);
            stats.push_back({{"shape", {1, count, 8400}},
                             {"dtype", "float32_dequantized"},
                             {"min", lo},
                             {"max", hi}});
            offset += count;
        }
        // Main production pipeline must return the exact same measurements.
        const auto actual = pipeline.infer(frame, fixture_time += 1000000);
        assert(actual.pose_ran && actual.pose_route.dual_expert && actual.display.ok);
        for (int c = 0; c < 2; ++c) {
            assert(actual.halves[c].valid == halves[c].valid);
            for (int k = 0; k < 4; ++k)
                assert(cv::norm(actual.halves[c].keypoints[k] - halves[c].keypoints[k]) < 1e-3);
        }
        Json record = {{"image", fixture.at("image")},
                       {"raw", halvesJson(halves)},
                       {"route",
                        {{"weight_far", route.weight_far},
                         {"max_area_fraction", route.max_area_fraction},
                         {"counts", route.counts},
                         {"reason", route.reason}}},
                       {"gain", image.sx},
                       {"pad", {image.pad_x, image.pad_y}},
                       {"outputs", stats},
                       {"preprocess_ms", ms(start, prepared)},
                       {"inference_ms", ms(prepared, inferred)},
                       {"decode_ms", ms(inferred, decoded)},
                       {"main_pipeline_ms", actual.total_ms},
                       {"main_pipeline_match", true},
                       {"reference_is_human_GT", false}};
        records.push_back(record);
        raw << record.dump() << "\n";
        filtered << Json({{"image", fixture.at("image")},
                          {"filtered", halvesJson(actual.display.halves)},
                          {"independent_image_not_temporal_validation", true}})
                        .dump()
                 << "\n";
        auto drawn = frame.clone();
        for (const auto& h : halves)
            if (h.valid) {
                const auto seen = [&](int k) {
                    auto p = h.keypoints[k];
                    return h.confidence[k] >= .5f && p.x >= 0 && p.y >= 0 && p.x < frame.cols &&
                           p.y < frame.rows;
                };
                for (int k = 0; k < 4; ++k)
                    if (seen(k))
                        cv::circle(drawn, h.keypoints[k], 5, {0, 255, 255}, -1);
                for (auto edge : {std::pair<int, int>{0, 2}, {1, 3}, {0, 1}, {2, 3}})
                    if (seen(edge.first) && seen(edge.second))
                        cv::line(drawn,
                                 h.keypoints[edge.first],
                                 h.keypoints[edge.second],
                                 {0, 255, 0},
                                 2);
            }
        cv::putText(drawn,
                    "RK3588 NPU / " + model.filename().string() + " / raw",
                    {20, 30},
                    cv::FONT_HERSHEY_SIMPLEX,
                    .7,
                    {0, 255, 255},
                    2);
        assert(cv::imwrite((out / (name + "_board.png")).string(), drawn));
        std::cout << name << " counts=" << route.counts[0] << "," << route.counts[1]
                  << " weight_far=" << route.weight_far << " infer_ms=" << ms(prepared, inferred)
                  << " main_match=1\n";
    }
    std::ofstream(out / "image_results.json") << records.dump(2) << "\n";
}
} // namespace
int main(int argc, char** argv) {
    try {
        cv::setNumThreads(1);
        unitTests();
        if (argc == 7 && std::string(argv[1]) == "--fixtures")
            fixtures(argv);
        else if (argc != 1)
            throw std::runtime_error(
                "usage: --fixtures model runtime manifest detector output_dir");
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
