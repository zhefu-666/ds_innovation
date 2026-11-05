#include "rescue/perception/rknn_output_layout.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace rescue;

namespace {

cv::Mat tensor(std::vector<int> shape, float start) {
    cv::Mat result(static_cast<int>(shape.size()), shape.data(), CV_32F);
    float* values = result.ptr<float>();
    for (size_t i = 0; i < result.total(); ++i)
        values[i] = start + static_cast<float>(i);
    return result;
}

template <class Fn> void rejects(Fn fn) {
    bool rejected = false;
    try {
        fn();
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    assert(rejected);
}

void layoutTests() {
    auto one = validateRknnOutputLayout("one", {{1, 8, 8400}});
    assert(one.channels == 8 && one.anchors == 8400);

    auto two = validateRknnOutputLayout("two", {{1, 4, 8400}, {1, 4, 8400}});
    assert(two.channels == 8 && two.anchors == 8400);

    auto six = validateRknnOutputLayout("six", {{1, 4, 8400}, {1, 6, 8400}});
    assert(six.channels == 10 && six.anchors == 8400);
    assert(validateRknnOutputLayout("six joined", {{1, 10, 8400}}).channels == 10);

    auto four = validateRknnOutputLayout(
        "four", {{1, 4, 8400}, {1, 2, 8400}, {1, 4, 2, 8400}, {1, 4, 1, 8400}});
    assert(four.channels == 18 && four.anchors == 8400);

    auto raw = validateRknnOutputLayout(
        "raw", {{1, 4, 8400}, {1, 2, 8400}, {1, 8, 8400}, {1, 8, 8400}, {1, 4, 8400}});
    assert(raw.channels == 26 && raw.anchors == 8400);
    rejects([] {
        validateRknnOutputLayout(
            "raw anchors", {{1, 4, 8399}, {1, 2, 8399}, {1, 8, 8399}, {1, 8, 8399}, {1, 4, 8399}});
    });
    rejects([] {
        validateRknnOutputLayout(
            "raw channels", {{1, 4, 8400}, {1, 2, 8400}, {1, 16, 8400}, {1, 8, 8400}, {1, 4, 8400}});
    });
    rejects([] {
        validateRknnOutputLayout("none", {});
    });
    rejects([] {
        validateRknnOutputLayout("three", {{1, 4, 10}, {1, 2, 10}, {1, 4, 2, 10}});
    });
    rejects([] {
        validateRknnOutputLayout("rank", {{1, 4, 10}, {1, 4, 10, 1}});
    });
    rejects([] {
        validateRknnOutputLayout("batch", {{2, 4, 10}, {2, 4, 10}});
    });
    rejects([] {
        validateRknnOutputLayout("detector channels", {{1, 4, 10}, {1, 5, 10}});
    });
    rejects([] {
        validateRknnOutputLayout("detector anchors", {{1, 4, 10}, {1, 4, 11}});
    });
    rejects([] {
        validateRknnOutputLayout("zero", {{1, 4, 0}, {1, 4, 0}});
    });
    rejects([] {
        validateRknnOutputLayout("pose xy", {{1, 4, 10}, {1, 2, 10}, {1, 3, 2, 10}, {1, 4, 1, 10}});
    });
    rejects([] {
        validateRknnOutputLayout("pose visibility",
                                 {{1, 4, 10}, {1, 2, 10}, {1, 4, 2, 10}, {1, 4, 2, 10}});
    });
}

void mergeTests() {
    cv::Mat boxes = tensor({1, 4, 3}, 0);
    cv::Mat scores = tensor({1, 4, 3}, 100);
    auto detector = mergeRknnOutputBranches({boxes, scores});
    assert(detector.dims == 3 && detector.size[0] == 1 && detector.size[1] == 8 &&
           detector.size[2] == 3);
    for (int row = 0; row < 8; ++row)
        for (int i = 0; i < 3; ++i)
            assert(detector.ptr<float>()[row * 3 + i] ==
                   (row < 4 ? row * 3 + i : 100 + (row - 4) * 3 + i));

    auto six = mergeRknnOutputBranches({boxes, tensor({1, 6, 3}, 100)});
    assert(six.size[1] == 10 && six.size[2] == 3);
    for (int row = 0; row < 10; ++row)
        for (int i = 0; i < 3; ++i)
            assert(six.ptr<float>()[row * 3 + i] ==
                   (row < 4 ? row * 3 + i : 100 + (row - 4) * 3 + i));

    cv::Mat pose_boxes = tensor({1, 4, 2}, 0);
    cv::Mat pose_scores = tensor({1, 2, 2}, 100);
    cv::Mat xy = tensor({1, 4, 2, 2}, 1000);
    cv::Mat visibility = tensor({1, 4, 1, 2}, 2000);
    auto pose = mergeRknnOutputBranches({pose_boxes, pose_scores, xy, visibility});
    assert(pose.dims == 3 && pose.size[1] == 18 && pose.size[2] == 2);
    const float* out = pose.ptr<float>();
    for (int i = 0; i < 2; ++i) {
        for (int row = 0; row < 4; ++row)
            assert(out[row * 2 + i] == row * 2 + i);
        for (int row = 0; row < 2; ++row)
            assert(out[(4 + row) * 2 + i] == 100 + row * 2 + i);
        for (int k = 0; k < 4; ++k) {
            assert(out[(6 + 3 * k) * 2 + i] == 1000 + (2 * k) * 2 + i);
            assert(out[(7 + 3 * k) * 2 + i] == 1000 + (2 * k + 1) * 2 + i);
            assert(out[(8 + 3 * k) * 2 + i] == 2000 + k * 2 + i);
        }
    }

    cv::Mat wrong_type = tensor({1, 4, 3}, 0);
    wrong_type.convertTo(wrong_type, CV_64F);
    rejects([&] {
        mergeRknnOutputBranches({wrong_type, scores});
    });
    rejects([&] {
        mergeRknnOutputBranches({boxes, tensor({1, 4, 2}, 0)});
    });
}

void rawXyTests() {
    std::vector<cv::Mat> branches;
    for (int channels : {4, 2, 8, 8, 4})
        branches.push_back(tensor({1, channels, 8400}, float(branches.size())));
    auto merged = mergeRknnOutputBranches(branches);
    const auto* values = merged.ptr<float>();
    for (int row = 0; row < 26; ++row) {
        for (int i = 0; i < 8400; ++i) {
            float expected;
            if (row < 6) {
                const int b = row < 4 ? 0 : 1;
                expected = branches[b].ptr<float>()[(row < 4 ? row : row - 4) * 8400 + i];
            } else if (row >= 22) {
                expected = branches[4].ptr<float>()[(row - 22) * 8400 + i];
            } else {
                const int b = row < 14 ? 2 : 3;
                const int channel = (row - 6) % 8;
                const int side = i < 6400 ? 80 : (i < 8000 ? 40 : 20);
                const int offset = i < 6400 ? 0 : (i < 8000 ? 6400 : 8000);
                const int cell = i - offset;
                const int grid = channel % 2 == 0 ? cell % side : cell / side;
                expected = (2 * branches[b].ptr<float>()[channel * 8400 + i] + grid) * (640 / side);
            }
            assert(values[row * 8400 + i] == expected);
        }
    }
    // The returned matrix owns its data after the SDK branch buffers are released.
    branches.clear();
    assert(merged.size[1] == 26 && values[0] == 0);
}

} // namespace

int main() {
    layoutTests();
    mergeTests();
    rawXyTests();
    std::cout << "rknn output layout tests passed\n";
    return 0;
}
