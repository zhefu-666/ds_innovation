#include "rescue/perception/rknn_model.hpp"
#include "rescue/perception/rknn_output_layout.hpp"
#include <rknn_api.h>
#include <dlfcn.h>
#include <stdexcept>
#include <iostream>
namespace rescue {
namespace {
void check(int code, const std::string& operation) {
    if (code != RKNN_SUCC)
        throw std::runtime_error(operation + ": " + std::to_string(code));
}
template <class T> T symbol(void* lib, const char* name) {
    auto p = dlsym(lib, name);
    if (!p)
        throw std::runtime_error(std::string("Missing runtime symbol: ") + name);
    return reinterpret_cast<T>(p);
}
} // namespace
struct RknnModel::Runtime {
    void* lib = nullptr;
    rknn_context context = 0;
    bool initialized = false;
    decltype(&rknn_init) init = nullptr;
    decltype(&rknn_destroy) destroy = nullptr;
    decltype(&rknn_query) query = nullptr;
    decltype(&rknn_set_core_mask) set_core_mask = nullptr;
    decltype(&rknn_inputs_set) inputs_set = nullptr;
    decltype(&rknn_run) run = nullptr;
    decltype(&rknn_outputs_get) outputs_get = nullptr;
    decltype(&rknn_outputs_release) outputs_release = nullptr;
    int size = 0, channels = 0, anchors = 0;
    std::vector<std::vector<uint32_t>> output_dims;
    ~Runtime() {
        if (initialized && destroy)
            destroy(context);
        if (lib)
            dlclose(lib);
    }
};
RknnModel::RknnModel(const std::string& library, const std::string& model, int core_mask)
    : r_(std::make_unique<Runtime>()) {
    auto& r = *r_;
    r.lib = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!r.lib)
        throw std::runtime_error(std::string("Cannot load RKNN runtime: ") + dlerror());
#define LOAD(name) r.name = symbol<decltype(r.name)>(r.lib, "rknn_" #name)
    LOAD(init);
    LOAD(destroy);
    LOAD(query);
    LOAD(set_core_mask);
    LOAD(inputs_set);
    LOAD(run);
    LOAD(outputs_get);
    LOAD(outputs_release);
#undef LOAD
    check(r.init(&r.context, const_cast<char*>(model.c_str()), 0, 0, nullptr),
          "rknn_init " + model);
    r.initialized = true;
    if (core_mask >= 0)
        check(r.set_core_mask(r.context, static_cast<rknn_core_mask>(core_mask)),
              "rknn_set_core_mask");
    rknn_input_output_num num{};
    check(r.query(r.context, RKNN_QUERY_IN_OUT_NUM, &num, sizeof(num)), "query counts");
    if (num.n_input != 1)
        throw std::runtime_error("Expected one RKNN input: " + model);
    rknn_tensor_attr input{};
    check(r.query(r.context, RKNN_QUERY_INPUT_ATTR, &input, sizeof(input)), "query input");
    if (input.n_dims != 4 || input.dims[0] != 1 || input.dims[1] != input.dims[2] ||
        input.dims[1] == 0 || input.dims[3] != 3 || input.fmt != RKNN_TENSOR_NHWC)
        throw std::runtime_error(
            "RKNN tensor contract mismatch (expected square NHWC RGB input): " + model);
    rknn_sdk_version version{};
    check(r.query(r.context, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)), "query SDK");
    std::clog << "[RKNN_MODEL] path=" << model << " api=" << version.api_version
              << " driver=" << version.drv_version << " input=" << input.dims[1] << "x"
              << input.dims[2] << " type=" << get_type_string(input.type) << " zp=" << input.zp
              << " scale=" << input.scale << "\n";
    r.output_dims.resize(num.n_output);
    for (unsigned i = 0; i < num.n_output; ++i) {
        rknn_tensor_attr output{};
        output.index = i;
        check(r.query(r.context, RKNN_QUERY_OUTPUT_ATTR, &output, sizeof(output)), "query output");
        if (output.n_dims < 1 || output.n_dims > 4)
            throw std::runtime_error("Unsupported RKNN output rank: " + model);
        auto& shape = r.output_dims[i];
        shape.reserve(output.n_dims);
        size_t count = 1;
        for (unsigned j = 0; j < output.n_dims; ++j) {
            shape.push_back(output.dims[j]);
            count *= output.dims[j];
        }
        if (count != output.n_elems)
            throw std::runtime_error("RKNN output element count mismatch: " + model);
        std::clog << "[RKNN_OUTPUT] index=" << i << " name=" << output.name << " shape=";
        for (auto dim : shape)
            std::clog << dim << ",";
        std::clog << " type=" << get_type_string(output.type)
                  << " format=" << get_format_string(output.fmt) << " zp=" << output.zp
                  << " scale=" << output.scale << " want_float=1\n";
    }
    const auto layout = validateRknnOutputLayout(model, r.output_dims);
    if (layout.channels == 26) {
        const char* decoded_names[] = {"boxes", "scores", "xy_experts", "old_visibility"};
        const char* raw_names[] = {"boxes", "scores", "base_xy_raw", "far_xy_raw", "visibility"};
        const bool raw_xy = num.n_output == 5;
        const auto names = raw_xy ? raw_names : decoded_names;
        for (unsigned i = 0; i < num.n_output; ++i) {
            rknn_tensor_attr attr{};
            attr.index = i;
            check(r.query(r.context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)),
                  "query expert name");
            if (std::string(attr.name) != names[i])
                throw std::runtime_error("Dual-expert output name/index mismatch: " + model);
            if (raw_xy && attr.type != RKNN_TENSOR_INT8)
                throw std::runtime_error("Raw-XY pose requires INT8 output tensors: " + model);
        }
        if (input.dims[1] != 640)
            throw std::runtime_error("Dual-expert pose requires 640 RGB input");
    }
    r.size = int(input.dims[1]);
    r.channels = layout.channels;
    r.anchors = layout.anchors;
}
RknnModel::~RknnModel() = default;
int RknnModel::inputSize() const {
    return r_->size;
}
int RknnModel::outputChannels() const {
    return r_->channels;
}
int RknnModel::outputAnchors() const {
    return r_->anchors;
}
cv::Mat RknnModel::run(const cv::Mat& rgb) {
    auto& r = *r_;
    if (rgb.rows != r.size || rgb.cols != r.size || rgb.type() != CV_8UC3 || !rgb.isContinuous())
        throw std::runtime_error("RKNN input must be a continuous " + std::to_string(r.size) + "x" +
                                 std::to_string(r.size) + " RGB image");
    rknn_input input{};
    input.index = 0;
    input.buf = rgb.data;
    input.size = static_cast<uint32_t>(rgb.total() * 3);
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.pass_through = 0;
    check(r.inputs_set(r.context, 1, &input), "inputs_set");
    check(r.run(r.context, nullptr), "run");
    std::vector<rknn_output> outputs(r.output_dims.size());
    for (unsigned i = 0; i < outputs.size(); ++i) {
        outputs[i].index = i;
        outputs[i].want_float = 1;
    }
    check(r.outputs_get(r.context, outputs.size(), outputs.data(), nullptr), "outputs_get");
    struct Release {
        Runtime& r;
        std::vector<rknn_output>& o;
        ~Release() {
            r.outputs_release(r.context, o.size(), o.data());
        }
    } release{r, outputs};
    std::vector<cv::Mat> branches;
    branches.reserve(outputs.size());
    for (unsigned i = 0; i < outputs.size(); ++i) {
        size_t elements = 1;
        std::vector<int> shape;
        shape.reserve(r.output_dims[i].size());
        for (const auto dim : r.output_dims[i]) {
            shape.push_back(int(dim));
            elements *= dim;
        }
        if (!outputs[i].buf || outputs[i].size < elements * sizeof(float))
            throw std::runtime_error("Truncated RKNN output");
        branches.emplace_back(static_cast<int>(shape.size()), shape.data(), CV_32F, outputs[i].buf);
        if (!branches.back().isContinuous())
            throw std::runtime_error("Non-contiguous RKNN output");
    }
    return mergeRknnOutputBranches(branches);
}
} // namespace rescue
