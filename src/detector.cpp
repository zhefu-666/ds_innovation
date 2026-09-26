#include "rescue/detector.hpp"
#include "rescue/model_io.hpp"
#include "../benchmark_results/rknn_api.h"
#include <dlfcn.h>
#include <chrono>
#include <stdexcept>
#include <cctype>
#include <algorithm>
namespace rescue {
namespace {
void check(int code,const char *operation){if(code!=RKNN_SUCC)throw std::runtime_error(std::string(operation)+": "+std::to_string(code));}
template<class T>T symbol(void *lib,const char *name) {
    auto p=dlsym(lib,name);if(!p)throw std::runtime_error(std::string("Missing runtime symbol: ")+name);
    return reinterpret_cast<T>(p);
}
}
struct YoloRknnDetector::Runtime {
    void *lib=nullptr;rknn_context context=0;bool initialized=false;
    decltype(&rknn_init) init=nullptr;
    decltype(&rknn_destroy) destroy=nullptr;
    decltype(&rknn_query) query=nullptr;
    decltype(&rknn_inputs_set) inputs_set=nullptr;
    decltype(&rknn_run) run=nullptr;
    decltype(&rknn_outputs_get) outputs_get=nullptr;
    decltype(&rknn_outputs_release) outputs_release=nullptr;
    ~Runtime(){if(initialized&&destroy)destroy(context);if(lib)dlclose(lib);}
};
YoloRknnDetector::~YoloRknnDetector()=default;
YoloRknnDetector::YoloRknnDetector(const Config &c):runtime_(std::make_unique<Runtime>()),config_(c) {
    const std::vector<std::string> expected{"core","wounded","red","dangerous","normal","main","blue"};
    if(c.input_size!=448 || c.class_names!=expected || c.require_instance_masks)
        throw std::runtime_error("RKNN profile requires size 448, original seven-class order and box-only output");
    auto &r=*runtime_;
    r.lib=dlopen(c.rknn_library.c_str(),RTLD_NOW|RTLD_LOCAL);
    if(!r.lib)throw std::runtime_error(std::string("Cannot load RKNN runtime: ")+dlerror());
#define LOAD(name) r.name=symbol<decltype(r.name)>(r.lib,"rknn_" #name)
    LOAD(init);LOAD(destroy);LOAD(query);LOAD(inputs_set);LOAD(run);LOAD(outputs_get);LOAD(outputs_release);
#undef LOAD
    check(r.init(&r.context,const_cast<char*>(c.model_path.c_str()),0,0,nullptr),"rknn_init");r.initialized=true;
    rknn_input_output_num num{};check(r.query(r.context,RKNN_QUERY_IN_OUT_NUM,&num,sizeof(num)),"query counts");
    if(num.n_input!=1 || num.n_output!=1)throw std::runtime_error("Expected one input and one output");
    rknn_tensor_attr input{},output{};
    check(r.query(r.context,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input)),"query input");
    check(r.query(r.context,RKNN_QUERY_OUTPUT_ATTR,&output,sizeof(output)),"query output");
    if(input.n_dims!=4 || input.dims[0]!=1 || input.dims[1]!=448 || input.dims[2]!=448 || input.dims[3]!=3 || input.fmt!=RKNN_TENSOR_NHWC ||
       output.n_dims!=3 || output.dims[0]!=1 || output.dims[1]!=11 || output.dims[2]!=4116 || output.n_elems!=11*4116)
        throw std::runtime_error("RKNN tensor contract mismatch (expected NHWC 448 and [1,11,4116])");
}
std::vector<SegDetection> YoloRknnDetector::infer(const cv::Mat &frame) {
    auto timestamp=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto image=prepareModelImage(frame,448);auto &r=*runtime_;
    rknn_input input{};input.buf=image.rgb.data;input.size=static_cast<uint32_t>(image.rgb.total()*3);
    input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=0;
    check(r.inputs_set(r.context,1,&input),"inputs_set");check(r.run(r.context,nullptr),"run");
    rknn_output output{};output.want_float=1;
    check(r.outputs_get(r.context,1,&output,nullptr),"outputs_get");
    struct Release {Runtime &r;rknn_output &o;~Release(){r.outputs_release(r.context,1,&o);}} release{r,output};
    if(!output.buf || output.size<11*4116*sizeof(float))throw std::runtime_error("Truncated RKNN output");
    int shape[]={1,11,4116};cv::Mat tensor(3,shape,CV_32F,output.buf);
    return decodeModelOutput(tensor,image,frame.size(),config_,timestamp);
}
std::unique_ptr<IDetector> makeDetector(const Config &config) {
    auto pos=config.model_path.find_last_of('.');auto ext=pos==std::string::npos?"":config.model_path.substr(pos);
    std::transform(ext.begin(),ext.end(),ext.begin(),[](unsigned char ch){return std::tolower(ch);});
    if(ext==".rknn")return std::make_unique<YoloRknnDetector>(config);
    if(ext==".onnx")return std::make_unique<YoloOnnxDetector>(config);
    throw std::runtime_error("Model must be .rknn or .onnx");
}

} // namespace rescue
