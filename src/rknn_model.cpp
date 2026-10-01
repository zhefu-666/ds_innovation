#include "rescue/rknn_model.hpp"
#include "../benchmark_results/rknn_api.h"
#include <dlfcn.h>
#include <stdexcept>
namespace rescue {
namespace {
void check(int code,const std::string &operation){if(code!=RKNN_SUCC)throw std::runtime_error(operation+": "+std::to_string(code));}
template<class T>T symbol(void *lib,const char *name) {
    auto p=dlsym(lib,name);if(!p)throw std::runtime_error(std::string("Missing runtime symbol: ")+name);
    return reinterpret_cast<T>(p);
}
}
struct RknnModel::Runtime {
    void *lib=nullptr;rknn_context context=0;bool initialized=false;
    decltype(&rknn_init) init=nullptr;
    decltype(&rknn_destroy) destroy=nullptr;
    decltype(&rknn_query) query=nullptr;
    decltype(&rknn_set_core_mask) set_core_mask=nullptr;
    decltype(&rknn_inputs_set) inputs_set=nullptr;
    decltype(&rknn_run) run=nullptr;
    decltype(&rknn_outputs_get) outputs_get=nullptr;
    decltype(&rknn_outputs_release) outputs_release=nullptr;
    int size=0,channels=0,anchors=0;
    ~Runtime(){if(initialized&&destroy)destroy(context);if(lib)dlclose(lib);}
};
RknnModel::RknnModel(const std::string &library,const std::string &model,int core_mask):r_(std::make_unique<Runtime>()) {
    auto &r=*r_;
    r.lib=dlopen(library.c_str(),RTLD_NOW|RTLD_LOCAL);
    if(!r.lib)throw std::runtime_error(std::string("Cannot load RKNN runtime: ")+dlerror());
#define LOAD(name) r.name=symbol<decltype(r.name)>(r.lib,"rknn_" #name)
    LOAD(init);LOAD(destroy);LOAD(query);LOAD(set_core_mask);LOAD(inputs_set);LOAD(run);LOAD(outputs_get);LOAD(outputs_release);
#undef LOAD
    check(r.init(&r.context,const_cast<char*>(model.c_str()),0,0,nullptr),"rknn_init "+model);
    r.initialized=true;
    if(core_mask>=0)check(r.set_core_mask(r.context,static_cast<rknn_core_mask>(core_mask)),"rknn_set_core_mask");
    rknn_input_output_num num{};check(r.query(r.context,RKNN_QUERY_IN_OUT_NUM,&num,sizeof(num)),"query counts");
    if(num.n_input!=1 || num.n_output!=1)throw std::runtime_error("Expected one input and one output: "+model);
    rknn_tensor_attr input{},output{};
    check(r.query(r.context,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input)),"query input");
    check(r.query(r.context,RKNN_QUERY_OUTPUT_ATTR,&output,sizeof(output)),"query output");
    if(input.n_dims!=4 || input.dims[0]!=1 || input.dims[1]!=input.dims[2] || input.dims[1]==0 || input.dims[3]!=3 ||
       input.fmt!=RKNN_TENSOR_NHWC || output.n_dims!=3 || output.dims[0]!=1 || output.dims[1]==0 || output.dims[2]==0 ||
       output.n_elems!=output.dims[1]*output.dims[2])
        throw std::runtime_error("RKNN tensor contract mismatch (expected square NHWC RGB input and one [1,C,N] output): "+model);
    r.size=int(input.dims[1]);r.channels=int(output.dims[1]);r.anchors=int(output.dims[2]);
}
RknnModel::~RknnModel()=default;
int RknnModel::inputSize() const {return r_->size;}
int RknnModel::outputChannels() const {return r_->channels;}
int RknnModel::outputAnchors() const {return r_->anchors;}
cv::Mat RknnModel::run(const cv::Mat &rgb) {
    auto &r=*r_;
    if(rgb.rows!=r.size||rgb.cols!=r.size||rgb.type()!=CV_8UC3||!rgb.isContinuous())
        throw std::runtime_error("RKNN input must be a continuous "+std::to_string(r.size)+"x"+std::to_string(r.size)+" RGB image");
    rknn_input input{};input.index=0;input.buf=rgb.data;input.size=static_cast<uint32_t>(rgb.total()*3);
    input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=0;
    check(r.inputs_set(r.context,1,&input),"inputs_set");check(r.run(r.context,nullptr),"run");
    rknn_output output{};output.want_float=1;
    check(r.outputs_get(r.context,1,&output,nullptr),"outputs_get");
    struct Release {Runtime &r;rknn_output &o;~Release(){r.outputs_release(r.context,1,&o);}} release{r,output};
    if(!output.buf || output.size<size_t(r.channels)*r.anchors*sizeof(float))throw std::runtime_error("Truncated RKNN output");
    int shape[]={1,r.channels,r.anchors};
    return cv::Mat(3,shape,CV_32F,output.buf).clone();
}
} // namespace rescue
