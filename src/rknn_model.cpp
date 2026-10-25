#include "rescue/stage_audit.hpp"
#include "rescue/rknn_model.hpp"
#include "../benchmark_results/rknn_api.h"
#include <dlfcn.h>
#include <stdexcept>
#include <fstream>
#include <array>
#include <cstdlib>
#include <cstring>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif
namespace rescue {
namespace {
void check(int code,const std::string &operation){if(code!=RKNN_SUCC)throw std::runtime_error(operation+": "+std::to_string(code));}
template<class T>T symbol(void *lib,const char *name) {
    auto p=dlsym(lib,name);if(!p)throw std::runtime_error(std::string("Missing runtime symbol: ")+name);
    return reinterpret_cast<T>(p);
}
}
// Restrict native normalization to the exact model validated against the old path.
// FNV-1a is a content identity guard, not a security checksum.
static bool validatedPoseModel(const std::string &path) {
    std::ifstream f(path,std::ios::binary);if(!f)return false;
    uint64_t hash=14695981039346656037ULL;std::array<char,65536> bytes;
    while(f){f.read(bytes.data(),bytes.size());for(std::streamsize i=0;i<f.gcount();++i){hash^=static_cast<unsigned char>(bytes[i]);hash*=1099511628211ULL;}}
    return f.eof()&&hash==0x592ca2315fd05ddeULL;
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
    std::string audit_prefix;
    rknn_tensor_mem *native_input=nullptr;
    decltype(&rknn_create_mem) create_mem=nullptr;
    decltype(&rknn_destroy_mem) destroy_mem=nullptr;
    decltype(&rknn_set_io_mem) set_io_mem=nullptr;
    decltype(&rknn_mem_sync) mem_sync=nullptr;
    int size=0,channels=0,anchors=0;
    std::vector<rknn_tensor_attr> outputs;
    ~Runtime(){if(native_input&&destroy_mem)destroy_mem(context,native_input);if(initialized&&destroy)destroy(context);if(lib)dlclose(lib);}
};
RknnModel::RknnModel(const std::string &library,const std::string &model,int core_mask):r_(std::make_unique<Runtime>()) {
    auto &r=*r_;
    r.audit_prefix=model.find("pose")!=std::string::npos?"pose":"detector";
    r.lib=dlopen(library.c_str(),RTLD_NOW|RTLD_LOCAL);
    if(!r.lib)throw std::runtime_error(std::string("Cannot load RKNN runtime: ")+dlerror());
#define LOAD(name) r.name=symbol<decltype(r.name)>(r.lib,"rknn_" #name)
    LOAD(init);LOAD(destroy);LOAD(query);LOAD(set_core_mask);LOAD(inputs_set);LOAD(run);LOAD(outputs_get);LOAD(outputs_release);
#undef LOAD
    check(r.init(&r.context,const_cast<char*>(model.c_str()),0,0,nullptr),"rknn_init "+model);
    r.initialized=true;
    if(core_mask>=0)check(r.set_core_mask(r.context,static_cast<rknn_core_mask>(core_mask)),"rknn_set_core_mask");
    rknn_input_output_num num{};check(r.query(r.context,RKNN_QUERY_IN_OUT_NUM,&num,sizeof(num)),"query counts");
    if(num.n_input!=1 || (num.n_output!=1 && num.n_output!=2 && num.n_output!=4))throw std::runtime_error("Expected one input and one, two or four outputs: "+model);
    rknn_tensor_attr input{};
    check(r.query(r.context,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input)),"query input");
    if(input.n_dims!=4 || input.dims[0]!=1 || input.dims[1]!=input.dims[2] || input.dims[1]==0 || input.dims[3]!=3 || input.fmt!=RKNN_TENSOR_NHWC)
        throw std::runtime_error("Expected square NHWC RGB input: "+model);
    r.outputs.resize(num.n_output);
    for(unsigned i=0;i<num.n_output;++i) {
        auto &attr=r.outputs[i];attr.index=i;
        check(r.query(r.context,RKNN_QUERY_OUTPUT_ATTR,&attr,sizeof(attr)),"query output");
        if((attr.n_dims!=3 && attr.n_dims!=4) || attr.dims[0]!=1)
            throw std::runtime_error("Expected batch-one 3D or 4D output: "+model);
        size_t elements=1;
        for(unsigned j=0;j<attr.n_dims;++j){if(!attr.dims[j])throw std::runtime_error("Empty RKNN dimension");elements*=attr.dims[j];}
        if(elements!=attr.n_elems)throw std::runtime_error("RKNN element count mismatch");
        if(num.n_output!=4 && attr.n_dims!=3)throw std::runtime_error("Expected [1,C,N] output: "+model);
        std::cerr<<"[RKNN_OUTPUT] index="<<i<<" channels="<<attr.dims[1]<<" anchors="<<attr.dims[attr.n_dims-1]
                 <<" type="<<get_type_string(attr.type)<<" scale="<<attr.scale<<" zp="<<attr.zp<<"\n";
    }
    if(num.n_output==2 && (r.outputs[0].dims[1]!=4 || r.outputs[1].dims[1]!=4 || r.outputs[0].dims[2]!=r.outputs[1].dims[2]))
        throw std::runtime_error("Expected boxes then four-class scores with equal anchors: "+model);
    if(num.n_output==4) {
        const auto n=r.outputs[0].dims[2];
        const std::vector<std::vector<uint32_t>> expected{{1,4,n},{1,2,n},{1,4,2,n},{1,4,1,n}};
        for(unsigned i=0;i<4;++i) {
            const auto &a=r.outputs[i];
            if(a.n_dims!=expected[i].size())throw std::runtime_error("Pose branch rank mismatch");
            for(unsigned j=0;j<a.n_dims;++j)if(a.dims[j]!=expected[i][j])throw std::runtime_error("Pose branch shape mismatch");
        }
    }
    const auto &output=r.outputs[0];
    if(StageAudit::instance().enabled()) {
        rknn_tensor_attr native{};
        const int rc=r.query(r.context,RKNN_QUERY_NATIVE_INPUT_ATTR,&native,sizeof(native));
        std::cerr << "[AUDIT_INPUT] " << r.audit_prefix << " declared_type=" << get_type_string(input.type)
                  << " declared_format=" << get_format_string(input.fmt) << " bytes=" << input.size;
        if(rc==0){std::cerr << " native_type=" << get_type_string(native.type) << " native_format=" << get_format_string(native.fmt)
                           << " native_bytes=" << native.size << " native_stride_bytes=" << native.size_with_stride << " dims=";
            for(unsigned i=0;i<native.n_dims;++i)std::cerr<<native.dims[i]<<":";
        }
        std::cerr << "\n";
    }
    r.size=int(input.dims[1]);r.channels=num.n_output==4?18:int(output.dims[1])+(num.n_output==2?int(r.outputs[1].dims[1]):0);r.anchors=int(output.dims[2]);
#if defined(__aarch64__)
    const char *native_env=std::getenv("RESCUE_POSE_NATIVE_FP16");
    if((!native_env||std::string(native_env)!="0") && r.audit_prefix=="pose" && validatedPoseModel(model)) {
        rknn_tensor_attr attr{};
        check(r.query(r.context,RKNN_QUERY_NATIVE_INPUT_ATTR,&attr,sizeof(attr)),"native input attr");
        if(attr.type!=RKNN_TENSOR_FLOAT16||attr.fmt!=RKNN_TENSOR_NHWC||attr.n_dims!=4||
           attr.dims[0]!=1||attr.dims[1]!=640||attr.dims[2]!=640||attr.dims[3]!=3||
           attr.size_with_stride!=640*640*3*2||attr.size!=attr.size_with_stride)
            throw std::runtime_error("Validated pose native input contract changed");
        r.create_mem=symbol<decltype(r.create_mem)>(r.lib,"rknn_create_mem");
        r.destroy_mem=symbol<decltype(r.destroy_mem)>(r.lib,"rknn_destroy_mem");
        r.set_io_mem=symbol<decltype(r.set_io_mem)>(r.lib,"rknn_set_io_mem");
        r.mem_sync=symbol<decltype(r.mem_sync)>(r.lib,"rknn_mem_sync");
        r.native_input=r.create_mem(r.context,attr.size_with_stride);
        if(!r.native_input)throw std::runtime_error("Cannot allocate native pose input");
        attr.pass_through=1;check(r.set_io_mem(r.context,r.native_input,&attr),"bind native pose input");
        std::cerr<<"[RKNN] pose native FP16 input enabled (validated model, RGB/255); RESCUE_POSE_NATIVE_FP16=0 restores old path\n";
    }
#endif
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
    #if defined(__aarch64__)
    if(r.native_input) {
        AuditScope audit(r.audit_prefix+".native_input_prepare");
        const auto *src=rgb.ptr<unsigned char>();auto *dst=static_cast<float16_t*>(r.native_input->virt_addr);
        const size_t count=rgb.total()*3;size_t i=0;
        for(;i+8<=count;i+=8){
            const uint16x8_t u=vmovl_u8(vld1_u8(src+i));
            const float32x4_t lo=vmulq_n_f32(vcvtq_f32_u32(vmovl_u16(vget_low_u16(u))),1.f/255.f);
            const float32x4_t hi=vmulq_n_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(u))),1.f/255.f);
            vst1q_f16(dst+i,vcombine_f16(vcvt_f16_f32(lo),vcvt_f16_f32(hi)));
        }
        for(;i<count;++i)dst[i]=static_cast<float16_t>(float(src[i])*(1.f/255.f));
        check(r.mem_sync(r.context,r.native_input,RKNN_MEMORY_SYNC_TO_DEVICE),"sync native pose input");
    } else
#endif
    {AuditScope audit(r.audit_prefix+".inputs_set");check(r.inputs_set(r.context,1,&input),"inputs_set");}
    {AuditScope audit(r.audit_prefix+".rknn_run");check(r.run(r.context,nullptr),"run");}
    std::vector<rknn_output> outputs(r.outputs.size());
    for(unsigned i=0;i<outputs.size();++i){outputs[i].index=i;outputs[i].want_float=1;}
    {AuditScope audit(r.audit_prefix+".outputs_get");check(r.outputs_get(r.context,outputs.size(),outputs.data(),nullptr),"outputs_get");}
    if(StageAudit::instance().enabled()) {
        rknn_perf_run perf{};
        if(r.query(r.context,RKNN_QUERY_PERF_RUN,&perf,sizeof(perf))==0&&perf.run_duration>0)
            StageAudit::instance().add(r.audit_prefix+".runtime_reported_npu",perf.run_duration/1000.0,0);
    }
    struct Release {Runtime &r;std::vector<rknn_output> &o;~Release(){AuditScope audit(r.audit_prefix+".outputs_release");r.outputs_release(r.context,o.size(),o.data());}} release{r,outputs};
    int shape[]={1,r.channels,r.anchors};
    AuditScope audit(r.audit_prefix+".output_clone");
    cv::Mat result(3,shape,CV_32F);size_t offset=0;
    for(unsigned i=0;i<outputs.size();++i) {
        const size_t bytes=size_t(r.outputs[i].n_elems)*sizeof(float);
        if(!outputs[i].buf || outputs[i].size<bytes)throw std::runtime_error("Truncated RKNN output");
        // want_float=1 lets Runtime apply each branch's own quantization scale.
        if(outputs.size()!=4 || i<2) {
            std::memcpy(result.ptr<float>()+offset,outputs[i].buf,bytes);offset+=r.outputs[i].n_elems;
        }
    }
    if(outputs.size()==4) {
        const float *xy=static_cast<const float*>(outputs[2].buf),*visibility=static_cast<const float*>(outputs[3].buf);
        float *dst=result.ptr<float>();
        for(int k=0;k<4;++k) {
            std::memcpy(dst+(6+3*k)*r.anchors,xy+(2*k)*r.anchors,r.anchors*sizeof(float));
            std::memcpy(dst+(7+3*k)*r.anchors,xy+(2*k+1)*r.anchors,r.anchors*sizeof(float));
            std::memcpy(dst+(8+3*k)*r.anchors,visibility+k*r.anchors,r.anchors*sizeof(float));
        }
    }
    return result;
}
} // namespace rescue
