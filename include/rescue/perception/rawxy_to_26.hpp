#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>
// Input buffers MUST be float32 from want_float=1, in queried output order.
// No smoothing, clipping, routing or letterbox reversal happens here.
inline std::vector<float> pose_rawxy_to_26(const std::array<const float*,5>& out,
                                          const std::array<std::size_t,5>& count) {
    constexpr std::size_t N=8400;
    const std::array<std::size_t,5> expected{4*N,2*N,8*N,8*N,4*N};
    if(count!=expected) throw std::invalid_argument("Expected 5 outputs: 4,2,8,8,4 channels, 8400 anchors");
    for(std::size_t j=0;j<5;++j) {
        if(!out[j]) throw std::invalid_argument("Null RKNN output buffer");
        for(std::size_t i=0;i<count[j];++i)
            if(!std::isfinite(out[j][i])) throw std::invalid_argument("Nonfinite RKNN output");
    }
    std::vector<float> result(26*N);
    for(std::size_t i=0;i<4*N;++i) result[i]=out[0][i];
    for(std::size_t i=0;i<2*N;++i) result[4*N+i]=out[1][i];
    for(std::size_t i=0;i<4*N;++i) result[22*N+i]=out[4][i];
    std::size_t offset=0;
    for(int side: {80,40,20}) {
        const float stride=640.f/side;
        for(int gy=0;gy<side;++gy) for(int gx=0;gx<side;++gx) {
            const auto i=offset+gy*side+gx;
            for(std::size_t expert=0;expert<2;++expert) for(std::size_t slot=0;slot<4;++slot) {
                result[(6+8*expert+2*slot)*N+i]=(out[2+expert][2*slot*N+i]*2.f+gx)*stride;
                result[(7+8*expert+2*slot)*N+i]=(out[2+expert][(2*slot+1)*N+i]*2.f+gy)*stride;
            }
        }
        offset+=side*side;
    }
    return result;
}
