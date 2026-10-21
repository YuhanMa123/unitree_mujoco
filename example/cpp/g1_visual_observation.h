#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace g1_visual {
using Joints = std::array<float, 29>;
struct Observation {
    uint64_t sequence = 0;
    double time = 0;
    Joints q{}, dq{}; // policy joint order
    std::array<float,3> omega{}, gravity{};
    float height = 0;
    std::array<float,36*64> depth{}; // image-plane metres
};

// Recover at most five missing 20ms samples. Interpolate proprioception while
// holding the last available depth image; never invent a new camera measurement.
// Queued, intact packets are processed individually and do not use this path.
inline std::vector<Observation> RecoverObservationGap(const Observation& previous,
                                                      const Observation& current, double dt) {
    const double elapsed=current.time-previous.time;
    if(elapsed<=0 || elapsed>6*dt+1e-5 || current.sequence<=previous.sequence)
        throw std::runtime_error("Observation gap exceeds 120ms recovery window or sequence is invalid");
    const int steps=std::lround(elapsed/dt);
    if(steps<1 || std::abs(elapsed-steps*dt)>1e-5 ||
       current.sequence-previous.sequence>static_cast<std::uint64_t>(steps))
        throw std::runtime_error("Observation sequence/time mismatch");
    std::vector<Observation> frames;
    for(int step=1;step<steps;++step) {
        Observation frame=previous;
        const float u=float(step)/steps;
        frame.time=previous.time+step*dt;
        auto interpolate=[&](auto& output,const auto& a,const auto& b) {
            for(std::size_t i=0;i<output.size();++i) output[i]=a[i]+u*(b[i]-a[i]);
        };
        interpolate(frame.q,previous.q,current.q);interpolate(frame.dq,previous.dq,current.dq);
        interpolate(frame.omega,previous.omega,current.omega);
        interpolate(frame.gravity,previous.gravity,current.gravity);
        float norm=0;for(float v:frame.gravity) norm+=v*v;
        if(norm>1e-8) for(float& v:frame.gravity) v/=std::sqrt(norm);
        frame.height=previous.height+u*(current.height-previous.height);
        frames.push_back(frame);
    }
    frames.push_back(current);
    return frames;
}

// Local UDP protocol: little endian, 32-byte header and 2369 float32 values.
inline Observation Decode(const unsigned char* bytes,std::size_t size,uint32_t signature) {
    constexpr std::size_t count=29+29+3+3+1+36*64;
    if(size!=32+count*4 || std::memcmp(bytes,"G1PKR001",8)!=0) throw std::runtime_error("Invalid observation packet");
    uint32_t order=0,n=0; Observation result;
    std::memcpy(&result.sequence,bytes+8,8);std::memcpy(&result.time,bytes+16,8);
    std::memcpy(&order,bytes+24,4);std::memcpy(&n,bytes+28,4);
    if(order!=signature || n!=count || !std::isfinite(result.time)) throw std::runtime_error("Observation joint order/count/time mismatch");
    std::array<float,count> values{};std::memcpy(values.data(),bytes+32,count*4);
    std::copy_n(values.begin(),29,result.q.begin());std::copy_n(values.begin()+29,29,result.dq.begin());
    std::copy_n(values.begin()+58,3,result.omega.begin());std::copy_n(values.begin()+61,3,result.gravity.begin());
    result.height=values[64];std::copy_n(values.begin()+65,36*64,result.depth.begin());
    for(int i=0;i<65;++i) if(!std::isfinite(values[i])) throw std::runtime_error("Non-finite observation state");
    for(float d:result.depth) if(std::isnan(d)||d<0) throw std::runtime_error("Invalid observation depth");
    return result;
}
} // namespace g1_visual
