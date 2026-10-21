#pragma once
#include <onnxruntime_cxx_api.h>
#include "g1_visual_config.h"
#include "g1_visual_observation.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_parkour {
using g1_visual::Joints;
using g1_visual::Observation;
using g1_visual::RecoverObservationGap;
using g1_visual::Decode;
using Image = std::array<float,18*32>;

class Policy : public g1_visual::JointConfig {
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "g1_parkour"};
    std::unique_ptr<Ort::Session> encoder_, actor_;
    std::string encoder_in_, encoder_out_, actor_in_, actor_out_;
    std::deque<std::array<float,96>> history_;
    std::deque<Image> images_;
    Joints last_action_{};
    int delay_ = 0;

    static void Validate(Ort::Session& session, const std::vector<int64_t>& input,
                         const std::vector<int64_t>& output, std::string& in, std::string& out) {
        if (session.GetInputCount()!=1 || session.GetOutputCount()!=1)
            throw std::runtime_error("Expected single-input single-output network");
        const auto input_type = session.GetInputTypeInfo(0);
        const auto output_type = session.GetOutputTypeInfo(0);
        const auto a = input_type.GetTensorTypeAndShapeInfo();
        const auto b = output_type.GetTensorTypeAndShapeInfo();
        if (a.GetShape()!=input || b.GetShape()!=output ||
            a.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            b.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("Incompatible parkour ONNX tensor shape/type");
        Ort::AllocatorWithDefaultOptions allocator;
        in=session.GetInputNameAllocated(0,allocator).get();
        out=session.GetOutputNameAllocated(0,allocator).get();
    }
    static std::vector<float> Run(Ort::Session& session, std::vector<float>& data,
                                  const std::vector<int64_t>& shape,
                                  const std::string& input, const std::string& output) {
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto tensor=Ort::Value::CreateTensor<float>(memory,data.data(),data.size(),shape.data(),shape.size());
        const char* in=input.c_str(); const char* out=output.c_str();
        auto result=session.Run(Ort::RunOptions{nullptr},&in,&tensor,1,&out,1);
        const auto count=result[0].GetTensorTypeAndShapeInfo().GetElementCount();
        const auto* values=result[0].GetTensorData<float>();
        for(std::size_t i=0;i<count;++i)
            if(!std::isfinite(values[i])) throw std::runtime_error("Non-finite network output");
        return {values,values+count};
    }
public:
    const std::array<float,3> command_min{0,0,-1}, command_max{1,0,1};
    Policy(const std::string& models, const std::string& config) : JointConfig(config) {
        const auto cfg=YAML::LoadFile(config);
        delay_=cfg["depth_delay_frames"].as<int>();
        if(delay_!=0 && delay_!=1) throw std::runtime_error("Expected depth delay 0 or 1");
        Ort::SessionOptions options; options.SetIntraOpNumThreads(1); options.SetInterOpNumThreads(1);
        encoder_=std::make_unique<Ort::Session>(env_,(models+"/0-depth_encoder.onnx").c_str(),options);
        actor_=std::make_unique<Ort::Session>(env_,(models+"/actor.onnx").c_str(),options);
        Validate(*encoder_,{1,8,18,32},{1,128},encoder_in_,encoder_out_);
        Validate(*actor_,{1,896},{1,29},actor_in_,actor_out_);
    }
    void Reset() { history_.clear(); images_.clear(); last_action_.fill(0); }
    static Image Preprocess(const std::array<float,36*64>& depth) {
        for(float v:depth) if(std::isnan(v) || v<0) throw std::runtime_error("Invalid raw depth");
        Image image{};
        const double w0=std::exp(-.5), sum=1+2*w0;
        const std::array<double,3> weights{w0/sum,1/sum,w0/sum};
        auto reflect=[](int v,int size) {return v<0 ? -v : (v>=size ? 2*size-2-v : v);};
        for(int y=0;y<18;++y) for(int x=0;x<32;++x) {
            double value=0;
            for(int j=-1;j<=1;++j) for(int i=-1;i<=1;++i) {
                float d=depth[(reflect(y+j,18)+18)*64+reflect(x+i,32)+16];
                // An infinite no-hit remains far after blur and depth clipping.
                value+=weights[j+1]*weights[i+1]*(std::isinf(d)?1e30:double(d));
            }
            image[y*32+x]=std::clamp(value,0.,2.5)/2.5;
        }
        return image;
    }
    std::pair<std::vector<float>,std::vector<float>> Observe(const Observation& obs,
                                                           const std::array<float,3>& command) {
        std::array<float,96> frame{};
        for(int i=0;i<3;++i) {frame[i]=obs.omega[i]*.25f;frame[3+i]=obs.gravity[i];frame[6+i]=command[i];}
        for(int i=0;i<29;++i) {frame[9+i]=obs.q[i]-neutral[i];frame[38+i]=obs.dq[i]*.05f;frame[67+i]=last_action_[i];}
        for(float v:frame) if(!std::isfinite(v)) throw std::runtime_error("Non-finite proprioception");
        const auto image=Preprocess(obs.depth);
        if(history_.empty()) history_.assign(8,frame); else {history_.pop_front();history_.push_back(frame);}
        if(images_.empty()) images_.assign(37,image); else {images_.pop_front();images_.push_back(image);}
        std::vector<float> proprio, depth;
        proprio.reserve(768); depth.reserve(8*18*32);
        const std::array<int,6> starts{0,3,6,9,38,67}, sizes{3,3,3,29,29,29};
        for(int term=0;term<6;++term) for(const auto& f:history_)
            proprio.insert(proprio.end(),f.begin()+starts[term],f.begin()+starts[term]+sizes[term]);
        for(int offset=35;offset>=0;offset-=5) {
            const auto& im=images_[36-delay_-offset]; depth.insert(depth.end(),im.begin(),im.end());
        }
        return {proprio,depth};
    }
    Joints Step(const Observation& obs,const std::array<float,3>& command) {
        return Infer(Observe(obs,command));
    }
    Joints Infer(std::pair<std::vector<float>,std::vector<float>> inputs) {
        if(inputs.first.size()!=768 || inputs.second.size()!=8*18*32)
            throw std::runtime_error("Invalid prepared parkour inputs");
        auto latent=Run(*encoder_,inputs.second,{1,8,18,32},encoder_in_,encoder_out_);
        inputs.first.insert(inputs.first.end(),latent.begin(),latent.end());
        auto actions=Run(*actor_,inputs.first,{1,896},actor_in_,actor_out_);
        Joints target{};
        for(int i=0;i<29;++i) {
            last_action_[i]=actions[i]; target[ids[i]]=sign[i]*(neutral[i]+scale[i]*actions[i]);
            if(!std::isfinite(target[ids[i]])) throw std::runtime_error("Non-finite target");
        }
        return target;
    }
};

} // namespace g1_parkour
