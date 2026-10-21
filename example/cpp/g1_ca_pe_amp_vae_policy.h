#pragma once
#include "g1_visual_config.h"
#include "g1_visual_observation.h"
#include <onnxruntime_cxx_api.h>
#include <deque>
#include <memory>

namespace g1_ca_pe_amp_vae {
using g1_visual::Observation;
using g1_visual::Joints;
using g1_visual::Decode;
using g1_visual::RecoverObservationGap;
using Image = std::array<float,32*32>;

class Policy : public g1_visual::JointConfig {
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING,"g1_ca_pe_amp_vae"};
    std::unique_ptr<Ort::Session> encoder_,actor_;
    std::deque<std::array<float,96>> history_;
    std::deque<Image> images_;
    Joints last_action_{},previous_action_{};
    int delay_=0;
    int history_action_delay_=1;
    float action_clip_=100,omega_scale_=.2f,dq_scale_=.05f;

    static bool Shape(const std::vector<int64_t>& actual,const std::vector<int64_t>& expected) {
        if(actual.size()!=expected.size()) return false;
        for(std::size_t i=0;i<actual.size();++i)
            if(actual[i]>=0 && actual[i]!=expected[i]) return false;
        return true;
    }
    static void Validate(Ort::Session& session,
                         const std::vector<std::pair<std::string,std::vector<int64_t>>>& inputs,
                         const std::string& output,const std::vector<int64_t>& shape) {
        if(session.GetInputCount()!=inputs.size() || session.GetOutputCount()!=1)
            throw std::runtime_error("Invalid CA PE network input/output count");
        Ort::AllocatorWithDefaultOptions allocator;
        for(std::size_t i=0;i<=inputs.size();++i) {
            const bool in=i<inputs.size();
            const auto type=in?session.GetInputTypeInfo(i):session.GetOutputTypeInfo(0);
            const auto info=type.GetTensorTypeAndShapeInfo();
            const std::string name=in?session.GetInputNameAllocated(i,allocator).get():session.GetOutputNameAllocated(0,allocator).get();
            if(name!=(in?inputs[i].first:output) || !Shape(info.GetShape(),in?inputs[i].second:shape) ||
               info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
                throw std::runtime_error("Incompatible CA PE tensor: "+name);
        }
    }
    static std::vector<float> Output(Ort::Value& tensor,const std::vector<int64_t>& shape) {
        if(tensor.GetTensorTypeAndShapeInfo().GetShape()!=shape)
            throw std::runtime_error("Unexpected runtime CA PE output shape");
        const auto count=tensor.GetTensorTypeAndShapeInfo().GetElementCount();
        const float* values=tensor.GetTensorData<float>();
        for(std::size_t i=0;i<count;++i)
            if(!std::isfinite(values[i])) throw std::runtime_error("Non-finite CA PE network output");
        return {values,values+count};
    }
public:
    struct Inputs {std::vector<float> current,history,depth;};
    std::array<float,3> command_min{},command_max{};
    Policy(const std::string& models,const std::string& config) : JointConfig(config) {
        const auto cfg=YAML::LoadFile(config);
        if(cfg["policy_type"].as<std::string>()!="ca_pe_amp_vae" || cfg["state_body"].as<std::string>()!="pelvis" ||
           cfg["proprio_history_frames"].as<int>()!=5 || cfg["depth_history_frames"].as<int>()!=8 ||
           cfg["depth_history_stride"].as<int>()!=5 || Read<4>(cfg["depth_crop"])!=std::array<float,4>{4,0,16,16} ||
           Read<2>(cfg["depth_range"])!=std::array<float,2>{0,2.5} || cfg["depth_blur_sigma"].as<float>()!=1)
            throw std::runtime_error("Unsupported CA PE observation configuration");
        delay_=cfg["depth_delay_frames"].as<int>();action_clip_=cfg["action_clip"].as<float>();
        history_action_delay_=cfg["history_action_delay"].as<int>(1);
        if(history_action_delay_!=0 && history_action_delay_!=1)
            throw std::runtime_error("history_action_delay must be 0 or 1");
        omega_scale_=cfg["angular_velocity_scale"].as<float>();dq_scale_=cfg["joint_velocity_scale"].as<float>();
        command_min=Read<3>(cfg["command_min"]);command_max=Read<3>(cfg["command_max"]);
        if((delay_!=0 && delay_!=1) || !std::isfinite(action_clip_) || action_clip_<=0 ||
           !std::isfinite(omega_scale_) || !std::isfinite(dq_scale_)) throw std::runtime_error("Invalid CA PE scaling/delay");
        for(int i=0;i<3;++i) if(command_min[i]>0 || command_max[i]<0 || command_min[i]>command_max[i])
            throw std::runtime_error("Invalid CA PE command bounds");
        Ort::SessionOptions options;options.SetIntraOpNumThreads(1);options.SetInterOpNumThreads(1);
        encoder_=std::make_unique<Ort::Session>(env_,(models+"/"+cfg["depth_model"].as<std::string>()).c_str(),options);
        actor_=std::make_unique<Ort::Session>(env_,(models+"/"+cfg["actor_model"].as<std::string>()).c_str(),options);
        Validate(*encoder_,{{"depth_image",{1,8,32,32}}},"depth_tokens",{1,64,64});
        Validate(*actor_,{{"actor_obs",{1,96}},{"obs_history",{1,480}},{"depth_tokens",{1,64,64}}},"actions",{1,29});
    }
    void Reset() {history_.clear();images_.clear();last_action_.fill(0);previous_action_.fill(0);}
    static Image Preprocess(const std::array<float,36*64>& depth) {
        for(float v:depth) if(std::isnan(v)||v<0) throw std::runtime_error("Invalid raw depth");
        Image image{};
        const double w=std::exp(-.5),sum=1+2*w;
        const std::array<double,3> weights{w/sum,1/sum,w/sum};
        auto reflect=[](int v) {return v<0?-v:(v>=32?62-v:v);};
        for(int y=0;y<32;++y) for(int x=0;x<32;++x) {
            double value=0;
            for(int j=-1;j<=1;++j) for(int i=-1;i<=1;++i) {
                const float d=depth[(reflect(y+j)+4)*64+reflect(x+i)+16];
                value+=weights[j+1]*weights[i+1]*(std::isinf(d)?1e30:double(d));
            }
            image[y*32+x]=std::clamp(value,0.,2.5)/2.5;
        }
        return image;
    }
    Inputs Observe(const Observation& obs,const std::array<float,3>& command) {
        std::array<float,96> frame{};
        for(int i=0;i<3;++i) {frame[i]=obs.omega[i]*omega_scale_;frame[3+i]=obs.gravity[i];
            if(!std::isfinite(command[i])) throw std::runtime_error("Invalid velocity command");
            frame[6+i]=std::clamp(command[i],command_min[i],command_max[i]);}
        for(int i=0;i<29;++i) {frame[9+i]=obs.q[i]-neutral[i];frame[38+i]=obs.dq[i]*dq_scale_;frame[67+i]=last_action_[i];}
        for(float v:frame) if(!std::isfinite(v)) throw std::runtime_error("Non-finite proprioception");
        const auto image=Preprocess(obs.depth);
        // Training policy.last_action reads action_manager.action, whereas
        // ActorHistoryObservation._actor_frame reads action_manager.prev_action.
        // process_action shifts action -> prev_action before applying the new action.
        auto history_frame=frame;
        // Newer training snapshots use action in both current and history frames.
        if(history_action_delay_==1)
            for(int i=0;i<29;++i) history_frame[67+i]=previous_action_[i];
        if(history_.empty()) history_.assign(5,history_frame);else {history_.pop_front();history_.push_back(history_frame);}
        if(images_.empty()) images_.assign(37,image);else {images_.pop_front();images_.push_back(image);}
        Inputs result;result.current.assign(frame.begin(),frame.end());
        for(const auto& f:history_) result.history.insert(result.history.end(),f.begin(),f.end());
        for(int offset=35;offset>=0;offset-=5) {
            const auto& im=images_[36-delay_-offset];result.depth.insert(result.depth.end(),im.begin(),im.end());
        }
        return result;
    }
    Joints Infer(Inputs inputs) {
        if(inputs.current.size()!=96 || inputs.history.size()!=480 || inputs.depth.size()!=8192)
            throw std::runtime_error("Invalid CA PE prepared inputs");
        for(const auto* input:{&inputs.current,&inputs.history,&inputs.depth})
            for(float v:*input) if(!std::isfinite(v)) throw std::runtime_error("Non-finite prepared inputs");
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
        auto tensor=[&](std::vector<float>& data,const std::vector<int64_t>& shape) {
            return Ort::Value::CreateTensor<float>(memory,data.data(),data.size(),shape.data(),shape.size());};
        auto depth=tensor(inputs.depth,{1,8,32,32});
        const char* depth_in="depth_image";const char* depth_out="depth_tokens";
        auto encoded=encoder_->Run(Ort::RunOptions{nullptr},&depth_in,&depth,1,&depth_out,1);
        auto tokens=Output(encoded[0],{1,64,64});
        std::array<Ort::Value,3> tensors{tensor(inputs.current,{1,96}),tensor(inputs.history,{1,480}),tensor(tokens,{1,64,64})};
        const char* names[]={"actor_obs","obs_history","depth_tokens"};const char* output="actions";
        auto result=actor_->Run(Ort::RunOptions{nullptr},names,tensors.data(),3,&output,1);
        auto actions=Output(result[0],{1,29});Joints target{};
        previous_action_=last_action_;
        for(int i=0;i<29;++i) {
            // The training wrapper clips raw actions before feedback and scaling.
            last_action_[i]=std::clamp(actions[i],-action_clip_,action_clip_);
            target[ids[i]]=sign[i]*(neutral[i]+scale[i]*last_action_[i]);
            if(!std::isfinite(target[ids[i]])) throw std::runtime_error("Non-finite target");
        }
        return target;
    }
    Joints Step(const Observation& obs,const std::array<float,3>& command) {return Infer(Observe(obs,command));}
};
} // namespace g1_ca_pe_amp_vae
