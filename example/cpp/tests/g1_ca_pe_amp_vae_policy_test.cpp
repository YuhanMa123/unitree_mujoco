#include "g1_ca_pe_amp_vae_policy.h"
#include <iostream>
#include <limits>
#include <filesystem>

void Check(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
int main(int argc,char** argv) {
    try {
        Check(argc==3,"Expected model directory and config");
        g1_ca_pe_amp_vae::Policy policy(argv[1],argv[2]);
        const int history_delay=YAML::LoadFile(argv[2])["history_action_delay"].as<int>(1);
        const std::array<int,29> expected{0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28};
        Check(policy.ids==expected,"Pelvis-root joint mapping");
        g1_ca_pe_amp_vae::Observation obs;obs.q=policy.neutral;obs.gravity={0,0,-1};
        g1_ca_pe_amp_vae::Policy::Inputs inputs;
        for(int t=0;t<40;++t) {
            obs.omega.fill(t*.01f);obs.depth.fill(t*.01f);
            inputs=policy.Observe(obs,{.3f,.1f,-.2f});
        }
        Check(inputs.current.size()==96 && inputs.history.size()==480 && inputs.depth.size()==8192,"Model input dimensions");
        for(int i=0;i<5;++i) {
            Check(std::abs(inputs.history[96*i]-(35+i)*.01f*.2f)<1e-6,"Frame-wise oldest-to-newest history");
            Check(inputs.history[96*i+6]==.3f && inputs.history[96*i+7]==.1f,"Command position in each history frame");
        }
        for(int i=0;i<8;++i)
            Check(std::abs(inputs.depth[i*1024]-(4+5*i)*.01f/2.5f)<1e-6,"Depth history stride/order");
        // A ramp distinguishes the new 32-row crop from parkour's bottom-half crop.
        for(int y=0;y<36;++y) for(int x=0;x<64;++x) obs.depth[y*64+x]=.01f*y+.001f*x;
        auto image=policy.Preprocess(obs.depth);
        Check(std::abs(image[10*32+10]-(.14f+.026f)/2.5f)<1e-6,"Depth crop coordinates and Gaussian interior");
        const float w=std::exp(-.5f),edge=(.04f+.016f+2*w/(1+2*w)*.011f)/2.5f;
        Check(std::abs(image[0]-edge)<1e-6,"Gaussian reflect padding at cropped border");
        obs.depth.fill(std::numeric_limits<float>::infinity());
        for(float v:policy.Preprocess(obs.depth)) Check(v==1,"No-hit depth must saturate far");
        for(float bad:{-1.f,std::numeric_limits<float>::quiet_NaN()}) {
            obs.depth[0]=bad;bool rejected=false;
            try {policy.Preprocess(obs.depth);} catch(const std::runtime_error&) {rejected=true;}
            Check(rejected,"Invalid camera input rejected");
        }
        obs.depth.fill(1.2f);obs.omega.fill(0);policy.Reset();
        const auto initial=policy.Observe(obs,{0,0,0});
        for(int i=0;i<5;++i)
            Check(std::equal(initial.current.begin(),initial.current.end(),initial.history.begin()+96*i),"Reset must fill history with first measurement");
        const auto target=policy.Infer(initial);
        // When exported together, the full graph must agree with encoder + actor.
        const auto unified_path=std::filesystem::path(argv[1])/"exported/policy.onnx";
        if(std::filesystem::exists(unified_path)) {
            Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"ca_pe_export_equivalence");
            Ort::SessionOptions options;options.SetIntraOpNumThreads(1);
            Ort::Session unified(env,unified_path.c_str(),options);
            Check(unified.GetInputCount()==3 && unified.GetOutputCount()==1,"Unified export interface");
            auto prepared=initial;
            auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
            auto tensor=[&](std::vector<float>& values,const std::vector<int64_t>& shape) {
                return Ort::Value::CreateTensor<float>(memory,values.data(),values.size(),shape.data(),shape.size());
            };
            std::array<Ort::Value,3> tensors{tensor(prepared.current,{1,96}),tensor(prepared.history,{1,480}),tensor(prepared.depth,{1,8192})};
            const char* names[]={"actor_obs","obs_history","depth_image"};const char* output="actions";
            auto result=unified.Run(Ort::RunOptions{nullptr},names,tensors.data(),3,&output,1);
            Check(result[0].GetTensorTypeAndShapeInfo().GetShape()==std::vector<int64_t>{1,29},"Unified export action shape");
            const auto* actions=result[0].GetTensorData<float>();
            const float clip=YAML::LoadFile(argv[2])["action_clip"].as<float>();
            for(int i=0;i<29;++i) {
                const float expected=policy.sign[i]*(policy.neutral[i]+policy.scale[i]*std::clamp(actions[i],-clip,clip));
                Check(std::isfinite(expected) && std::abs(expected-target[policy.ids[i]])<2e-4,"Unified and split ONNX exports must agree");
            }
            std::cout<<"Unified policy.onnx agrees with depth encoder + actor\n";
        }
        const auto feedback=policy.Observe(obs,{0,0,0});
        for(int i=0;i<29;++i) {
            const float action=(policy.sign[i]*target[policy.ids[i]]-policy.neutral[i])/policy.scale[i];
            Check(std::abs(action-feedback.current[67+i])<1e-5,"Scaled action -> DDS -> feedback consistency");
            Check(feedback.history[4*96+67+i]==(history_delay?0:feedback.current[67+i]),"History action must match the training snapshot timing");
        }
        policy.Infer(feedback);
        const auto second_feedback=policy.Observe(obs,{0,0,0});
        for(int i=0;i<29;++i)
            Check(second_feedback.history[4*96+67+i]==(history_delay?feedback.current[67+i]:second_feedback.current[67+i]),"History action must advance on the second inference");
        policy.Reset();const auto repeated=policy.Step(obs,{0,0,0});
        for(int i=0;i<29;++i) Check(std::abs(target[i]-repeated[i])<1e-6,"Deterministic dual-network inference after reset");
        auto changed_history=initial;
        for(int i=0;i<5;++i) changed_history.history[i*96+9]+=.2f;
        const auto history_target=policy.Infer(changed_history);
        float history_delta=0;
        for(int i=0;i<29;++i) history_delta+=std::abs(history_target[i]-target[i]);
        Check(history_delta>1e-4,"VAE history must independently influence actor targets");
        // Verify that both independent inputs influence real model outputs.
        policy.Reset();obs.depth.fill(.25f);const auto near=policy.Step(obs,{0,0,0});
        policy.Reset();obs.depth.fill(2.5f);const auto far=policy.Step(obs,{0,0,0});
        float visual_delta=0;for(int i=0;i<29;++i) visual_delta+=std::abs(near[i]-far[i]);
        Check(visual_delta>1e-4,"Real depth tokens must influence actor targets");
        policy.Reset();const auto commanded=policy.Step(obs,{.5f,.1f,.2f});
        float command_delta=0;for(int i=0;i<29;++i) command_delta+=std::abs(commanded[i]-far[i]);
        Check(command_delta>1e-4,"Velocity commands must influence actor targets");
        auto invalid=initial;invalid.depth[0]=std::numeric_limits<float>::quiet_NaN();bool rejected=false;
        try {policy.Infer(invalid);} catch(const std::runtime_error&) {rejected=true;}
        Check(rejected,"Prepared NaN must fail before inference");
        policy.Reset();const auto cleared=policy.Observe(obs,{0,0,0});
        for(int i=0;i<29;++i) Check(cleared.current[67+i]==0,"Reset must clear previous action");
        std::cout<<"CA PE AMP VAE: mapping, crop/blur, history, real dual-network visual/command response, feedback and reset passed\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
