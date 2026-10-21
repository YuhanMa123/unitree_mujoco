#include "g1_parkour_policy.h"
#include <iostream>
#include <limits>

void Check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
int main(int argc,char** argv) {
    try {
        Check(argc==3,"Expected models and config");
        g1_parkour::Policy policy(argv[1],argv[2]);
        g1_parkour::Observation obs;obs.q=policy.neutral;obs.gravity={0,0,-1};
        std::pair<std::vector<float>,std::vector<float>> inputs;
        for(int t=0;t<40;++t) {
            obs.omega.fill(t*.01f);obs.depth.fill(t*.01f);
            inputs=policy.Observe(obs,{0,0,0});
        }
        for(int i=0;i<8;++i) {
            Check(std::abs(inputs.first[i*3]-(32+i)*.01f*.25f)<1e-6,"Proprio history order");
            Check(std::abs(inputs.second[i*18*32]-(4+5*i)*.01f/2.5f)<1e-6,"Depth stride/order");
        }
        policy.Reset();obs.depth.fill(2.5f);obs.omega.fill(0);
        auto target=policy.Step(obs,{0,0,0});
        auto after=policy.Observe(obs,{0,0,0});
        for(int i=0;i<29;++i) {
            const auto raw=(policy.sign[i]*target[policy.ids[i]]-policy.neutral[i])/policy.scale[i];
            Check(std::abs(after.first[768-29+i]-raw)<1e-5,"Last action feedback");
        }
        obs.depth.fill(std::numeric_limits<float>::infinity());
        for(float v:policy.Preprocess(obs.depth)) Check(v==1,"Infinite no-hit preprocessing");
        obs.depth[0]=-1;bool rejected=false;
        try {policy.Preprocess(obs.depth);} catch(const std::runtime_error&) {rejected=true;}
        Check(rejected,"Negative depth must fail");
        std::array<unsigned char,9508> bytes{};std::memcpy(bytes.data(),"G1PKR001",8);
        uint32_t count=2369;std::memcpy(bytes.data()+24,&policy.signature,4);std::memcpy(bytes.data()+28,&count,4);
        auto decoded=g1_parkour::Decode(bytes.data(),bytes.size(),policy.signature);
        Check(decoded.sequence==0,"Packet decoding");
        rejected=false;try {g1_parkour::Decode(bytes.data(),bytes.size()-1,policy.signature);} catch(...) {rejected=true;}
        Check(rejected,"Truncated packet must fail");
        g1_parkour::Observation previous,current;
        previous.sequence=10;previous.time=1.;previous.q.fill(0);previous.gravity={0,0,-1};previous.depth.fill(1);
        current=previous;current.sequence=13;current.time=1.06;current.q.fill(.3f);current.depth.fill(2);
        auto recovered=g1_parkour::RecoverObservationGap(previous,current,.02);
        Check(recovered.size()==3,"Recover two lost samples");
        Check(std::abs(recovered[0].time-1.02)<1e-8 && std::abs(recovered[0].q[0]-.1f)<1e-6,
              "Recovered proprioception must preserve 20ms timing");
        Check(recovered[0].depth[0]==1 && recovered[1].depth[0]==1 && recovered[2].depth[0]==2,
              "Missing camera frames must hold previous image");
        current.sequence=11; // Publisher skipped sampling rather than network packet loss.
        Check(g1_parkour::RecoverObservationGap(previous,current,.02).size()==3,"Recover sender timing gap");
        current.time=1.14;rejected=false;
        try {g1_parkour::RecoverObservationGap(previous,current,.02);} catch(...) {rejected=true;}
        Check(rejected,"Large gaps must retain protection");
        current.time=1.025;rejected=false;
        try {g1_parkour::RecoverObservationGap(previous,current,.02);} catch(...) {rejected=true;}
        Check(rejected,"Non-periodic timestamps must fail");
        std::cout<<"Parkour history, depth, feedback and protocol checks passed\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
