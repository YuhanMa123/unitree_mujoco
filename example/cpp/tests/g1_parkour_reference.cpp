// Numerical cross-check of the real UDP publisher and deployed C++ policy.
#include "parkour_observation.h"
#include "g1_parkour_policy.h"
#include <fstream>
#include <memory>

int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("Expected repository and temporary output path");
        const std::string root=argv[1],config=root+"/example/exported/g1_parkour/g1_torsobase_cfg.json";
        char error[2048]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(
            (root+"/unitree_robots/g1/scene_29dof_flat.xml").c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
        if(!model) throw std::runtime_error(error);
        std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
        const int fd=socket(AF_INET,SOCK_DGRAM,0);
        if(fd<0) throw std::runtime_error("Cannot create reference receiver");
        struct Socket {int fd;~Socket(){close(fd);}} socket_guard{fd};
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))) throw std::runtime_error("Cannot bind reference receiver");
        socklen_t length=sizeof(address);getsockname(fd,reinterpret_cast<sockaddr*>(&address),&length);
        timeval timeout{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        unitree_mujoco::ParkourObservationPublisher publisher(model.get(),data.get(),config,ntohs(address.sin_port));
        g1_parkour::Policy policy(root+"/example/exported/g1_parkour",config);
        const auto cfg=YAML::LoadFile(config);
        std::ofstream output(argv[2],std::ios::binary);
        auto write=[&](const auto& values) {output.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(values[0]));};
        for(int t=0;t<40;++t) {
            data->time=t*.02;
            data->qpos[0]=t*.025;data->qpos[2]=.78;
            mjtNum axis[3]={0,0,1};mju_axisAngle2Quat(data->qpos+3,axis,t*.004);
            for(int i=0;i<29;++i) {
                const int joint=mj_name2id(model.get(),mjOBJ_JOINT,cfg["joint_names"][i].as<std::string>().c_str());
                data->qpos[model->jnt_qposadr[joint]]=policy.sign[i]*(policy.neutral[i]+.01*std::sin(t*.1+i));
            }
            for(int i=0;i<model->nv;++i) data->qvel[i]=.02*std::cos(t*.1+i);
            mj_forward(model.get(),data.get());publisher.Update();
            std::array<unsigned char,9508> bytes{};
            if(recv(fd,bytes.data(),bytes.size(),0)!=bytes.size()) throw std::runtime_error("UDP reference packet missing");
            const auto obs=g1_parkour::Decode(bytes.data(),bytes.size(),policy.signature);
            auto inputs=policy.Observe(obs,{.3,0,.1});
            const auto target=policy.Infer(inputs);
            write(std::vector<double>(data->qpos,data->qpos+model->nq));
            write(std::vector<double>(data->qvel,data->qvel+model->nv));
            write(bytes);write(inputs.first);write(inputs.second);write(target);
        }
        if(!output) throw std::runtime_error("Reference output write failed");
        return 0;
    } catch(const std::exception& e) {std::fprintf(stderr,"%s\n",e.what());return 1;}
}
