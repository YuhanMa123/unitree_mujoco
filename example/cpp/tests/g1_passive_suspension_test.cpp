#include "parkour_observation.h"
#include "elastic_band.h"
#include <unitree_mujoco/motor_control.h>
#include <algorithm>
#include <iostream>
#include <memory>

void Check(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
struct NoCommand {
    double q() const {return 0;} double dq() const {return 0;}
    double kp() const {return 0;} double kd() const {return 0;} double tau() const {return 0;}
};
int main(int argc,char** argv) {
    try {
        Check(argc==2,"Expected repository root");const std::string root=argv[1];
        for(const char* task:{"g1_parkour/g1_torsobase_cfg.json",
                              "g1_test/g1_ca_pe_amp_vae_cfg.json",
                              "g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json"}) {
            const std::string path=root+"/example/exported/"+task;const auto cfg=YAML::LoadFile(path);
            char error[1024]{};
            std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(
                (root+"/unitree_robots/g1/scene_29dof_flat.xml").c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
            Check(bool(model),error);auto* m=model.get();m->opt.timestep=cfg["physics_dt"].as<double>();
            std::vector<double> damping(m->dof_damping,m->dof_damping+m->nv);
            std::vector<double> friction(m->dof_frictionloss,m->dof_frictionloss+m->nv);
            std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(m),mj_deleteData);auto* d=data.get();
            unitree_mujoco::ParkourObservationPublisher observations(m,d,path,17999);
            unitree_mujoco::MotorControl control(m);std::array<NoCommand,29> commands{};
            ElasticBand band;double late_peak=0,sum=0;int n=0;
            for(int step=0;step<int(20./m->opt.timestep);++step) {
                const bool active=control.Apply(d,commands);Check(!active,"Zero commands enabled motor control");
                observations.SetMotorControlActive(active);
                mj_forward(m,d);band.Apply(m,d);mj_step(m,d);
                for(int i=0;i<29;++i) Check(d->ctrl[i]==0,"Idle simulation applied motor torque");
                if(d->time>10.) for(int i=6;i<m->nv;++i) {
                    Check(std::isfinite(d->qvel[i]),"Nonfinite idle suspension velocity");
                    late_peak=std::max(late_peak,std::abs(d->qvel[i]));sum+=d->qvel[i]*d->qvel[i];++n;
                }
            }
            Check(late_peak<.05,"Unpowered suspended robot failed to settle");
            for(int i=0;i<29;++i) {
                const int v=m->jnt_dofadr[m->actuator_trnid[2*i]];
                Check(m->dof_damping[v]==damping[v] && m->dof_frictionloss[v]==friction[v],
                      "Idle model lost original passive damping/friction");
            }
            observations.SetMotorControlActive(true);
            for(int i=0;i<29;++i) {
                const int v=m->jnt_dofadr[m->actuator_trnid[2*i]];
                Check(m->dof_damping[v]==0 && m->dof_frictionloss[v]==0,
                      "Active motors retained extra passive losses instead of training dynamics");
            }
            observations.SetMotorControlActive(false);
            for(int i=0;i<29;++i) {
                const int v=m->jnt_dofadr[m->actuator_trnid[2*i]];
                Check(m->dof_damping[v]==damping[v] && m->dof_frictionloss[v]==friction[v],
                      "Disabling motors failed to restore idle losses");
            }
            for(int i=0;i<mjNWARNING;++i) Check(!d->warning[i].number,"Idle suspension instability warning");
            std::cout<<task<<": zero motor torque for 20s; last 10s peak="<<late_peak
                     <<" rad/s, RMS="<<std::sqrt(sum/n)<<" rad/s; active/idle dynamics passed\n";
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
