#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <unitree_mujoco/motor_control.h>
#include "elastic_band.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>

void Check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
struct Command {
    double position=0, velocity=0, stiffness=0, damping=0, torque=0;
    double q() const {return position;} double dq() const {return velocity;}
    double kp() const {return stiffness;} double kd() const {return damping;}
    double tau() const {return torque;}
};

int main(int argc, char** argv) {
    try {
        Check(argc==2, "Expected repository root");
        const std::string root=argv[1]; char error[1024]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(
            (root+"/unitree_robots/g1/scene_29dof_flat.xml").c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
        Check(bool(model),error);
        auto* m=model.get();
        std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(m),mj_deleteData);
        auto* d=data.get(); mj_forward(m,d);
        const auto cfg=YAML::LoadFile(root+"/example/exported/g1_locomotion/g1_locomotion.yaml");
        unitree_mujoco::MotorControl control(m);
        std::array<Command,29> commands{};
        std::array<double,29> target{};
        for(int i=0;i<29;++i) {
            target[cfg["joint_ids_map"][i].as<int>()]=cfg["default_joint_pos"][i].as<double>();
            commands[i].stiffness=cfg["stiffness"][i].as<double>();
            commands[i].damping=cfg["damping"][i].as<double>();
        }
        // Fresh generalized state must win over stale sensor feedback.
        const int joint=m->actuator_trnid[0];
        d->qpos[m->jnt_qposadr[joint]]=.1; d->qvel[m->jnt_dofadr[joint]]=.2;
        control.Apply(d,commands);
        Check(std::abs(d->ctrl[0]-(-.1*commands[0].stiffness-.2*commands[0].damping))<1e-9,
              "PD used stale sensors instead of current joint state");

        // Only the waist moves: the pelvis is stationary but the force point is not.
        mj_resetData(m,d);
        const int waist=mj_name2id(m,mjOBJ_JOINT,"waist_pitch_joint");
        d->qvel[m->jnt_dofadr[waist]]=1.; mj_forward(m,d);
        ElasticBand band;
        const int body=ElasticBandBody(m); const auto* p=d->xipos+3*body;
        band.point_={p[0]+1,p[1],p[2]}; band.length_=1.;
        band.Apply(m,d);
        mjtNum velocity[6]; mj_objectVelocity(m,d,mjOBJ_BODY,body,velocity,0);
        Check(std::abs(velocity[3])>.1,"Waist test failed to move attachment COM");
        Check(std::abs(band.f_[0]+band.damping_*velocity[3])<1e-9,
              "Band damping did not follow attached COM velocity");
        Check(band.f_[0]*velocity[3]<0,"Band damping injected energy");
        band.point_={p[0],p[1],p[2]};band.Apply(m,d);
        Check(mju_norm3(band.f_.data())==0,"Coincident band anchor produced invalid force");
        band.enable_=false;d->xfrc_applied[6*body]=123;band.Apply(m,d);
        Check(d->xfrc_applied[6*body]==0,"Band release retained external force");

        // Real suspended G1: emulate a viewer catching up eight steps at a time.
        // PD must refresh inside the batch, even if no DDS thread runs between steps.
        mj_resetData(m,d);band=ElasticBand();double peak=0, sum=0;int count=0;
        for(int batch=0;batch<500;++batch) for(int substep=0;substep<8;++substep) {
            for(int i=0;i<29;++i)
                commands[i].position=target[i]*std::min(1.,d->time/3.);
            mj_forward(m,d);band.Apply(m,d);control.Apply(d,commands);mj_step(m,d);
            if(d->time>4.) for(int i=6;i<m->nv;++i) {
                Check(std::isfinite(d->qvel[i]),"Nonfinite suspended joint velocity");
                peak=std::max(peak,std::abs(d->qvel[i]));sum+=d->qvel[i]*d->qvel[i];++count;
            }
        }
        double max_error=0;
        for(int i=0;i<29;++i) max_error=std::max(max_error,
            std::abs(d->qpos[m->jnt_qposadr[m->actuator_trnid[2*i]]]-target[i]));
        std::cout<<"Suspended G1: peak joint speed="<<peak<<", RMS="<<std::sqrt(sum/count)
                 <<", final max pose error="<<max_error<<"\n";
        Check(peak<.2 && max_error<.08,"Suspended G1 failed to settle with per-step PD");
        for(int i=0;i<mjNWARNING;++i) Check(!d->warning[i].number,"MuJoCo reported an instability warning");
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
