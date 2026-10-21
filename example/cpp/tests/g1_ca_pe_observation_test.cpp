// Compare production UDP proprioception with the MuJoCo pelvis IMU and the
// Euler -> quaternion -> inverse-gravity path used by CtrlZ.
#include "parkour_observation.h"
#include "g1_ca_pe_amp_vae_policy.h"
#include <iostream>
#include <memory>
#ifdef G1_HAVE_CTRLZ_SO3
#include "Utils/so3.hpp"
#endif

void Require(bool ok,const char* message) {
    if(!ok) throw std::runtime_error(message);
}

int main(int argc,char** argv) {
    try {
        Require(argc==3,"Expected repository root and task directory");
        const std::string root=argv[1],task=argv[2],config=task+"/g1_ca_pe_amp_vae_cfg.json";
        const auto cfg=YAML::LoadFile(config);
        char error[2048]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(
            (root+"/unitree_robots/g1/scene_29dof_flat.xml").c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
        Require(bool(model),error);
        model->opt.timestep=cfg["physics_dt"].as<double>();
        std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
        const int fd=socket(AF_INET,SOCK_DGRAM,0);
        Require(fd>=0,"Cannot create local observation receiver");
        struct Socket {int fd;~Socket(){close(fd);}} guard{fd};
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        Require(bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"Cannot bind local receiver");
        socklen_t length=sizeof(address);
        Require(getsockname(fd,reinterpret_cast<sockaddr*>(&address),&length)==0,"Cannot get receiver port");
        timeval timeout{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        unitree_mujoco::ParkourObservationPublisher publisher(model.get(),data.get(),config,ntohs(address.sin_port));
        g1_ca_pe_amp_vae::Policy policy(task,config);
        const int site=mj_name2id(model.get(),mjOBJ_SITE,"imu");
        const int gyro=mj_name2id(model.get(),mjOBJ_SENSOR,"imu_gyro");
        const int pelvis=mj_name2id(model.get(),mjOBJ_BODY,"pelvis");
        const int torso=mj_name2id(model.get(),mjOBJ_BODY,"torso_link");
        Require(site>=0 && gyro>=0 && model->site_bodyid[site]==pelvis,"Expected pelvis IMU site");
        double max_omega=0,max_gravity=0,max_frame=0,torso_difference=0,max_legacy_error=0;
        double max_com_velocity_error=0,max_origin_velocity_difference=0;
        int count=0;
        for(double roll:{-.6,0.,.6}) for(double pitch:{-.5,0.,.5})
        for(double yaw:{-3.,-1.57,0.,1.57,3.}) {
            data->time=count*.02;data->qpos[2]=.8;
            const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
            const double cy=std::cos(yaw/2),sy=std::sin(yaw/2);
            data->qpos[3]=cr*cp*cy+sr*sp*sy;data->qpos[4]=sr*cp*cy-cr*sp*sy;
            data->qpos[5]=cr*sp*cy+sr*cp*sy;data->qpos[6]=cr*cp*sy-sr*sp*cy;
            for(int i=0;i<model->nv;++i) data->qvel[i]=.2*std::sin(i+.3*count);
            for(int i=0;i<29;++i) {
                const int joint=mj_name2id(model.get(),mjOBJ_JOINT,cfg["joint_names"][i].as<std::string>().c_str());
                data->qpos[model->jnt_qposadr[joint]]=policy.sign[i]*(policy.neutral[i]+.15*std::sin(i+.2*count));
            }
            // Move the waist independently so accidentally reading the torso is detectable.
            const int waist=mj_name2id(model.get(),mjOBJ_JOINT,"waist_roll_joint");
            data->qpos[model->jnt_qposadr[waist]]=.25;
            mj_forward(model.get(),data.get());publisher.Update();
            std::array<unsigned char,9508> bytes{};
            Require(recv(fd,bytes.data(),bytes.size(),0)==static_cast<ssize_t>(bytes.size()),"Missing UDP sample");
            const auto obs=g1_visual::Decode(bytes.data(),bytes.size(),policy.signature);
            const auto* rotation=data->site_xmat+9*site;
            // MujocoImu::Rot2RPY reads the site matrix in this exact order.
            const float r=std::atan2(rotation[7],rotation[8]);
            const float p=std::atan2(-rotation[6],std::hypot(rotation[7],rotation[8]));
            const float y=std::atan2(rotation[3],rotation[0]);
            std::array<float,3> gravity{};
#ifdef G1_HAVE_CTRLZ_SO3
            const auto quat=z::math::quat_from_euler_xyz(z::math::Vector<float,3>{r,p,y});
            const auto projected=z::math::quat_rotate_inverse(quat,z::math::Vector<float,3>{0,0,-1});
            for(int i=0;i<3;++i) gravity[i]=projected[i];
#else
            (void)y;
            gravity={std::sin(p),-std::cos(p)*std::sin(r),-std::cos(p)*std::cos(r)};
#endif
            const std::array<float,3> command{.5f,-.2f,.3f};
            const auto prepared=policy.Observe(obs,command);
            // Commands are already expressed in the training convention. A change
            // in measured roll/pitch/yaw must not rotate or rescale them again.
            for(int i=0;i<3;++i)
                Require(prepared.current[6+i]==command[i],"Velocity command was rotated, reordered or rescaled");
            mjtNum legacy[6];mj_objectVelocity(model.get(),data.get(),mjOBJ_BODY,pelvis,legacy,1);
            mjtNum com_velocity[6],link_velocity[6],jacobian_velocity[3];
            mj_objectVelocity(model.get(),data.get(),mjOBJ_BODY,pelvis,com_velocity,0);
            mj_objectVelocity(model.get(),data.get(),mjOBJ_XBODY,pelvis,link_velocity,0);
            std::vector<mjtNum> jacobian(3*model->nv);
            mj_jacBodyCom(model.get(),data.get(),jacobian.data(),nullptr,pelvis);
            mju_mulMatVec(jacobian_velocity,jacobian.data(),data->qvel,3,model->nv);
            for(int i=0;i<3;++i) {
                max_com_velocity_error=std::max(max_com_velocity_error,std::abs(com_velocity[3+i]-jacobian_velocity[i]));
                max_origin_velocity_difference=std::max(max_origin_velocity_difference,std::abs(com_velocity[3+i]-link_velocity[3+i]));
                const double omega=data->sensordata[model->sensor_adr[gyro]+i];
                max_legacy_error=std::max(max_legacy_error,std::abs(legacy[i]-omega));
                max_omega=std::max(max_omega,std::abs(double(obs.omega[i])-omega));
                max_gravity=std::max(max_gravity,std::abs(double(obs.gravity[i])-gravity[i]));
                max_frame=std::max(max_frame,std::abs(double(prepared.current[i])-omega*.2));
                max_frame=std::max(max_frame,std::abs(double(prepared.current[3+i])-gravity[i]));
                torso_difference=std::max(torso_difference,std::abs(double(obs.gravity[i])+data->xmat[9*torso+6+i]));
            }
            ++count;
        }
        Require(max_omega<1e-6,"UDP omega disagrees with pelvis IMU gyro");
        Require(max_gravity<1e-6,"Projected gravity disagrees with Euler inverse rotation");
        Require(max_frame<1e-6,"Actor IMU observation scaling mismatch");
        Require(torso_difference>.1,"Test must distinguish pelvis from torso orientation");
        Require(max_com_velocity_error<1e-8,"COM velocity disagrees with the body COM Jacobian");
        Require(max_origin_velocity_difference>1e-3,"Test must distinguish link-origin from COM velocity");
        // Verify both camera alignments against an analytic horizontal plane.
        // Isolate the floor from robot occlusion, retaining actual UDP sampling.
        for(int g=0;g<model->ngeom;++g) if(model->geom_bodyid[g]>0) model->geom_group[g]=1;
        double camera_error=0;
        for(const char* alignment:{"body","yaw"}) {
            auto camera_cfg=YAML::Clone(cfg);
            camera_cfg["camera"]["ray_alignment"]=alignment;
            camera_cfg["camera"]["include_robot"]=false;
            camera_cfg["camera"]["position"]=std::vector<double>{0,0,0};
            char filename[]="/tmp/unitree-camera-config-XXXXXX";
            const int temporary=mkstemp(filename);Require(temporary>=0,"Cannot create temporary camera config");
            struct ConfigFile {const char* name;int fd;~ConfigFile(){close(fd);unlink(name);}} config_file{filename,temporary};
            const std::string serialized=YAML::Dump(camera_cfg);
            Require(write(temporary,serialized.data(),serialized.size())==static_cast<ssize_t>(serialized.size()),
                    "Cannot write temporary camera config");
            unitree_mujoco::ParkourObservationPublisher camera(model.get(),data.get(),filename,ntohs(address.sin_port));
            std::array<double,2> ratios{};int pose=0;
            for(double angle:{0.,.2}) {
                mj_resetData(model.get(),data.get());data->time=pose*.02;data->qpos[2]=1.2;
                data->qpos[3]=std::cos(angle/2);data->qpos[5]=std::sin(angle/2);
                mj_forward(model.get(),data.get());camera.Update();
                std::array<unsigned char,9508> bytes{};
                Require(recv(fd,bytes.data(),bytes.size(),0)==static_cast<ssize_t>(bytes.size()),"Missing camera sample");
                const auto obs=g1_visual::Decode(bytes.data(),bytes.size(),policy.signature);
                mjtNum ray[3]={1,-.5/32*cfg["camera"]["horizontal_aperture"].as<double>()/2,
                                -.5/18*cfg["camera"]["vertical_aperture"].as<double>()/2};
                mju_normalize3(ray);const double cosine=ray[0];
                mjtNum quat[4],matrix[9],local[3],direction[3];
                for(int i=0;i<4;++i) quat[i]=cfg["camera"]["quaternion"][i].as<double>();
                mju_normalize4(quat);mju_quat2Mat(matrix,quat);mju_mulMatVec3(local,matrix,ray);
                if(std::string(alignment)=="yaw") mju_copy3(direction,local);
                else mju_mulMatVec3(direction,data->xmat+9*torso,local);
                const double height=data->xpos[3*torso+2],expected=-height/direction[2]*cosine;
                camera_error=std::max(camera_error,std::abs(obs.depth[18*64+32]-expected));
                ratios[pose++]=obs.depth[18*64+32]/height;
            }
            if(std::string(alignment)=="yaw") Require(std::abs(ratios[1]-ratios[0])<1e-5,"Yaw camera incorrectly follows pitch");
            else Require(std::abs(ratios[1]-ratios[0])>.1,"Body camera did not follow torso pitch");
        }
        Require(camera_error<1e-5,"Camera depth disagrees with analytic plane distance");
        std::cout<<"PASS "<<count<<" mixed roll/pitch/yaw poses with independent waist motion; max gyro error="
                 <<max_omega<<", gravity error="<<max_gravity<<", actor IMU error="<<max_frame
                 <<", pelvis/torso gravity difference="<<torso_difference
                 <<", old inertial-axis gyro error="<<max_legacy_error;
        std::cout<<", body/yaw camera plane error="<<camera_error;
        std::cout<<", COM velocity Jacobian error="<<max_com_velocity_error
                 <<", link-origin/COM velocity difference="<<max_origin_velocity_difference
                 <<"; command order/scale invariant under all poses";
#ifdef G1_HAVE_CTRLZ_SO3
        std::cout<<"; using actual CtrlZ SO3 templates";
#endif
        std::cout<<'\n';return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
