#pragma once
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace unitree_mujoco {

// Called on the physics thread under Simulate::mtx, immediately before mj_step.
// Matches simulate_python/parkour_observation.py and g1_parkour::Decode.
class ParkourObservationPublisher {
    mjModel* model_;
    mjData* data_;
    int fd_ = -1, torso_ = -1, state_body_ = -1;
    bool include_robot_ = false;
    bool camera_yaw_only_ = false;
    double min_distance_ = 0;
    std::vector<int> camera_geoms_;
    std::array<mjtNum,3> camera_offset_{.0487988662332928,.01,.4378029937970051};
    sockaddr_in address_{};
    std::array<int,29> qadr_{}, vadr_{}, motors_{};
    std::array<double,29> signs_{}, limits_{}, initial_q_{};
    std::array<double,29> idle_damping_{}, idle_friction_{};
    std::array<mjtNum,36*64*3> rays_{}, directions_{};
    std::array<mjtNum,36*64> cosines_{}, distances_{};
    std::array<int,36*64> hits_{};
    std::uint32_t signature_ = 2166136261u;
    std::uint64_t sequence_ = 0;
    double period_ = .02, next_time_ = -1, last_time_ = -1;

public:
    ParkourObservationPublisher(mjModel* model, mjData* data,
                                const std::string& config, int port)
        : model_(model), data_(data) {
        if (port < 1024 || port > 65535 || model->nu != 29)
            throw std::runtime_error("Parkour requires G1 29DOF and UDP port 1024..65535");
        const auto cfg = YAML::LoadFile(config);
        const auto names = cfg["joint_names"].as<std::vector<std::string>>();
        period_ = cfg["step_dt"].as<double>();
        const double physics_dt = cfg["physics_dt"].as<double>();
        if (names.size()!=29 || std::abs(period_-.02)>1e-8 ||
            !std::isfinite(period_) || !std::isfinite(physics_dt) ||
            std::abs(model->opt.timestep-physics_dt)>1e-8)
            throw std::runtime_error("Parkour expects 20ms observations and matching physics_dt");
        for (const char* field : {"joint_sign", "effort_limit", "armature", "default_joint_pos"})
            if (!cfg[field].IsSequence() || cfg[field].size()!=29)
                throw std::runtime_error("Expected 29 joint signs and effort limits");
        torso_ = mj_name2id(model, mjOBJ_BODY, "torso_link");
        if (torso_<0) throw std::runtime_error("Missing torso_link for parkour observations");
        const std::string state_name=cfg["state_body"] ? cfg["state_body"].as<std::string>() : "torso_link";
        state_body_=mj_name2id(model,mjOBJ_BODY,state_name.c_str());
        if(state_body_<0) throw std::runtime_error("Missing observation state body: "+state_name);
        std::array<bool,29> seen{};
        for (int i=0;i<29;++i) {
            const int joint = mj_name2id(model,mjOBJ_JOINT,names[i].c_str());
            if (joint<0 || model->jnt_type[joint]!=mjJNT_HINGE)
                throw std::runtime_error("Missing parkour hinge joint: "+names[i]);
            qadr_[i]=model->jnt_qposadr[joint]; vadr_[i]=model->jnt_dofadr[joint];
            int motor=-1;
            for(int a=0;a<model->nu;++a)
                if(model->actuator_trntype[a]==mjTRN_JOINT && model->actuator_trnid[2*a]==joint) {
                    if(motor>=0) throw std::runtime_error("Multiple motors for parkour joint");
                    motor=a;
                }
            if(motor<0 || seen[motor]) throw std::runtime_error("Invalid parkour motor mapping");
            seen[motor]=true; motors_[i]=motor;
            for(int k=0;k<6;++k)
                if(model->actuator_gear[6*motor+k]!=(k==0?1:0))
                    throw std::runtime_error("Parkour requires direct unit-gear motors");
            signs_[i]=cfg["joint_sign"][i].as<double>();
            initial_q_[i]=signs_[i]*cfg["default_joint_pos"][i].as<double>();
            const double armature=cfg["armature"][i].as<double>();
            if(!std::isfinite(initial_q_[i]) || !std::isfinite(armature) || armature<0)
                throw std::runtime_error("Invalid parkour reset pose or armature");
            // Keep unpowered motor losses until PD supplies the training damping.
            idle_damping_[i]=model->dof_damping[vadr_[i]];
            idle_friction_[i]=model->dof_frictionloss[vadr_[i]];
            // Match training inertia; passive losses switch with command activity.
            model->dof_armature[vadr_[i]]=armature;
            const double effort=cfg["effort_limit"][i].as<double>();
            if((signs_[i]!=1 && signs_[i]!=-1) || !std::isfinite(effort) || effort<=0)
                throw std::runtime_error("Invalid parkour sign or effort limit");
            limits_[i]=effort;
            if(model->actuator_ctrllimited[motor])
                limits_[i]=std::min(effort,std::max(std::abs(model->actuator_ctrlrange[2*motor]),
                                                   std::abs(model->actuator_ctrlrange[2*motor+1])));
            for(unsigned char c:names[i]+":"+std::to_string(int(signs_[i]))+"\n")
                signature_=(signature_^c)*16777619u;
        }
        // Training camera: x forward, y left, z up, with fixed torso extrinsics.
        mjtNum quaternion[4]={.9135367613482678,.004363309284746571,.4067366430758002,0};
        double horizontal=2*std::tan(89.51*3.14159265358979323846/360),
               vertical=2*std::tan(58.29*3.14159265358979323846/360), focal=1;
        if(const auto camera=cfg["camera"]) {
            const auto alignment=camera["ray_alignment"].as<std::string>("body");
            if(alignment!="body" && alignment!="yaw")
                throw std::runtime_error("Camera ray_alignment must be body or yaw");
            camera_yaw_only_=alignment=="yaw";
            torso_=mj_name2id(model,mjOBJ_BODY,camera["body"].as<std::string>().c_str());
            if(torso_<0 || camera["position"].size()!=3 || camera["quaternion"].size()!=4)
                throw std::runtime_error("Invalid visual camera extrinsics");
            for(int i=0;i<3;++i) camera_offset_[i]=camera["position"][i].as<double>();
            for(int i=0;i<4;++i) quaternion[i]=camera["quaternion"][i].as<double>();
            horizontal=camera["horizontal_aperture"].as<double>();
            vertical=camera["vertical_aperture"].as<double>();focal=camera["focal_length"].as<double>();
            include_robot_=camera["include_robot"].as<bool>();
            min_distance_=camera["min_distance"].as<double>(0);
            if(!std::isfinite(min_distance_) || min_distance_<0)
                throw std::runtime_error("Invalid camera minimum distance");
            // Training sees ground and robot visual meshes, not collision proxies.
            for(int g=0;g<model->ngeom;++g)
                if(model->geom_rgba[4*g+3]>0 &&
                   (model->geom_group[g]==1 || (model->geom_group[g]==0 && model->body_weldid[model->geom_bodyid[g]]==0)))
                    camera_geoms_.push_back(g);
            double norm=0;
            for(double v:quaternion) {if(!std::isfinite(v)) throw std::runtime_error("Non-finite camera quaternion");norm+=v*v;}
            for(double v:camera_offset_) if(!std::isfinite(v)) throw std::runtime_error("Non-finite camera position");
            if(norm<1e-12 || !std::isfinite(horizontal) || !std::isfinite(vertical) || !std::isfinite(focal) ||
               horizontal<=0 || vertical<=0 || focal<=0) throw std::runtime_error("Invalid camera intrinsics");
        }
        mju_normalize4(quaternion);
        mjtNum rotation[9]; mju_quat2Mat(rotation,quaternion);
        for(int y=0;y<36;++y) for(int x=0;x<64;++x) {
            const int i=y*64+x;
            mjtNum ray[3]={1,-(x+.5-32)/32*(horizontal/(2*focal)),
                            -(y+.5-18)/18*(vertical/(2*focal))};
            mju_normalize3(ray);cosines_[i]=ray[0];
            mju_mulMatVec3(rays_.data()+3*i,rotation,ray);
        }
        const std::uint16_t endian=1;
        if(*reinterpret_cast<const unsigned char*>(&endian)!=1)
            throw std::runtime_error("Parkour UDP protocol requires little endian");
        fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
        if(fd_<0) throw std::runtime_error("Cannot create parkour UDP socket");
        address_.sin_family=AF_INET;address_.sin_port=htons(port);
        address_.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    }
    ~ParkourObservationPublisher() {if(fd_>=0) close(fd_);}
    ParkourObservationPublisher(const ParkourObservationPublisher&)=delete;
    ParkourObservationPublisher& operator=(const ParkourObservationPublisher&)=delete;

    // Call before mj_forward. No position hold or motor torque is added while idle.
    void SetMotorControlActive(bool active) {
        for(int i=0;i<29;++i) {
            model_->dof_damping[vadr_[i]]=active?0:idle_damping_[i];
            model_->dof_frictionloss[vadr_[i]]=active?0:idle_friction_[i];
        }
    }

    // Called after mj_resetData, before publishing the first post-reset observation.
    // Do not modify model->qpos0: hinge reference angles affect kinematics.
    void ResetPose() {
        for(int i=0;i<29;++i) data_->qpos[qadr_[i]]=initial_q_[i];
        mj_forward(model_,data_);
    }

    void Update() {
        const double now=data_->time;
        if(now<last_time_) next_time_=now;
        last_time_=now;
        if(now+1e-9<next_time_) return;
        next_time_=now+period_;
        std::array<float,2369> values{};
        for(int i=0;i<29;++i) {
            values[i]=signs_[i]*data_->qpos[qadr_[i]];
            values[29+i]=signs_[i]*data_->qvel[vadr_[i]];
        }
        // mjOBJ_BODY uses the inertial principal axes (ximat), which can differ
        // from the link/IMU axes. Gravity below uses xmat, so omega must too.
        mjtNum velocity[6];mj_objectVelocity(model_,data_,mjOBJ_XBODY,state_body_,velocity,1);
        const auto* state_rotation=data_->xmat+9*state_body_;
        const auto* rotation=data_->xmat+9*torso_;
        for(int i=0;i<3;++i) {values[58+i]=velocity[i];values[61+i]=-state_rotation[6+i];}
        values[64]=data_->xpos[3*state_body_+2];
        mjtNum origin[3];mju_mulMatVec3(origin,rotation,camera_offset_.data());
        mju_addTo3(origin,data_->xpos+3*torso_);
        mjtNum yaw_rotation[9];
        const mjtNum* ray_rotation=rotation;
        if(camera_yaw_only_) {
            const double yaw=std::atan2(rotation[3],rotation[0]),c=std::cos(yaw),s=std::sin(yaw);
            const mjtNum matrix[9]={c,-s,0,s,c,0,0,0,1};
            mju_copy(yaw_rotation,matrix,9);ray_rotation=yaw_rotation;
        }
        for(int i=0;i<36*64;++i)
            mju_mulMatVec3(directions_.data()+3*i,ray_rotation,rays_.data()+3*i);
        if(!include_robot_) {
            const mjtByte groups[6]={1,0,0,0,0,0};
            mj_multiRay(model_,data_,origin,directions_.data(),groups,1,torso_,
                        hits_.data(),distances_.data(),36*64,100.);
        } else {
            // Match grouped training rays: ignore each mesh's nearest hit when
            // it is inside min_distance, then consider the other meshes.
            // Filtering the final nearest hit would incorrectly hide the ground.
            for(int i=0;i<36*64;++i) {
                double nearest=100.;int hit=-1;
                const mjtNum* direction=directions_.data()+3*i;
                for(int g:camera_geoms_) {
                    const int type=model_->geom_type[g];
                    const double radius=model_->geom_rbound[g];
                    if(type!=mjGEOM_PLANE && radius>0) {
                        mjtNum relative[3];mju_sub3(relative,data_->geom_xpos+3*g,origin);
                        const double along=mju_dot3(relative,direction);
                        if(along+radius<0 || along-radius>nearest ||
                           mju_dot3(relative,relative)-along*along>radius*radius) continue;
                    }
                    double distance=-1;
                    if(type==mjGEOM_MESH) distance=mj_rayMesh(model_,data_,g,origin,direction);
                    else if(type==mjGEOM_HFIELD) distance=mj_rayHfield(model_,data_,g,origin,direction);
                    else distance=mju_rayGeom(data_->geom_xpos+3*g,data_->geom_xmat+9*g,
                                             model_->geom_size+3*g,origin,direction,type);
                    if(distance>min_distance_ && distance<nearest) {nearest=distance;hit=g;}
                }
                hits_[i]=hit;distances_[i]=hit<0?-1:nearest;
            }
        }
        for(int i=0;i<36*64;++i)
            values[65+i]=distances_[i]<0 ? std::numeric_limits<float>::infinity() : distances_[i]*cosines_[i];
        std::array<unsigned char,32+2369*4> packet{};
        std::memcpy(packet.data(),"G1PKR001",8);
        const std::uint32_t count=values.size();
        std::memcpy(packet.data()+8,&sequence_,8);std::memcpy(packet.data()+16,&now,8);
        std::memcpy(packet.data()+24,&signature_,4);std::memcpy(packet.data()+28,&count,4);
        std::memcpy(packet.data()+32,values.data(),values.size()*4);
        ++sequence_;
        if(sendto(fd_,packet.data(),packet.size(),0,reinterpret_cast<sockaddr*>(&address_),sizeof(address_))<0 &&
            errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)
            throw std::runtime_error("Cannot send parkour observations");
    }
    void LimitTorque() {
        for(int i=0;i<29;++i) {
            auto& torque=data_->ctrl[motors_[i]];
            if(!std::isfinite(torque)) throw std::runtime_error("Non-finite parkour torque");
            torque=std::clamp(torque,-limits_[i],limits_[i]);
        }
    }
};
} // namespace unitree_mujoco
