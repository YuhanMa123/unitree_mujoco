#pragma once
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_visual {
using Joints = std::array<float,29>;
// Shared policy-order joint mapping and actuator parameters for visual controllers.
class JointConfig {
protected:
    template<std::size_t N>
    static std::array<float,N> Read(const YAML::Node& node) {
        if (!node.IsSequence() || node.size()!=N) throw std::runtime_error("Invalid parkour configuration array");
        std::array<float,N> values{};
        for (std::size_t i=0; i<N; ++i) {
            values[i]=node[i].as<float>();
            if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite configuration");
        }
        return values;
    }
public:
    std::array<int,29> ids{}; // policy -> DDS
    Joints neutral{}, kp{}, kd{}, scale{}, sign{}, effort{}; // policy order
    uint32_t signature=2166136261u;
    double dt=.02;

    explicit JointConfig(const std::string& config) {
        const auto cfg=YAML::LoadFile(config);
        neutral=Read<29>(cfg["default_joint_pos"]);
        kp=Read<29>(cfg["kp"]); kd=Read<29>(cfg["kd"]); scale=Read<29>(cfg["action_scale"]);
        sign=Read<29>(cfg["joint_sign"]);
        effort=Read<29>(cfg["effort_limit"]);
        dt=cfg["step_dt"].as<double>();
        if(!std::isfinite(dt) || std::abs(dt-.02)>1e-8)
            throw std::runtime_error("Expected 20ms visual policy observations");
        const std::vector<std::string> dds={
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint",
            "left_ankle_pitch_joint","left_ankle_roll_joint","right_hip_pitch_joint","right_hip_roll_joint",
            "right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint","left_shoulder_pitch_joint",
            "left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint",
            "left_wrist_pitch_joint","left_wrist_yaw_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint",
            "right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"};
        auto names=cfg["joint_names"].as<std::vector<std::string>>();
        if(names.size()!=29) throw std::runtime_error("Expected 29 policy joints");
        std::array<bool,29> seen{};
        for(int i=0;i<29;++i) {
            auto found=std::find(dds.begin(),dds.end(),names[i]);
            if(found==dds.end() || seen[found-dds.begin()]) throw std::runtime_error("Unknown or duplicate joint");
            ids[i]=found-dds.begin(); seen[ids[i]]=true;
            if(sign[i]!=1 && sign[i]!=-1) throw std::runtime_error("joint_sign must be +1 or -1");
            for(unsigned char c:names[i]+":"+std::to_string(static_cast<int>(sign[i]))+"\n") signature=(signature^c)*16777619u;
            if(kp[i]<=0 || kd[i]<0 || effort[i]<=0 || scale[i]<=0)
                throw std::runtime_error("Expected positive Kp, effort limits and action scale, and nonnegative Kd");
        }
    }
    Joints DefaultDds() const {Joints result{};for(int i=0;i<29;++i) result[ids[i]]=sign[i]*neutral[i];return result;}
};
} // namespace g1_visual
