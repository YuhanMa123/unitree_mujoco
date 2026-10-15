#pragma once

// G1 velocity interface from unitreerobotics/unitree_rl_lab (Apache-2.0).
// No Isaac runtime is used. Arrays from DDS are reordered before inference.
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

namespace g1_policy {
constexpr int kJoints = 29;
constexpr int kFrame = 96;
constexpr int kHistory = 5;
constexpr int kObservation = kFrame * kHistory;
using Joints = std::array<float, kJoints>;
struct State {
    Joints q{}, dq{};                       // DDS order
    std::array<float, 4> quaternion{1, 0, 0, 0}; // w,x,y,z
    std::array<float, 3> omega{};           // body frame (IMU)
};

template<class T, std::size_t N>
std::array<T, N> ReadArray(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() != N)
        throw std::runtime_error("Wrong policy configuration array length");
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = node[i].as<T>();
        if (!std::isfinite(static_cast<double>(result[i])))
            throw std::runtime_error("Non-finite policy configuration");
    }
    return result;
}

class Policy {
public:
    Policy(const std::string& model, const std::string& config)
        : env_(ORT_LOGGING_LEVEL_WARNING, "g1_locomotion") {
        const auto cfg = YAML::LoadFile(config);
        ids_ = ReadArray<int, kJoints>(cfg["joint_ids_map"]);
        std::array<bool, kJoints> seen{};
        for (int id : ids_) {
            if (id < 0 || id >= kJoints || seen[id])
                throw std::runtime_error("joint_ids_map must be a permutation of 0..28");
            seen[id] = true;
        }
        default_ = ReadArray<float, kJoints>(cfg["default_joint_pos"]);
        kp = ReadArray<float, kJoints>(cfg["stiffness"]); // upstream stores gains in DDS order
        kd = ReadArray<float, kJoints>(cfg["damping"]);
        for (int i = 0; i < kJoints; ++i)
            if (kp[i] < 0 || kd[i] < 0) throw std::runtime_error("Negative PD gains");
        dt = cfg["step_dt"].as<double>();
        if (!std::isfinite(dt) || dt < 0.002) throw std::runtime_error("Invalid policy step_dt");
        const auto action = cfg["actions"]["JointPositionAction"];
        scale_ = ReadArray<float, kJoints>(action["scale"]);
        offset_ = ReadArray<float, kJoints>(action["offset"]);
        if (!action["joint_ids"].IsNull())
            throw std::runtime_error("This interface requires full-joint actions");
        raw_clip_ = action["raw_clip"].as<float>(std::numeric_limits<float>::max());
        if (!std::isfinite(raw_clip_) || raw_clip_ <= 0)
            throw std::runtime_error("Invalid raw action clip");
        for (int i = 0; i < kJoints; ++i) {
            target_clip_[i] = {-std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            if (!action["clip"].IsNull()) {
                if (!action["clip"].IsSequence() || action["clip"].size() != kJoints)
                    throw std::runtime_error("Expected 29 joint target clip bounds");
                target_clip_[i] = ReadArray<float, 2>(action["clip"][i]);
                if (target_clip_[i][0] > target_clip_[i][1])
                    throw std::runtime_error("Reversed joint target clip bounds");
            }
        }
        const auto observations = cfg["observations"];
        const std::array<const char*, 6> names{"base_ang_vel", "projected_gravity",
            "velocity_commands", "joint_pos_rel", "joint_vel_rel", "last_action"};
        const std::array<int, 6> sizes{3, 3, 3, 29, 29, 29};
        if (observations.size() != names.size()) throw std::runtime_error("Unexpected observation terms");
        int term = 0, index = 0;
        for (const auto& entry : observations) {
            if (entry.first.as<std::string>() != names[term])
                throw std::runtime_error("Unexpected observation order");
            const auto node = entry.second;
            if (node["history_length"].as<int>() != kHistory ||
                !node["scale"].IsSequence() || node["scale"].size() != std::size_t(sizes[term]))
                throw std::runtime_error("Expected five-frame observations");
            std::array<float, 2> clip{-std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            if (!node["clip"].IsNull()) clip = ReadArray<float, 2>(node["clip"]);
            if (clip[0] > clip[1]) throw std::runtime_error("Reversed observation clip bounds");
            for (int i = 0; i < sizes[term]; ++i) {
                float value = node["scale"][i].as<float>();
                if (!std::isfinite(value)) throw std::runtime_error("Non-finite observation scale");
                obs_clip_[index] = clip;
                obs_scale_[index++] = value;
            }
            ++term;
        }
        const auto ranges = cfg["commands"]["base_velocity"]["ranges"];
        const std::array<const char*, 3> velocity_names{"lin_vel_x", "lin_vel_y", "ang_vel_z"};
        for (int i = 0; i < 3; ++i) {
            limits_[i] = ReadArray<float, 2>(ranges[velocity_names[i]]);
            if (limits_[i][0] > 0 || limits_[i][1] < 0)
                throw std::runtime_error("Command limits must include zero");
        }
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(env_, model.c_str(), options);
        if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1)
            throw std::runtime_error("Expected one policy input and output");
        auto in_type = session_->GetInputTypeInfo(0);
        auto out_type = session_->GetOutputTypeInfo(0);
        auto in = in_type.GetTensorTypeAndShapeInfo();
        auto out = out_type.GetTensorTypeAndShapeInfo();
        if (in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            in.GetShape() != std::vector<int64_t>({1, kObservation}) ||
            out.GetShape() != std::vector<int64_t>({1, kJoints}))
            throw std::runtime_error("Expected float32 model: [1,480] -> [1,29]");
        Ort::AllocatorWithDefaultOptions allocator;
        input_name_ = session_->GetInputNameAllocated(0, allocator).get();
        output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
        Reset();
    }

    void Reset() { last_action_.fill(0); initialized_ = false; }
    Joints DefaultDds() const {
        Joints result{};
        for (int i = 0; i < kJoints; ++i) result[ids_[i]] = default_[i];
        return result;
    }
    void ClampVelocity(std::array<float, 3>& command) const {
        for (int i = 0; i < 3; ++i) command[i] = std::clamp(command[i], limits_[i][0], limits_[i][1]);
    }

    // Term-major history, oldest -> newest within each term (Isaac Lab default).
    std::array<float, kObservation> Observe(const State& state, const std::array<float, 3>& velocity) {
        std::array<float, kFrame> frame{};
        float norm = 0;
        for (float v : state.quaternion) norm += v*v;
        if (!std::isfinite(norm) || norm < 1e-8f) throw std::runtime_error("Invalid IMU quaternion");
        norm = std::sqrt(norm);
        const float w = state.quaternion[0]/norm, x = state.quaternion[1]/norm;
        const float y = state.quaternion[2]/norm, z = state.quaternion[3]/norm;
        for (int i = 0; i < 3; ++i) frame[i] = state.omega[i];
        frame[3] = 2*(w*y-x*z);
        frame[4] = -2*(y*z+w*x);
        frame[5] = 2*(x*x+y*y)-1;
        for (int i = 0; i < 3; ++i) frame[6+i] = velocity[i];
        for (int i = 0; i < kJoints; ++i) {
            frame[9+i] = state.q[ids_[i]] - default_[i];
            frame[38+i] = state.dq[ids_[i]];
            frame[67+i] = last_action_[i];
        }
        for (int i = 0; i < kFrame; ++i) {
            frame[i] *= obs_scale_[i];
            if (!std::isfinite(frame[i])) throw std::runtime_error("Non-finite observation");
            // WalkNet1 clips the scaled observation before model input.
            frame[i] = std::clamp(frame[i], obs_clip_[i][0], obs_clip_[i][1]);
        }
        if (!initialized_) { history_.fill(frame); initialized_ = true; }
        else {
            for (int h = 0; h < kHistory-1; ++h) history_[h] = history_[h+1];
            history_.back() = frame;
        }
        std::array<float, kObservation> observation{};
        int offset = 0, cursor = 0;
        for (int size : {3, 3, 3, 29, 29, 29}) {
            for (const auto& entry : history_)
                for (int i = 0; i < size; ++i) observation[cursor++] = entry[offset+i];
            offset += size;
        }
        return observation;
    }

    Joints Step(const State& state, const std::array<float, 3>& velocity) {
        auto observation = Observe(state, velocity);
        const std::array<int64_t, 2> shape{1, kObservation};
        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto tensor = Ort::Value::CreateTensor<float>(memory, observation.data(), observation.size(), shape.data(), shape.size());
        const char* inputs[]{input_name_.c_str()};
        const char* outputs[]{output_name_.c_str()};
        auto result = session_->Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
        const auto* actions = result.front().GetTensorData<float>();
        Joints raw{};
        std::copy_n(actions, kJoints, raw.begin());
        return ProcessActions(raw);
    }

    Joints ProcessActions(const Joints& actions) {
        Joints target{};
        for (int i = 0; i < kJoints; ++i) {
            if (!std::isfinite(actions[i])) throw std::runtime_error("Non-finite policy action");
            last_action_[i] = actions[i]; // raw, unscaled actions are fed back
            target[ids_[i]] = offset_[i] + scale_[i]*std::clamp(actions[i], -raw_clip_, raw_clip_);
            if (!std::isfinite(target[ids_[i]])) throw std::runtime_error("Non-finite policy target");
            target[ids_[i]] = std::clamp(target[ids_[i]], target_clip_[i][0], target_clip_[i][1]);
        }
        return target;
    }

    Joints kp{}, kd{};
    double dt = 0.02;
private:
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_, output_name_;
    std::array<int, kJoints> ids_{};
    Joints default_{}, scale_{}, offset_{}, last_action_{};
    std::array<float, kFrame> obs_scale_{};
    std::array<std::array<float, 2>, kFrame> obs_clip_{};
    std::array<std::array<float, 2>, kJoints> target_clip_{};
    float raw_clip_ = std::numeric_limits<float>::max();
    std::array<std::array<float, kFrame>, kHistory> history_{};
    std::array<std::array<float, 2>, 3> limits_{};
    bool initialized_ = false;
};
} // namespace g1_policy
