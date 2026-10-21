#pragma once
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <vector>

namespace unitree_mujoco {
// Cache actuator-to-joint addresses, rather than assuming sensor or joint order.
class MotorControl {
    std::vector<int> qadr_, vadr_;
public:
    explicit MotorControl(const mjModel* model) {
        for (int i = 0; i < model->nu; ++i) {
            const int joint = model->actuator_trnid[2*i];
            if (model->actuator_trntype[i] != mjTRN_JOINT || joint < 0 ||
                (model->jnt_type[joint] != mjJNT_HINGE && model->jnt_type[joint] != mjJNT_SLIDE))
                throw std::runtime_error("Motor PD requires scalar joint actuators");
            qadr_.push_back(model->jnt_qposadr[joint]);
            vadr_.push_back(model->jnt_dofadr[joint]);
        }
    }

    template<class Commands>
    bool Apply(mjData* data, const Commands& commands) const {
        bool active = false;
        for (std::size_t i = 0; i < qadr_.size(); ++i) {
            const auto& motor = commands[i];
            active |= motor.kp() != 0 || motor.kd() != 0 || motor.tau() != 0;
            data->ctrl[i] = motor.tau() + motor.kp() * (motor.q() - data->qpos[qadr_[i]]) +
                           motor.kd() * (motor.dq() - data->qvel[vadr_[i]]);
        }
        return active;
    }
};
} // namespace unitree_mujoco
