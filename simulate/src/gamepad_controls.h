#pragma once

#include <mujoco/mujoco.h>
#include <unitree_mujoco/gamepad.h>
#include "elastic_band.h"

namespace unitree_mujoco {

// Called under Simulate::mtx, the same lock used by the render thread.
class SimulatorGamepad {
public:
    explicit SimulatorGamepad(const gamepad::Config& config) : config_(config) {}

    template<class Band>
    bool Update(const gamepad::State& state, const mjModel* model, const mjData* data,
                mjvCamera& camera, Band& band, bool band_available, double now) {
        const double dt = std::clamp(now-last_time_, 0.0, 0.02);
        last_time_ = now;
        const auto previous = previous_;
        previous_ = state;
        if (!state.connected) {
            armed_ = false;
            trigger_since_ = -1;
            center_follow_ = false;
            return false;
        }
        if (!previous.connected) { armed_ = false; return false; }
        if (!armed_) {
            armed_ = !state.a && !state.y && !state.x && !state.rb && !state.lb &&
                     state.lt < config_.trigger_threshold;
            return false;
        }
        if (state.rb && !previous.rb) return true;
        int base = -1;
        for (int i = 0; i < model->njnt; ++i) {
            if (model->jnt_type[i] == mjJNT_FREE) { base = model->jnt_bodyid[i]; break; }
        }
        if (base < 0) return false;
        const auto* position = data->xpos+3*base;
        if (band_available) {
            if (state.x && !previous.x) {
                band.enable_ = false;
            } else if (state.a || state.y || (state.b && !previous.b)) {
                if (!band.enable_) {
                    // Match the center of mass where xfrc_applied applies the force.
                    const auto* attachment = data->xipos + 3*ElasticBandBody(model);
                    band.point_ = {attachment[0], attachment[1], attachment[2]+2.0};
                    const double weight = mj_getTotalmass(model)*mju_norm3(model->opt.gravity);
                    band.length_ = std::max(0.0, 2.0-weight/band.stiffness_);
                    band.enable_ = true;
                }
                band.length_ = std::clamp(band.length_+
                    (int(state.a)-int(state.y))*config_.band_speed*dt, 0.0, 5.0);
            }
        }
        if (state.lb && !previous.lb) {
            center_follow_ = !center_follow_;
            if (center_follow_) SetRearView(camera, position, data->xmat+9*base);
        }
        // Follow position only; preserve all mouse edits to orbit and zoom.
        if (center_follow_) mju_copy3(camera.lookat, position);
        bool temporary_heading = false;
        if (state.lt >= config_.trigger_threshold) {
            if (trigger_since_ < 0) trigger_since_ = now;
            temporary_heading = now-trigger_since_ >= config_.heading_hold_seconds;
        } else {
            trigger_since_ = -1;
        }
        // Releasing LT frees the angle; LB independently controls centering.
        if (temporary_heading) {
            SetRearView(camera, position, data->xmat+9*base);
        }
        return false;
    }
private:
    void SetRearView(mjvCamera& camera, const mjtNum* position, const mjtNum* rotation) {
        camera.type = mjCAMERA_FREE;
        camera.fixedcamid = -1;
        mju_copy3(camera.lookat, position);
        const double yaw = std::atan2(rotation[3], rotation[0]);
        camera.azimuth = std::fmod(yaw*180.0/3.14159265358979323846+360.0, 360.0);
        camera.elevation = config_.camera_elevation;
        camera.distance = config_.camera_distance;
    }
    gamepad::Config config_;
    gamepad::State previous_;
    bool armed_ = false;
    bool center_follow_ = false;
    double trigger_since_ = -1, last_time_ = 0;
};
} // namespace unitree_mujoco
