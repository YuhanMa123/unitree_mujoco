#pragma once

#include <mujoco/mujoco.h>
#include <unitree_mujoco/gamepad.h>

namespace unitree_mujoco {

// Called under Simulate::mtx, the same lock used by the render thread.
class SimulatorGamepad {
public:
    explicit SimulatorGamepad(const gamepad::Config& config) : config_(config) {}

    template<class Band>
    void Update(const gamepad::State& state, const mjModel* model, const mjData* data,
                mjvCamera& camera, Band& band, bool band_available, double now) {
        const double dt = std::clamp(now-last_time_, 0.0, 0.02);
        last_time_ = now;
        const auto previous = previous_;
        previous_ = state;
        if (!state.connected) {
            RestoreCamera(camera);
            heading_locked_ = false;
            armed_ = false;
            trigger_since_ = -1;
            return;
        }
        if (!previous.connected) { armed_ = false; return; }
        if (!armed_) {
            armed_ = !state.a && !state.y && !state.x && !state.rb && !state.lb &&
                     state.lt < config_.trigger_threshold;
            return;
        }
        int base = -1;
        for (int i = 0; i < model->njnt; ++i) {
            if (model->jnt_type[i] == mjJNT_FREE) { base = model->jnt_bodyid[i]; break; }
        }
        if (base < 0) return;
        const auto* position = data->xpos+3*base;
        if (band_available) {
            if (state.x && !previous.x) {
                band.enable_ = false;
            } else if (state.a || state.y || (state.rb && !previous.rb) || (state.b && !previous.b)) {
                if (!band.enable_) {
                    // Reattach above the current base, without pulling back to the origin.
                    band.point_ = {position[0], position[1], position[2]+2.0};
                    const double weight = mj_getTotalmass(model)*mju_norm3(model->opt.gravity);
                    band.length_ = std::max(0.0, 2.0-weight/band.stiffness_);
                    band.enable_ = true;
                }
                band.length_ = std::clamp(band.length_+
                    (int(state.a)-int(state.y))*config_.band_speed*dt, 0.0, 5.0);
            }
        }
        if (state.lb && !previous.lb) heading_locked_ = !heading_locked_;
        bool temporary_heading = false;
        if (state.lt >= config_.trigger_threshold) {
            if (trigger_since_ < 0) trigger_since_ = now;
            temporary_heading = now-trigger_since_ >= config_.heading_hold_seconds;
        } else {
            trigger_since_ = -1;
        }
        if (heading_locked_ || temporary_heading) {
            if (!heading_) {
                saved_camera_ = camera;
                heading_ = true;
            }
        } else {
            RestoreCamera(camera);
        }
        if (heading_) {
            camera.type = mjCAMERA_FREE;
            camera.fixedcamid = -1;
            mju_copy3(camera.lookat, position);
            const auto* rotation = data->xmat+9*base;
            const double yaw = std::atan2(rotation[3], rotation[0]);
            camera.azimuth = std::fmod(yaw*180.0/3.14159265358979323846+360.0, 360.0);
            camera.elevation = config_.camera_elevation;
            camera.distance = config_.camera_distance;
        }
    }
private:
    void RestoreCamera(mjvCamera& camera) {
        if (!heading_) return;
        camera.azimuth = saved_camera_.azimuth;
        camera.elevation = saved_camera_.elevation;
        camera.distance = saved_camera_.distance;
        heading_ = false;
    }
    gamepad::Config config_;
    gamepad::State previous_;
    bool armed_ = false, heading_locked_ = false, heading_ = false;
    double trigger_since_ = -1, last_time_ = 0;
    mjvCamera saved_camera_{};
};
} // namespace unitree_mujoco
