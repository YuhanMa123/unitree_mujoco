#include <iostream>
#include <stdexcept>
#include <vector>
#include "gamepad_controls.h"
#include "simulation_reset.h"

namespace pad = unitree_mujoco::gamepad;
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void Near(double a, double b) { Check(std::isfinite(a) && std::abs(a-b) < 1e-5, "Unexpected numeric result"); }

void CheckDrive(const pad::Config& config) {
    pad::RawState raw;
    auto sample = raw.Snapshot();
    Near(sample.lt, 0); Near(sample.rt, 0);
    raw.axes[ABS_Y] = -32768; raw.axes[ABS_X] = -32768; raw.axes[ABS_RX] = 32767;
    raw.axes[ABS_Z] = 32767; raw.axes[ABS_HAT0Y] = -32767;
    raw.buttons[BTN_NORTH] = true; raw.buttons[BTN_WEST] = true;
    sample = raw.Snapshot();
    Near(sample.forward, 1); Near(sample.left, 1); Near(sample.turn, -1); Near(sample.lt, 1);
    Check(sample.x && sample.y && sample.up, "Xbox Linux button/hat decoding failed");
    Near(config.Deadzone(0.05f), 0);

    pad::Drive drive(config);
    pad::State state; state.connected = true;
    std::array<float, 3> velocity{};
    auto tick = [&](bool active = true) {
        const auto action = drive.Update(state, velocity, active);
        if (action.stop || action.reset || action.suspend || action.simulation_reset) velocity.fill(0);
        if (action.planar_update) { velocity[0] = action.velocity[0]; velocity[1] = action.velocity[1]; }
        if (action.yaw_update) velocity[2] = action.velocity[2];
        return action;
    };
    tick(); tick();
    state.up = true; tick(); Near(velocity[0], 0.1);
    for (int i=0; i<50; ++i) tick();
    Near(velocity[0], 0.1); // Held D-pad does not repeat.
    state.up = false; tick(); state.up = true; tick(); Near(velocity[0], 0.2);
    state.up = false; state.left_pad = true; tick(); Near(velocity[1], 0.1);
    state.left_pad = false; tick();
    state.forward = 0.5f; tick();
    Near(velocity[0], config.Deadzone(0.5f)*config.forward_max); Near(velocity[1], 0);
    state.up = true; tick();
    const float cruise = config.Deadzone(0.5f)*config.forward_max+config.velocity_step;
    Near(velocity[0], cruise);
    state.up = false; tick(); Near(velocity[0], cruise); // D-pad can take over a held stick.
    state.forward = 0; tick(); Near(velocity[0], cruise);
    state.forward = -1; tick(); Near(velocity[0], -config.backward_max);
    state.forward = 0; tick(); Near(velocity[0], 0);
    state.turn = -1; tick(); Near(velocity[2], -config.yaw_max);
    state.turn = 0; tick(); Near(velocity[2], 0);
    for (int i=0; i<20; ++i) { state.up = true; tick(); state.up = false; tick(); }
    Near(velocity[0], config.forward_max);

    state.back = true; Check(tick().stop, "View must stop velocity"); Near(velocity[0], 0);
    state.back = false; tick(); tick();
    state.b = true; Check(tick(false).reset, "B must request reset before policy entry");
    Check(!tick(false).reset, "Held B must not repeatedly reset");
    state.b = false; state.start = true; Check(tick(false).start, "Menu must request policy entry");
    state.start = false; state.y = true; Check(tick().suspend, "Band movement must suspend walking");
    state.y = false; tick(); tick();
    state.rb = true;
    const auto reset_simulation = tick();
    Check(reset_simulation.simulation_reset && !reset_simulation.reset && !reset_simulation.suspend,
          "RB must request simulation reset without exiting policy for a joint reset");
    Check(!tick().simulation_reset, "Held RB must not repeatedly reset simulation");
    Near(velocity[0], 0);
    state.rb = false; tick(); tick();
    state.up = true; tick(); Check(velocity[0] > 0, "D-pad must work after neutral rearm");
    state.connected = false; Check(tick().stop, "Disconnect must stop velocity"); Near(velocity[0], 0);
    state.connected = true; state.b = true;
    Check(!tick(false).reset && !tick(false).reset, "Reconnection must suppress held buttons");
    state.b = false; tick(false); state.b = true; Check(tick(false).reset, "Reset must work after releasing button");

    pad::Drive startup(config);
    state = {}; state.connected = true; state.forward = 1;
    startup.Update(state, {}, true);
    Check(!startup.Update(state, {}, true).planar_update, "Held stick at startup must wait for neutral");
    state.forward = 0; startup.Update(state, {}, true);
    state.forward = 1; Check(startup.Update(state, {}, true).planar_update, "Neutral rearm must enable stick");
}

struct Band {
    bool enable_ = true;
    double length_ = 1, stiffness_ = 200;
    std::vector<double> point_{0,0,3};
};

void CheckSimulator(const pad::Config& config) {
    const char xml[] = "<mujoco><worldbody><body name='base' pos='1 2 1'><freejoint/><geom type='sphere' size='.1' mass='1'/></body></worldbody></mujoco>";
    mjVFS vfs; mj_defaultVFS(&vfs);
    Check(mj_addBufferVFS(&vfs, "gamepad.xml", xml, sizeof(xml)) == 0, "Cannot create camera test model");
    char error[1024]{};
    mjModel* model = mj_loadXML("gamepad.xml", &vfs, error, sizeof(error));
    mj_deleteVFS(&vfs);
    if (!model) throw std::runtime_error(error);
    mjData* data = mj_makeData(model);
    data->qpos[3] = std::sqrt(0.5); data->qpos[6] = std::sqrt(0.5); // base faces +Y
    mj_forward(model, data);
    mjvCamera camera; mjv_defaultCamera(&camera);
    camera.azimuth = 37; camera.elevation = -25; camera.distance = 4;
    Band band;
    unitree_mujoco::SimulatorGamepad ui(config);
    pad::State state; state.connected = true;
    double now = 0;
    auto tick = [&]() { now += 0.02; ui.Update(state, model, data, camera, band, true, now); };
    tick(); tick();
    const double initial = band.length_;
    state.y = true;
    for (int i=0; i<20; ++i) tick();
    Near(band.length_, initial-config.band_speed*0.4);
    state.y = false; state.a = true;
    for (int i=0; i<20; ++i) tick();
    Near(band.length_, initial);
    state.a = false; state.x = true; tick(); Check(!band.enable_, "X must release the band");
    state.x = false; state.b = true; tick(); Check(band.enable_, "B must reattach the band");
    Near(band.point_[0], 1); Near(band.point_[1], 2); Near(band.point_[2], 3);
    state.b = false; state.lb = true; tick();
    Near(camera.lookat[0], 1); Near(camera.lookat[1], 2); Near(camera.lookat[2], 1);
    Near(camera.azimuth, 90); Near(camera.distance, config.camera_distance);
    // LB toggles position follow; holding it must not toggle again or lock orbit.
    camera.azimuth = 37; camera.elevation = -25; camera.distance = 4;
    tick(); Near(camera.azimuth, 37); Near(camera.elevation, -25); Near(camera.distance, 4);
    data->qpos[0] = 5;
    data->qpos[3] = std::cos(3.14159265358979323846/8);
    data->qpos[6] = std::sin(3.14159265358979323846/8);
    mj_forward(model, data); tick();
    Near(camera.lookat[0], 5); Near(camera.azimuth, 37);
    state.lb = false; tick(); Near(camera.azimuth, 37);
    state.lb = true; tick(); // Second press disables centering without changing orbit.
    Near(camera.lookat[0], 5); Near(camera.azimuth, 37); Near(camera.distance, 4);
    data->qpos[0] = 6; mj_forward(model, data);
    tick(); Near(camera.lookat[0], 5); // Held LB does not turn follow back on.
    state.lb = false; tick();
    state.lb = true; tick(); // Third press enables and aligns once again.
    Near(camera.lookat[0], 6); Near(camera.azimuth, 45);
    state.lb = false; state.lt = 1; tick();
    for (int i=0; i<20; ++i) tick();
    Near(camera.azimuth, 45);
    state.lt = 0; tick();
    camera.azimuth = 37; camera.elevation = -25; camera.distance = 4;
    data->qpos[0] = 5;
    data->qpos[3] = data->qpos[6] = std::sqrt(0.5); mj_forward(model, data);
    tick(); Near(camera.lookat[0], 5); Near(camera.azimuth, 37);
    Near(camera.elevation, -25); Near(camera.distance, 4); // LB remains enabled after LT release.
    state.lb = true; tick(); // Disable centering for the independent LT checks below.
    state.lb = false; tick();
    data->qpos[3] = std::cos(3.14159265358979323846/8);
    data->qpos[6] = std::sin(3.14159265358979323846/8); mj_forward(model, data);
    camera.azimuth = 37; camera.elevation = -25; camera.distance = 4;
    state.lt = 1; tick();
    Near(camera.azimuth, 37); // LT must be held long enough.
    for (int i=0; i<20; ++i) tick();
    Near(camera.azimuth, 45); Near(camera.distance, config.camera_distance);
    // Only held LT continuously follows translation and turns.
    data->qpos[0] = 6;
    data->qpos[3] = data->qpos[6] = std::sqrt(0.5); mj_forward(model, data);
    tick(); Near(camera.lookat[0], 6); Near(camera.azimuth, 90);
    mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(model, &scene, 100);
    mjvOption option; mjv_defaultOption(&option);
    mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
    if (scene.camera[0].forward[1] <= 0.9)
        throw std::runtime_error("Heading camera direction: " + std::to_string(scene.camera[0].forward[0]) +
                                 ", " + std::to_string(scene.camera[0].forward[1]) + ", " + std::to_string(scene.camera[0].forward[2]));
    mjv_freeScene(&scene);
    state.lt = 0; tick(); Near(camera.azimuth, 90);
    Near(camera.elevation, config.camera_elevation); Near(camera.distance, config.camera_distance);
    data->qpos[0] = 5;
    data->qpos[3] = std::cos(3.14159265358979323846/8);
    data->qpos[6] = std::sin(3.14159265358979323846/8);
    mj_forward(model, data); tick();
    Near(camera.lookat[0], 6); Near(camera.azimuth, 90); // No follow after release.
    camera.azimuth = 37; camera.elevation = -25; camera.distance = 4;
    camera.lookat[0] = 8;
    tick(); Near(camera.azimuth, 37); Near(camera.elevation, -25);
    Near(camera.distance, 4); Near(camera.lookat[0], 8); // Mouse edits remain free.
    state.x = true; tick(); Check(!band.enable_, "Release before reset test");
    state.x = false; state.b = true; tick();
    Check(band.enable_, "B reset must reattach the support band");
    Near(band.point_[0], 5); Near(band.point_[1], 2);
    state.b = false;
    state.lt = 1; tick(); for (int i=0; i<20; ++i) tick();
    Near(camera.azimuth, 45); Near(camera.lookat[0], 5); // A new hold resumes follow.
    state.connected = false; tick(); Near(camera.azimuth, 45);
    data->qpos[0] = 7; mj_forward(model, data); tick(); Near(camera.lookat[0], 5);
    // Reconnect with RB held must not reset until it is released and pressed again.
    state.connected = true; state.rb = true;
    auto update = [&]() {
        now += 0.02;
        return ui.Update(state, model, data, camera, band, true, now);
    };
    Check(!update() && !update(), "Held RB on reconnect must not reset");
    state.rb = false; state.lt = 0; update();
    const auto generation = unitree_mujoco::simulation_reset_generation;
    data->time = 12; data->qvel[0] = 3; data->xfrc_applied[0] = 10;
    state.rb = true;
    Check(update(), "RB press must request full simulation reset");
    unitree_mujoco::ResetSimulation(model, data);
    Near(data->time, 0); Near(data->qpos[0], model->qpos0[0]);
    Near(data->qvel[0], 0); Near(data->xfrc_applied[0], 0);
    Check(unitree_mujoco::simulation_reset_generation == generation+1, "Reset generation must advance");
    data->time = 1;
    Check(!update(), "Holding RB must not repeatedly reset");
    Near(data->time, 1);
    mj_deleteData(data); mj_deleteModel(model);
}

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "Expected gamepad configuration path");
        const auto config = pad::Config::Load(argv[1]);
        CheckDrive(config);
        CheckSimulator(config);
        std::cout << "Xbox decoding, button edges, cruise/stick takeover, limits, reconnect, band and camera checks passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
