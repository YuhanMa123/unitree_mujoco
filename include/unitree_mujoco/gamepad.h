#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace unitree_mujoco::gamepad {

struct Config {
    bool enabled = true;
    std::string device = "/dev/input/js0";
    float deadzone = 0.12f;
    float forward_max = 0.6f, backward_max = 0.4f, lateral_max = 0.4f, yaw_max = 1.0f;
    float velocity_step = 0.1f;
    double band_speed = 0.4, trigger_threshold = 0.5, heading_hold_seconds = 0.35;
    double camera_distance = 2.5, camera_elevation = -15;

    static Config Load(const std::string& path) {
        const auto yaml = YAML::LoadFile(path);
        Config c;
        c.enabled = yaml["enabled"].as<bool>(c.enabled);
        c.device = yaml["device"].as<std::string>(c.device);
        c.deadzone = yaml["deadzone"].as<float>(c.deadzone);
        c.forward_max = yaml["forward_max"].as<float>(c.forward_max);
        c.backward_max = yaml["backward_max"].as<float>(c.backward_max);
        c.lateral_max = yaml["lateral_max"].as<float>(c.lateral_max);
        c.yaw_max = yaml["yaw_max"].as<float>(c.yaw_max);
        c.velocity_step = yaml["velocity_step"].as<float>(c.velocity_step);
        c.band_speed = yaml["band_speed"].as<double>(c.band_speed);
        c.trigger_threshold = yaml["trigger_threshold"].as<double>(c.trigger_threshold);
        c.heading_hold_seconds = yaml["heading_hold_seconds"].as<double>(c.heading_hold_seconds);
        c.camera_distance = yaml["camera_distance"].as<double>(c.camera_distance);
        c.camera_elevation = yaml["camera_elevation"].as<double>(c.camera_elevation);
        for (double v : {double(c.forward_max), double(c.backward_max), double(c.lateral_max),
                         double(c.yaw_max), double(c.velocity_step), c.band_speed,
                         c.heading_hold_seconds, c.camera_distance}) {
            if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Invalid positive gamepad setting");
        }
        if (!std::isfinite(c.deadzone) || c.deadzone < 0 || c.deadzone >= 1 ||
            !std::isfinite(c.trigger_threshold) || c.trigger_threshold <= 0 || c.trigger_threshold >= 1 ||
            !std::isfinite(c.camera_elevation) || c.camera_elevation <= -90 || c.camera_elevation >= 0 ||
            c.device.empty()) throw std::runtime_error("Invalid gamepad deadzone, trigger, camera or device setting");
        return c;
    }

    float Deadzone(float value) const {
        if (std::abs(value) <= deadzone) return 0;
        return std::copysign((std::min(std::abs(value), 1.0f)-deadzone)/(1-deadzone), value);
    }
    void Clamp(std::array<float, 3>& velocity) const {
        velocity[0] = std::clamp(velocity[0], -backward_max, forward_max);
        velocity[1] = std::clamp(velocity[1], -lateral_max, lateral_max);
        velocity[2] = std::clamp(velocity[2], -yaw_max, yaw_max);
    }
};

struct State {
    bool connected = false;
    // Body-frame convention: forward, left and left turn are positive.
    float forward = 0, left = 0, turn = 0, lt = 0, rt = 0;
    bool a = false, b = false, x = false, y = false, lb = false, rb = false;
    bool back = false, start = false, guide = false, ls = false, rs = false;
    bool up = false, down = false, left_pad = false, right_pad = false;
};

// Translate Linux input codes rather than assuming a particular js button index.
struct RawState {
    std::array<std::int16_t, ABS_CNT> axes{};
    std::array<bool, KEY_CNT> buttons{};
    RawState() { axes[ABS_Z] = axes[ABS_RZ] = -32768; }
    static float Signed(std::int16_t value) {
        return value < 0 ? value/32768.0f : value/32767.0f;
    }
    State Snapshot() const {
        State s;
        s.connected = true;
        s.forward = -Signed(axes[ABS_Y]);
        s.left = -Signed(axes[ABS_X]);
        s.turn = -Signed(axes[ABS_RX]);
        s.lt = (double(axes[ABS_Z])+32768)/65535;
        s.rt = (double(axes[ABS_RZ])+32768)/65535;
        s.a = buttons[BTN_SOUTH]; s.b = buttons[BTN_EAST];
        s.x = buttons[BTN_NORTH]; s.y = buttons[BTN_WEST];
        s.lb = buttons[BTN_TL]; s.rb = buttons[BTN_TR];
        s.back = buttons[BTN_SELECT]; s.start = buttons[BTN_START];
        s.guide = buttons[BTN_MODE]; s.ls = buttons[BTN_THUMBL]; s.rs = buttons[BTN_THUMBR];
        s.up = axes[ABS_HAT0Y] < -16384; s.down = axes[ABS_HAT0Y] > 16384;
        s.left_pad = axes[ABS_HAT0X] < -16384; s.right_pad = axes[ABS_HAT0X] > 16384;
        return s;
    }
};

class Reader {
public:
    explicit Reader(const Config& config) : config_(config) {}
    ~Reader() { if (fd_ >= 0) close(fd_); }
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    const std::string& Name() const { return name_; }
    const std::string& Error() const { return error_; }

    State Poll() {
        if (!config_.enabled) return {};
        const auto now = std::chrono::steady_clock::now();
        if (fd_ < 0) {
            if (now < retry_at_) return {};
            retry_at_ = now + std::chrono::seconds(1);
            fd_ = open(config_.device.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd_ < 0) { error_ = std::strerror(errno); return {}; }
            raw_ = RawState{};
            unsigned char axes = 0, buttons = 0;
            char name[128] = {};
            if (ioctl(fd_, JSIOCGAXES, &axes) < 0 || ioctl(fd_, JSIOCGBUTTONS, &buttons) < 0 ||
                ioctl(fd_, JSIOCGAXMAP, axis_map_.data()) < 0 ||
                ioctl(fd_, JSIOCGBTNMAP, button_map_.data()) < 0) {
                Disconnect(std::strerror(errno)); return {};
            }
            if (axes > axis_map_.size() || buttons > button_map_.size()) {
                Disconnect("Unsupported joystick axis/button count"); return {};
            }
            ioctl(fd_, JSIOCGNAME(sizeof(name)), name);
            name[sizeof(name)-1] = '\0';
            name_ = name;
            error_.clear();
        }
        // Drain queued events; do not turn initialization events into button presses.
        for (;;) {
            js_event event{};
            const auto count = read(fd_, &event, sizeof(event));
            if (count == sizeof(event)) {
                const auto type = event.type & ~JS_EVENT_INIT;
                if (type == JS_EVENT_AXIS && event.number < axis_map_.size()) {
                    const auto code = axis_map_[event.number];
                    if (code < raw_.axes.size()) raw_.axes[code] = event.value;
                } else if (type == JS_EVENT_BUTTON && event.number < button_map_.size()) {
                    const auto code = button_map_[event.number];
                    if (code < raw_.buttons.size()) raw_.buttons[code] = event.value != 0;
                }
            } else if (count < 0 && errno == EINTR) {
                continue;
            } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break;
            } else {
                Disconnect(count < 0 ? std::strerror(errno) : "Joystick disconnected");
                return {};
            }
        }
        return raw_.Snapshot();
    }
private:
    void Disconnect(const std::string& error) {
        if (fd_ >= 0) close(fd_);
        fd_ = -1;
        raw_ = RawState{};
        error_ = error;
    }
    Config config_;
    int fd_ = -1;
    RawState raw_;
    std::array<unsigned char, ABS_CNT> axis_map_{};
    std::array<unsigned short, KEY_MAX-BTN_MISC+1> button_map_{};
    std::string name_, error_;
    std::chrono::steady_clock::time_point retry_at_{};
};

struct Actions {
    bool reset = false, start = false, stop = false, suspend = false;
    bool planar_update = false, yaw_update = false;
    std::array<float, 3> velocity{};
};

class Drive {
public:
    explicit Drive(const Config& config) : config_(config) {}
    void Clear() { mode_ = Mode::Idle; yaw_owned_ = false; armed_ = false; cruise_ = {}; }
    Actions Update(const State& state, const std::array<float, 3>& current, bool active) {
        Actions action;
        action.velocity = current;
        const auto previous = previous_;
        previous_ = state;
        if (!state.connected) {
            action.stop = previous.connected;
            Clear();
            return action;
        }
        if (!previous.connected) { Clear(); return action; }
        action.reset = state.b && !previous.b;
        action.start = !action.reset && state.start && !previous.start;
        action.suspend = (state.a && !previous.a) || (state.y && !previous.y) || (state.rb && !previous.rb);
        action.stop = (state.back && !previous.back) || (state.rs && !previous.rs);
        if (action.reset || action.start || action.stop || action.suspend || !active) {
            Clear(); return action;
        }
        const float forward = config_.Deadzone(state.forward), left = config_.Deadzone(state.left);
        const float turn = config_.Deadzone(state.turn);
        const bool neutral = forward == 0 && left == 0;
        if (!armed_) {
            if (neutral && turn == 0) armed_ = true;
            return action;
        }
        const int dx = int(state.up && !previous.up)-int(state.down && !previous.down);
        const int dy = int(state.left_pad && !previous.left_pad)-int(state.right_pad && !previous.right_pad);
        if (dx || dy) {
            cruise_ = current;
            cruise_[0] += dx*config_.velocity_step;
            cruise_[1] += dy*config_.velocity_step;
            config_.Clamp(cruise_);
            mode_ = Mode::Cruise;
            takeover_armed_ = neutral;
        } else if (!neutral && (mode_ != Mode::Cruise || takeover_armed_)) {
            mode_ = Mode::Stick;
        }
        if (neutral) takeover_armed_ = true;
        if (mode_ == Mode::Cruise) {
            action.velocity[0] = cruise_[0]; action.velocity[1] = cruise_[1];
            action.planar_update = true;
        } else if (mode_ == Mode::Stick) {
            action.velocity[0] = forward*(forward >= 0 ? config_.forward_max : config_.backward_max);
            action.velocity[1] = left*config_.lateral_max;
            action.planar_update = true;
            if (neutral) mode_ = Mode::Idle;
        }
        if (turn != 0) yaw_owned_ = true;
        if (yaw_owned_) {
            action.velocity[2] = turn*config_.yaw_max;
            action.yaw_update = true;
            if (turn == 0) yaw_owned_ = false;
        }
        return action;
    }
private:
    enum class Mode { Idle, Stick, Cruise };
    Config config_;
    State previous_;
    Mode mode_ = Mode::Idle;
    std::array<float, 3> cruise_{};
    bool armed_ = false, takeover_armed_ = false, yaw_owned_ = false;
};
} // namespace unitree_mujoco::gamepad
