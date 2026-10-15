// G1 29DOF MuJoCo joint reset: current position -> default pose -> hold.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <cerrno>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <boost/property_tree/xml_parser.hpp>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;
using Clock = std::chrono::steady_clock;

namespace {
constexpr std::size_t kJointCount = 29;
constexpr auto kPeriod = std::chrono::milliseconds(2);  // 500 Hz
volatile std::sig_atomic_t stop_requested = 0;
void Stop(int) { stop_requested = 1; }

// Read single keys without Enter; restore terminal settings on every exit.
class Keyboard {
public:
    Keyboard() {
        if (!isatty(STDIN_FILENO)) return;
        if (tcgetattr(STDIN_FILENO, &original_) != 0)
            throw std::runtime_error("Cannot read terminal settings");
        auto settings = original_;
        settings.c_lflag &= ~(ICANON | ECHO);  // keep Ctrl+C signal handling
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) != 0)
            throw std::runtime_error("Cannot enable keyboard input");
        restore_ = true;
    }
    ~Keyboard() {
        if (restore_) tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
    char Read() {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 0);
        if (ready < 0) {
            if (errno == EINTR) return 0;
            throw std::runtime_error("Cannot poll keyboard input");
        }
        if (!ready) return 0;
        if (input.revents & (POLLERR | POLLNVAL))
            throw std::runtime_error("Keyboard input closed");
        char key = 0;
        const auto count = read(STDIN_FILENO, &key, 1);
        if (count == 0) return 'q';  // closed pipe/terminal
        if (count < 0 && errno != EINTR && errno != EAGAIN)
            throw std::runtime_error("Cannot read keyboard input");
        return count > 0 ? key : 0;
    }
private:
    termios original_{};
    bool restore_ = false;
};

// Joint names in G1 29DOF DDS order. XML order is independent.
constexpr std::array<const char*, kJointCount> kJointNames{
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
};

struct ResetConfig {
    std::array<float, kJointCount> default_pos{};
    std::array<float, kJointCount> kp{};
    std::array<float, kJointCount> kd{};
};

ResetConfig LoadConfig(const std::string& path) {
    boost::property_tree::ptree tree;
    boost::property_tree::read_xml(path, tree,
        boost::property_tree::xml_parser::trim_whitespace);
    const auto& root = tree.get_child("g1_reset");
    ResetConfig config;
    std::array<bool, kJointCount> seen{};
    for (const auto& entry : root) {
        if (entry.first == "<xmlcomment>") continue;
        if (entry.first != "joint")
            throw std::runtime_error("Unexpected XML element: " + entry.first);
        const auto name = entry.second.get<std::string>("<xmlattr>.name");
        const auto joint = std::find_if(kJointNames.begin(), kJointNames.end(),
            [&](const char* expected) { return name == expected; });
        if (joint == kJointNames.end())
            throw std::runtime_error("Unknown joint in XML: " + name);
        const auto i = static_cast<std::size_t>(joint - kJointNames.begin());
        if (seen[i]) throw std::runtime_error("Duplicate joint in XML: " + name);
        config.default_pos[i] = entry.second.get<float>("<xmlattr>.default_pos");
        config.kp[i] = entry.second.get<float>("<xmlattr>.kp");
        config.kd[i] = entry.second.get<float>("<xmlattr>.kd");
        if (!std::isfinite(config.default_pos[i]) || !std::isfinite(config.kp[i]) ||
            !std::isfinite(config.kd[i]) || config.kp[i] < 0 || config.kd[i] < 0)
            throw std::runtime_error("Invalid default_pos/kp/kd for joint: " + name);
        seen[i] = true;
    }
    for (std::size_t i = 0; i < kJointCount; ++i)
        if (!seen[i]) throw std::runtime_error(std::string("Missing joint in XML: ") + kJointNames[i]);
    return config;
}

#ifndef G1_RESET_DEFAULT_CONFIG
#define G1_RESET_DEFAULT_CONFIG "g1_reset.xml"
#endif

// SDK wire CRC, excluding the final crc field. memcpy avoids aliasing.
uint32_t CommandCrc(const LowCmd_& command) {
    static_assert(sizeof(LowCmd_) % sizeof(uint32_t) == 0);
    const auto* bytes = reinterpret_cast<const unsigned char*>(&command);
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < sizeof(LowCmd_) / 4 - 1; ++i) {
        uint32_t word;
        std::memcpy(&word, bytes + i * 4, sizeof(word));
        for (uint32_t mask = 0x80000000u; mask; mask >>= 1) {
            const bool high_bit = (crc & 0x80000000u) != 0;
            crc <<= 1;
            if (high_bit) crc ^= 0x04C11DB7u;
            if (word & mask) crc ^= 0x04C11DB7u;
        }
    }
    return crc;
}
}  // namespace

class JointReset {
public:
    explicit JointReset(const ResetConfig& config) : config_(config), publisher_("rt/lowcmd"), target_publisher_("rt/g1/reset_target"), enable_publisher_("rt/g1/reset_enable"), subscriber_("rt/lowstate") {
        publisher_.InitChannel();
        target_publisher_.InitChannel();
        enable_publisher_.InitChannel();
        subscriber_.InitChannel(
            std::bind(&JointReset::OnState, this, std::placeholders::_1), 1);
    }
    ~JointReset() { subscriber_.CloseChannel(); }

    int Run(double duration) {
        std::cout << "Waiting for G1 29DOF lowstate (5 s timeout)...\n";
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        LowState_ state{};
        while (!stop_requested && !Snapshot(state)) {
            if (Clock::now() >= deadline) {
                std::cerr << "No lowstate. Check G1 scene, DDS domain and interface.\n";
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (stop_requested) return 0;
        // C++ bridge publishes 4 for 23DOF, 5 for 29DOF. Python bridge
        // leaves mode_machine at 0, so its scene must be checked by the user.
        if (state.mode_machine() != 0 && state.mode_machine() != 5) {
            std::cerr << "Expected 29DOF mode_machine 5, received "
                      << unsigned(state.mode_machine()) << ".\n";
            return 1;
        }
        std::array<float, kJointCount> start_angles{};

        LowCmd_ command{};
        command.mode_pr() = 0;  // pitch/roll joint coordinates, not A/B motors
        command.mode_machine() = state.mode_machine();
        Keyboard keyboard;
        std::cout << "Ready. Command output DISABLED. R: set reset target (" << duration
                  << " s), E: enable output, D: disable output, Space / Q / Ctrl+C: exit.\n";
        auto start = Clock::now();
        auto next = start;
        bool active = false;
        bool output_enabled = false;
        bool publishing = false;
        bool holding = false;
        auto publish_enable = [&](bool enabled) {
            LowCmd_ status{};
            for (std::size_t i = 0; i < kJointCount; ++i)
                status.motor_cmd()[i].mode() = enabled ? 1 : 0;
            status.crc() = CommandCrc(status);
            enable_publisher_.Write(status);
        };
        auto release = [&]() {
            if (!publishing) return;
            // MuJoCo caches the last command and ignores motor.mode.
            for (auto& motor : command.motor_cmd()) {
                motor.mode() = 0;
                motor.kp() = motor.kd() = motor.tau() = motor.dq() = 0;
            }
            command.crc() = CommandCrc(command);
            publisher_.Write(command);
            publishing = false;
        };
        int result = 0;
        while (!stop_requested) {
            if (!Snapshot(state) || state.mode_machine() != command.mode_machine()) {
                std::cerr << "Lowstate stale/invalid or robot mode changed; stopping.\n";
                result = 1;
                break;
            }
            char key = 0;
            try {
                key = keyboard.Read();
            } catch (const std::exception& error) {
                std::cerr << error.what() << '\n';
                result = 1;
                break;
            }
            if (key == ' ' || key == 'q' || key == 'Q') break;
            if (key == 'd' || key == 'D') {
                release();
                output_enabled = false;
                std::cout << "Command output DISABLED; targets only, no publishing.\n";
            }
            if ((key == 'e' || key == 'E') && !output_enabled) {
                output_enabled = true;
                if (active) {
                    // Never jump to a target that finished while output was off.
                    for (std::size_t i = 0; i < kJointCount; ++i)
                        start_angles[i] = state.motor_state()[i].q();
                    start = Clock::now();
                    holding = false;
                }
                std::cout << "Command output ENABLED."
                          << (active ? " Moving from current angles.\n" : " Press R to set the reset target.\n");
            }
            if (key == 'r' || key == 'R') {
                for (std::size_t i = 0; i < kJointCount; ++i)
                    start_angles[i] = state.motor_state()[i].q();
                start = Clock::now();
                active = true;
                holding = false;
                std::cout << "Reset target generated from current joint angles."
                          << (output_enabled ? " Publishing enabled.\n" : " Output disabled; targets only.\n");
            }
            publish_enable(output_enabled);
            if (!active) {
                next += kPeriod;
                if (next < Clock::now()) next = Clock::now() + kPeriod;
                std::this_thread::sleep_until(next);
                continue;  // do not publish commands until R is pressed
            }
            const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            const double u = std::clamp(elapsed / duration, 0.0, 1.0);
            const double blend = u * u * (3.0 - 2.0 * u);
            for (std::size_t i = 0; i < kJointCount; ++i) {
                auto& motor = command.motor_cmd()[i];
                motor.mode() = 1;
                motor.q() = u == 1.0 ? config_.default_pos[i] :
                    static_cast<float>(start_angles[i] +
                        blend * (config_.default_pos[i] - start_angles[i]));
                motor.dq() = 0;
                motor.tau() = 0;
                motor.kp() = config_.kp[i];
                motor.kd() = config_.kd[i];
            }
            command.crc() = CommandCrc(command);
            // Telemetry only: the simulator does not subscribe to this topic.
            target_publisher_.Write(command);
            if (output_enabled) {
                publisher_.Write(command);
                publishing = true;
            }
            if (u == 1.0 && !holding) {
                std::cout << "Target interpolation complete."
                          << (output_enabled ? " Holding default pose.\n" : " Output disabled; target only.\n");
                holding = true;
            }
            next += kPeriod;
            if (next < Clock::now()) next = Clock::now() + kPeriod;
            std::this_thread::sleep_until(next);
        }
        release();
        publish_enable(false);
        return result;
    }

private:
    void OnState(const void* message) {
        const auto& state = *static_cast<const LowState_*>(message);
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = false;
        for (std::size_t i = 0; i < kJointCount; ++i)
            if (!std::isfinite(state.motor_state()[i].q())) return;
        state_ = state;
        received_at_ = Clock::now();
        valid_ = true;
    }
    bool Snapshot(LowState_& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_ || Clock::now() - received_at_ > std::chrono::milliseconds(500))
            return false;
        state = state_;
        return true;
    }

    const ResetConfig config_;
    std::mutex mutex_;
    LowState_ state_{};
    Clock::time_point received_at_{};
    bool valid_ = false;
    unitree::robot::ChannelPublisher<LowCmd_> publisher_;
    unitree::robot::ChannelPublisher<LowCmd_> target_publisher_;
    unitree::robot::ChannelPublisher<LowCmd_> enable_publisher_;
    unitree::robot::ChannelSubscriber<LowState_> subscriber_;
};

int main(int argc, char** argv) {
    if (argc > 5 || (argc > 1 && std::string(argv[1]) == "--help")) {
        std::cout << "Usage: " << argv[0] << " [reset_seconds=3] [interface=lo] [domain=1] [config_xml]\n"
                  << "MuJoCo G1 29DOF only; domain must be positive.\n";
        return argc > 5 ? 1 : 0;
    }
    try {
        std::size_t consumed = 0;
        const std::string duration_text = argc > 1 ? argv[1] : "3";
        const double duration = std::stod(duration_text, &consumed);
        if (consumed != duration_text.size() || !std::isfinite(duration) || duration <= 0)
            throw std::invalid_argument("reset_seconds must be finite and positive");
        const std::string domain_text = argc > 3 ? argv[3] : "1";
        const int domain = std::stoi(domain_text, &consumed);
        if (consumed != domain_text.size() || domain <= 0)
            throw std::invalid_argument("simulation domain must be positive");
        const std::string config_path = argc > 4 ? argv[4] : G1_RESET_DEFAULT_CONFIG;
        const auto config = LoadConfig(config_path);
        std::cout << "Loaded reset configuration: " << config_path << '\n';
        std::signal(SIGINT, Stop);
        std::signal(SIGTERM, Stop);
        unitree::robot::ChannelFactory::Instance()->Init(domain, argc > 2 ? argv[2] : "lo");
        JointReset controller(config);
        return controller.Run(duration);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
