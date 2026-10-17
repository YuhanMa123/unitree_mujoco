#include <iomanip>
#include <iostream>
#include <thread>
#include <unitree_mujoco/gamepad.h>

int main(int argc, char** argv) {
    try {
        const bool once = argc > 1 && std::string(argv[1]) == "--once";
        auto config = unitree_mujoco::gamepad::Config::Load(G1_GAMEPAD_DEFAULT_CONFIG);
        config.enabled = true;
        unitree_mujoco::gamepad::Reader reader(config);
        bool connected = false;
        int last_mask = -1;
        std::array<float, 5> last_axes{};
        const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(3);
        std::cout << "Read-only Xbox monitor: " << config.device << "; Ctrl+C to exit.\n";
        for (;;) {
            const auto state = reader.Poll();
            if (state.connected != connected) {
                std::cout << (state.connected ? "Connected: " + reader.Name() : "Disconnected: " + reader.Error()) << std::endl;
                connected = state.connected;
            }
            if (state.connected) {
                const bool buttons[]{state.a, state.b, state.x, state.y, state.lb, state.rb, state.back,
                    state.start, state.guide, state.ls, state.rs, state.up, state.down, state.left_pad, state.right_pad};
                const char* names[]{"A", "B", "X", "Y", "LB", "RB", "View", "Menu", "Guide", "L3", "R3", "Up", "Down", "Left", "Right"};
                int mask = 0;
                for (int i=0; i<15; ++i) if (buttons[i]) mask |= 1<<i;
                const std::array<float, 5> axes{state.forward, state.left, state.turn, state.lt, state.rt};
                bool moved = mask != last_mask;
                for (int i=0; i<5; ++i) moved |= std::abs(axes[i]-last_axes[i]) > 0.02;
                if (moved || once) {
                    std::cout << std::fixed << std::setprecision(3) << "forward=" << axes[0] << " left=" << axes[1]
                              << " turn=" << axes[2] << " LT=" << axes[3] << " RT=" << axes[4] << " buttons:";
                    for (int i=0; i<15; ++i) if (buttons[i]) std::cout << ' ' << names[i];
                    std::cout << std::endl;
                    last_mask = mask; last_axes = axes;
                }
                if (once) return 0;
            } else if (once && std::chrono::steady_clock::now() >= deadline) {
                std::cerr << "Cannot read controller: " << reader.Error() << '\n'; return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
