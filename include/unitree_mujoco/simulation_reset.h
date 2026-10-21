#pragma once
#include <cstdint>

namespace unitree_mujoco {
// Simulation-only LowState reserve[0:2]: marker and reset generation.
constexpr std::uint32_t kSimulationResetMarker = 0x47315253;
}
