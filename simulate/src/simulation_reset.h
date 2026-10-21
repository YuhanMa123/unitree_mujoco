#pragma once
#include <mujoco/mujoco.h>
#include <unitree_mujoco/simulation_reset.h>

namespace unitree_mujoco {
// Access under the same simulation mutex as physics and the DDS bridge.
inline std::uint32_t simulation_reset_generation = 0;
inline void ResetSimulation(const mjModel* model, mjData* data) {
    mj_resetData(model, data);
    mj_forward(model, data);
    ++simulation_reset_generation;
}
}
