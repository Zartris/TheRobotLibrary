#pragma once
#include <common/types.hpp>
#include <simulation/bridge/model_adapter.hpp>
#include <mujoco/mujoco.h>

namespace robotlib::sim {

class ActuatorAdapter {
public:
    static void applyTwist(const mjModel* m, mjData* d, const Twist& twist,
                           const SimVehicleParams& params);
};

}  // namespace robotlib::sim
