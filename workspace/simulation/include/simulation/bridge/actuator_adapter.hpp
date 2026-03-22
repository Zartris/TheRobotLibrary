#pragma once
#include <common/types.hpp>
#include <mujoco/mujoco.h>

namespace robotlib::sim {

class ActuatorAdapter {
public:
    static void applyTwist(const mjModel* m, mjData* d, const Twist& twist,
                           double wheelRadius, double trackWidth);
};

}  // namespace robotlib::sim
