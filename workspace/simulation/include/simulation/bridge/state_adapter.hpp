#pragma once
#include <common/types.hpp>
#include <mujoco/mujoco.h>

namespace robotlib::sim {

class StateAdapter {
public:
    static Pose2D extractPose2D(const mjModel* m, const mjData* d, int bodyId = 1);
    static Twist extractTwist(const mjModel* m, const mjData* d, int bodyId = 1);
};

}  // namespace robotlib::sim
