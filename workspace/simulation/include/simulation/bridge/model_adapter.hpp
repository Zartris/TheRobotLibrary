#pragma once
#include <common/types.hpp>
#include <mujoco/mujoco.h>
#include <string>

namespace robotlib::sim {

struct SimVehicleParams {
    double wheelRadius{0.05};
    double trackWidth{0.3};
    double mass{5.0};
    int bodyId{1};
    int leftWheelActuator{0};
    int rightWheelActuator{1};
};

class ModelAdapter {
public:
    static SimVehicleParams extractVehicleParams(const mjModel* m);
};

}  // namespace robotlib::sim
