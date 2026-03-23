#include <simulation/bridge/actuator_adapter.hpp>

namespace robotlib::sim {

void ActuatorAdapter::applyTwist(const mjModel* m, mjData* d, const Twist& twist,
                                  const SimVehicleParams& params) {
    if (params.wheelRadius <= 0.0) return;

    // Convert twist to wheel velocities
    double vl = (twist.linear - twist.angular * params.trackWidth / 2.0) / params.wheelRadius;
    double vr = (twist.linear + twist.angular * params.trackWidth / 2.0) / params.wheelRadius;

    // Apply to the extracted actuator indices
    if (params.leftWheelActuator < m->nu) {
        d->ctrl[params.leftWheelActuator] = vl;
    }
    if (params.rightWheelActuator < m->nu) {
        d->ctrl[params.rightWheelActuator] = vr;
    }
}

}  // namespace robotlib::sim
