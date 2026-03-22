#include <simulation/bridge/actuator_adapter.hpp>

namespace robotlib::sim {

void ActuatorAdapter::applyTwist(const mjModel* m, mjData* d, const Twist& twist,
                                  double wheelRadius, double trackWidth) {
    // Convert twist to wheel velocities
    double vl = (twist.linear - twist.angular * trackWidth / 2.0) / wheelRadius;
    double vr = (twist.linear + twist.angular * trackWidth / 2.0) / wheelRadius;

    // Apply to first two actuators (left, right wheel)
    if (m->nu >= 2) {
        d->ctrl[0] = vl;
        d->ctrl[1] = vr;
    }
}

}  // namespace robotlib::sim
