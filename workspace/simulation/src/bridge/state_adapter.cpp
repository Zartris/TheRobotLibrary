#include <simulation/bridge/state_adapter.hpp>
#include <common/geometry.hpp>
#include <cmath>

namespace robotlib::sim {

Pose2D StateAdapter::extractPose2D(const mjModel* m, const mjData* d, int bodyId) {
    if (bodyId < 0 || bodyId >= m->nbody) {
        return Pose2D{};
    }
    // MuJoCo stores body positions in d->xpos (3 per body)
    // and quaternions in d->xquat (4 per body)
    const double* pos = d->xpos + 3 * bodyId;
    const double* quat = d->xquat + 4 * bodyId;

    // Extract yaw from quaternion (w, x, y, z format in MuJoCo)
    double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    return Pose2D{pos[0], pos[1], yaw};
}

Twist StateAdapter::extractTwist(const mjModel* m, const mjData* d, int bodyId) {
    if (bodyId < 0 || bodyId >= m->nbody) {
        return Twist{};
    }
    // Body velocities in d->cvel (6 per body: 3 angular + 3 linear)
    const double* vel = d->cvel + 6 * bodyId;
    // vel[0..2] = angular velocity, vel[3..5] = linear velocity
    double vx = vel[3];
    double vy = vel[4];
    double linear = std::sqrt(vx * vx + vy * vy);
    double angular = vel[2];  // rotation about z
    return Twist{linear, angular};
}

}  // namespace robotlib::sim
