#include <simulation/bridge/sensor_adapter.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace robotlib::sim {

LaserScan SensorAdapter::generateLidar(const mjModel* m, const mjData* d,
                                         int bodyId, const SimLidarConfig& config) {
    LaserScan scan;
    scan.angleMin = config.angleMin;
    scan.angleMax = config.angleMax;
    scan.rangeMin = 0.1;
    scan.rangeMax = config.rangeMax;
    scan.angleIncrement = (config.angleMax - config.angleMin) / std::max(1, config.numRays - 1);
    scan.ranges.resize(config.numRays);

    if (bodyId < 0 || bodyId >= m->nbody) {
        std::fill(scan.ranges.begin(), scan.ranges.end(),
                  static_cast<float>(config.rangeMax));
        return scan;
    }

    // Get robot position and orientation
    const double* pos = d->xpos + 3 * bodyId;
    const double* quat = d->xquat + 4 * bodyId;
    double w = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    double yaw = std::atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // Use mj_ray for each beam
    for (int i = 0; i < config.numRays; ++i) {
        double angle = yaw + config.angleMin + i * scan.angleIncrement;
        double pnt[3] = {pos[0], pos[1], pos[2]};
        double vec[3] = {std::cos(angle), std::sin(angle), 0.0};

        // Exclude the robot body from ray intersection
        mjtByte geomgroup[mjNGROUP] = {1, 1, 1, 1, 1, 1};
        int geomid = -1;
        mjtNum dist = mj_ray(m, d, pnt, vec, geomgroup, 1, bodyId, &geomid);

        if (dist >= 0 && dist <= config.rangeMax) {
            scan.ranges[i] = static_cast<float>(dist);
        } else {
            scan.ranges[i] = static_cast<float>(config.rangeMax);
        }
    }

    return scan;
}

}  // namespace robotlib::sim
