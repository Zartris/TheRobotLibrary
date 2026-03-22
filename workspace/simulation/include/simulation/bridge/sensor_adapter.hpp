#pragma once
#include <common/laser_scan.hpp>
#include <common/types.hpp>
#include <mujoco/mujoco.h>
#include <vector>

namespace robotlib::sim {

struct SimLidarConfig {
    int numRays{180};
    double angleMin{-M_PI};
    double angleMax{M_PI};
    double rangeMax{10.0};
};

class SensorAdapter {
public:
    static LaserScan generateLidar(const mjModel* m, const mjData* d,
                                    int bodyId, const SimLidarConfig& config);
};

}  // namespace robotlib::sim
