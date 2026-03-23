#pragma once
#include <common/types.hpp>
#include <common/occupancy_grid.hpp>
#include <common/laser_scan.hpp>

namespace robotlib {

class ILocalPlanner {
public:
    virtual ~ILocalPlanner() = default;
    virtual Twist compute(const Pose2D& pose, const Twist& currentVelocity,
                          const Path& globalPath, const LaserScan& scan,
                          const OccupancyGrid& grid) = 0;
};

}  // namespace robotlib
