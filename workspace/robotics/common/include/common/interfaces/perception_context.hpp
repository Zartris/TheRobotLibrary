#pragma once
#include <common/laser_scan.hpp>
#include <common/occupancy_grid.hpp>
#include <vector>

namespace robotlib {

struct TrackedObstacle {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double radius{0.5};
    int trackId{-1};  ///< -1 = unassociated; stable ID assigned by M4 tracker
};

/// Forward-compatible perception bundle for local planners.
/// Introduced in M2 to allow M4 to add obstacle tracking without
/// breaking the ILocalPlanner interface.
///
/// Pre-conditions:
///   - scan and grid must be consistent (same coordinate frame)
///   - trackedObstacles is empty until M4 populates it
/// Post-conditions: none (read-only input bundle)
struct PerceptionContext {
    LaserScan scan;
    OccupancyGrid grid;
    std::vector<TrackedObstacle> trackedObstacles;  ///< Empty until M4
};

}  // namespace robotlib
