#pragma once
#include <common/types.hpp>
#include <common/occupancy_grid.hpp>
#include <optional>

namespace robotlib {

/// Interface for global path planners operating on an occupancy grid.
///
/// A global planner computes a collision-free path from start to goal.
///
/// Pre-conditions:
///   - grid must be initialized (non-zero resolution and dimensions)
///   - start and goal must be within the grid bounds
///
/// Post-conditions:
///   - Returns std::nullopt if no path exists or the goal is unreachable
///   - Returned path (if present) is non-empty and ends at the goal pose
///
/// Thread safety: not thread-safe; do not call concurrently on the same instance.
class IGlobalPlanner {
public:
    virtual ~IGlobalPlanner() = default;
    virtual std::optional<Path> plan(const Pose2D& start, const Pose2D& goal, const OccupancyGrid& grid) = 0;
};

}  // namespace robotlib
