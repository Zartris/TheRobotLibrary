#pragma once
#include <common/types.hpp>
#include <common/occupancy_grid.hpp>
#include <optional>

namespace robotlib {

class IGlobalPlanner {
public:
    virtual ~IGlobalPlanner() = default;
    virtual std::optional<Path> plan(const Pose2D& start, const Pose2D& goal, const OccupancyGrid& grid) = 0;
};

}  // namespace robotlib
