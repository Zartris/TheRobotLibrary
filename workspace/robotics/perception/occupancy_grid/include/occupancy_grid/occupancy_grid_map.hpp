#pragma once
#include <common/laser_scan.hpp>
#include <common/occupancy_grid.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <memory>

namespace robotlib {

struct OccupancyGridConfig {
    int logOddsHit{3};
    int logOddsMiss{-1};
    int logOddsMax{100};
    int logOddsMin{-100};
};

class OccupancyGridMap {
public:
    OccupancyGridMap(int width, int height, double resolution,
                     const OccupancyGridConfig& config = {});

    void updateFromScan(const Pose2D& robotPose, const LaserScan& scan);
    void reset();

    /// Create an inflated copy of the grid. Occupied cells are expanded by the given
    /// radius (in metres). The original grid is not modified.
    /// @param radiusMetres Inflation radius in metres (typically robot radius)
    /// @return New OccupancyGrid with inflated obstacles
    OccupancyGrid inflate(double radiusMetres) const;

    const OccupancyGrid& grid() const { return m_grid; }
    OccupancyGrid& grid() { return m_grid; }

private:
    void markRay(int x0, int y0, int x1, int y1);

    OccupancyGrid m_grid;
    OccupancyGridConfig m_config;
    std::shared_ptr<ILogger> m_logger;
    int m_updateCount{0};
};

}  // namespace robotlib
