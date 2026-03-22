#pragma once
#include <common/occupancy_grid.hpp>
#include <common/laser_scan.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <optional>

namespace robotlib {

struct RayCastResult {
    bool hit{false};
    double range{0.0};
    Point2D endpoint;
};

struct ScanConfig {
    double angleMin{-M_PI};
    double angleMax{M_PI};
    double angleIncrement{0.01};
    double maxRange{30.0};
};

class RayCaster {
public:
    explicit RayCaster();

    RayCastResult castRay(const OccupancyGrid& grid, const Point2D& origin,
                          double angle, double maxRange) const;

    LaserScan castScan(const OccupancyGrid& grid, const Pose2D& pose,
                       const ScanConfig& config) const;

private:
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
