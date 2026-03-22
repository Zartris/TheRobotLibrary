#include <ray_casting/ray_caster.hpp>
#include <cmath>
#include <sstream>
#include <chrono>

namespace robotlib {

RayCaster::RayCaster() : m_logger(getLogger("ray_casting")) {
    m_logger->debug("RayCaster initialized");
}

RayCastResult RayCaster::castRay(const OccupancyGrid& grid, const Point2D& origin,
                                  double angle, double maxRange) const {
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);

    // Bresenham-style DDA ray traversal
    auto [gx, gy] = grid.toGrid(origin.x, origin.y);
    const double stepSize = grid.resolution * 0.5;
    const int maxSteps = static_cast<int>(maxRange / stepSize) + 1;

    for (int i = 0; i < maxSteps; ++i) {
        const double dist = i * stepSize;
        const double wx = origin.x + dx * dist;
        const double wy = origin.y + dy * dist;
        auto [cx, cy] = grid.toGrid(wx, wy);

        if (!grid.isValid(cx, cy)) {
            return RayCastResult{false, maxRange, {origin.x + dx * maxRange, origin.y + dy * maxRange}};
        }

        if (grid.cellState(cx, cy) == CellState::Occupied) {
            return RayCastResult{true, dist, {wx, wy}};
        }
    }

    return RayCastResult{false, maxRange, {origin.x + dx * maxRange, origin.y + dy * maxRange}};
}

LaserScan RayCaster::castScan(const OccupancyGrid& grid, const Pose2D& pose,
                               const ScanConfig& config) const {
    auto start = std::chrono::high_resolution_clock::now();

    LaserScan scan;
    scan.angleMin = config.angleMin;
    scan.angleMax = config.angleMax;
    scan.angleIncrement = config.angleIncrement;
    scan.rangeMax = config.maxRange;
    scan.rangeMin = 0.0;

    const int numRays = static_cast<int>((config.angleMax - config.angleMin) / config.angleIncrement) + 1;
    scan.ranges.resize(numRays);

    Point2D origin{pose.x, pose.y};

    for (int i = 0; i < numRays; ++i) {
        double angle = pose.theta + config.angleMin + i * config.angleIncrement;
        auto result = castRay(grid, origin, angle, config.maxRange);
        scan.ranges[i] = static_cast<float>(result.range);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "castScan: " << numRays << " rays in " << us << " us";
    m_logger->debug(oss.str());

    return scan;
}

}  // namespace robotlib
