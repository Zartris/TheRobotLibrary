#include <occupancy_grid/occupancy_grid_map.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sstream>

namespace robotlib {

OccupancyGridMap::OccupancyGridMap(int width, int height, double resolution,
                                     const OccupancyGridConfig& config)
    : m_grid(width, height, resolution), m_config(config),
      m_logger(getLogger("occupancy_grid")) {
    std::ostringstream oss;
    oss << "OccupancyGridMap initialized: " << width << "x" << height
        << " resolution=" << resolution;
    m_logger->debug(oss.str());
}

void OccupancyGridMap::markRay(int x0, int y0, int x1, int y1) {
    // Bresenham's line algorithm for marking free cells along the ray
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int cx = x0;
    int cy = y0;

    while (cx != x1 || cy != y1) {
        if (m_grid.isValid(cx, cy)) {
            // Mark as free
            auto& cell = m_grid.at(cx, cy);
            cell = static_cast<int8_t>(std::clamp(
                static_cast<int>(cell) + m_config.logOddsMiss,
                m_config.logOddsMin, m_config.logOddsMax));
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            cx += sx;
        }
        if (e2 < dx) {
            err += dx;
            cy += sy;
        }
    }
}

void OccupancyGridMap::updateFromScan(const Pose2D& robotPose, const LaserScan& scan) {
    auto start = std::chrono::high_resolution_clock::now();

    auto [rx, ry] = m_grid.toGrid(robotPose.x, robotPose.y);

    for (int i = 0; i < scan.numRays(); ++i) {
        float range = scan.ranges[i];
        if (std::isnan(range) || range < scan.rangeMin || range > scan.rangeMax) continue;

        double angle = robotPose.theta + scan.angleAt(i);
        double hitX = robotPose.x + range * std::cos(angle);
        double hitY = robotPose.y + range * std::sin(angle);

        auto [hx, hy] = m_grid.toGrid(hitX, hitY);

        // Mark cells along the ray as free
        markRay(rx, ry, hx, hy);

        // Mark endpoint as occupied
        if (m_grid.isValid(hx, hy)) {
            auto& cell = m_grid.at(hx, hy);
            cell = static_cast<int8_t>(std::clamp(
                static_cast<int>(cell) + m_config.logOddsHit,
                m_config.logOddsMin, m_config.logOddsMax));
        }
    }

    ++m_updateCount;

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "updateFromScan #" << m_updateCount << ": " << scan.numRays() << " rays in " << us << " us";
    m_logger->debug(oss.str());
}

void OccupancyGridMap::reset() {
    std::fill(m_grid.cells.begin(), m_grid.cells.end(), 0);
    m_updateCount = 0;
    m_logger->debug("OccupancyGridMap reset");
}

}  // namespace robotlib
