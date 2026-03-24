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
        // range >= rangeMax means "no obstacle detected" — mark free cells but skip occupied endpoint
        if (std::isnan(range) || range < scan.rangeMin) continue;
        bool hasHit = (range < scan.rangeMax);

        double angle = robotPose.theta + scan.angleAt(i);
        double hitX = robotPose.x + range * std::cos(angle);
        double hitY = robotPose.y + range * std::sin(angle);

        auto [hx, hy] = m_grid.toGrid(hitX, hitY);

        // Mark cells along the ray as free
        markRay(rx, ry, hx, hy);

        // Only mark endpoint as occupied when there was an actual hit
        if (hasHit && m_grid.isValid(hx, hy)) {
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
    m_logger->trace(oss.str());
}

void OccupancyGridMap::reset() {
    std::fill(m_grid.cells.begin(), m_grid.cells.end(), 0);
    m_updateCount = 0;
    m_logger->debug("OccupancyGridMap reset");
}

OccupancyGrid OccupancyGridMap::inflate(double radiusMetres) const {
    auto start = std::chrono::high_resolution_clock::now();

    double radius = std::max(0.0, radiusMetres);
    OccupancyGrid inflated = m_grid;  // copy
    int radiusCells = static_cast<int>(std::ceil(radius / m_grid.resolution));

    // For each occupied cell, mark cells within radius as occupied
    for (int y = 0; y < m_grid.height; ++y) {
        for (int x = 0; x < m_grid.width; ++x) {
            if (m_grid.cells[y * m_grid.width + x] <= 0) continue;  // not occupied

            // Inflate in a square region, check circular distance
            for (int dy = -radiusCells; dy <= radiusCells; ++dy) {
                for (int dx = -radiusCells; dx <= radiusCells; ++dx) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (!inflated.isValid(nx, ny)) continue;

                    // Check if within circular radius
                    double dist =
                        std::sqrt(static_cast<double>(dx * dx + dy * dy)) * m_grid.resolution;
                    if (dist <= radiusMetres) {
                        int idx = ny * inflated.width + nx;
                        if (inflated.cells[idx] <= 0) {
                            inflated.cells[idx] = 1;  // mark as occupied (inflation)
                        }
                    }
                }
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "Inflate: " << us << " us, radius=" << radius << "m (" << radiusCells
        << " cells)";
    m_logger->trace(oss.str());

    return inflated;
}

}  // namespace robotlib
