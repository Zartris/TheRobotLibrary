#include <velocity_profiling/trapezoidal_profiler.hpp>
#include <common/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <chrono>

namespace robotlib {

TrapezoidalProfiler::TrapezoidalProfiler() : m_logger(getLogger("velocity_profiling")) {
    m_logger->debug("TrapezoidalProfiler initialized");
}

double TrapezoidalProfiler::curvatureAtIndex(const Path& path, size_t idx) const {
    if (idx == 0 || idx >= path.size() - 1) return 0.0;

    double dx1 = path[idx].x - path[idx - 1].x;
    double dy1 = path[idx].y - path[idx - 1].y;
    double dx2 = path[idx + 1].x - path[idx].x;
    double dy2 = path[idx + 1].y - path[idx].y;

    double cross = dx1 * dy2 - dy1 * dx2;
    double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    if (d1 < 1e-6 || d2 < 1e-6) return 0.0;
    return std::abs(cross) / (d1 * d2);
}

TimedPath TrapezoidalProfiler::profile(const Path& path,
                                        const VelocityConstraints& constraints) {
    auto startTime = std::chrono::high_resolution_clock::now();

    if (path.empty()) return {};
    if (path.size() == 1) {
        return {{path[0], 0.0, 0.0}};
    }

    // Compute cumulative distances
    std::vector<double> dist(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        dist[i] = dist[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    double totalDist = dist.back();

    // Compute max velocity at each point (reduced at turns)
    std::vector<double> maxVel(path.size(), constraints.maxVelocity);
    for (size_t i = 0; i < path.size(); ++i) {
        double curv = curvatureAtIndex(path, i);
        if (curv > 0.01) {
            maxVel[i] = std::min(maxVel[i], constraints.maxVelocity / (1.0 + 5.0 * curv));
        }
    }
    maxVel.front() = 0.0;
    maxVel.back() = 0.0;

    // Forward pass: accelerate
    std::vector<double> vel(path.size(), 0.0);
    vel[0] = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double ds = dist[i] - dist[i - 1];
        double vMax = std::sqrt(vel[i - 1] * vel[i - 1] + 2.0 * constraints.maxAcceleration * ds);
        vel[i] = std::min({vMax, maxVel[i], constraints.maxVelocity});
    }

    // Backward pass: decelerate
    vel.back() = 0.0;
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
        double ds = dist[i + 1] - dist[i];
        double vMax = std::sqrt(vel[i + 1] * vel[i + 1] + 2.0 * constraints.maxDeceleration * ds);
        vel[i] = std::min(vel[i], vMax);
    }

    // Compute times
    TimedPath result;
    result.reserve(path.size());
    double t = 0.0;
    result.push_back({path[0], 0.0, vel[0]});
    for (size_t i = 1; i < path.size(); ++i) {
        double ds = dist[i] - dist[i - 1];
        double avgVel = (vel[i] + vel[i - 1]) / 2.0;
        if (avgVel > 1e-6) {
            t += ds / avgVel;
        } else {
            t += ds / 0.01;
        }
        result.push_back({path[i], t, vel[i]});
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    std::ostringstream oss;
    oss << "TrapezoidalProfiler: " << path.size() << " waypoints, totalDist="
        << totalDist << " in " << us << " us";
    m_logger->trace(oss.str());

    return result;
}

}  // namespace robotlib
