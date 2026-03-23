#include <pure_pursuit/pure_pursuit_controller.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>

namespace robotlib {

PurePursuitController::PurePursuitController(const PurePursuitConfig& config)
    : m_config(config), m_logger(getLogger("pure_pursuit")) {
    m_logger->debug("PurePursuitController initialized");
}

void PurePursuitController::setPath(const Path& path) {
    m_path = path;
    std::ostringstream oss;
    oss << "Path set: " << path.size() << " waypoints";
    m_logger->debug(oss.str());
}

void PurePursuitController::reset() {
    m_path.clear();
    m_lookaheadPoint = {};
    m_logger->debug("PurePursuitController reset");
}

Twist PurePursuitController::compute(const Pose2D& current, const Pose2D& target, double dt) {
    auto start = std::chrono::high_resolution_clock::now();

    // If no path, go directly to target (degenerate case)
    if (m_path.empty()) {
        double dx = target.x - current.x;
        double dy = target.y - current.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < m_config.goalTolerance) return {0.0, 0.0};

        double targetAngle = std::atan2(dy, dx);
        double headingError = normalizeAngle(targetAngle - current.theta);
        double curvature = 2.0 * std::sin(headingError) / std::max(dist, 0.01);
        double v = m_config.maxLinearVelocity * std::cos(headingError);
        v = std::clamp(v, 0.0, m_config.maxLinearVelocity);
        double omega = std::clamp(v * curvature, -m_config.maxAngularVelocity, m_config.maxAngularVelocity);

        // Log timing
        auto end = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::ostringstream oss;
        oss << "PurePursuit compute: " << us << " us (no path, direct to target)";
        m_logger->trace(oss.str());

        return {v, omega};
    }

    // Compute adaptive lookahead distance
    // (we don't have current speed from IController, so use base lookahead)
    double lookaheadDist = m_config.lookaheadDistance;

    // Find lookahead point
    m_lookaheadPoint = findLookaheadPoint(current, lookaheadDist);

    // Check if goal reached
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    if (std::sqrt(dx * dx + dy * dy) < m_config.goalTolerance) {
        return {0.0, 0.0};
    }

    // Compute curvature
    double curvature = computeCurvature(current, m_lookaheadPoint);

    // Compute velocity: slow down near tight turns
    double v = m_config.maxLinearVelocity;
    if (std::abs(curvature) > 0.01) {
        v = std::min(v, 1.0 / std::abs(curvature));  // limit v for tight turns
    }
    v = std::clamp(v, 0.0, m_config.maxLinearVelocity);

    double omega = v * curvature;
    omega = std::clamp(omega, -m_config.maxAngularVelocity, m_config.maxAngularVelocity);

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "PurePursuit compute: " << us << " us, curvature=" << curvature;
    m_logger->trace(oss.str());

    return {v, omega};
}

Pose2D PurePursuitController::findLookaheadPoint(const Pose2D& current, double lookaheadDist) const {
    if (m_path.empty()) return current;

    // Find the closest point on the path (starting from last known closest)
    double minDist = std::numeric_limits<double>::max();
    size_t closestIdx = 0;
    for (size_t i = 0; i < m_path.size(); ++i) {
        double dx = m_path[i].x - current.x;
        double dy = m_path[i].y - current.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < minDist) {
            minDist = d;
            closestIdx = i;
        }
    }

    // Walk forward along path until we find a point at lookahead distance
    for (size_t i = closestIdx; i < m_path.size(); ++i) {
        double dx = m_path[i].x - current.x;
        double dy = m_path[i].y - current.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d >= lookaheadDist) {
            return m_path[i];
        }
    }

    // If no point found at lookahead distance, use last point
    return m_path.back();
}

double PurePursuitController::computeCurvature(const Pose2D& current, const Pose2D& lookahead) const {
    // Transform lookahead to robot frame
    double dx = lookahead.x - current.x;
    double dy = lookahead.y - current.y;
    double localX = std::cos(current.theta) * dx + std::sin(current.theta) * dy;
    double localY = -std::sin(current.theta) * dx + std::cos(current.theta) * dy;

    double l2 = localX * localX + localY * localY;
    if (l2 < 1e-9) return 0.0;

    // Pure pursuit curvature: kappa = 2 * y / L^2
    return 2.0 * localY / l2;
}

}  // namespace robotlib
