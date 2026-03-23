#include <dwa/dwa_planner.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <limits>

namespace robotlib {

DWAPlanner::DWAPlanner(const DWAConfig& config)
    : m_config(config), m_logger(getLogger("dwa")) {
    m_logger->debug("DWAPlanner initialized");
}

DWAPlanner::Trajectory DWAPlanner::simulateTrajectory(const Pose2D& startPose,
                                                        const Twist& vel) const {
    Trajectory traj;
    traj.velocity = vel;
    Pose2D p = startPose;
    traj.poses.push_back(p);

    const int steps = static_cast<int>(m_config.simTime / m_config.simDt);
    for (int i = 0; i < steps; ++i) {
        if (std::abs(vel.angular) < 1e-6) {
            p.x += vel.linear * std::cos(p.theta) * m_config.simDt;
            p.y += vel.linear * std::sin(p.theta) * m_config.simDt;
        } else {
            const double r = vel.linear / vel.angular;
            p.x += r * (std::sin(p.theta + vel.angular * m_config.simDt) - std::sin(p.theta));
            p.y += -r * (std::cos(p.theta + vel.angular * m_config.simDt) - std::cos(p.theta));
            p.theta = normalizeAngle(p.theta + vel.angular * m_config.simDt);
        }
        traj.poses.push_back(p);
    }
    return traj;
}

double DWAPlanner::minObstacleDistance(const Trajectory& traj, const LaserScan& scan,
                                        const Pose2D& robotPose) const {
    double minDist = std::numeric_limits<double>::max();
    // Convert scan to obstacle points
    for (int i = 0; i < scan.numRays(); ++i) {
        if (std::isnan(scan.ranges[i]) || scan.ranges[i] >= scan.rangeMax) continue;
        double angle = robotPose.theta + scan.angleAt(i);
        double ox = robotPose.x + scan.ranges[i] * std::cos(angle);
        double oy = robotPose.y + scan.ranges[i] * std::sin(angle);

        for (const auto& p : traj.poses) {
            double d = distance(p.x, p.y, ox, oy);
            minDist = std::min(minDist, d);
        }
    }
    return minDist;
}

double DWAPlanner::scoreCost(const Trajectory& traj, const Path& globalPath,
                              const LaserScan& scan, const Pose2D& robotPose) const {
    if (traj.poses.empty()) return std::numeric_limits<double>::max();

    // Goal distance cost - distance from trajectory end to nearest point on global path
    const auto& endPose = traj.poses.back();
    double goalDist = std::numeric_limits<double>::max();
    for (const auto& wp : globalPath) {
        double d = distance(endPose.x, endPose.y, wp.x, wp.y);
        goalDist = std::min(goalDist, d);
    }

    // Obstacle cost
    double obsDist = minObstacleDistance(traj, scan, robotPose);
    double obsCost = 0.0;
    if (obsDist < m_config.obstacleThreshold) {
        return std::numeric_limits<double>::max();  // collision
    }
    obsCost = 1.0 / (obsDist + 0.01);

    // Velocity cost (prefer higher speeds)
    double velCost = m_config.maxLinearVel - std::abs(traj.velocity.linear);

    return m_config.goalWeight * goalDist +
           m_config.obstacleWeight * obsCost +
           m_config.velocityWeight * velCost;
}

Twist DWAPlanner::compute(const Pose2D& pose, const Twist& vel,
                           const Path& path, const PerceptionContext& ctx) {
    auto startTime = std::chrono::high_resolution_clock::now();

    if (path.empty()) {
        m_logger->debug("DWA: empty global path, returning zero twist");
        return Twist{0.0, 0.0};
    }

    // Dynamic window
    double minV = std::max(-m_config.maxLinearVel,
                           vel.linear - m_config.linearAccel * m_config.simDt);
    double maxV = std::min(m_config.maxLinearVel,
                           vel.linear + m_config.linearAccel * m_config.simDt);
    double minW = std::max(-m_config.maxAngularVel,
                           vel.angular - m_config.angularAccel * m_config.simDt);
    double maxW = std::min(m_config.maxAngularVel,
                           vel.angular + m_config.angularAccel * m_config.simDt);

    Twist bestCmd{0.0, 0.0};
    double bestCost = std::numeric_limits<double>::max();

    const double dv = (maxV - minV) / std::max(1, m_config.linearSamples - 1);
    const double dw = (maxW - minW) / std::max(1, m_config.angularSamples - 1);

    for (int vi = 0; vi < m_config.linearSamples; ++vi) {
        for (int wi = 0; wi < m_config.angularSamples; ++wi) {
            Twist candidateVel{minV + vi * dv, minW + wi * dw};
            auto traj = simulateTrajectory(pose, candidateVel);
            double cost = scoreCost(traj, path, ctx.scan, pose);

            if (cost < bestCost) {
                bestCost = cost;
                bestCmd = candidateVel;
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    std::ostringstream oss;
    oss << "DWA compute: " << us << " us, best v=" << bestCmd.linear << " w=" << bestCmd.angular;
    m_logger->trace(oss.str());

    return bestCmd;
}

}  // namespace robotlib
