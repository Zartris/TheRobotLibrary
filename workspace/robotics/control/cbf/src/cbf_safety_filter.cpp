#include <cbf/cbf_safety_filter.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>

namespace robotlib {

CbfSafetyFilter::CbfSafetyFilter(std::unique_ptr<IController> nominalController,
                                   const CbfConfig& config)
    : m_nominal(std::move(nominalController)), m_config(config),
      m_logger(getLogger("cbf")) {
    if (!m_nominal) {
        m_logger->error("CbfSafetyFilter: null nominal controller");
    }
    m_logger->debug("CbfSafetyFilter initialized");
}

void CbfSafetyFilter::setObstacles(const std::vector<Obstacle2D>& obstacles) {
    m_obstacles = obstacles;
}

void CbfSafetyFilter::reset() {
    m_obstacles.clear();
    if (m_nominal) m_nominal->reset();
    m_logger->debug("CbfSafetyFilter reset");
}

Twist CbfSafetyFilter::compute(const Pose2D& current, const Pose2D& target, double dt) {
    auto start = std::chrono::high_resolution_clock::now();

    if (!m_nominal) {
        m_logger->error("CBF: no nominal controller set");
        return {0.0, 0.0};
    }

    // Get nominal control from wrapped controller
    Twist nominal = m_nominal->compute(current, target, dt);

    // If no obstacles or zero safety radius, pass through
    if (m_obstacles.empty() || m_config.safetyRadius <= 0.0) {
        auto end = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::ostringstream oss;
        oss << "CBF compute: " << us << " us (passthrough, no obstacles)";
        m_logger->trace(oss.str());
        return nominal;
    }

    Twist safe = projectToSafe(current, nominal);

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "CBF compute: " << us << " us, modified=" << (safe.linear != nominal.linear || safe.angular != nominal.angular);
    m_logger->trace(oss.str());

    return safe;
}

Twist CbfSafetyFilter::projectToSafe(const Pose2D& current, const Twist& nominal) const {
    // CBF-QP: minimize ||u - u_nom||^2 subject to CBF constraints
    //
    // For each obstacle i:
    //   h_i(x) = ||p - p_i||^2 - (r_safe + r_obs_i)^2
    //   h_dot_i = 2(p - p_i)^T * p_dot >= -alpha * h_i
    //
    // p_dot = [v*cos(theta), v*sin(theta)]
    //
    // The constraint is linear in v (angular velocity omega doesn't directly
    // affect p_dot for a point robot, so the constraint is on v only).
    //
    // For simplicity, we solve a 1D constrained optimization:
    // Find v closest to v_nom such that all CBF constraints are satisfied.
    // Omega passes through unchanged (CBF on position only).

    double omega = nominal.angular;
    double cosTheta = std::cos(current.theta);
    double sinTheta = std::sin(current.theta);

    // Track feasible interval [vLower, vUpper] across all obstacle constraints
    double vLower = -m_config.maxLinearVelocity;
    double vUpper = m_config.maxLinearVelocity;

    for (const auto& obs : m_obstacles) {
        double dx = current.x - obs.x;
        double dy = current.y - obs.y;
        double dist2 = dx * dx + dy * dy;
        double rTotal = m_config.safetyRadius + obs.radius;
        double h = dist2 - rTotal * rTotal;

        // Gradient of h w.r.t. position: [2*dx, 2*dy]
        // p_dot = v * [cos(theta), sin(theta)]
        // h_dot = 2*dx*v*cos(theta) + 2*dy*v*sin(theta)
        // Constraint: h_dot >= -alpha * h
        // => 2*(dx*cos + dy*sin) * v >= -alpha * h

        double grad_dot_heading = 2.0 * (dx * cosTheta + dy * sinTheta);

        if (std::abs(grad_dot_heading) < 1e-9) {
            // Heading is perpendicular to obstacle direction — no constraint on v
            continue;
        }

        // Constraint: grad_dot_heading * v >= -alpha * h
        double rhs = -m_config.alpha * h;

        if (grad_dot_heading > 0) {
            // v >= rhs / grad_dot_heading
            double vMin = rhs / grad_dot_heading;
            vLower = std::max(vLower, vMin);
        } else {
            // v <= rhs / grad_dot_heading (flipped inequality)
            double vMax = rhs / grad_dot_heading;
            vUpper = std::min(vUpper, vMax);
        }
    }

    // Infeasibility detection: if no valid v exists, stop the robot
    double v;
    if (vLower > vUpper) {
        m_logger->warn("CBF: constraints infeasible — stopping robot");
        v = 0.0;
    } else {
        v = std::clamp(nominal.linear, vLower, vUpper);
    }

    omega = std::clamp(omega, -m_config.maxAngularVelocity, m_config.maxAngularVelocity);

    return {v, omega};
}

}  // namespace robotlib
