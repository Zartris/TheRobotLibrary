#include <common/kinematics/ackermann.hpp>
#include <common/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace robotlib {

Ackermann::Ackermann(double wheelbase, double maxSteeringAngle)
    : m_wheelbase(wheelbase), m_maxSteeringAngle(maxSteeringAngle),
      m_logger(getLogger("common.ackermann")) {
    std::ostringstream oss;
    oss << "Ackermann initialized: wheelbase=" << wheelbase
        << " maxSteeringAngle=" << maxSteeringAngle;
    m_logger->debug(oss.str());
}

Pose2D Ackermann::step(const Pose2D& state, const Twist& control, double dt) {
    const double v = control.linear;
    const double delta = std::clamp(control.angular, -m_maxSteeringAngle, m_maxSteeringAngle);

    Pose2D result;
    if (std::abs(delta) < 1e-6 || std::abs(v) < 1e-9) {
        // Straight-line motion
        result.x = state.x + v * std::cos(state.theta) * dt;
        result.y = state.y + v * std::sin(state.theta) * dt;
        result.theta = state.theta;
    } else {
        // Bicycle model kinematics: arc integration
        const double omega = v * std::tan(delta) / m_wheelbase;
        const double r = v / omega;
        result.theta = normalizeAngle(state.theta + omega * dt);
        result.x = state.x + r * (std::sin(result.theta) - std::sin(state.theta));
        result.y = state.y - r * (std::cos(result.theta) - std::cos(state.theta));
    }
    return result;
}

ControlLimits Ackermann::getControlLimits() const {
    return ControlLimits{1.0, m_maxSteeringAngle, 0.5, 1.0};
}

Twist Ackermann::toTwist(const Eigen::Vector2d& wheelSpeeds) const {
    // wheelSpeeds(0) = speed, wheelSpeeds(1) = steering_angle
    return Twist{wheelSpeeds(0), wheelSpeeds(1)};
}

Eigen::Vector2d Ackermann::fromTwist(const Twist& twist) const {
    // twist.linear = speed, twist.angular = steering_angle
    return Eigen::Vector2d{twist.linear, twist.angular};
}

}  // namespace robotlib
