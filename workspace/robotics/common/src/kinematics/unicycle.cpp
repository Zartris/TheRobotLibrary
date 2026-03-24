#include <common/kinematics/unicycle.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <sstream>

namespace robotlib {

Unicycle::Unicycle(double maxSpeed, double maxOmega)
    : m_maxSpeed(maxSpeed), m_maxOmega(maxOmega),
      m_logger(getLogger("common.unicycle")) {
    std::ostringstream oss;
    oss << "Unicycle initialized: maxSpeed=" << maxSpeed
        << " maxOmega=" << maxOmega;
    m_logger->debug(oss.str());
}

Pose2D Unicycle::step(const Pose2D& state, const Twist& control, double dt) {
    Pose2D result;
    if (std::abs(control.angular) < 1e-6) {
        result.x = state.x + control.linear * std::cos(state.theta) * dt;
        result.y = state.y + control.linear * std::sin(state.theta) * dt;
        result.theta = state.theta;
    } else {
        const double r = control.linear / control.angular;
        result.theta = normalizeAngle(state.theta + control.angular * dt);
        result.x = state.x + r * (std::sin(result.theta) - std::sin(state.theta));
        result.y = state.y - r * (std::cos(result.theta) - std::cos(state.theta));
    }
    return result;
}

ControlLimits Unicycle::getControlLimits() const {
    return ControlLimits{m_maxSpeed, m_maxOmega, 0.5, 1.0};
}

Twist Unicycle::toTwist(const Eigen::Vector2d& wheelSpeeds) const {
    return Twist{wheelSpeeds(0), wheelSpeeds(1)};
}

Eigen::Vector2d Unicycle::fromTwist(const Twist& twist) const {
    return Eigen::Vector2d{twist.linear, twist.angular};
}

}  // namespace robotlib
