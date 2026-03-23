#include <common/kinematics/differential_drive.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <sstream>

namespace robotlib {

DifferentialDrive::DifferentialDrive(double wheelRadius, double trackWidth)
    : m_wheelRadius(wheelRadius), m_trackWidth(trackWidth),
      m_logger(getLogger("common.differential_drive")) {
    std::ostringstream oss;
    oss << "DifferentialDrive initialized: wheelRadius=" << wheelRadius
        << " trackWidth=" << trackWidth;
    m_logger->debug(oss.str());
}

Pose2D DifferentialDrive::step(const Pose2D& state, const Twist& control, double dt) {
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

ControlLimits DifferentialDrive::getControlLimits() const {
    return ControlLimits{1.0, 2.0, 0.5, 1.0};
}

Twist DifferentialDrive::toTwist(const Eigen::Vector2d& wheelSpeeds) const {
    const double vl = wheelSpeeds(0) * m_wheelRadius;
    const double vr = wheelSpeeds(1) * m_wheelRadius;
    return Twist{(vr + vl) / 2.0, (vr - vl) / m_trackWidth};
}

Eigen::Vector2d DifferentialDrive::fromTwist(const Twist& twist) const {
    const double vl = (twist.linear - twist.angular * m_trackWidth / 2.0) / m_wheelRadius;
    const double vr = (twist.linear + twist.angular * m_trackWidth / 2.0) / m_wheelRadius;
    return Eigen::Vector2d{vl, vr};
}

}  // namespace robotlib
