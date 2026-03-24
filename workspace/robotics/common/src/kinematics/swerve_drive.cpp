#include <common/kinematics/swerve_drive.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <sstream>

namespace robotlib {

SwerveDrive::SwerveDrive(double trackWidth, double wheelBase)
    : m_trackWidth(trackWidth), m_wheelBase(wheelBase),
      m_logger(getLogger("common.swerve_drive")) {
    std::ostringstream oss;
    oss << "SwerveDrive initialized: trackWidth=" << trackWidth
        << " wheelBase=" << wheelBase;
    m_logger->debug(oss.str());
}

Pose2D SwerveDrive::step(const Pose2D& state, const Twist& control, double dt) {
    // The IKinematicModel Twist interface expresses body-frame commands as
    // (linear forward speed, yaw rate). Full swerve holonomic capability
    // (vx, vy, omega) would require an extended interface. For compatibility,
    // this uses the same arc-integration model as the unicycle.
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

ControlLimits SwerveDrive::getControlLimits() const {
    return ControlLimits{1.0, 2.0, 0.5, 1.0};
}

Twist SwerveDrive::toTwist(const Eigen::Vector2d& wheelSpeeds) const {
    return Twist{wheelSpeeds(0), wheelSpeeds(1)};
}

Eigen::Vector2d SwerveDrive::fromTwist(const Twist& twist) const {
    return Eigen::Vector2d{twist.linear, twist.angular};
}

}  // namespace robotlib
