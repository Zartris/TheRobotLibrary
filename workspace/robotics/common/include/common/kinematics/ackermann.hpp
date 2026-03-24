#pragma once
#include <common/kinematics/i_kinematic_model.hpp>
#include <logging/get_logger.hpp>

namespace robotlib {

/// Ackermann (bicycle model) kinematics.
///
/// IMPORTANT: Unlike other IKinematicModel implementations where Twist.angular
/// is yaw rate (omega), Ackermann uses Twist.angular as the **steering angle (delta)**
/// in radians. This matches the natural bicycle model parameterization:
///   - Twist.linear = forward speed (v) in m/s
///   - Twist.angular = front-wheel steering angle (delta) in radians
///
/// Conversion: omega = v * tan(delta) / wheelbase
class Ackermann : public IKinematicModel {
public:
    Ackermann(double wheelbase, double maxSteeringAngle);

    Pose2D step(const Pose2D& state, const Twist& control, double dt) override;
    ControlLimits getControlLimits() const override;
    Twist toTwist(const Eigen::Vector2d& wheelSpeeds) const override;
    Eigen::Vector2d fromTwist(const Twist& twist) const override;

    double wheelbase() const noexcept { return m_wheelbase; }
    double maxSteeringAngle() const noexcept { return m_maxSteeringAngle; }
    double minTurningRadius() const noexcept { return m_wheelbase / std::tan(m_maxSteeringAngle); }

private:
    double m_wheelbase;
    double m_maxSteeringAngle;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
