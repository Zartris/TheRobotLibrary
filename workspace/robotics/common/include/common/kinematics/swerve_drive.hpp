#pragma once
#include <common/kinematics/i_kinematic_model.hpp>
#include <logging/get_logger.hpp>

namespace robotlib {

/// SwerveDrive kinematic model.
///
/// A swerve drive is holonomic — each wheel can be independently steered and
/// driven, allowing simultaneous translation and rotation in any direction.
/// The full capability (vx, vy, omega) would require an extended interface
/// beyond the current Twist{linear, angular} contract. This implementation
/// uses the IKinematicModel interface directly: Twist.linear is forward speed
/// and Twist.angular is yaw rate, matching the unicycle arc-integration model.
class SwerveDrive : public IKinematicModel {
public:
    explicit SwerveDrive(double trackWidth = 0.3, double wheelBase = 0.3);

    Pose2D step(const Pose2D& state, const Twist& control, double dt) override;
    ControlLimits getControlLimits() const override;
    Twist toTwist(const Eigen::Vector2d& wheelSpeeds) const override;
    Eigen::Vector2d fromTwist(const Twist& twist) const override;

private:
    double m_trackWidth;
    double m_wheelBase;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
