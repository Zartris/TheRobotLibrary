#pragma once
#include <common/kinematics/i_kinematic_model.hpp>
#include <logging/get_logger.hpp>

namespace robotlib {

class DifferentialDrive : public IKinematicModel {
public:
    DifferentialDrive(double wheelRadius, double trackWidth);

    Pose2D step(const Pose2D& state, const Twist& control, double dt) override;
    ControlLimits getControlLimits() const override;
    Twist toTwist(const Eigen::Vector2d& wheelSpeeds) const override;
    Eigen::Vector2d fromTwist(const Twist& twist) const override;

    double wheelRadius() const noexcept { return m_wheelRadius; }
    double trackWidth() const noexcept { return m_trackWidth; }

private:
    double m_wheelRadius;
    double m_trackWidth;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
