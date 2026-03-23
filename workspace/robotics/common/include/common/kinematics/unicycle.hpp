#pragma once
#include <common/kinematics/i_kinematic_model.hpp>
#include <logging/get_logger.hpp>

namespace robotlib {

class Unicycle : public IKinematicModel {
public:
    explicit Unicycle(double maxSpeed = 1.0, double maxOmega = 2.0);

    Pose2D step(const Pose2D& state, const Twist& control, double dt) override;
    ControlLimits getControlLimits() const override;
    Twist toTwist(const Eigen::Vector2d& wheelSpeeds) const override;
    Eigen::Vector2d fromTwist(const Twist& twist) const override;

private:
    double m_maxSpeed;
    double m_maxOmega;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
