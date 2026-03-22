#pragma once
#include <common/types.hpp>
#include <Eigen/Core>

namespace robotlib {

struct ControlLimits {
    double maxLinearVelocity{1.0};
    double maxAngularVelocity{1.0};
    double maxLinearAcceleration{0.5};
    double maxAngularAcceleration{1.0};
};

class IKinematicModel {
public:
    virtual ~IKinematicModel() = default;
    virtual Pose2D step(const Pose2D& state, const Twist& control, double dt) = 0;
    virtual ControlLimits getControlLimits() const = 0;
    virtual Twist toTwist(const Eigen::Vector2d& wheelSpeeds) const = 0;
    virtual Eigen::Vector2d fromTwist(const Twist& twist) const = 0;
};

}  // namespace robotlib
