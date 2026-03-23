#pragma once
#include <common/types.hpp>

namespace robotlib {

class IController {
public:
    virtual ~IController() = default;
    virtual Twist compute(const Pose2D& current, const Pose2D& target, double dt) = 0;
    virtual void reset() = 0;
};

}  // namespace robotlib
