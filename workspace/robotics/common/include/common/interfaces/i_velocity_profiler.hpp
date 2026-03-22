#pragma once
#include <common/types.hpp>

namespace robotlib {

struct VelocityConstraints {
    double maxVelocity{1.0};
    double maxAcceleration{0.5};
    double maxDeceleration{0.5};
};

class IVelocityProfiler {
public:
    virtual ~IVelocityProfiler() = default;
    virtual TimedPath profile(const Path& path, const VelocityConstraints& constraints) = 0;
};

}  // namespace robotlib
