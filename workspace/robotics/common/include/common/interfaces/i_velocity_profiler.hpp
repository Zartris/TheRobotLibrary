#pragma once
#include <common/types.hpp>

namespace robotlib {

struct VelocityConstraints {
    double maxVelocity{1.0};
    double maxAcceleration{0.5};
    double maxDeceleration{0.5};
};

/// Interface for velocity profile generators that assign speeds to waypoints.
///
/// Converts a geometric path into a timed path respecting kinematic limits.
///
/// Pre-conditions:
///   - path must not be empty
///   - constraints.maxVelocity, maxAcceleration, and maxDeceleration must be > 0
///
/// Post-conditions:
///   - Returned TimedPath has the same number of waypoints as the input path
///   - All assigned speeds respect the given VelocityConstraints
///
/// Thread safety: implementations must be stateless or externally synchronised.
class IVelocityProfiler {
public:
    virtual ~IVelocityProfiler() = default;
    virtual TimedPath profile(const Path& path, const VelocityConstraints& constraints) = 0;
};

}  // namespace robotlib
