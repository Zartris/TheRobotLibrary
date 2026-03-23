#pragma once
#include <common/types.hpp>

namespace robotlib {

/// Interface for feedback controllers that compute velocity commands.
///
/// A controller converts pose error into a Twist command each control cycle.
///
/// Pre-conditions:
///   - dt > 0 (callers must not pass zero or negative timesteps)
///   - current and target must be finite Pose2D values in the same frame
///
/// Post-conditions:
///   - Returned Twist magnitudes are bounded by the implementation's limits
///   - reset() leaves the controller in a clean initial state (no integral wind-up)
///
/// Thread safety: not thread-safe; call from a single control thread.
class IController {
public:
    virtual ~IController() = default;
    virtual Twist compute(const Pose2D& current, const Pose2D& target, double dt) = 0;
    virtual void reset() = 0;
};

}  // namespace robotlib
