#pragma once
#include <common/interfaces/perception_context.hpp>
#include <common/types.hpp>

namespace robotlib {

/// Interface for reactive local planners.
///
/// A local planner receives the robot's current state and perception data,
/// and returns an immediate velocity command safe for the next control cycle.
///
/// Pre-conditions:
///   - path must not be empty (callers must check hasGoal/path)
///   - pose must be a valid (finite) Pose2D in the map frame
///   - ctx.grid must be initialized (non-zero resolution)
///   - dt > 0 is assumed by implementations (not passed here; planners use
///     their own configured timestep for trajectory rollout)
///
/// Post-conditions:
///   - Returned Twist is within the robot's dynamic window limits
///   - If no safe velocity found, returns zero Twist
///
/// Thread safety: not thread-safe; call from a single control thread.
class ILocalPlanner {
public:
    virtual ~ILocalPlanner() = default;

    /// Compute a velocity command for the current control cycle.
    /// @param pose    Current estimated robot pose (map frame)
    /// @param vel     Current robot velocity
    /// @param path    Global reference path to follow
    /// @param ctx     Perception bundle: scan, grid, and (M4+) tracked obstacles
    /// @return        Velocity command (linear m/s, angular rad/s)
    virtual Twist compute(const Pose2D& pose, const Twist& vel,
                          const Path& path, const PerceptionContext& ctx) = 0;
};

}  // namespace robotlib
