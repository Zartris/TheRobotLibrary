#pragma once
#include <common/interfaces/i_controller.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <memory>

namespace robotlib {

struct PurePursuitConfig {
    double lookaheadDistance{0.5};   ///< Base lookahead distance (m)
    double lookaheadGain{0.1};       ///< Speed-dependent gain: L = base + gain * |v|
    double maxLinearVelocity{1.0};   ///< Max forward speed (m/s)
    double maxAngularVelocity{3.0};  ///< Max turning rate (rad/s)
    double goalTolerance{0.1};       ///< Distance to goal to stop (m)
};

class PurePursuitController : public IController {
public:
    explicit PurePursuitController(const PurePursuitConfig& config = {});

    Twist compute(const Pose2D& current, const Pose2D& target, double dt) override;
    void reset() override;

    /// Set the path for the controller to follow
    void setPath(const Path& path);

    /// Get the current lookahead point (for visualization)
    Pose2D getLookaheadPoint() const { return m_lookaheadPoint; }

    const PurePursuitConfig& config() const { return m_config; }

private:
    /// Find the lookahead point on the path
    Pose2D findLookaheadPoint(const Pose2D& current, double lookaheadDist) const;

    /// Compute curvature to reach a target point
    double computeCurvature(const Pose2D& current, const Pose2D& lookahead) const;

    PurePursuitConfig m_config;
    Path m_path;
    Pose2D m_lookaheadPoint;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
