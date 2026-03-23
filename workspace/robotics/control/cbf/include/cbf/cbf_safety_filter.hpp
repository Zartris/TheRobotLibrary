#pragma once
#include <common/interfaces/i_controller.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace robotlib {

struct Obstacle2D {
    double x{0.0};
    double y{0.0};
    double radius{0.0};  ///< Obstacle radius (0 for point obstacle)
};

struct CbfConfig {
    double safetyRadius{0.5};  ///< Minimum distance to maintain from obstacle centers
    double alpha{1.0};         ///< CBF decay rate (higher = more aggressive safety)
    double maxLinearVelocity{1.0};
    double maxAngularVelocity{3.0};
};

class CbfSafetyFilter : public IController {
public:
    CbfSafetyFilter(std::unique_ptr<IController> nominalController,
                     const CbfConfig& config = {});

    Twist compute(const Pose2D& current, const Pose2D& target, double dt) override;
    void reset() override;

    /// Update obstacle list each tick
    void setObstacles(const std::vector<Obstacle2D>& obstacles);

    /// Access the wrapped controller
    IController& nominalController() { return *m_nominal; }

    /// Get the safety radius
    double safetyRadius() const { return m_config.safetyRadius; }

    const CbfConfig& config() const { return m_config; }

private:
    /// Project nominal velocity onto the safe set defined by CBF constraints
    Twist projectToSafe(const Pose2D& current, const Twist& nominal) const;

    std::unique_ptr<IController> m_nominal;
    CbfConfig m_config;
    std::vector<Obstacle2D> m_obstacles;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
