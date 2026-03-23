#pragma once
#include <common/interfaces/i_local_planner.hpp>
#include <common/interfaces/perception_context.hpp>
#include <logging/get_logger.hpp>
#include <memory>
#include <vector>

namespace robotlib {

struct DWAConfig {
    double maxLinearVel{1.0};
    double maxAngularVel{2.0};
    double linearAccel{0.5};
    double angularAccel{2.0};
    int linearSamples{11};
    int angularSamples{21};
    double simTime{2.0};
    double simDt{0.1};
    double goalWeight{1.0};
    double obstacleWeight{1.0};
    double velocityWeight{0.1};
    double obstacleThreshold{0.3};
};

class DWAPlanner : public ILocalPlanner {
public:
    explicit DWAPlanner(const DWAConfig& config = {});

    Twist compute(const Pose2D& pose, const Twist& vel,
                  const Path& path, const PerceptionContext& ctx) override;

private:
    struct Trajectory {
        std::vector<Pose2D> poses;
        Twist velocity;
        double cost{0.0};
    };

    Trajectory simulateTrajectory(const Pose2D& pose, const Twist& vel) const;
    double scoreCost(const Trajectory& traj, const Path& globalPath,
                     const LaserScan& scan, const Pose2D& robotPose) const;
    double minObstacleDistance(const Trajectory& traj, const LaserScan& scan,
                               const Pose2D& robotPose) const;

    DWAConfig m_config;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
