#pragma once
#include <common/interfaces/i_controller.hpp>
#include <common/interfaces/i_global_planner.hpp>
#include <common/interfaces/i_local_planner.hpp>
#include <common/interfaces/i_state_estimator.hpp>
#include <common/interfaces/i_velocity_profiler.hpp>
#include <common/types.hpp>
#include <common/occupancy_grid.hpp>
#include <common/laser_scan.hpp>
#include <logging/get_logger.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace robotlib::sim {

class ModulePipeline {
public:
    ModulePipeline();

    void setController(std::unique_ptr<IController> controller);
    void setGlobalPlanner(std::unique_ptr<IGlobalPlanner> planner);
    void setLocalPlanner(std::unique_ptr<ILocalPlanner> planner);
    void setEstimator(std::unique_ptr<IStateEstimator> estimator);
    void setVelocityProfiler(std::unique_ptr<IVelocityProfiler> profiler);

    void setGoal(const Pose2D& goal);

    Twist tick(const Pose2D& measuredPose, const Twist& measuredVelocity,
              const LaserScan& scan, const OccupancyGrid& grid, double dt);

    bool hasGoal() const;
    Pose2D getEstimatedPose() const;
    const Path& getGlobalPath() const { return m_globalPath; }
    bool isGoalReached() const;

    /// Create and set a controller by name. Returns false if name unknown.
    bool selectController(const std::string& name);

    /// Select kinematic model by name (UI state only — the simulation bridge
    /// uses MJCF-defined kinematics directly). Returns false if name is unknown.
    bool selectKinematicModel(const std::string& name);

    /// Get available controller names
    static std::vector<std::string> availableControllers();

    /// Get available kinematic model names
    static std::vector<std::string> availableKinematicModels();

    /// Get currently selected controller name
    const std::string& currentControllerName() const { return m_controllerName; }

    /// Get currently selected kinematic model name
    const std::string& currentKinematicModelName() const { return m_kinematicModelName; }

private:
    std::unique_ptr<IController> m_controller;
    std::unique_ptr<IGlobalPlanner> m_globalPlanner;
    std::unique_ptr<ILocalPlanner> m_localPlanner;
    std::unique_ptr<IStateEstimator> m_estimator;
    std::unique_ptr<IVelocityProfiler> m_profiler;

    std::optional<Pose2D> m_goal;
    Path m_globalPath;
    bool m_needsReplan{true};
    double m_goalTolerance{0.3};

    std::string m_controllerName{"pid"};
    std::string m_kinematicModelName{"differential_drive"};

    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib::sim
