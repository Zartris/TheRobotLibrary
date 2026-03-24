#include <simulation/pipeline/module_pipeline.hpp>
#include <common/geometry.hpp>
#include <common/interfaces/perception_context.hpp>
#include <pid/pid_controller.hpp>
#include <pure_pursuit/pure_pursuit_controller.hpp>
#include <mpc/mpc_controller.hpp>
#include <cbf/cbf_safety_filter.hpp>
#include <cmath>
#include <sstream>
#include <chrono>

namespace robotlib::sim {

ModulePipeline::ModulePipeline()
    : m_logger(robotlib::getLogger("simulation.pipeline")) {
    m_logger->debug("ModulePipeline initialized");
}

void ModulePipeline::setController(std::unique_ptr<IController> controller) {
    m_controller = std::move(controller);
    m_logger->debug("Controller set");
}

void ModulePipeline::setGlobalPlanner(std::unique_ptr<IGlobalPlanner> planner) {
    m_globalPlanner = std::move(planner);
    m_logger->debug("Global planner set");
}

void ModulePipeline::setLocalPlanner(std::unique_ptr<ILocalPlanner> planner) {
    m_localPlanner = std::move(planner);
    m_logger->debug("Local planner set");
}

void ModulePipeline::setEstimator(std::unique_ptr<IStateEstimator> estimator) {
    m_estimator = std::move(estimator);
    m_logger->debug("Estimator set");
}

void ModulePipeline::setVelocityProfiler(std::unique_ptr<IVelocityProfiler> profiler) {
    m_profiler = std::move(profiler);
    m_logger->debug("Velocity profiler set");
}

void ModulePipeline::setGoal(const Pose2D& goal) {
    m_goal = goal;
    m_needsReplan = true;
    std::ostringstream oss;
    oss << "Goal set: (" << goal.x << ", " << goal.y << ", " << goal.theta << ")";
    m_logger->debug(oss.str());
}

bool ModulePipeline::hasGoal() const {
    return m_goal.has_value();
}

Pose2D ModulePipeline::getEstimatedPose() const {
    if (m_estimator) return m_estimator->getPose();
    return {};
}

bool ModulePipeline::isGoalReached() const {
    if (!m_goal || !m_estimator) return false;
    auto pose = m_estimator->getPose();
    double dx = pose.x - m_goal->x;
    double dy = pose.y - m_goal->y;
    return std::sqrt(dx * dx + dy * dy) < m_goalTolerance;
}

Twist ModulePipeline::tick(const Pose2D& measuredPose, const Twist& measuredVelocity,
                            const LaserScan& scan, const OccupancyGrid& grid, double dt) {
    auto start = std::chrono::high_resolution_clock::now();

    // 1. State estimation
    if (m_estimator) {
        m_estimator->predict(measuredVelocity, dt);
        // Direct pose update
        Eigen::VectorXd z(3);
        z << measuredPose.x, measuredPose.y, measuredPose.theta;
        Eigen::MatrixXd H = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd R = Eigen::Matrix3d::Identity() * 0.01;
        m_estimator->update(z, H, R);
    }

    Pose2D currentPose = m_estimator ? m_estimator->getPose() : measuredPose;

    if (!m_goal) return {0.0, 0.0};

    // Check if goal reached
    if (isGoalReached()) {
        m_logger->debug("Goal reached!");
        return {0.0, 0.0};
    }

    // 2. Global planning (replan if needed)
    if (m_needsReplan && m_globalPlanner) {
        auto path = m_globalPlanner->plan(currentPose, *m_goal, grid);
        if (path) {
            m_globalPath = *path;
            m_needsReplan = false;
            std::ostringstream oss;
            oss << "Global path computed: " << m_globalPath.size() << " waypoints";
            m_logger->debug(oss.str());
        } else {
            m_logger->warn("No global path found");
            return {0.0, 0.0};
        }
    }

    // Fallback: if no global planner is set, use a trivial path to the goal
    if (m_globalPath.empty() && !m_globalPlanner && m_goal) {
        m_globalPath = {*m_goal};
        m_logger->debug("No global planner — using direct path to goal");
    }

    if (m_globalPath.empty()) return {0.0, 0.0};

    // 3. Local planning
    Twist cmd;
    if (m_localPlanner) {
        PerceptionContext ctx;
        ctx.scan = scan;
        ctx.grid = grid;
        // ctx.trackedObstacles is empty (populated in M4)
        cmd = m_localPlanner->compute(currentPose, measuredVelocity,
                                      m_globalPath, ctx);
    } else if (m_controller) {
        // Fall back to simple controller toward next waypoint
        // Find nearest waypoint ahead
        Pose2D target = m_globalPath.back();
        for (const auto& wp : m_globalPath) {
            double dx = wp.x - currentPose.x;
            double dy = wp.y - currentPose.y;
            if (std::sqrt(dx * dx + dy * dy) > m_goalTolerance) {
                target = wp;
                break;
            }
        }
        cmd = m_controller->compute(currentPose, target, dt);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "Pipeline tick: " << us << " us";
    m_logger->trace(oss.str());

    return cmd;
}

std::vector<std::string> ModulePipeline::availableControllers() {
    return {"pid", "pure_pursuit", "mpc", "cbf_pid"};
}

std::vector<std::string> ModulePipeline::availableKinematicModels() {
    return {"differential_drive", "unicycle", "ackermann", "swerve"};
}

bool ModulePipeline::selectController(const std::string& name) {
    if (name == "pid") {
        PIDConfig hCfg{2.0, 0.0, 0.1, 3.0, 2.0};
        PIDConfig sCfg{1.0, 0.1, 0.0, 1.0, 1.0};
        setController(std::make_unique<HeadingSpeedController>(hCfg, sCfg));
    } else if (name == "pure_pursuit") {
        PurePursuitConfig cfg;
        cfg.lookaheadDistance = 0.8;
        cfg.maxLinearVelocity = 0.8;
        setController(std::make_unique<PurePursuitController>(cfg));
    } else if (name == "mpc") {
        MPCConfig cfg;
        cfg.horizon = 10;
        cfg.dt = 0.1;
        setController(std::make_unique<MPCController>(cfg));
    } else if (name == "cbf_pid") {
        PIDConfig hCfg{2.0, 0.0, 0.1, 3.0, 2.0};
        PIDConfig sCfg{1.0, 0.1, 0.0, 1.0, 1.0};
        auto pid = std::make_unique<HeadingSpeedController>(hCfg, sCfg);
        CbfConfig cbfCfg;
        cbfCfg.safetyRadius = 0.5;
        setController(std::make_unique<CbfSafetyFilter>(std::move(pid), cbfCfg));
    } else {
        m_logger->warn("Unknown controller: " + name);
        return false;
    }
    m_controllerName = name;
    std::ostringstream oss;
    oss << "Controller selected: " << name;
    m_logger->debug(oss.str());
    return true;
}

bool ModulePipeline::selectKinematicModel(const std::string& name) {
    auto models = availableKinematicModels();
    bool found = false;
    for (const auto& m : models) {
        if (m == name) {
            found = true;
            break;
        }
    }
    if (!found) {
        m_logger->warn("Unknown kinematic model: " + name);
        return false;
    }
    m_kinematicModelName = name;
    std::ostringstream oss;
    oss << "Kinematic model selected: " << name;
    m_logger->debug(oss.str());
    return true;
}

}  // namespace robotlib::sim
