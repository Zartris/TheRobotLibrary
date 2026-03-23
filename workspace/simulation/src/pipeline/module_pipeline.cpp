#include <simulation/pipeline/module_pipeline.hpp>
#include <common/geometry.hpp>
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
        cmd = m_localPlanner->compute(currentPose, measuredVelocity,
                                       m_globalPath, scan, grid);
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

}  // namespace robotlib::sim
