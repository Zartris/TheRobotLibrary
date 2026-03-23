#pragma once
#include <common/interfaces/i_controller.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace robotlib {

struct MPCConfig {
    int horizon{10};                     ///< Prediction horizon steps
    double dt{0.1};                      ///< Discretization timestep
    double maxLinearVelocity{1.0};       ///< Max v (m/s)
    double maxAngularVelocity{2.0};      ///< Max omega (rad/s)
    Eigen::Vector3d stateWeight{1.0, 1.0, 0.1};     ///< Q diagonal [px, py, theta]
    Eigen::Vector2d controlWeight{0.01, 0.01};       ///< R diagonal [v, omega]
    Eigen::Vector3d terminalWeight{10.0, 10.0, 1.0}; ///< Qf diagonal
};

class MPCController : public IController {
public:
    explicit MPCController(const MPCConfig& config = {});
    ~MPCController();

    Twist compute(const Pose2D& current, const Pose2D& target, double dt) override;
    void reset() override;

    /// Set a reference path for the MPC to track (optional — if not set, tracks target directly)
    void setReferencePath(const Path& path);

    /// Get the predicted trajectory (for visualization)
    const std::vector<Pose2D>& getPredictedTrajectory() const { return m_predictedTrajectory; }

    const MPCConfig& config() const { return m_config; }

private:
    /// Build linearized dynamics matrices at operating point
    void linearize(const Pose2D& state, double v_ref, double dt,
                   Eigen::Matrix3d& A, Eigen::Matrix<double, 3, 2>& B) const;

    /// Build and solve the QP
    Eigen::Vector2d solveQP(const Pose2D& current, const Pose2D& target);

    MPCConfig m_config;
    Path m_referencePath;
    std::vector<Pose2D> m_predictedTrajectory;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
