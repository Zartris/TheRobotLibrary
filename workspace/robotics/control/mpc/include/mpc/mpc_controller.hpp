#pragma once
#include <common/interfaces/i_controller.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace robotlib {

struct MPCConfig {
    int horizon{20};            ///< Prediction horizon steps
    double predictionTime{2.0}; ///< Total prediction time (s)
    double maxLinearVelocity{1.0};  ///< Max v (m/s)
    double maxAngularVelocity{2.0}; ///< Max omega (rad/s)
    double goalTolerance{0.1};      ///< Stop distance (m)

    // Cost weights (used by Eigen fallback; acados uses codegen values)
    Eigen::Vector3d stateWeight{10.0, 10.0, 1.0};
    Eigen::Vector2d controlWeight{0.1, 0.1};
    Eigen::Vector3d terminalWeight{100.0, 100.0, 10.0};
};

/// Model Predictive Controller for trajectory tracking.
///
/// Two backends:
///   1. **acados (NMPC)** -- if acados-generated solver is available in src/generated/.
///      Uses SQP-RTI with HPIPM QP solver. Nonlinear unicycle dynamics.
///      Generate with: `python tools/generate_solver.py`
///   2. **Eigen fallback (LTV-MPC)** -- if acados is not available.
///      Linearized dynamics, dense QP via Eigen LLT. Always available.
///
/// Pre-conditions:
///   - horizon >= 1, predictionTime > 0
///   - current and target must be finite Pose2D values
///
/// Post-conditions:
///   - Returned Twist respects velocity constraints
///   - getPredictedTrajectory() returns the planned horizon (acados) or linearized prediction
///     (fallback)
class MPCController : public IController {
public:
    explicit MPCController(const MPCConfig& config = {});
    ~MPCController();

    Twist compute(const Pose2D& current, const Pose2D& target, double dt) override;
    void reset() override;

    void setReferencePath(const Path& path);
    const std::vector<Pose2D>& getPredictedTrajectory() const { return m_predictedTrajectory; }
    const MPCConfig& config() const { return m_config; }

    /// Returns "acados" or "eigen" depending on active backend
    std::string backend() const;

private:
    // Eigen fallback solver
    Eigen::Vector2d solveEigen(const Pose2D& current, const Pose2D& target);
    void linearize(const Pose2D& state, double v_ref, double dt, Eigen::Matrix3d& A,
                   Eigen::Matrix<double, 3, 2>& B) const;

    MPCConfig m_config;
    Path m_referencePath;
    std::vector<Pose2D> m_predictedTrajectory;
    std::shared_ptr<ILogger> m_logger;

    // Opaque pointer to acados solver (nullptr if not available)
    struct AcadosSolver;
    std::unique_ptr<AcadosSolver> m_acados;
};

}  // namespace robotlib
