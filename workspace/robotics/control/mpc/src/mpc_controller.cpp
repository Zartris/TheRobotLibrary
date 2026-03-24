#include <mpc/mpc_controller.hpp>
#include <common/geometry.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

// Acados integration (conditional)
#ifdef ROBOTLIB_HAS_ACADOS
#include "generated/acados_solver_unicycle.h"
#endif

namespace robotlib {

// ============================================================
// Acados solver wrapper (compiled only when acados is available)
// ============================================================
struct MPCController::AcadosSolver {
#ifdef ROBOTLIB_HAS_ACADOS
    unicycle_solver_capsule* capsule{nullptr};
    int N{0};
    bool valid{false};

    explicit AcadosSolver(int horizon) : N(horizon) {
        capsule = unicycle_acados_create_capsule();
        if (capsule) {
            int status = unicycle_acados_create(capsule);
            valid = (status == 0);
        }
    }

    ~AcadosSolver() {
        if (capsule) {
            unicycle_acados_free(capsule);
            unicycle_acados_free_capsule(capsule);
        }
    }

    // Non-copyable
    AcadosSolver(const AcadosSolver&) = delete;
    AcadosSolver& operator=(const AcadosSolver&) = delete;

    Eigen::Vector2d solve(const Pose2D& current, const Pose2D& target, const Path& refPath,
                          double maxV, double maxOmega) {
        if (!valid) {
            return Eigen::Vector2d::Zero();
        }

        ocp_nlp_config* nlpConfig = unicycle_acados_get_nlp_config(capsule);
        ocp_nlp_dims* dims = unicycle_acados_get_nlp_dims(capsule);
        ocp_nlp_in* in = unicycle_acados_get_nlp_in(capsule);
        ocp_nlp_out* out = unicycle_acados_get_nlp_out(capsule);

        // Set initial state constraint
        double x0[3] = {current.x, current.y, current.theta};
        ocp_nlp_constraints_model_set(nlpConfig, dims, in, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlpConfig, dims, in, 0, "ubx", x0);

        // Set reference for each stage
        for (int k = 0; k <= N; ++k) {
            double xRef[3];
            if (!refPath.empty()) {
                size_t idx = std::min(static_cast<size_t>(k), refPath.size() - 1);
                xRef[0] = refPath[idx].x;
                xRef[1] = refPath[idx].y;
                xRef[2] = refPath[idx].theta;
            } else {
                double alpha = static_cast<double>(k) / N;
                xRef[0] = current.x + alpha * (target.x - current.x);
                xRef[1] = current.y + alpha * (target.y - current.y);
                xRef[2] = normalizeAngle(
                    current.theta + alpha * normalizeAngle(target.theta - current.theta));
            }

            if (k < N) {
                double yref[5] = {xRef[0], xRef[1], xRef[2], 0.0, 0.0};
                ocp_nlp_cost_model_set(nlpConfig, dims, in, k, "yref", yref);
            } else {
                ocp_nlp_cost_model_set(nlpConfig, dims, in, k, "yref", xRef);
            }
        }

        // Solve
        int status = unicycle_acados_solve(capsule);
        if (status != 0) {
            return Eigen::Vector2d::Zero();
        }

        // Extract first control
        double u0[2];
        ocp_nlp_out_get(nlpConfig, dims, out, 0, "u", u0);

        return Eigen::Vector2d{std::clamp(u0[0], -maxV, maxV),
                               std::clamp(u0[1], -maxOmega, maxOmega)};
    }

    void extractTrajectory(std::vector<Pose2D>& traj) {
        if (!valid) {
            return;
        }
        ocp_nlp_config* nlpConfig = unicycle_acados_get_nlp_config(capsule);
        ocp_nlp_dims* dims = unicycle_acados_get_nlp_dims(capsule);
        ocp_nlp_out* out = unicycle_acados_get_nlp_out(capsule);

        traj.clear();
        for (int k = 0; k <= N; ++k) {
            double xk[3];
            ocp_nlp_out_get(nlpConfig, dims, out, k, "x", xk);
            traj.push_back({xk[0], xk[1], normalizeAngle(xk[2])});
        }
    }
#else
    // Stub when acados is not available
    explicit AcadosSolver(int /*horizon*/) {}
#endif
};

// ============================================================
// MPCController implementation
// ============================================================

MPCController::MPCController(const MPCConfig& config) : m_config(config), m_logger(getLogger("mpc")) {
#ifdef ROBOTLIB_HAS_ACADOS
    m_acados = std::make_unique<AcadosSolver>(config.horizon);
    if (m_acados->valid) {
        m_logger->info("MPCController initialized with acados NMPC backend");
    } else {
        m_logger->warn("acados solver creation failed -- falling back to Eigen LTV-MPC");
        m_acados.reset();
    }
#else
    m_logger->info("MPCController initialized with Eigen LTV-MPC backend (acados not available)");
#endif
}

MPCController::~MPCController() = default;

void MPCController::setReferencePath(const Path& path) {
    m_referencePath = path;
    std::ostringstream oss;
    oss << "Reference path set: " << path.size() << " waypoints";
    m_logger->debug(oss.str());
}

void MPCController::reset() {
    m_referencePath.clear();
    m_predictedTrajectory.clear();
    m_logger->debug("MPCController reset");
}

std::string MPCController::backend() const {
#ifdef ROBOTLIB_HAS_ACADOS
    if (m_acados) {
        return "acados";
    }
#endif
    return "eigen";
}

Twist MPCController::compute(const Pose2D& current, const Pose2D& target, double /*dt*/) {
    auto start = std::chrono::high_resolution_clock::now();

    // Check if at goal
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    if (std::sqrt(dx * dx + dy * dy) < m_config.goalTolerance) {
        auto end = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::ostringstream oss;
        oss << "MPC compute: " << us << " us (goal reached)";
        m_logger->trace(oss.str());
        return {0.0, 0.0};
    }

    Eigen::Vector2d u;

#ifdef ROBOTLIB_HAS_ACADOS
    if (m_acados) {
        u = m_acados->solve(current, target, m_referencePath, m_config.maxLinearVelocity,
                            m_config.maxAngularVelocity);
        m_acados->extractTrajectory(m_predictedTrajectory);
    } else {
        u = solveEigen(current, target);
    }
#else
    u = solveEigen(current, target);
#endif

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "MPC compute: " << us << " us, v=" << u(0) << " omega=" << u(1)
        << " backend=" << backend();
    m_logger->trace(oss.str());

    return {u(0), u(1)};
}

// ============================================================
// Eigen fallback solver (LTV-MPC)
// ============================================================

void MPCController::linearize(const Pose2D& state, double v_ref, double dt, Eigen::Matrix3d& A,
                              Eigen::Matrix<double, 3, 2>& B) const {
    double theta = state.theta;
    A = Eigen::Matrix3d::Identity();
    A(0, 2) = -v_ref * std::sin(theta) * dt;
    A(1, 2) = v_ref * std::cos(theta) * dt;

    B.setZero();
    B(0, 0) = std::cos(theta) * dt;
    B(1, 0) = std::sin(theta) * dt;
    B(2, 1) = dt;
}

Eigen::Vector2d MPCController::solveEigen(const Pose2D& current, const Pose2D& target) {
    const int N = m_config.horizon;
    const int nx = 3;
    const int nu = 2;

    if (N <= 0) {
        m_logger->error("MPC horizon must be >= 1");
        return Eigen::Vector2d::Zero();
    }

    double dt = m_config.predictionTime / N;
    if (dt <= 0.0) {
        m_logger->error("MPC prediction time must be > 0");
        return Eigen::Vector2d::Zero();
    }

    Eigen::Matrix3d Q = m_config.stateWeight.asDiagonal();
    Eigen::Matrix2d R = m_config.controlWeight.asDiagonal();
    Eigen::Matrix3d Qf = m_config.terminalWeight.asDiagonal();

    // Build reference trajectory
    std::vector<Eigen::Vector3d> xRef(N + 1);
    if (!m_referencePath.empty()) {
        for (int k = 0; k <= N; ++k) {
            size_t idx = std::min(static_cast<size_t>(k), m_referencePath.size() - 1);
            xRef[k] = Eigen::Vector3d{m_referencePath[idx].x, m_referencePath[idx].y,
                                      m_referencePath[idx].theta};
        }
    } else {
        Eigen::Vector3d goalVec{target.x, target.y, target.theta};
        Eigen::Vector3d startVec{current.x, current.y, current.theta};
        for (int k = 0; k <= N; ++k) {
            double alpha = static_cast<double>(k) / N;
            xRef[k] = startVec + alpha * (goalVec - startVec);
            xRef[k](2) = normalizeAngle(xRef[k](2));
        }
    }

    // Derive v_ref from reference for linearization
    double v_ref = 0.1;
    if (xRef.size() > 1) {
        Eigen::Vector2d delta = (xRef[1] - xRef[0]).head<2>();
        v_ref = std::max(delta.norm() / dt, 0.1);
    }

    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;
    linearize(current, v_ref, dt, A, B);

    // Condensed formulation: X = S*x0 + T*U
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(N * nx, nx);
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(N * nx, N * nu);

    Eigen::Matrix3d Apow = A;
    for (int k = 0; k < N; ++k) {
        S.block(k * nx, 0, nx, nx) = Apow;
        for (int j = 0; j <= k; ++j) {
            Eigen::Matrix3d Adiff = Eigen::Matrix3d::Identity();
            for (int p = 0; p < k - j; ++p) {
                Adiff = A * Adiff;
            }
            T.block(k * nx, j * nu, nx, nu) = Adiff * B;
        }
        Apow = A * Apow;
    }

    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(N * nx, N * nx);
    for (int k = 0; k < N - 1; ++k) {
        Qbar.block(k * nx, k * nx, nx, nx) = Q;
    }
    Qbar.block((N - 1) * nx, (N - 1) * nx, nx, nx) = Qf;

    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(N * nu, N * nu);
    for (int k = 0; k < N; ++k) {
        Rbar.block(k * nu, k * nu, nu, nu) = R;
    }

    Eigen::VectorXd xRefStacked(N * nx);
    for (int k = 0; k < N; ++k) {
        xRefStacked.segment(k * nx, nx) = xRef[k + 1];
    }

    Eigen::Vector3d x0{current.x, current.y, current.theta};
    Eigen::VectorXd freeResp = S * x0;

    Eigen::MatrixXd H = T.transpose() * Qbar * T + Rbar;
    Eigen::VectorXd f = T.transpose() * Qbar * (freeResp - xRefStacked);
    H = 0.5 * (H + H.transpose());

    Eigen::LLT<Eigen::MatrixXd> llt(H);
    Eigen::VectorXd Ustar;
    if (llt.info() == Eigen::Success) {
        Ustar = llt.solve(-f);
    } else {
        m_logger->error("MPC QP solve failed: H not positive definite");
        return Eigen::Vector2d::Zero();
    }

    // Clamp and extract predicted trajectory
    for (int k = 0; k < N; ++k) {
        Ustar(k * nu + 0) =
            std::clamp(Ustar(k * nu + 0), -m_config.maxLinearVelocity, m_config.maxLinearVelocity);
        Ustar(k * nu + 1) = std::clamp(Ustar(k * nu + 1), -m_config.maxAngularVelocity,
                                        m_config.maxAngularVelocity);
    }

    m_predictedTrajectory.clear();
    m_predictedTrajectory.push_back(current);
    Eigen::VectorXd xPred = S * x0 + T * Ustar;
    for (int k = 0; k < N; ++k) {
        m_predictedTrajectory.push_back(
            {xPred(k * nx + 0), xPred(k * nx + 1), normalizeAngle(xPred(k * nx + 2))});
    }

    return Ustar.head<2>();
}

}  // namespace robotlib
