#include <mpc/mpc_controller.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>

namespace robotlib {

MPCController::MPCController(const MPCConfig& config)
    : m_config(config), m_logger(getLogger("mpc")) {
    m_logger->debug("MPCController initialized");
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

void MPCController::linearize(const Pose2D& state, double v_ref, double dt,
                               Eigen::Matrix3d& A, Eigen::Matrix<double, 3, 2>& B) const {
    double theta = state.theta;
    // Jacobian of discrete unicycle: x[k+1] = f(x[k], u[k])
    // A = df/dx at (state, u_ref)
    A = Eigen::Matrix3d::Identity();
    A(0, 2) = -v_ref * std::sin(theta) * dt;
    A(1, 2) = v_ref * std::cos(theta) * dt;

    // B = df/du at (state, u_ref)
    B.setZero();
    B(0, 0) = std::cos(theta) * dt;
    B(1, 0) = std::sin(theta) * dt;
    B(2, 1) = dt;
}

Eigen::Vector2d MPCController::solveQP(const Pose2D& current, const Pose2D& target) {
    const int N = m_config.horizon;
    const int nx = 3;  // state dim
    const int nu = 2;  // control dim

    // Build Q, R, Qf weight matrices
    Eigen::Matrix3d Q = m_config.stateWeight.asDiagonal();
    Eigen::Matrix2d R = m_config.controlWeight.asDiagonal();
    Eigen::Matrix3d Qf = m_config.terminalWeight.asDiagonal();

    // Reference state for each step (simple: interpolate toward target or use path)
    std::vector<Eigen::Vector3d> xRef(N + 1);
    if (!m_referencePath.empty()) {
        // Use reference path
        for (int k = 0; k <= N; ++k) {
            size_t idx = std::min(static_cast<size_t>(k), m_referencePath.size() - 1);
            xRef[k] = Eigen::Vector3d{m_referencePath[idx].x, m_referencePath[idx].y,
                                      m_referencePath[idx].theta};
        }
    } else {
        // Track target directly — interpolate linearly from current to target
        Eigen::Vector3d goalVec{target.x, target.y, target.theta};
        Eigen::Vector3d startVec{current.x, current.y, current.theta};
        for (int k = 0; k <= N; ++k) {
            double alpha = static_cast<double>(k) / N;
            xRef[k] = startVec + alpha * (goalVec - startVec);
            xRef[k](2) = normalizeAngle(xRef[k](2));
        }
    }

    // Linearize around current state with zero velocity reference
    double v_ref = 0.0;
    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;
    linearize(current, v_ref, m_config.dt, A, B);

    // Build prediction matrices using condensed formulation
    // X = S * x0 + T * U
    // S: (N*nx) x nx   — free response (no control)
    // T: (N*nx) x (N*nu) — control-to-state mapping
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(N * nx, nx);
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(N * nx, N * nu);

    // Build S: row k holds A^(k+1)
    Eigen::Matrix3d Apow = A;
    for (int k = 0; k < N; ++k) {
        S.block(k * nx, 0, nx, nx) = Apow;

        // T[k, j] = A^(k-j) * B  for j <= k
        for (int j = 0; j <= k; ++j) {
            Eigen::Matrix3d Adiff = Eigen::Matrix3d::Identity();
            for (int p = 0; p < k - j; ++p) {
                Adiff = A * Adiff;
            }
            T.block(k * nx, j * nu, nx, nu) = Adiff * B;
        }

        Apow = A * Apow;
    }

    // Build block-diagonal weight matrices
    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(N * nx, N * nx);
    for (int k = 0; k < N - 1; ++k) {
        Qbar.block(k * nx, k * nx, nx, nx) = Q;
    }
    Qbar.block((N - 1) * nx, (N - 1) * nx, nx, nx) = Qf;

    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(N * nu, N * nu);
    for (int k = 0; k < N; ++k) {
        Rbar.block(k * nu, k * nu, nu, nu) = R;
    }

    // Build reference vector (steps 1..N)
    Eigen::VectorXd xRefStacked(N * nx);
    for (int k = 0; k < N; ++k) {
        xRefStacked.segment(k * nx, nx) = xRef[k + 1];
    }

    // QP: minimize 0.5 U' H U + f' U
    // H = T' Qbar T + Rbar
    // f = T' Qbar (S * x0 - xRef)
    Eigen::Vector3d x0{current.x, current.y, current.theta};
    Eigen::VectorXd freeResp = S * x0;

    Eigen::MatrixXd H = T.transpose() * Qbar * T + Rbar;
    Eigen::VectorXd f = T.transpose() * Qbar * (freeResp - xRefStacked);

    // Symmetrize H (numerical cleanup)
    H = 0.5 * (H + H.transpose());

    // Solve unconstrained QP: U* = -H^-1 f via Cholesky decomposition
    Eigen::VectorXd Ustar = H.llt().solve(-f);

    // Clamp all controls to box limits
    for (int k = 0; k < N; ++k) {
        Ustar(k * nu + 0) = std::clamp(Ustar(k * nu + 0),
                                        -m_config.maxLinearVelocity,
                                        m_config.maxLinearVelocity);
        Ustar(k * nu + 1) = std::clamp(Ustar(k * nu + 1),
                                        -m_config.maxAngularVelocity,
                                        m_config.maxAngularVelocity);
    }

    // Store predicted trajectory for visualization
    m_predictedTrajectory.clear();
    m_predictedTrajectory.push_back(current);
    Eigen::VectorXd xPred = S * x0 + T * Ustar;
    for (int k = 0; k < N; ++k) {
        Pose2D p;
        p.x = xPred(k * nx + 0);
        p.y = xPred(k * nx + 1);
        p.theta = normalizeAngle(xPred(k * nx + 2));
        m_predictedTrajectory.push_back(p);
    }

    return Ustar.head<2>();  // return first control input
}

Twist MPCController::compute(const Pose2D& current, const Pose2D& target, double dt) {
    auto start = std::chrono::high_resolution_clock::now();

    // Check if already at goal
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    if (std::sqrt(dx * dx + dy * dy) < 0.1) {
        return {0.0, 0.0};
    }

    Eigen::Vector2d u = solveQP(current, target);

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "MPC compute: " << us << " us, v=" << u(0) << " omega=" << u(1);
    m_logger->trace(oss.str());

    return {u(0), u(1)};
}

}  // namespace robotlib
