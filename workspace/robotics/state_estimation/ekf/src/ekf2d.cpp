#include <ekf/ekf2d.hpp>
#include <common/geometry.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include <chrono>

namespace robotlib {

EKF2D::EKF2D(const EKFConfig& config)
    : m_state(Eigen::Vector3d::Zero()),
      m_P(config.P0),
      m_Q(config.Q),
      m_logger(getLogger("ekf")) {
    m_logger->debug("EKF2D initialized");
}

void EKF2D::predict(const Twist& twist, double dt) {
    auto start = std::chrono::high_resolution_clock::now();

    const double theta = m_state(2);
    const double v = twist.linear;
    const double w = twist.angular;

    // Motion model: Euler integration
    if (std::abs(w) < 1e-6) {
        m_state(0) += v * std::cos(theta) * dt;
        m_state(1) += v * std::sin(theta) * dt;
    } else {
        m_state(0) += (v / w) * (std::sin(theta + w * dt) - std::sin(theta));
        m_state(1) += (v / w) * (-std::cos(theta + w * dt) + std::cos(theta));
        m_state(2) = normalizeAngle(theta + w * dt);
    }

    // Jacobian of motion model w.r.t. state
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    if (std::abs(w) < 1e-6) {
        F(0, 2) = -v * std::sin(theta) * dt;
        F(1, 2) = v * std::cos(theta) * dt;
    } else {
        F(0, 2) = (v / w) * (std::cos(theta + w * dt) - std::cos(theta));
        F(1, 2) = (v / w) * (std::sin(theta + w * dt) - std::sin(theta));
    }

    m_P = F * m_P * F.transpose() + m_Q;
    ++m_predictCount;

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "predict #" << m_predictCount << ": " << us << " us";
    m_logger->debug(oss.str());
}

void EKF2D::update(const Eigen::VectorXd& measurement, const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& R) {
    auto start = std::chrono::high_resolution_clock::now();

    // Innovation
    Eigen::VectorXd y = measurement - H * m_state;
    // Wrap angle if needed (last component)
    if (y.size() >= 3) {
        y(2) = normalizeAngle(y(2));
    }

    // Innovation covariance
    Eigen::MatrixXd S = H * m_P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXd K = m_P * H.transpose() * S.inverse();

    // State update
    m_state += K * y;
    m_state(2) = normalizeAngle(m_state(2));

    // Covariance update (Joseph form for numerical stability)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd IKH = I - K * H;
    m_P = IKH * m_P * IKH.transpose() + K * R * K.transpose();

    ++m_updateCount;

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "update #" << m_updateCount << ": " << us << " us";
    m_logger->debug(oss.str());
}

Pose2D EKF2D::getPose() const {
    return Pose2D{m_state(0), m_state(1), m_state(2)};
}

Eigen::Matrix3d EKF2D::getCovariance() const {
    return m_P;
}

void EKF2D::reset(const Pose2D& pose) {
    m_state = Eigen::Vector3d{pose.x, pose.y, pose.theta};
    m_P = Eigen::Matrix3d::Identity() * 0.1;
    m_predictCount = 0;
    m_updateCount = 0;
    m_logger->debug("EKF2D reset");
}

}  // namespace robotlib
