#pragma once
#include <common/interfaces/i_state_estimator.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <Eigen/Core>

namespace robotlib {

struct EKFConfig {
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 0.01;  // process noise
    Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * 0.1;  // initial covariance
};

class EKF2D : public IStateEstimator {
public:
    explicit EKF2D(const EKFConfig& config = {});

    void predict(const Twist& twist, double dt) override;
    void update(const Eigen::VectorXd& measurement, const Eigen::MatrixXd& H,
                const Eigen::MatrixXd& R) override;
    Pose2D getPose() const override;
    Eigen::Matrix3d getCovariance() const override;
    void reset(const Pose2D& pose) override;

private:
    Eigen::Vector3d m_state;  // [x, y, theta]
    Eigen::Matrix3d m_P;      // covariance
    Eigen::Matrix3d m_Q;      // process noise
    std::shared_ptr<ILogger> m_logger;
    int m_predictCount{0};
    int m_updateCount{0};
};

}  // namespace robotlib
