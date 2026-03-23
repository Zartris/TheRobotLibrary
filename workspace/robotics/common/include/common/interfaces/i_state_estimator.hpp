#pragma once
#include <common/types.hpp>
#include <Eigen/Core>

namespace robotlib {

class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;
    virtual void predict(const Twist& twist, double dt) = 0;
    virtual void update(const Eigen::VectorXd& measurement, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) = 0;
    virtual Pose2D getPose() const = 0;
    virtual Eigen::Matrix3d getCovariance() const = 0;
    virtual void reset(const Pose2D& pose) = 0;
};

}  // namespace robotlib
