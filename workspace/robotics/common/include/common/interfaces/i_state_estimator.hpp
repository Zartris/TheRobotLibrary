#pragma once
#include <common/types.hpp>
#include <Eigen/Core>

namespace robotlib {

/// Interface for probabilistic state estimators (e.g., EKF, UKF).
///
/// Follows the standard predict/update Bayesian filter cycle.
///
/// Pre-conditions:
///   - dt > 0 for predict(); calling with dt <= 0 is undefined behaviour
///   - H and R dimensions must be consistent with the state vector in update()
///
/// Post-conditions:
///   - getPose() returns the MAP estimate of the current robot pose
///   - getCovariance() returns a positive semi-definite 3x3 matrix
///   - reset() replaces the state with the given pose and resets covariance
///
/// Thread safety: not thread-safe; predict/update must be called from one thread.
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
