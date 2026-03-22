#pragma once
/// @file types_3d.hpp
/// @brief Convenience header for 3D types needed by the MuJoCo bridge.
/// Delegates to the transforms sub-module (SE3, SO3) which provides
/// Position3D-equivalent (Eigen::Vector3d), Quaternion (Eigen::Quaterniond),
/// and Transform3D (SE3) functionality.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robotlib {

/// 3D position alias — used by simulation bridge adapters.
using Position3D = Eigen::Vector3d;

/// Quaternion alias — used for 3D orientation.
using QuaternionD = Eigen::Quaterniond;

/// Simple 3D rigid-body transform (translation + quaternion).
struct Transform3D {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};

    [[nodiscard]] static Transform3D identity() noexcept {
        return {};
    }

    [[nodiscard]] Eigen::Vector3d transformPoint(const Eigen::Vector3d& p) const noexcept {
        return rotation * p + translation;
    }

    [[nodiscard]] Transform3D inverse() const noexcept {
        auto invR = rotation.conjugate();
        return {-(invR * translation), invR};
    }

    [[nodiscard]] Transform3D compose(const Transform3D& other) const noexcept {
        return {translation + rotation * other.translation,
                (rotation * other.rotation).normalized()};
    }
};

}  // namespace robotlib
