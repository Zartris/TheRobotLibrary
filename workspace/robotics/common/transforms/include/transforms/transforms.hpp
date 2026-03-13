#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

/// @file transforms.hpp
/// @brief Header-only rigid-body transform types: SE2, SE3, SO3.
///
/// All types are pure value types (no heap allocation, no virtual dispatch).
/// They are designed to be composed, inverted, and applied to points in the
/// most numerically straightforward way possible.  For a full mathematical
/// treatment see docs/theory.md.

namespace transforms {

// ─────────────────────────────────────────────────────────────────────────────
// SE2 — 2-D Special Euclidean group
// Represents a planar rigid-body pose: translation (x, y) + rotation theta.
// ─────────────────────────────────────────────────────────────────────────────

struct SE2 {
    double x{0.0};
    double y{0.0};
    double theta{0.0};  ///< Rotation angle in radians

    // ── Factories ────────────────────────────────────────────────────────────

    /// Returns the identity transform (zero translation, zero rotation).
    [[nodiscard]] static SE2 identity() noexcept { return SE2{0.0, 0.0, 0.0}; }

    // ── Group operations ─────────────────────────────────────────────────────

    /// Compose this transform with @p other: T_result = T_this * T_other.
    /// Applies @p other in the local frame of this transform.
    [[nodiscard]] SE2 compose(const SE2& other) const noexcept {
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        return SE2{
            x + c * other.x - s * other.y,
            y + s * other.x + c * other.y,
            normalizeAngle(theta + other.theta)
        };
    }

    /// Returns the inverse transform such that compose(inverse()) == identity().
    [[nodiscard]] SE2 inverse() const noexcept {
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        return SE2{
            -(c * x + s * y),
            -(-s * x + c * y),
            normalizeAngle(-theta)
        };
    }

    // ── Point transformation ─────────────────────────────────────────────────

    /// Apply this transform to a 2-D point expressed in the child frame.
    [[nodiscard]] Eigen::Vector2d transformPoint(const Eigen::Vector2d& p) const noexcept {
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        return Eigen::Vector2d{
            c * p.x() - s * p.y() + x,
            s * p.x() + c * p.y() + y
        };
    }

    // ── Matrix representation ────────────────────────────────────────────────

    /// Returns the homogeneous 3x3 matrix representation:
    /// [ cos -sin  tx ]
    /// [ sin  cos  ty ]
    /// [   0    0   1 ]
    [[nodiscard]] Eigen::Matrix3d toMatrix() const noexcept {
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        Eigen::Matrix3d m;
        m << c, -s, x,
             s,  c, y,
             0,  0, 1;
        return m;
    }

    /// Reconstruct an SE2 from a homogeneous 3x3 matrix.
    /// @pre The matrix must be a valid SE(2) element (orthonormal rotation block,
    ///      last row [0 0 1]).
    [[nodiscard]] static SE2 fromMatrix(const Eigen::Matrix3d& m) noexcept {
        return SE2{
            m(0, 2),
            m(1, 2),
            std::atan2(m(1, 0), m(0, 0))
        };
    }

    // ── Interpolation ────────────────────────────────────────────────────────

    /// Linear interpolation between this transform and @p other.
    /// @p t == 0 returns *this; @p t == 1 returns @p other.
    /// Translation is linearly blended; rotation uses shortest-arc blending.
    [[nodiscard]] SE2 interpolate(const SE2& other, double t) const noexcept {
        const double ix = x + t * (other.x - x);
        const double iy = y + t * (other.y - y);
        // Shortest-arc angular interpolation
        double dtheta = normalizeAngle(other.theta - theta);
        const double itheta = normalizeAngle(theta + t * dtheta);
        return SE2{ix, iy, itheta};
    }

    // ── Comparison ───────────────────────────────────────────────────────────

    /// Equality with a small epsilon tolerance for floating-point comparison.
    [[nodiscard]] bool operator==(const SE2& other) const noexcept {
        constexpr double kEps = 1e-9;
        return std::abs(x - other.x) < kEps &&
               std::abs(y - other.y) < kEps &&
               std::abs(normalizeAngle(theta - other.theta)) < kEps;
    }

    [[nodiscard]] bool operator!=(const SE2& other) const noexcept {
        return !(*this == other);
    }

private:
    /// Wrap angle to (-pi, pi].
    [[nodiscard]] static double normalizeAngle(double a) noexcept {
        while (a >  std::numbers::pi) a -= 2.0 * std::numbers::pi;
        while (a <= -std::numbers::pi) a += 2.0 * std::numbers::pi;
        return a;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// SO3 — 3-D Special Orthogonal group
// Represents a pure 3-D rotation stored as a unit quaternion.
// Declared before SE3 because SE3::toSE2 needs SE2, and SE3 uses SO3 helpers.
// ─────────────────────────────────────────────────────────────────────────────

struct SO3 {
    Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};

    // ── Factories ────────────────────────────────────────────────────────────

    /// Returns the identity rotation (no rotation).
    [[nodiscard]] static SO3 identity() noexcept {
        return SO3{Eigen::Quaterniond::Identity()};
    }

    /// Construct from an axis-angle representation.
    /// @param axis  Unit vector specifying the rotation axis (need not be normalised;
    ///              normalisation is performed internally).
    /// @param angle Rotation angle in radians.
    [[nodiscard]] static SO3 fromAxisAngle(const Eigen::Vector3d& axis, double angle) noexcept {
        return SO3{Eigen::Quaterniond{Eigen::AngleAxisd{angle, axis.normalized()}}};
    }

    /// Construct from roll-pitch-yaw angles (intrinsic ZYX convention):
    /// R = Rz(yaw) * Ry(pitch) * Rx(roll)
    [[nodiscard]] static SO3 fromRPY(double roll, double pitch, double yaw) noexcept {
        const Eigen::Quaterniond quat =
            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
        return SO3{quat};
    }

    // ── Group operations ─────────────────────────────────────────────────────

    /// Compose rotations: R_result = R_this * R_other.
    [[nodiscard]] SO3 compose(const SO3& other) const noexcept {
        return SO3{(q * other.q).normalized()};
    }

    /// Returns the inverse rotation (conjugate of the unit quaternion).
    [[nodiscard]] SO3 inverse() const noexcept {
        return SO3{q.conjugate()};
    }

    // ── Vector rotation ──────────────────────────────────────────────────────

    /// Rotate a 3-D vector by this rotation.
    [[nodiscard]] Eigen::Vector3d rotate(const Eigen::Vector3d& v) const noexcept {
        return q * v;
    }

    // ── Matrix representation ────────────────────────────────────────────────

    /// Returns the 3x3 rotation matrix.
    [[nodiscard]] Eigen::Matrix3d toMatrix() const noexcept {
        return q.toRotationMatrix();
    }

    // ── Euler angles ─────────────────────────────────────────────────────────

    /// Extract roll, pitch, yaw angles (intrinsic ZYX convention).
    /// Returns Eigen::Vector3d{roll, pitch, yaw}.
    [[nodiscard]] Eigen::Vector3d toRPY() const noexcept {
        // eulerAngles(2,1,0) returns [yaw, pitch, roll] in ZYX order
        const Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
        return Eigen::Vector3d{euler(2), euler(1), euler(0)};  // {roll, pitch, yaw}
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// SE3 — 3-D Special Euclidean group
// Represents a 3-D rigid-body pose: translation vector + unit quaternion.
// ─────────────────────────────────────────────────────────────────────────────

struct SE3 {
    Eigen::Vector3d    translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};

    // ── Factories ────────────────────────────────────────────────────────────

    /// Returns the identity transform.
    [[nodiscard]] static SE3 identity() noexcept {
        return SE3{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
    }

    /// Construct a pure-translation transform.
    [[nodiscard]] static SE3 fromTranslation(const Eigen::Vector3d& t) noexcept {
        return SE3{t, Eigen::Quaterniond::Identity()};
    }

    /// Construct a pure-rotation transform.
    [[nodiscard]] static SE3 fromRotation(const Eigen::Quaterniond& q) noexcept {
        return SE3{Eigen::Vector3d::Zero(), q.normalized()};
    }

    /// Reconstruct from a homogeneous 4x4 matrix.
    /// @pre The matrix must be a valid SE(3) element (orthonormal 3x3 rotation
    ///      block, last row [0 0 0 1]).
    [[nodiscard]] static SE3 fromMatrix(const Eigen::Matrix4d& m) noexcept {
        Eigen::Matrix3d R = m.block<3, 3>(0, 0);
        Eigen::Vector3d t = m.block<3, 1>(0, 3);
        return SE3{t, Eigen::Quaterniond{R}.normalized()};
    }

    // ── Group operations ─────────────────────────────────────────────────────

    /// Compose: T_result = T_this * T_other.
    [[nodiscard]] SE3 compose(const SE3& other) const noexcept {
        return SE3{
            translation + rotation * other.translation,
            (rotation * other.rotation).normalized()
        };
    }

    /// Returns the inverse transform: T^{-1}.
    [[nodiscard]] SE3 inverse() const noexcept {
        const Eigen::Quaterniond inv_rot = rotation.conjugate();
        return SE3{
            -(inv_rot * translation),
            inv_rot
        };
    }

    // ── Point transformation ─────────────────────────────────────────────────

    /// Apply this transform to a 3-D point expressed in the child frame.
    [[nodiscard]] Eigen::Vector3d transformPoint(const Eigen::Vector3d& p) const noexcept {
        return rotation * p + translation;
    }

    // ── Matrix representation ────────────────────────────────────────────────

    /// Returns the homogeneous 4x4 matrix representation:
    /// [ R  t ]
    /// [ 0  1 ]
    [[nodiscard]] Eigen::Matrix4d toMatrix() const noexcept {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        m.block<3, 1>(0, 3) = translation;
        return m;
    }

    // ── Projection to 2-D ────────────────────────────────────────────────────

    /// Project this 3-D transform to a 2-D SE2 by discarding the z component
    /// of translation and using the yaw angle of the rotation.
    /// Useful for projecting a 3-D robot pose (e.g. from wheel odometry fused
    /// with IMU) onto the ground plane.
    [[nodiscard]] SE2 toSE2() const noexcept {
        // Extract yaw from quaternion (rotation about Z axis)
        const Eigen::Vector3d euler = SO3{rotation}.toRPY();  // {roll, pitch, yaw}
        return SE2{translation.x(), translation.y(), euler(2)};
    }
};

}  // namespace transforms
