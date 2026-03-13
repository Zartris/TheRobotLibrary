#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <transforms/transforms.hpp>
#include <numbers>

using Catch::Approx;
using namespace transforms;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static constexpr double kEps = 1e-9;

static bool se2Near(const SE2& a, const SE2& b, double eps = 1e-9) {
    auto wrap = [](double angle) {
        while (angle >  std::numbers::pi) angle -= 2.0 * std::numbers::pi;
        while (angle <= -std::numbers::pi) angle += 2.0 * std::numbers::pi;
        return angle;
    };
    return std::abs(a.x - b.x) < eps &&
           std::abs(a.y - b.y) < eps &&
           std::abs(wrap(a.theta - b.theta)) < eps;
}

static bool vec2Near(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double eps = 1e-9) {
    return (a - b).norm() < eps;
}

static bool vec3Near(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double eps = 1e-9) {
    return (a - b).norm() < eps;
}

static bool mat3Near(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b, double eps = 1e-9) {
    return (a - b).norm() < eps;
}

static bool mat4Near(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, double eps = 1e-9) {
    return (a - b).norm() < eps;
}

static bool se3Near(const SE3& a, const SE3& b, double eps = 1e-9) {
    return (a.translation - b.translation).norm() < eps &&
           std::abs(std::abs(a.rotation.dot(b.rotation)) - 1.0) < eps;
}

// ─────────────────────────────────────────────────────────────────────────────
// SE2 tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SE2 identity compose leaves transform unchanged", "[SE2]") {
    const SE2 pose{1.0, 2.0, 0.5};
    const SE2 result = pose.compose(SE2::identity());
    REQUIRE(se2Near(result, pose));
}

TEST_CASE("SE2 identity composed on the left leaves transform unchanged", "[SE2]") {
    const SE2 pose{1.0, 2.0, 0.5};
    const SE2 result = SE2::identity().compose(pose);
    REQUIRE(se2Near(result, pose));
}

TEST_CASE("SE2 compose translates then rotates correctly", "[SE2]") {
    // T1: pure translation (3, 0, 0)
    // T2: pure translation (0, 1, 0) in the local frame of T1
    // result should be (3, 1, 0)
    const SE2 t1{3.0, 0.0, 0.0};
    const SE2 t2{0.0, 1.0, 0.0};
    const SE2 result = t1.compose(t2);
    REQUIRE(result.x     == Approx(3.0).epsilon(kEps));
    REQUIRE(result.y     == Approx(1.0).epsilon(kEps));
    REQUIRE(result.theta == Approx(0.0).epsilon(kEps));
}

TEST_CASE("SE2 compose with rotation applies rotation to child translation", "[SE2]") {
    // T1: 90-degree rotation about origin
    // T2: translate by (1, 0, 0) in local frame
    // Expected: T1 * T2 should place the origin at (0, 1) in world frame
    const SE2 t1{0.0, 0.0, std::numbers::pi / 2.0};
    const SE2 t2{1.0, 0.0, 0.0};
    const SE2 result = t1.compose(t2);
    REQUIRE(result.x     == Approx(0.0).margin(1e-9));
    REQUIRE(result.y     == Approx(1.0).epsilon(1e-9));
    REQUIRE(result.theta == Approx(std::numbers::pi / 2.0).epsilon(1e-9));
}

TEST_CASE("SE2 inverse cancels original", "[SE2]") {
    const SE2 pose{3.0, -1.5, 0.8};
    const SE2 result = pose.compose(pose.inverse());
    REQUIRE(se2Near(result, SE2::identity(), 1e-9));
}

TEST_CASE("SE2 inverse on left cancels original", "[SE2]") {
    const SE2 pose{3.0, -1.5, 0.8};
    const SE2 result = pose.inverse().compose(pose);
    REQUIRE(se2Near(result, SE2::identity(), 1e-9));
}

TEST_CASE("SE2 transformPoint translates a point correctly", "[SE2]") {
    const SE2 t{2.0, 3.0, 0.0};
    const Eigen::Vector2d p{1.0, 0.0};
    const Eigen::Vector2d result = t.transformPoint(p);
    REQUIRE(vec2Near(result, Eigen::Vector2d{3.0, 3.0}));
}

TEST_CASE("SE2 transformPoint applies rotation correctly", "[SE2]") {
    // 90-degree rotation about origin, point at (1, 0) should map to (0, 1)
    const SE2 t{0.0, 0.0, std::numbers::pi / 2.0};
    const Eigen::Vector2d p{1.0, 0.0};
    const Eigen::Vector2d result = t.transformPoint(p);
    REQUIRE(result.x() == Approx(0.0).margin(1e-9));
    REQUIRE(result.y() == Approx(1.0).epsilon(1e-9));
}

TEST_CASE("SE2 toMatrix / fromMatrix round-trip", "[SE2]") {
    const SE2 original{1.5, -2.3, std::numbers::pi / 4.0};
    const Eigen::Matrix3d mat = original.toMatrix();
    const SE2 recovered = SE2::fromMatrix(mat);
    REQUIRE(se2Near(recovered, original, 1e-9));
}

TEST_CASE("SE2 toMatrix has correct structure", "[SE2]") {
    const SE2 t{1.0, 2.0, 0.0};
    const Eigen::Matrix3d m = t.toMatrix();
    // Last row must be [0 0 1]
    REQUIRE(m(2, 0) == Approx(0.0).margin(1e-12));
    REQUIRE(m(2, 1) == Approx(0.0).margin(1e-12));
    REQUIRE(m(2, 2) == Approx(1.0).epsilon(1e-12));
    // Translation column
    REQUIRE(m(0, 2) == Approx(1.0).epsilon(1e-12));
    REQUIRE(m(1, 2) == Approx(2.0).epsilon(1e-12));
}

TEST_CASE("SE2 interpolate at t=0 returns self", "[SE2]") {
    const SE2 a{0.0, 0.0, 0.0};
    const SE2 b{2.0, 4.0, std::numbers::pi / 2.0};
    const SE2 result = a.interpolate(b, 0.0);
    REQUIRE(se2Near(result, a));
}

TEST_CASE("SE2 interpolate at t=1 returns other", "[SE2]") {
    const SE2 a{0.0, 0.0, 0.0};
    const SE2 b{2.0, 4.0, std::numbers::pi / 2.0};
    const SE2 result = a.interpolate(b, 1.0);
    REQUIRE(se2Near(result, b, 1e-9));
}

TEST_CASE("SE2 interpolate at t=0.5 gives midpoint", "[SE2]") {
    const SE2 a{0.0, 0.0, 0.0};
    const SE2 b{2.0, 4.0, std::numbers::pi};
    const SE2 mid = a.interpolate(b, 0.5);
    REQUIRE(mid.x     == Approx(1.0).epsilon(1e-9));
    REQUIRE(mid.y     == Approx(2.0).epsilon(1e-9));
    REQUIRE(mid.theta == Approx(std::numbers::pi / 2.0).epsilon(1e-9));
}

TEST_CASE("SE2 operator== with identical values", "[SE2]") {
    const SE2 a{1.0, 2.0, 0.5};
    const SE2 b{1.0, 2.0, 0.5};
    REQUIRE(a == b);
}

TEST_CASE("SE2 operator!= with different values", "[SE2]") {
    const SE2 a{1.0, 2.0, 0.5};
    const SE2 b{1.0, 2.0, 0.6};
    REQUIRE(a != b);
}

// ─────────────────────────────────────────────────────────────────────────────
// SE3 tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SE3 identity compose leaves transform unchanged", "[SE3]") {
    const SE3 pose{Eigen::Vector3d{1.0, 2.0, 3.0},
                   Eigen::Quaterniond{Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitZ()}}};
    const SE3 result = pose.compose(SE3::identity());
    REQUIRE(se3Near(result, pose));
}

TEST_CASE("SE3 identity composed on the left leaves transform unchanged", "[SE3]") {
    const SE3 pose{Eigen::Vector3d{1.0, 2.0, 3.0},
                   Eigen::Quaterniond{Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitZ()}}};
    const SE3 result = SE3::identity().compose(pose);
    REQUIRE(se3Near(result, pose));
}

TEST_CASE("SE3 compose then inverse equals identity", "[SE3]") {
    const SE3 pose{
        Eigen::Vector3d{1.0, -2.0, 0.5},
        Eigen::Quaterniond{Eigen::AngleAxisd{0.7, Eigen::Vector3d{1.0, 0.5, 0.3}.normalized()}}
    };
    const SE3 result = pose.compose(pose.inverse());
    REQUIRE(se3Near(result, SE3::identity(), 1e-9));
}

TEST_CASE("SE3 inverse on left compose equals identity", "[SE3]") {
    const SE3 pose{
        Eigen::Vector3d{-0.5, 1.2, 3.0},
        Eigen::Quaterniond{Eigen::AngleAxisd{1.1, Eigen::Vector3d::UnitX()}}
    };
    const SE3 result = pose.inverse().compose(pose);
    REQUIRE(se3Near(result, SE3::identity(), 1e-9));
}

TEST_CASE("SE3 transformPoint pure translation", "[SE3]") {
    const SE3 t = SE3::fromTranslation(Eigen::Vector3d{1.0, 2.0, 3.0});
    const Eigen::Vector3d p{0.0, 0.0, 0.0};
    const Eigen::Vector3d result = t.transformPoint(p);
    REQUIRE(vec3Near(result, Eigen::Vector3d{1.0, 2.0, 3.0}));
}

TEST_CASE("SE3 transformPoint pure rotation", "[SE3]") {
    // 90 degrees about Z: (1,0,0) -> (0,1,0)
    const SE3 t = SE3::fromRotation(
        Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()}});
    const Eigen::Vector3d p{1.0, 0.0, 0.0};
    const Eigen::Vector3d result = t.transformPoint(p);
    REQUIRE(result.x() == Approx(0.0).margin(1e-9));
    REQUIRE(result.y() == Approx(1.0).epsilon(1e-9));
    REQUIRE(result.z() == Approx(0.0).margin(1e-9));
}

TEST_CASE("SE3 toMatrix / fromMatrix round-trip", "[SE3]") {
    const SE3 original{
        Eigen::Vector3d{1.0, -2.0, 0.5},
        Eigen::Quaterniond{Eigen::AngleAxisd{0.4, Eigen::Vector3d{0.0, 0.0, 1.0}}}
    };
    const Eigen::Matrix4d mat = original.toMatrix();
    const SE3 recovered = SE3::fromMatrix(mat);
    REQUIRE(se3Near(recovered, original, 1e-9));
}

TEST_CASE("SE3 toMatrix last row is [0 0 0 1]", "[SE3]") {
    const SE3 t{Eigen::Vector3d{1.0, 2.0, 3.0},
                Eigen::Quaterniond{Eigen::AngleAxisd{0.5, Eigen::Vector3d::UnitY()}}};
    const Eigen::Matrix4d m = t.toMatrix();
    REQUIRE(m(3, 0) == Approx(0.0).margin(1e-12));
    REQUIRE(m(3, 1) == Approx(0.0).margin(1e-12));
    REQUIRE(m(3, 2) == Approx(0.0).margin(1e-12));
    REQUIRE(m(3, 3) == Approx(1.0).epsilon(1e-12));
}

TEST_CASE("SE3 toSE2 projects translation and yaw correctly", "[SE3]") {
    const double yaw = std::numbers::pi / 4.0;
    const SE3 pose{
        Eigen::Vector3d{2.0, 3.0, 1.5},
        Eigen::Quaterniond{Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}}
    };
    const SE2 projected = pose.toSE2();
    REQUIRE(projected.x     == Approx(2.0).epsilon(1e-9));
    REQUIRE(projected.y     == Approx(3.0).epsilon(1e-9));
    REQUIRE(projected.theta == Approx(yaw).epsilon(1e-9));
}

TEST_CASE("SE3 fromTranslation has identity rotation", "[SE3]") {
    const SE3 t = SE3::fromTranslation(Eigen::Vector3d{1.0, 2.0, 3.0});
    REQUIRE(t.rotation.isApprox(Eigen::Quaterniond::Identity(), 1e-12));
}

TEST_CASE("SE3 fromRotation has zero translation", "[SE3]") {
    const SE3 t = SE3::fromRotation(
        Eigen::Quaterniond{Eigen::AngleAxisd{0.5, Eigen::Vector3d::UnitX()}});
    REQUIRE(t.translation.isZero(1e-12));
}

// ─────────────────────────────────────────────────────────────────────────────
// SO3 tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SO3 identity rotation leaves vector unchanged", "[SO3]") {
    const SO3 r = SO3::identity();
    const Eigen::Vector3d v{1.0, 2.0, 3.0};
    REQUIRE(vec3Near(r.rotate(v), v));
}

TEST_CASE("SO3 fromAxisAngle rotates correctly", "[SO3]") {
    // 90-degree rotation about Z: (1,0,0) -> (0,1,0)
    const SO3 r = SO3::fromAxisAngle(Eigen::Vector3d::UnitZ(), std::numbers::pi / 2.0);
    const Eigen::Vector3d result = r.rotate(Eigen::Vector3d{1.0, 0.0, 0.0});
    REQUIRE(result.x() == Approx(0.0).margin(1e-9));
    REQUIRE(result.y() == Approx(1.0).epsilon(1e-9));
    REQUIRE(result.z() == Approx(0.0).margin(1e-9));
}

TEST_CASE("SO3 fromRPY toRPY round-trip (zero angles)", "[SO3]") {
    const SO3 r = SO3::fromRPY(0.0, 0.0, 0.0);
    const Eigen::Vector3d rpy = r.toRPY();
    REQUIRE(rpy(0) == Approx(0.0).margin(1e-9));
    REQUIRE(rpy(1) == Approx(0.0).margin(1e-9));
    REQUIRE(rpy(2) == Approx(0.0).margin(1e-9));
}

TEST_CASE("SO3 fromRPY toRPY round-trip (non-trivial angles)", "[SO3]") {
    const double roll  =  0.1;
    const double pitch =  0.2;
    const double yaw   = -0.3;
    const SO3 r = SO3::fromRPY(roll, pitch, yaw);
    const Eigen::Vector3d rpy = r.toRPY();
    REQUIRE(rpy(0) == Approx(roll).epsilon(1e-9));
    REQUIRE(rpy(1) == Approx(pitch).epsilon(1e-9));
    REQUIRE(rpy(2) == Approx(yaw).epsilon(1e-9));
}

TEST_CASE("SO3 toMatrix is orthonormal", "[SO3]") {
    const SO3 r = SO3::fromRPY(0.3, -0.2, 1.1);
    const Eigen::Matrix3d R = r.toMatrix();
    // R^T * R should be identity
    const Eigen::Matrix3d RtR = R.transpose() * R;
    REQUIRE(mat3Near(RtR, Eigen::Matrix3d::Identity(), 1e-9));
    // det(R) should be +1
    REQUIRE(R.determinant() == Approx(1.0).epsilon(1e-9));
}

TEST_CASE("SO3 compose and then inverse gives identity", "[SO3]") {
    const SO3 r = SO3::fromAxisAngle(Eigen::Vector3d{1.0, 1.0, 0.0}.normalized(),
                                     std::numbers::pi / 3.0);
    const SO3 result = r.compose(r.inverse());
    const Eigen::Matrix3d R = result.toMatrix();
    REQUIRE(mat3Near(R, Eigen::Matrix3d::Identity(), 1e-9));
}

TEST_CASE("SO3 compose applies rotations in correct order", "[SO3]") {
    // R1: 90 deg about Z,  R2: 90 deg about X
    // R1 * R2: first rotate 90 about Z, then 90 about X (in local frame)
    const SO3 r1 = SO3::fromAxisAngle(Eigen::Vector3d::UnitZ(), std::numbers::pi / 2.0);
    const SO3 r2 = SO3::fromAxisAngle(Eigen::Vector3d::UnitX(), std::numbers::pi / 2.0);
    const SO3 combined = r1.compose(r2);
    // Apply to (1,0,0): first r1 gives (0,1,0), then r2 rotates that:
    //   r2 about X on (0,1,0): y->z so result is (0,0,1)
    const Eigen::Vector3d result = combined.rotate(Eigen::Vector3d{1.0, 0.0, 0.0});
    REQUIRE(result.x() == Approx(0.0).margin(1e-9));
    REQUIRE(result.y() == Approx(0.0).margin(1e-9));
    REQUIRE(result.z() == Approx(1.0).epsilon(1e-9));
}

TEST_CASE("SO3 inverse of identity is identity", "[SO3]") {
    const SO3 r = SO3::identity();
    const SO3 inv = r.inverse();
    REQUIRE(mat3Near(inv.toMatrix(), Eigen::Matrix3d::Identity(), 1e-9));
}
