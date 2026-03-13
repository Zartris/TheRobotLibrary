#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <visual_odometry/visual_odometry.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <numbers>
#include <vector>

using Catch::Approx;
using namespace visual_odometry;

// ── Helpers ───────────────────────────────────────────────────────────────

static CameraIntrinsics makeTestCamera() {
    CameraIntrinsics cam;
    cam.fx     = 500.0;
    cam.fy     = 500.0;
    cam.cx     = 320.0;
    cam.cy     = 240.0;
    cam.width  = 640;
    cam.height = 480;
    return cam;
}

/// Returns the rotation angle (radians) of an isometry, using the angle-axis
/// representation of the rotation matrix.
static double rotationAngle(const Eigen::Isometry3d& T) {
    const Eigen::AngleAxisd aa(T.rotation());
    return std::abs(aa.angle());
}

// ── CameraIntrinsics ──────────────────────────────────────────────────────

TEST_CASE("CameraIntrinsics project roundtrip", "[CameraIntrinsics]") {
    const CameraIntrinsics cam = makeTestCamera();

    // 3D point in front of the camera.
    const Eigen::Vector3d point3d{0.5, -0.3, 2.0};

    const auto pixel_opt = cam.project(point3d);
    REQUIRE(pixel_opt.has_value());

    const Eigen::Vector2d pixel = *pixel_opt;

    // Unproject back to a bearing vector and verify it points in the same direction.
    const Eigen::Vector3d bearing = cam.unproject(pixel);

    // bearing (unit vector) should be proportional to point3d / point3d.z()
    // i.e. the normalised direction of point3d must equal bearing.
    const Eigen::Vector3d expectedDir = point3d.normalized();
    REQUIRE(bearing.dot(expectedDir) == Approx(1.0).epsilon(1e-9));
}

TEST_CASE("CameraIntrinsics project behind camera returns nullopt", "[CameraIntrinsics]") {
    const CameraIntrinsics cam = makeTestCamera();

    // Point with negative z (behind the camera).
    const Eigen::Vector3d behind{0.0, 0.0, -1.0};
    REQUIRE_FALSE(cam.project(behind).has_value());

    // Point exactly on the image plane (z == 0) is also invalid.
    const Eigen::Vector3d onPlane{1.0, 0.0, 0.0};
    REQUIRE_FALSE(cam.project(onPlane).has_value());
}

TEST_CASE("CameraIntrinsics K matrix has correct structure", "[CameraIntrinsics]") {
    const CameraIntrinsics cam = makeTestCamera();
    const Eigen::Matrix3d K = cam.K();

    // Diagonal entries.
    CHECK(K(0, 0) == Approx(cam.fx).epsilon(1e-12));
    CHECK(K(1, 1) == Approx(cam.fy).epsilon(1e-12));
    CHECK(K(2, 2) == Approx(1.0).epsilon(1e-12));

    // Principal point.
    CHECK(K(0, 2) == Approx(cam.cx).epsilon(1e-12));
    CHECK(K(1, 2) == Approx(cam.cy).epsilon(1e-12));

    // Off-diagonal (non-principal-point) entries should be zero.
    CHECK(K(0, 1) == Approx(0.0).margin(1e-12));
    CHECK(K(1, 0) == Approx(0.0).margin(1e-12));
    CHECK(K(2, 0) == Approx(0.0).margin(1e-12));
    CHECK(K(2, 1) == Approx(0.0).margin(1e-12));
    CHECK(K(1, 2) == Approx(cam.cy).epsilon(1e-12));
}

// ── VisualOdometry construction ───────────────────────────────────────────

TEST_CASE("VisualOdometry construction does not crash", "[VisualOdometry]") {
    const CameraIntrinsics cam = makeTestCamera();
    REQUIRE_NOTHROW([&]() {
        VisualOdometry vo(cam);
        CHECK(vo.camera().fx == Approx(cam.fx).epsilon(1e-12));
        CHECK(vo.config().minMatchesRequired == 8);
    }());
}

// ── Insufficient matches ──────────────────────────────────────────────────

TEST_CASE("VisualOdometry returns InsufficientMatches for fewer than 8 matches",
          "[VisualOdometry]") {
    const CameraIntrinsics cam = makeTestCamera();
    VisualOdometryConfig cfg;
    cfg.minMatchesRequired = 8;
    VisualOdometry vo(cam, cfg);

    // Provide only 5 matches — well below the threshold.
    std::vector<PointMatch> sparse;
    for (int i = 0; i < 5; ++i) {
        sparse.push_back(PointMatch{Eigen::Vector2d{320.0 + i * 10.0, 240.0},
                                    Eigen::Vector2d{325.0 + i * 10.0, 240.0}});
    }

    const auto result = vo.estimatePose(sparse);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == VoError::Code::InsufficientMatches);
}

// ── Synthetic pure-translation test ──────────────────────────────────────

TEST_CASE("VisualOdometry pure translation: recovered rotation is near identity",
          "[VisualOdometry]") {
    // Known ground-truth transform: translate 0.1 m along +X, no rotation.
    const Eigen::Isometry3d T_gt = []{
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d{0.1, 0.0, 0.0};
        return T;
    }();

    const CameraIntrinsics cam = makeTestCamera();

    // Generate 3D world points distributed in front of the camera.
    std::vector<Eigen::Vector3d> worldPoints;
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 5; ++col) {
            const double X = (col - 2) * 0.15;
            const double Y = (row - 2) * 0.10;
            const double Z = 2.0 + static_cast<double>(row + col) * 0.05;
            worldPoints.push_back({X, Y, Z});
        }
    }
    // 25 points total — well above the minimum of 8.

    // Project into frame 1 (identity camera) and frame 2 (translated camera).
    std::vector<PointMatch> matches;
    for (const auto& P : worldPoints) {
        const auto px1 = cam.project(P);
        if (!px1.has_value()) { continue; }

        // Transform point into frame-2 coordinates.
        const Eigen::Vector3d P2 = T_gt.inverse() * P;
        const auto px2 = cam.project(P2);
        if (!px2.has_value()) { continue; }

        matches.push_back(PointMatch{*px1, *px2});
    }

    REQUIRE(static_cast<int>(matches.size()) >= 8);

    VisualOdometryConfig cfg;
    cfg.minMatchesRequired  = 8;
    cfg.ransacThresholdPx   = 2.0;
    cfg.maxRansacIterations = 1000;
    cfg.minInlierRatio      = 0.3;

    VisualOdometry vo(cam, cfg);
    const auto result = vo.estimatePose(matches);

    // For a well-conditioned synthetic case with noise-free projections the
    // estimator should succeed.
    REQUIRE(result.has_value());
    CHECK(result->isValid);
    CHECK(result->inlierCount > 0);

    // The rotation component of the recovered pose should be close to identity
    // (within 5 degrees) because ground truth is a pure translation.
    constexpr double FIVE_DEG_RAD = 5.0 * std::numbers::pi / 180.0;
    const double angle = rotationAngle(result->relativePose);
    CHECK(angle < FIVE_DEG_RAD);
}
