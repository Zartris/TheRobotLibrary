#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <ekf/ekf2d.hpp>
#include <logging/get_logger.hpp>
#include <testing/recording_logger.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("EKF predict with zero twist - pose unchanged, covariance grows", "[ekf]") {
    EKF2D ekf;
    ekf.reset({1.0, 2.0, 0.5});
    auto covBefore = ekf.getCovariance();

    ekf.predict({0.0, 0.0}, 1.0);

    auto pose = ekf.getPose();
    REQUIRE_THAT(pose.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(pose.y, Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(pose.theta, Catch::Matchers::WithinAbs(0.5, 1e-6));

    auto covAfter = ekf.getCovariance();
    // Covariance should grow
    REQUIRE(covAfter.trace() > covBefore.trace());
}

TEST_CASE("EKF predict with known twist for 1s", "[ekf]") {
    EKF2D ekf;
    ekf.reset({0.0, 0.0, 0.0});

    ekf.predict({1.0, 0.0}, 1.0);

    auto pose = ekf.getPose();
    REQUIRE_THAT(pose.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(pose.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("EKF update with perfect measurement - covariance shrinks", "[ekf]") {
    EKF2D ekf;
    ekf.reset({0.0, 0.0, 0.0});
    ekf.predict({1.0, 0.0}, 1.0);

    auto covBefore = ekf.getCovariance();

    // Perfect measurement at the true position
    Eigen::VectorXd z(3);
    z << 1.0, 0.0, 0.0;
    Eigen::MatrixXd H = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R = Eigen::Matrix3d::Identity() * 0.01;

    ekf.update(z, H, R);

    auto covAfter = ekf.getCovariance();
    REQUIRE(covAfter.trace() < covBefore.trace());

    auto pose = ekf.getPose();
    REQUIRE_THAT(pose.x, Catch::Matchers::WithinAbs(1.0, 0.1));
}

TEST_CASE("EKF predict-update cycle converges", "[ekf]") {
    EKFConfig cfg;
    cfg.Q = Eigen::Matrix3d::Identity() * 0.001;
    EKF2D ekf(cfg);
    ekf.reset({0.0, 0.0, 0.0});

    const double trueX = 5.0;
    Eigen::VectorXd z(3);
    z << trueX, 0.0, 0.0;
    Eigen::MatrixXd H = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R = Eigen::Matrix3d::Identity() * 0.1;

    for (int i = 0; i < 50; ++i) {
        ekf.predict({0.0, 0.0}, 0.1);
        ekf.update(z, H, R);
    }

    auto pose = ekf.getPose();
    REQUIRE_THAT(pose.x, Catch::Matchers::WithinAbs(trueX, 0.5));
}

TEST_CASE("EKF2D logging and observability", "[ekf][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr,
        [](void*) { robotlib::clearLoggerRegistry(); });

    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("ekf", mockLogger);

    EKFConfig cfg;
    EKF2D ekf(cfg);

    // Verify DEBUG log on initialization
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "EKF2D initialized"));

    // Verify no errors during nominal construction
    REQUIRE(mockLogger->hasNoErrors());

    // Run predict and check TRACE timing log
    ekf.predict({1.0, 0.0}, 0.1);
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
}
