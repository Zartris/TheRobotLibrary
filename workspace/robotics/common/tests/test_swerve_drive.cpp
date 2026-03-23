#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/kinematics/swerve_drive.hpp>
#include <logging/get_logger.hpp>
#include <numbers>
#include <testing/recording_logger.hpp>

using namespace robotlib;

TEST_CASE("SwerveDrive forward motion", "[common][kinematics]") {
    SwerveDrive model(0.3, 0.3);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{1.0, 0.0};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("SwerveDrive rotation in place", "[common][kinematics]") {
    SwerveDrive model(0.3, 0.3);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{0.0, 1.0};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("SwerveDrive combined arc motion", "[common][kinematics]") {
    SwerveDrive model(0.3, 0.3);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{1.0, 1.0};
    auto result = model.step(start, cmd, 1.0);
    // Arc integration: r = v/omega = 1.0, theta_new = 1.0 rad
    const double r = 1.0;
    const double theta_new = 1.0;
    const double expected_x = r * (std::sin(theta_new) - std::sin(0.0));
    const double expected_y = -r * (std::cos(theta_new) - std::cos(0.0));
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(expected_x, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(expected_y, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(theta_new, 1e-6));
}

TEST_CASE("SwerveDrive toTwist/fromTwist round-trip", "[common][kinematics]") {
    SwerveDrive model(0.3, 0.3);
    Twist original{0.5, 1.0};
    auto wheels = model.fromTwist(original);
    auto recovered = model.toTwist(wheels);
    REQUIRE_THAT(recovered.linear, Catch::Matchers::WithinAbs(original.linear, 1e-9));
    REQUIRE_THAT(recovered.angular, Catch::Matchers::WithinAbs(original.angular, 1e-9));
}

TEST_CASE("SwerveDrive logging and observability", "[common][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("common.swerve_drive", mockLogger);

    SwerveDrive model(0.3, 0.3);

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));
    REQUIRE(mockLogger->hasNoErrors());
}
