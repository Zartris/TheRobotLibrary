#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/kinematics/differential_drive.hpp>
#include <logging/get_logger.hpp>
#include <numbers>
#include <testing/recording_logger.hpp>

using namespace robotlib;

TEST_CASE("DifferentialDrive straight line", "[common][kinematics]") {
    DifferentialDrive dd(0.05, 0.3);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{1.0, 0.0};
    auto result = dd.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("DifferentialDrive pure rotation", "[common][kinematics]") {
    DifferentialDrive dd(0.05, 0.3);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{0.0, std::numbers::pi / 2};
    auto result = dd.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(std::numbers::pi / 2, 1e-6));
}

TEST_CASE("DifferentialDrive toTwist/fromTwist round-trip", "[common][kinematics]") {
    DifferentialDrive dd(0.05, 0.3);
    Twist original{0.5, 1.0};
    auto wheels = dd.fromTwist(original);
    auto recovered = dd.toTwist(wheels);
    REQUIRE_THAT(recovered.linear, Catch::Matchers::WithinAbs(original.linear, 1e-9));
    REQUIRE_THAT(recovered.angular, Catch::Matchers::WithinAbs(original.angular, 1e-9));
}

TEST_CASE("DifferentialDrive zero twist", "[common][kinematics]") {
    DifferentialDrive dd(0.05, 0.3);
    Pose2D start{1.0, 2.0, 0.5};
    Twist cmd{0.0, 0.0};
    auto result = dd.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(0.5, 1e-9));
}

TEST_CASE("DifferentialDrive logging on construction", "[common][kinematics]") {
    // Verify construction doesn't crash (logger is initialized)
    DifferentialDrive dd(0.1, 0.5);
    REQUIRE(dd.wheelRadius() == 0.1);
    REQUIRE(dd.trackWidth() == 0.5);
}

TEST_CASE("DifferentialDrive logs initialization at DEBUG level", "[common][logging]") {
    // Register mock logger before constructing DifferentialDrive.
    // The module uses getLogger("common.differential_drive") internally.
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("common.differential_drive", mockLogger);

    // Construct a DifferentialDrive (triggers init logging)
    DifferentialDrive dd(0.05, 0.24);

    // Verify init was logged at DEBUG level with expected content
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "DifferentialDrive initialized"));

    // Verify no errors during construction
    REQUIRE(mockLogger->hasNoErrors());

    // Clean up registry so other tests are unaffected
    robotlib::clearLoggerRegistry();
}
