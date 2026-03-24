#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/kinematics/unicycle.hpp>
#include <logging/get_logger.hpp>
#include <numbers>
#include <testing/recording_logger.hpp>

using namespace robotlib;

TEST_CASE("Unicycle forward motion", "[common][kinematics]") {
    Unicycle model(1.0, 2.0);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{1.0, 0.0};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Unicycle pure rotation", "[common][kinematics]") {
    Unicycle model(1.0, 2.0);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{0.0, std::numbers::pi / 2};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(std::numbers::pi / 2, 1e-6));
}

TEST_CASE("Unicycle zero input", "[common][kinematics]") {
    Unicycle model(1.0, 2.0);
    Pose2D start{1.0, 2.0, 0.5};
    Twist cmd{0.0, 0.0};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(0.5, 1e-9));
}

TEST_CASE("Unicycle toTwist/fromTwist round-trip", "[common][kinematics]") {
    Unicycle model(1.0, 2.0);
    Twist original{0.5, 1.0};
    auto wheels = model.fromTwist(original);
    auto recovered = model.toTwist(wheels);
    REQUIRE_THAT(recovered.linear, Catch::Matchers::WithinAbs(original.linear, 1e-9));
    REQUIRE_THAT(recovered.angular, Catch::Matchers::WithinAbs(original.angular, 1e-9));
}

TEST_CASE("Unicycle logging and observability", "[common][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("common.unicycle", mockLogger);

    Unicycle model(1.0, 2.0);

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));
    REQUIRE(mockLogger->hasNoErrors());
}
