#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/kinematics/ackermann.hpp>
#include <logging/get_logger.hpp>
#include <numbers>
#include <testing/recording_logger.hpp>

using namespace robotlib;

TEST_CASE("Ackermann straight drive (delta=0)", "[common][kinematics]") {
    Ackermann model(1.0, std::numbers::pi / 4);
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{1.0, 0.0};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Ackermann zero speed produces no motion", "[common][kinematics]") {
    Ackermann model(1.0, std::numbers::pi / 4);
    Pose2D start{1.0, 2.0, 0.5};
    Twist cmd{0.0, std::numbers::pi / 4};
    auto result = model.step(start, cmd, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(result.theta, Catch::Matchers::WithinAbs(0.5, 1e-9));
}

TEST_CASE("Ackermann minTurningRadius formula", "[common][kinematics]") {
    const double wheelbase = 1.0;
    const double maxSteering = std::numbers::pi / 4;
    Ackermann model(wheelbase, maxSteering);
    const double expected = wheelbase / std::tan(maxSteering);
    REQUIRE_THAT(model.minTurningRadius(), Catch::Matchers::WithinAbs(expected, 1e-9));
}

TEST_CASE("Ackermann turning radius at max steering >= minTurningRadius", "[common][kinematics]") {
    const double wheelbase = 1.0;
    const double maxSteering = std::numbers::pi / 4;
    Ackermann model(wheelbase, maxSteering);

    // Step at max steering angle for a short duration; measure arc radius
    const double v = 1.0;
    const double dt = 0.01;
    Pose2D start{0.0, 0.0, 0.0};
    Twist cmd{v, maxSteering};
    auto result = model.step(start, cmd, dt);

    // Compute actual turning radius from travelled arc
    const double dx = result.x - start.x;
    const double dy = result.y - start.y;
    const double chordLen = std::sqrt(dx * dx + dy * dy);
    const double dtheta = std::abs(result.theta - start.theta);

    // For small dt, chord ≈ arc; actual radius ~ chord / (2 sin(dtheta/2))
    double actualRadius = chordLen / (2.0 * std::sin(dtheta / 2.0) + 1e-12);
    REQUIRE(actualRadius >= model.minTurningRadius() - 1e-6);
}

TEST_CASE("Ackermann toTwist/fromTwist round-trip", "[common][kinematics]") {
    Ackermann model(1.0, std::numbers::pi / 4);
    Twist original{0.8, 0.3};
    auto wheels = model.fromTwist(original);
    auto recovered = model.toTwist(wheels);
    REQUIRE_THAT(recovered.linear, Catch::Matchers::WithinAbs(original.linear, 1e-9));
    REQUIRE_THAT(recovered.angular, Catch::Matchers::WithinAbs(original.angular, 1e-9));
}

TEST_CASE("Ackermann logging and observability", "[common][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("common.ackermann", mockLogger);

    Ackermann model(1.0, std::numbers::pi / 4);

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));
    REQUIRE(mockLogger->hasNoErrors());
}
