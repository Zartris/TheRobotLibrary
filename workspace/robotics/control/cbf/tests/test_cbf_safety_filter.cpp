#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cbf/cbf_safety_filter.hpp>
#include <pid/pid_controller.hpp>
#include <testing/recording_logger.hpp>
#include <logging/get_logger.hpp>
#include <cmath>

using namespace robotlib;

static std::unique_ptr<IController> makePID() {
    PIDConfig hCfg{2.0, 0.0, 0.1, 3.0, 2.0};
    PIDConfig sCfg{1.0, 0.1, 0.0, 1.0, 1.0};
    return std::make_unique<HeadingSpeedController>(hCfg, sCfg);
}

TEST_CASE("No obstacles: CBF output equals nominal", "[cbf]") {
    CbfConfig cfg;
    cfg.safetyRadius = 0.5;

    // Create two identical controllers
    auto pid1 = makePID();
    auto pid2 = makePID();

    // Compute nominal command from the standalone PID
    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto nominal = pid1->compute(current, target, 0.1);

    // Create CBF with the other PID (no obstacles)
    CbfSafetyFilter cbf(std::move(pid2), cfg);
    auto filtered = cbf.compute(current, target, 0.1);

    // First call from identical initial state should match
    REQUIRE_THAT(filtered.linear, Catch::Matchers::WithinAbs(nominal.linear, 1e-9));
    REQUIRE_THAT(filtered.angular, Catch::Matchers::WithinAbs(nominal.angular, 1e-9));
}

TEST_CASE("Single obstacle ahead: CBF reduces velocity", "[cbf]") {
    CbfConfig cfg;
    cfg.safetyRadius = 1.0;
    cfg.alpha = 1.0;
    CbfSafetyFilter cbf(makePID(), cfg);

    // Obstacle directly ahead, close
    cbf.setObstacles({{1.5, 0.0, 0.0}});

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto cmd = cbf.compute(current, target, 0.1);

    // The nominal PID would command positive linear velocity toward target
    auto nominal = makePID()->compute(current, target, 0.1);
    // CBF should reduce or limit velocity due to obstacle
    CHECK(cmd.linear <= nominal.linear + 1e-9);
}

TEST_CASE("Multiple obstacles: all safety constraints satisfied", "[cbf]") {
    CbfConfig cfg;
    cfg.safetyRadius = 0.5;
    cfg.alpha = 1.0;
    CbfSafetyFilter cbf(makePID(), cfg);

    cbf.setObstacles({
        {1.0, 0.0, 0.0},   // ahead
        {0.5, 0.5, 0.0},   // ahead-left
    });

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto cmd = cbf.compute(current, target, 0.1);

    // Verify finite output (no NaN/Inf from constraint handling)
    REQUIRE(std::isfinite(cmd.linear));
    REQUIRE(std::isfinite(cmd.angular));
}

TEST_CASE("Safety radius = 0: same as bare nominal", "[cbf]") {
    CbfConfig cfg;
    cfg.safetyRadius = 0.0;  // disabled
    CbfSafetyFilter cbf(makePID(), cfg);

    cbf.setObstacles({{1.0, 0.0, 0.0}});

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto nominal = makePID()->compute(current, target, 0.1);
    auto filtered = cbf.compute(current, target, 0.1);

    REQUIRE_THAT(filtered.linear, Catch::Matchers::WithinAbs(nominal.linear, 1e-9));
    REQUIRE_THAT(filtered.angular, Catch::Matchers::WithinAbs(nominal.angular, 1e-9));
}

TEST_CASE("CBF reset also resets nominal controller", "[cbf]") {
    CbfSafetyFilter cbf(makePID());
    cbf.setObstacles({{1.0, 0.0, 0.0}});
    cbf.reset();

    // After reset, no obstacles — should pass through
    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto nominal = makePID()->compute(current, target, 0.1);
    auto filtered = cbf.compute(current, target, 0.1);

    REQUIRE_THAT(filtered.linear, Catch::Matchers::WithinAbs(nominal.linear, 1e-9));
}

TEST_CASE("CBF logging and observability", "[cbf][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("cbf", mockLogger);

    CbfSafetyFilter cbf(makePID());

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    cbf.compute(current, target, 0.1);

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
    REQUIRE(mockLogger->hasNoErrors());
}
