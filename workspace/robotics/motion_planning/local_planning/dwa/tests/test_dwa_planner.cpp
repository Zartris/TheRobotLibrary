#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <dwa/dwa_planner.hpp>
#include <logging/get_logger.hpp>
#include <testing/recording_logger.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("DWA open space with goal ahead - forward velocity", "[dwa]") {
    DWAPlanner planner;

    Pose2D pose{0.0, 0.0, 0.0};
    Twist vel{0.0, 0.0};
    Path path = {{5.0, 0.0, 0.0}};
    PerceptionContext ctx;  // empty scan = no obstacles, empty grid

    auto cmd = planner.compute(pose, vel, path, ctx);

    REQUIRE(cmd.linear > 0.0);
}

TEST_CASE("DWA obstacle ahead - turning velocity", "[dwa]") {
    DWAConfig cfg;
    cfg.obstacleThreshold = 0.5;
    cfg.linearAccel = 2.0;
    cfg.angularAccel = 4.0;
    DWAPlanner planner(cfg);

    Pose2D pose{0.0, 0.0, 0.0};
    Twist vel{0.0, 0.0};
    Path path = {{5.0, 0.0, 0.0}};

    // Dense obstacle wall directly ahead at 0.6m
    PerceptionContext ctx;
    ctx.scan.angleMin = -0.3;
    ctx.scan.angleMax = 0.3;
    ctx.scan.angleIncrement = 0.05;
    ctx.scan.rangeMin = 0.1;
    ctx.scan.rangeMax = 10.0;
    const int numRays = static_cast<int>((ctx.scan.angleMax - ctx.scan.angleMin) / ctx.scan.angleIncrement) + 1;
    ctx.scan.ranges.resize(numRays, 0.6f);

    auto cmd = planner.compute(pose, vel, path, ctx);

    // Should turn to avoid obstacle
    REQUIRE(std::abs(cmd.angular) > 0.01);
}

TEST_CASE("DWA boxed in - zero twist", "[dwa]") {
    DWAConfig cfg;
    cfg.obstacleThreshold = 0.5;
    DWAPlanner planner(cfg);

    Pose2D pose{0.0, 0.0, 0.0};
    Twist vel{0.0, 0.0};
    Path path = {{5.0, 0.0, 0.0}};

    // Obstacles all around at close range
    PerceptionContext ctx;
    ctx.scan.angleMin = -std::numbers::pi;
    ctx.scan.angleMax = std::numbers::pi;
    ctx.scan.angleIncrement = std::numbers::pi / 4;
    ctx.scan.rangeMin = 0.1;
    ctx.scan.rangeMax = 10.0;
    ctx.scan.ranges.resize(9, 0.2f);

    auto cmd = planner.compute(pose, vel, path, ctx);

    REQUIRE_THAT(cmd.linear, Catch::Matchers::WithinAbs(0.0, 0.3));
}

TEST_CASE("DWA respects dynamic window", "[dwa]") {
    DWAConfig cfg;
    cfg.linearAccel = 0.5;
    cfg.simDt = 0.1;
    DWAPlanner planner(cfg);

    Pose2D pose{0.0, 0.0, 0.0};
    Twist vel{0.0, 0.0};
    Path path = {{5.0, 0.0, 0.0}};
    PerceptionContext ctx;  // empty scan and grid

    auto cmd = planner.compute(pose, vel, path, ctx);

    // Linear velocity shouldn't exceed acceleration * dt from current
    REQUIRE(std::abs(cmd.linear) <= cfg.linearAccel * cfg.simDt + 1e-6);
}

TEST_CASE("DWAPlanner logging and observability", "[dwa][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr,
        [](void*) { robotlib::clearLoggerRegistry(); });

    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("dwa", mockLogger);

    DWAPlanner planner;

    // Verify DEBUG log on initialization
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "DWAPlanner initialized"));

    // Verify no errors during nominal construction
    REQUIRE(mockLogger->hasNoErrors());

    // Run compute and check TRACE timing log
    Pose2D pose{0.0, 0.0, 0.0};
    Twist vel{0.0, 0.0};
    Path path = {{5.0, 0.0, 0.0}};
    PerceptionContext ctx;
    planner.compute(pose, vel, path, ctx);
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
}
