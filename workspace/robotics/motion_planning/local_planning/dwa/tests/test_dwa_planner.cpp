#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <dwa/dwa_planner.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("DWA open space with goal ahead - forward velocity", "[dwa]") {
    DWAPlanner planner;

    Pose2D pose{0.0, 0.0, 0.0};
    Twist vel{0.0, 0.0};
    Path path = {{5.0, 0.0, 0.0}};
    LaserScan scan;  // empty scan = no obstacles
    OccupancyGrid grid;

    auto cmd = planner.compute(pose, vel, path, scan, grid);

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
    LaserScan scan;
    scan.angleMin = -0.3;
    scan.angleMax = 0.3;
    scan.angleIncrement = 0.05;
    scan.rangeMin = 0.1;
    scan.rangeMax = 10.0;
    const int numRays = static_cast<int>((scan.angleMax - scan.angleMin) / scan.angleIncrement) + 1;
    scan.ranges.resize(numRays, 0.6f);

    OccupancyGrid grid;
    auto cmd = planner.compute(pose, vel, path, scan, grid);

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
    LaserScan scan;
    scan.angleMin = -std::numbers::pi;
    scan.angleMax = std::numbers::pi;
    scan.angleIncrement = std::numbers::pi / 4;
    scan.rangeMin = 0.1;
    scan.rangeMax = 10.0;
    scan.ranges.resize(9, 0.2f);

    OccupancyGrid grid;
    auto cmd = planner.compute(pose, vel, path, scan, grid);

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
    LaserScan scan;
    OccupancyGrid grid;

    auto cmd = planner.compute(pose, vel, path, scan, grid);

    // Linear velocity shouldn't exceed acceleration * dt from current
    REQUIRE(std::abs(cmd.linear) <= cfg.linearAccel * cfg.simDt + 1e-6);
}
