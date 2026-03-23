#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <logging/get_logger.hpp>
#include <ray_casting/ray_caster.hpp>
#include <testing/recording_logger.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("Ray into empty grid returns max range", "[ray_casting]") {
    OccupancyGrid grid(100, 100, 0.1);
    // Mark all cells as free
    for (auto& c : grid.cells) c = -1;

    RayCaster caster;
    auto result = caster.castRay(grid, {5.0, 5.0}, 0.0, 10.0);

    // Should not hit anything within the grid
    REQUIRE_FALSE(result.hit);
}

TEST_CASE("Ray toward wall at known distance", "[ray_casting]") {
    OccupancyGrid grid(100, 100, 0.1);
    // All free
    for (auto& c : grid.cells) c = -1;
    // Place wall at x=7.0 (column 70)
    for (int y = 0; y < 100; ++y) {
        grid.at(70, y) = 10;  // occupied
    }

    RayCaster caster;
    auto result = caster.castRay(grid, {5.0, 5.0}, 0.0, 10.0);

    REQUIRE(result.hit);
    REQUIRE_THAT(result.range, Catch::Matchers::WithinAbs(2.0, grid.resolution));
}

TEST_CASE("Diagonal ray at 45 degrees", "[ray_casting]") {
    OccupancyGrid grid(100, 100, 0.1);
    for (auto& c : grid.cells) c = -1;
    // Place wall at (7, 7) area
    grid.at(70, 70) = 10;
    grid.at(71, 70) = 10;
    grid.at(70, 71) = 10;

    RayCaster caster;
    auto result = caster.castRay(grid, {5.0, 5.0}, std::numbers::pi / 4, 10.0);

    REQUIRE(result.hit);
    double expectedDist = std::sqrt(2.0) * 2.0;
    REQUIRE_THAT(result.range, Catch::Matchers::WithinAbs(expectedDist, grid.resolution * 2));
}

TEST_CASE("castScan produces correct LaserScan structure", "[ray_casting]") {
    OccupancyGrid grid(100, 100, 0.1);
    for (auto& c : grid.cells) c = -1;

    RayCaster caster;
    ScanConfig config;
    config.angleMin = -0.1;
    config.angleMax = 0.1;
    config.angleIncrement = 0.1;
    config.maxRange = 5.0;

    auto scan = caster.castScan(grid, {5.0, 5.0, 0.0}, config);

    REQUIRE(scan.numRays() == 3);
    REQUIRE_THAT(scan.angleMin, Catch::Matchers::WithinAbs(-0.1, 1e-9));
    REQUIRE_THAT(scan.angleMax, Catch::Matchers::WithinAbs(0.1, 1e-9));
}

TEST_CASE("RayCaster logging and observability", "[ray_casting][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr,
        [](void*) { robotlib::clearLoggerRegistry(); });

    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("ray_casting", mockLogger);

    RayCaster caster;

    // Verify DEBUG log on initialization
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "RayCaster initialized"));

    // Verify no errors during nominal construction
    REQUIRE(mockLogger->hasNoErrors());

    // Run castScan and check TRACE timing log
    OccupancyGrid grid(100, 100, 0.1);
    for (auto& c : grid.cells) c = -1;
    ScanConfig config;
    config.angleMin = -0.1;
    config.angleMax = 0.1;
    config.angleIncrement = 0.1;
    config.maxRange = 5.0;
    caster.castScan(grid, {5.0, 5.0, 0.0}, config);
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
}
