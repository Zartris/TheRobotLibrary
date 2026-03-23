#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <logging/get_logger.hpp>
#include <occupancy_grid/occupancy_grid_map.hpp>
#include <testing/recording_logger.hpp>
#include <cmath>
#include <numbers>

using namespace robotlib;

TEST_CASE("Empty grid has all cells Unknown", "[occupancy_grid]") {
    OccupancyGridMap map(50, 50, 0.1);
    const auto& grid = map.grid();
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            REQUIRE(grid.cellState(x, y) == CellState::Unknown);
        }
    }
}

TEST_CASE("Single ray update marks free and occupied", "[occupancy_grid]") {
    OccupancyGridMap map(100, 100, 0.1);

    // Robot at center (5, 5), fire a ray to the right at 2m
    Pose2D robotPose{5.0, 5.0, 0.0};
    LaserScan scan;
    scan.angleMin = 0.0;
    scan.angleMax = 0.0;
    scan.angleIncrement = 1.0;
    scan.rangeMin = 0.1;
    scan.rangeMax = 10.0;
    scan.ranges = {2.0f};

    map.updateFromScan(robotPose, scan);

    const auto& grid = map.grid();

    // Cells along the ray (between robot and hit) should be free
    auto [rx, ry] = grid.toGrid(5.0, 5.0);
    auto [hx, hy] = grid.toGrid(7.0, 5.0);

    // A cell halfway along the ray should be free
    int midX = (rx + hx) / 2;
    CHECK(grid.cellState(midX, ry) == CellState::Free);

    // Hit cell should be occupied
    CHECK(grid.cellState(hx, hy) == CellState::Occupied);
}

TEST_CASE("Multiple updates accumulate log-odds", "[occupancy_grid]") {
    OccupancyGridMap map(100, 100, 0.1);

    Pose2D robotPose{5.0, 5.0, 0.0};
    LaserScan scan;
    scan.angleMin = 0.0;
    scan.angleMax = 0.0;
    scan.angleIncrement = 1.0;
    scan.rangeMin = 0.1;
    scan.rangeMax = 10.0;
    scan.ranges = {2.0f};

    map.updateFromScan(robotPose, scan);
    auto [hx, hy] = map.grid().toGrid(7.0, 5.0);
    int8_t firstVal = map.grid().at(hx, hy);

    map.updateFromScan(robotPose, scan);
    int8_t secondVal = map.grid().at(hx, hy);

    // Log-odds should accumulate (increase for occupied)
    REQUIRE(secondVal > firstVal);
}

TEST_CASE("Coordinate conversion round-trip", "[occupancy_grid]") {
    OccupancyGrid grid(100, 100, 0.1, {1.0, 2.0});

    int gx = 25;
    int gy = 30;
    auto wp = grid.toWorld(gx, gy);
    auto [rgx, rgy] = grid.toGrid(wp.x, wp.y);

    REQUIRE(rgx == gx);
    REQUIRE(rgy == gy);
}

TEST_CASE("OccupancyGridMap reset clears all cells", "[occupancy_grid]") {
    OccupancyGridMap map(50, 50, 0.1);

    Pose2D robotPose{2.5, 2.5, 0.0};
    LaserScan scan;
    scan.angleMin = 0.0;
    scan.angleMax = 0.0;
    scan.angleIncrement = 1.0;
    scan.rangeMin = 0.1;
    scan.rangeMax = 10.0;
    scan.ranges = {1.0f};
    map.updateFromScan(robotPose, scan);

    map.reset();

    const auto& grid = map.grid();
    for (const auto& c : grid.cells) {
        REQUIRE(c == 0);
    }
}

TEST_CASE("OccupancyGridMap logging and observability", "[occupancy_grid][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr,
        [](void*) { robotlib::clearLoggerRegistry(); });

    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("occupancy_grid", mockLogger);

    OccupancyGridMap map(50, 50, 0.1);

    // Verify DEBUG log on initialization (contains "OccupancyGridMap initialized")
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "OccupancyGridMap initialized"));

    // Verify no errors during nominal construction
    REQUIRE(mockLogger->hasNoErrors());

    // Run updateFromScan and check TRACE timing log
    Pose2D robotPose{2.5, 2.5, 0.0};
    LaserScan scan;
    scan.angleMin = 0.0;
    scan.angleMax = 0.0;
    scan.angleIncrement = 1.0;
    scan.rangeMin = 0.1;
    scan.rangeMax = 10.0;
    scan.ranges = {1.0f};
    map.updateFromScan(robotPose, scan);
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
}
