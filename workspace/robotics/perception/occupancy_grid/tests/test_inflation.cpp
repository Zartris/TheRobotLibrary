#include <catch2/catch_test_macros.hpp>
#include <occupancy_grid/occupancy_grid_map.hpp>
#include <cmath>

using namespace robotlib;

TEST_CASE("Single occupied cell inflated to circle", "[occupancy_grid][inflation]") {
    OccupancyGridMap map(20, 20, 0.1);  // 2m x 2m, 0.1m resolution
    auto& grid = map.grid();

    // Set center cell as occupied
    grid.cells[10 * 20 + 10] = 10;  // occupied

    auto inflated = map.inflate(0.3);  // 3-cell radius

    // Center should still be occupied
    REQUIRE(inflated.cells[10 * 20 + 10] > 0);

    // Cells within radius should be occupied
    REQUIRE(inflated.cells[10 * 20 + 11] > 0);  // 1 cell right
    REQUIRE(inflated.cells[10 * 20 + 12] > 0);  // 2 cells right
    REQUIRE(inflated.cells[10 * 20 + 13] > 0);  // 3 cells right (0.3m / 0.1m = 3)

    // Cells beyond radius should not be inflated
    // 5 cells away = 0.5m > 0.3m -> should remain free/unknown
    CHECK(inflated.cells[10 * 20 + 15] <= 0);
}

TEST_CASE("Inflation doesn't exceed grid bounds", "[occupancy_grid][inflation]") {
    OccupancyGridMap map(10, 10, 0.1);
    auto& grid = map.grid();

    // Occupied cell at corner (0, 0)
    grid.cells[0] = 10;

    // This should not crash even with large radius
    auto inflated = map.inflate(0.5);

    // Corner should be occupied
    REQUIRE(inflated.cells[0] > 0);
    // Some nearby cells should be inflated
    REQUIRE(inflated.cells[1] > 0);   // (1, 0)
    REQUIRE(inflated.cells[10] > 0);  // (0, 1)
}

TEST_CASE("Re-inflation after map update", "[occupancy_grid][inflation]") {
    OccupancyGridMap map(20, 20, 0.1);
    auto& grid = map.grid();

    // Initial occupied cell
    grid.cells[5 * 20 + 5] = 10;
    auto inflated1 = map.inflate(0.2);

    // Add another occupied cell
    grid.cells[15 * 20 + 15] = 10;
    auto inflated2 = map.inflate(0.2);

    // Second inflation should have more occupied cells
    int count1 = 0, count2 = 0;
    for (auto c : inflated1.cells)
        if (c > 0) ++count1;
    for (auto c : inflated2.cells)
        if (c > 0) ++count2;
    REQUIRE(count2 > count1);
}

TEST_CASE("Zero radius inflation returns same grid", "[occupancy_grid][inflation]") {
    OccupancyGridMap map(10, 10, 0.1);
    auto& grid = map.grid();
    grid.cells[5 * 10 + 5] = 10;

    auto inflated = map.inflate(0.0);

    // Should be identical to original
    REQUIRE(inflated.cells == grid.cells);
}
