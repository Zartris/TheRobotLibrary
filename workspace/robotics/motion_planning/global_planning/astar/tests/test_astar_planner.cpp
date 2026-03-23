#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <astar/astar_planner.hpp>

using namespace robotlib;

static OccupancyGrid makeEmptyGrid(int size = 50, double res = 0.1) {
    OccupancyGrid grid(size, size, res);
    for (auto& c : grid.cells) c = -1;  // all free
    return grid;
}

TEST_CASE("A* straight line in empty grid", "[astar]") {
    auto grid = makeEmptyGrid();
    AStarPlanner planner;

    auto path = planner.plan({0.5, 0.5, 0.0}, {4.0, 0.5, 0.0}, grid);

    REQUIRE(path.has_value());
    REQUIRE(path->size() > 1);
}

TEST_CASE("A* L-shaped obstacle", "[astar]") {
    auto grid = makeEmptyGrid();
    // Place L-shaped wall
    for (int x = 20; x < 30; ++x) grid.at(x, 25) = 10;
    for (int y = 15; y < 25; ++y) grid.at(20, y) = 10;

    AStarPlanner planner;
    auto path = planner.plan({1.0, 2.5, 0.0}, {3.0, 2.5, 0.0}, grid);

    REQUIRE(path.has_value());
    REQUIRE(path->size() > 2);  // must go around
}

TEST_CASE("A* no path possible", "[astar]") {
    auto grid = makeEmptyGrid();
    // Surround the goal completely
    for (int x = 38; x <= 42; ++x)
        for (int y = 38; y <= 42; ++y)
            grid.at(x, y) = 10;

    AStarPlanner planner;
    auto path = planner.plan({0.5, 0.5, 0.0}, {4.0, 4.0, 0.0}, grid);

    REQUIRE_FALSE(path.has_value());
}

TEST_CASE("A* start equals goal", "[astar]") {
    auto grid = makeEmptyGrid();
    AStarPlanner planner;

    auto path = planner.plan({2.0, 2.0, 0.0}, {2.0, 2.0, 0.0}, grid);

    REQUIRE(path.has_value());
    REQUIRE(path->size() == 1);
}

TEST_CASE("A* start in occupied cell", "[astar]") {
    auto grid = makeEmptyGrid();
    auto [gx, gy] = grid.toGrid(0.5, 0.5);
    grid.at(gx, gy) = 10;

    AStarPlanner planner;
    auto path = planner.plan({0.5, 0.5, 0.0}, {4.0, 4.0, 0.0}, grid);

    REQUIRE_FALSE(path.has_value());
}
