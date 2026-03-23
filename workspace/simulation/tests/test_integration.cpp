#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <simulation/app/app_shell.hpp>
#include <simulation/pipeline/module_pipeline.hpp>
#include <simulation/bridge/state_adapter.hpp>
#include <simulation/bridge/model_adapter.hpp>
#include <simulation/bridge/sensor_adapter.hpp>
#include <simulation/bridge/actuator_adapter.hpp>
#include <common/kinematics/differential_drive.hpp>
#include <common/occupancy_grid.hpp>

#include <pid/pid_controller.hpp>
#include <astar/astar_planner.hpp>
#include <dwa/dwa_planner.hpp>
#include <ekf/ekf2d.hpp>
#include <velocity_profiling/trapezoidal_profiler.hpp>

#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

using namespace robotlib;
using namespace robotlib::sim;

// --- Helpers ---

static std::string findScenario(const std::string& name) {
    std::vector<std::string> candidates = {
        "simulation/scenarios/" + name,
        "../simulation/scenarios/" + name,
        "../../simulation/scenarios/" + name,
        "../../../workspace/simulation/scenarios/" + name,
        "../../../../workspace/simulation/scenarios/" + name,
    };
    const char* srcDir = CMAKE_SOURCE_DIR;
    if (srcDir) {
        candidates.push_back(std::string(srcDir) + "/simulation/scenarios/" + name);
    }
    for (const auto& p : candidates) {
        if (std::filesystem::exists(p)) return p;
    }
    return "scenarios/" + name;
}

/// Mark a rectangular region of the grid as occupied.
/// (wx1, wy1) and (wx2, wy2) are world-coordinate corners (any order).
static void markWall(OccupancyGrid& grid, double wx1, double wy1, double wx2, double wy2) {
    if (wx1 > wx2) std::swap(wx1, wx2);
    if (wy1 > wy2) std::swap(wy1, wy2);

    auto [gx1, gy1] = grid.toGrid(wx1, wy1);
    auto [gx2, gy2] = grid.toGrid(wx2, wy2);

    gx1 = std::max(0, gx1);
    gy1 = std::max(0, gy1);
    gx2 = std::min(grid.width - 1, gx2);
    gy2 = std::min(grid.height - 1, gy2);

    for (int y = gy1; y <= gy2; ++y) {
        for (int x = gx1; x <= gx2; ++x) {
            grid.at(x, y) = 1;  // Occupied (CellState::Occupied = positive)
        }
    }
}

/// Create a grid covering the arena, initialized as free, with boundary walls marked.
static OccupancyGrid makeGrid(double arenaHalf, double resolution) {
    int cells = static_cast<int>(2.0 * arenaHalf / resolution);
    OccupancyGrid grid(cells, cells, resolution, {-arenaHalf, -arenaHalf});

    // Initialize all cells as free (negative = CellState::Free)
    std::fill(grid.cells.begin(), grid.cells.end(), static_cast<int8_t>(-1));

    // Mark boundary walls (wall thickness ~ 0.1m = 2*0.05 half-size)
    double w = 0.15;  // slightly wider than geom to ensure coverage
    markWall(grid, -arenaHalf, -arenaHalf, arenaHalf, -arenaHalf + w);  // south
    markWall(grid, -arenaHalf, arenaHalf - w, arenaHalf, arenaHalf);    // north
    markWall(grid, -arenaHalf, -arenaHalf, -arenaHalf + w, arenaHalf);  // west
    markWall(grid, arenaHalf - w, -arenaHalf, arenaHalf, arenaHalf);    // east

    return grid;
}

/// Wire up the full M1 pipeline with all modules.
static void wireFullPipeline(ModulePipeline& pipeline) {
    PIDConfig hCfg{2.0, 0.0, 0.1, 3.0, 2.0};
    PIDConfig sCfg{1.0, 0.1, 0.0, 1.0, 1.0};
    pipeline.setController(std::make_unique<HeadingSpeedController>(hCfg, sCfg));
    pipeline.setEstimator(std::make_unique<EKF2D>());
    pipeline.setGlobalPlanner(std::make_unique<AStarPlanner>());
    pipeline.setLocalPlanner(std::make_unique<DWAPlanner>());
    pipeline.setVelocityProfiler(std::make_unique<TrapezoidalProfiler>());
}

/// Run simulation loop and return whether goal was reached.
static bool runNavigation(mjModel* m, mjData* d, ModulePipeline& pipeline,
                          OccupancyGrid& grid, const Pose2D& goal,
                          double maxTime, double pipelineDt) {
    auto params = ModelAdapter::extractVehicleParams(m);
    SimLidarConfig lidarCfg;  // defaults: 180 rays, -pi..pi, 10m range

    pipeline.setGoal(goal);

    double lastPipelineTime = 0.0;

    while (d->time < maxTime) {
        mj_step(m, d);

        if (d->time - lastPipelineTime >= pipelineDt) {
            auto pose = StateAdapter::extractPose2D(m, d, params.bodyId);
            auto twist = StateAdapter::extractTwist(m, d, params.bodyId);
            auto scan = SensorAdapter::generateLidar(m, d, params.bodyId, lidarCfg);

            auto cmd = pipeline.tick(pose, twist, scan, grid, pipelineDt);
            ActuatorAdapter::applyTwist(m, d, cmd, params);

            if (pipeline.isGoalReached()) return true;

            lastPipelineTime = d->time;
        }
    }
    return pipeline.isGoalReached();
}

// --- Tests ---

TEST_CASE("Integration: Open room navigation", "[integration][headless]") {
    auto path = findScenario("open_room.xml");
    REQUIRE(std::filesystem::exists(path));

    char error[1000] = "";
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);
    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    ModulePipeline pipeline;
    wireFullPipeline(pipeline);

    auto grid = makeGrid(5.0, 0.25);

    // Mark obstacles matching open_room.xml:
    // obstacle1: pos=(2,1), half-size=(0.3, 0.3) -> covers [1.7,2.3] x [0.7,1.3]
    markWall(grid, 1.7, 0.7, 2.3, 1.3);
    // obstacle2: pos=(-1,3), half-size=(0.4, 0.2) -> covers [-1.4,-0.6] x [2.8,3.2]
    markWall(grid, -1.4, 2.8, -0.6, 3.2);
    // obstacle3: pos=(3,-1), half-size=(0.25, 0.35) -> covers [2.75,3.25] x [-1.35,-0.65]
    markWall(grid, 2.75, -1.35, 3.25, -0.65);

    Pose2D goal{4.0, 4.0, 0.0};
    bool reached = runNavigation(m, d, pipeline, grid, goal, 60.0, 0.02);

    CHECK(reached);

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Integration: Maze navigation", "[integration][headless]") {
    auto path = findScenario("maze.xml");
    REQUIRE(std::filesystem::exists(path));

    char error[1000] = "";
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);
    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    ModulePipeline pipeline;
    wireFullPipeline(pipeline);

    auto grid = makeGrid(5.0, 0.25);

    // Mark internal maze wall matching maze.xml:
    // maze_wall1: pos=(-1,0), half-size=(3.0, 0.1) -> covers [-4,2] x [-0.1,0.1]
    markWall(grid, -4.0, -0.1, 2.0, 0.1);

    Pose2D goal{0.0, 3.0, 0.0};
    bool reached = runNavigation(m, d, pipeline, grid, goal, 90.0, 0.02);

    CHECK(reached);

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Integration: L-shaped room navigation", "[integration][headless]") {
    auto path = findScenario("l_shaped_room.xml");
    REQUIRE(std::filesystem::exists(path));

    char error[1000] = "";
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);
    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    ModulePipeline pipeline;
    wireFullPipeline(pipeline);

    auto grid = makeGrid(5.0, 0.25);

    // Mark the L-shape block matching l_shaped_room.xml:
    // l_block: pos=(2.5,2.5), half-size=(2.5, 2.5) -> covers [0,5] x [0,5]
    markWall(grid, 0.0, 0.0, 5.0, 5.0);

    Pose2D goal{-3.0, 3.0, 0.0};
    bool reached = runNavigation(m, d, pipeline, grid, goal, 90.0, 0.02);

    CHECK(reached);

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Integration: Module swap mid-run doesn't crash", "[integration][headless]") {
    auto path = findScenario("open_room.xml");
    REQUIRE(std::filesystem::exists(path));

    char error[1000] = "";
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);
    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    ModulePipeline pipeline;
    wireFullPipeline(pipeline);

    auto grid = makeGrid(5.0, 0.25);
    auto params = ModelAdapter::extractVehicleParams(m);
    SimLidarConfig lidarCfg;

    pipeline.setGoal({4.0, 4.0, 0.0});

    // Run for 5 seconds
    double lastPipelineTime = 0.0;
    while (d->time < 5.0) {
        mj_step(m, d);
        if (d->time - lastPipelineTime >= 0.02) {
            auto pose = StateAdapter::extractPose2D(m, d, params.bodyId);
            auto twist = StateAdapter::extractTwist(m, d, params.bodyId);
            auto scan = SensorAdapter::generateLidar(m, d, params.bodyId, lidarCfg);
            auto cmd = pipeline.tick(pose, twist, scan, grid, 0.02);
            ActuatorAdapter::applyTwist(m, d, cmd, params);
            lastPipelineTime = d->time;
        }
    }

    // Swap controller mid-run (should not crash)
    PIDConfig hCfg{3.0, 0.0, 0.2, 4.0, 3.0};
    PIDConfig sCfg{2.0, 0.2, 0.0, 2.0, 2.0};
    pipeline.setController(std::make_unique<HeadingSpeedController>(hCfg, sCfg));
    pipeline.setEstimator(std::make_unique<EKF2D>());

    // Continue running for 5 more seconds
    while (d->time < 10.0) {
        mj_step(m, d);
        if (d->time - lastPipelineTime >= 0.02) {
            auto pose = StateAdapter::extractPose2D(m, d, params.bodyId);
            auto twist = StateAdapter::extractTwist(m, d, params.bodyId);
            auto scan = SensorAdapter::generateLidar(m, d, params.bodyId, lidarCfg);
            auto cmd = pipeline.tick(pose, twist, scan, grid, 0.02);
            ActuatorAdapter::applyTwist(m, d, cmd, params);
            lastPipelineTime = d->time;
        }
    }

    // If we got here without crash, the test passes
    REQUIRE(d->time >= 10.0);

    mj_deleteData(d);
    mj_deleteModel(m);
}
