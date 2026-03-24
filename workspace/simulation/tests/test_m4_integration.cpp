#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <simulation/app/app_shell.hpp>
#include <simulation/pipeline/module_pipeline.hpp>
#include <obstacle_detection/obstacle_detector.hpp>
#include <lidar_processing/line_extractor.hpp>
#include <occupancy_grid/occupancy_grid_map.hpp>
#include <common/laser_scan.hpp>
#include <ekf/ekf2d.hpp>

#include <mujoco/mujoco.h>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace robotlib;
using namespace robotlib::sim;

struct MjScope {
    mjModel* m{nullptr};
    mjData* d{nullptr};
    explicit MjScope(const std::string& path) {
        char error[1000] = "";
        m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
        if (m) d = mj_makeData(m);
    }
    ~MjScope() {
        if (d) mj_deleteData(d);
        if (m) mj_deleteModel(m);
    }
    MjScope(const MjScope&) = delete;
    MjScope& operator=(const MjScope&) = delete;
};

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

TEST_CASE("Obstacle detector finds clusters in scan", "[m4][integration]") {
    // Create a simple scan with obstacle points
    ObstacleDetectorConfig cfg;
    cfg.clusterEps = 0.5;
    cfg.clusterMinPoints = 2;
    ObstacleDetector detector(cfg);

    // Simulated scan: group of close points
    LaserScan scan;
    scan.angleMin = -0.5;
    scan.angleMax = 0.5;
    scan.angleIncrement = 0.05;
    int numRays = static_cast<int>((scan.angleMax - scan.angleMin) / scan.angleIncrement) + 1;
    scan.ranges.resize(numRays, 50.0f);
    // Cluster of points at ~2m ahead
    for (int i = 8; i <= 12; ++i) scan.ranges[i] = 2.0f;

    auto obstacles = detector.detect(scan, {0, 0, 0});
    REQUIRE(obstacles.size() >= 1);
    // Check the obstacle is roughly at (2, 0)
    CHECK(obstacles[0].position.x() > 1.5);
    CHECK(obstacles[0].position.x() < 2.5);
}

TEST_CASE("Line extractor finds wall segments", "[m4][integration]") {
    LineExtractorConfig cfg;
    cfg.minInliers = 3;
    cfg.distanceThreshold = 0.1;
    LineExtractor extractor(cfg);

    // Simulated scan: straight wall at 2m ahead
    LaserScan scan;
    scan.angleMin = -0.5;
    scan.angleMax = 0.5;
    scan.angleIncrement = 0.02;
    int numRays = static_cast<int>((scan.angleMax - scan.angleMin) / scan.angleIncrement) + 1;
    scan.ranges.resize(numRays);
    for (int i = 0; i < numRays; ++i) {
        double angle = scan.angleMin + i * scan.angleIncrement;
        scan.ranges[i] = static_cast<float>(2.0 / std::cos(angle));
    }

    auto lines = extractor.extractLines(scan, {0, 0, 0});
    REQUIRE(lines.size() >= 1);
    CHECK(lines[0].inlierCount >= 3);
}

TEST_CASE("Inflation expands obstacles for planning", "[m4][integration]") {
    OccupancyGridMap map(40, 40, 0.1);
    auto& grid = map.grid();

    // Place a wall
    for (int y = 15; y <= 25; ++y) {
        grid.cells[y * 40 + 20] = 10;
    }

    auto inflated = map.inflate(0.2);  // 2-cell radius

    // Original wall cell should be occupied
    REQUIRE(inflated.cells[20 * 40 + 20] > 0);
    // Adjacent cells should be inflated
    REQUIRE(inflated.cells[20 * 40 + 21] > 0);
    REQUIRE(inflated.cells[20 * 40 + 22] > 0);
    // Far cells should not be inflated
    CHECK(inflated.cells[20 * 40 + 25] <= 0);
}

TEST_CASE("Dynamic obstacles scenario loads", "[m4][integration][headless]") {
    auto path = findScenario("dynamic_obstacles.xml");
    REQUIRE(std::filesystem::exists(path));

    MjScope mj(path);
    REQUIRE(mj.m != nullptr);
    REQUIRE(mj.d != nullptr);

    // Step physics
    for (int i = 0; i < 100; ++i) {
        mj_step(mj.m, mj.d);
    }
    REQUIRE(mj.d->time > 0.0);

    // Verify dynamic obstacle bodies exist
    int dynObs1 = mj_name2id(mj.m, mjOBJ_BODY, "dyn_obs_1");
    int dynObs2 = mj_name2id(mj.m, mjOBJ_BODY, "dyn_obs_2");
    REQUIRE(dynObs1 >= 0);
    REQUIRE(dynObs2 >= 0);
}

TEST_CASE("Full perception pipeline headless run", "[m4][integration][headless]") {
    auto path = findScenario("flat_ground.xml");
    REQUIRE(std::filesystem::exists(path));

    AppConfig config;
    config.scenarioPath = path;
    config.headless = true;

    AppShell app(config);
    REQUIRE(app.initialize());

    auto& pipeline = app.pipeline();
    pipeline.selectController("pid");
    pipeline.setEstimator(std::make_unique<EKF2D>());
    pipeline.setGoal({3.0, 3.0, 0.0});

    // Run headless — perception modules are linked and available
    app.runHeadless(3.0, 0.02);
    REQUIRE(app.data()->time > 2.5);
}
