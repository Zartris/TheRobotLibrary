#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <simulation/app/app_shell.hpp>
#include <simulation/bridge/state_adapter.hpp>
#include <simulation/bridge/model_adapter.hpp>

#include <pid/pid_controller.hpp>
#include <astar/astar_planner.hpp>
#include <dwa/dwa_planner.hpp>
#include <ekf/ekf2d.hpp>
#include <velocity_profiling/trapezoidal_profiler.hpp>

#include <mujoco/mujoco.h>
#include <filesystem>
#include <string>

using namespace robotlib;
using namespace robotlib::sim;

// Find the scenario file relative to the test binary or source tree
static std::string findScenario() {
    // Try relative paths from common run locations
    std::vector<std::string> candidates = {
        "simulation/scenarios/flat_ground.xml",
        "../simulation/scenarios/flat_ground.xml",
        "../../simulation/scenarios/flat_ground.xml",
        "../../../workspace/simulation/scenarios/flat_ground.xml",
        "../../../../workspace/simulation/scenarios/flat_ground.xml",
    };
    // Also try from the workspace source directory
    const char* srcDir = CMAKE_SOURCE_DIR;
    if (srcDir) {
        candidates.push_back(std::string(srcDir) + "/simulation/scenarios/flat_ground.xml");
    }

    for (const auto& path : candidates) {
        if (std::filesystem::exists(path)) return path;
    }
    return "scenarios/flat_ground.xml";  // fallback
}

TEST_CASE("MuJoCo model loads and steps", "[simulation][headless]") {
    auto scenarioPath = findScenario();
    REQUIRE(std::filesystem::exists(scenarioPath));

    char error[1000] = "";
    mjModel* m = mj_loadXML(scenarioPath.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    // Step physics
    for (int i = 0; i < 100; ++i) {
        mj_step(m, d);
    }

    REQUIRE(d->time > 0.0);

    // Extract robot state
    auto pose = StateAdapter::extractPose2D(m, d, 1);
    // Robot should still be near origin after 100 steps with no control
    REQUIRE_THAT(pose.x, Catch::Matchers::WithinAbs(0.0, 1.0));
    REQUIRE_THAT(pose.y, Catch::Matchers::WithinAbs(0.0, 1.0));

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Model adapter extracts vehicle params", "[simulation][headless]") {
    auto scenarioPath = findScenario();
    REQUIRE(std::filesystem::exists(scenarioPath));

    char error[1000] = "";
    mjModel* m = mj_loadXML(scenarioPath.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    auto params = ModelAdapter::extractVehicleParams(m);
    REQUIRE(params.bodyId >= 1);
    REQUIRE(params.mass > 0.0);

    mj_deleteModel(m);
}

TEST_CASE("Headless simulation runs without crash", "[simulation][headless]") {
    auto scenarioPath = findScenario();
    REQUIRE(std::filesystem::exists(scenarioPath));

    AppConfig config;
    config.scenarioPath = scenarioPath;
    config.headless = true;

    AppShell app(config);
    REQUIRE(app.initialize());

    // Set up pipeline with just a controller (no planner — avoids
    // empty-grid A* failure in the unit test)
    auto& pipeline = app.pipeline();
    PIDConfig hCfg{2.0, 0.0, 0.1, 3.0, 2.0};
    PIDConfig sCfg{1.0, 0.1, 0.0, 1.0, 1.0};
    pipeline.setController(std::make_unique<HeadingSpeedController>(hCfg, sCfg));
    pipeline.setEstimator(std::make_unique<EKF2D>());

    // Run for a very short duration (physics-only smoke test)
    app.runHeadless(0.1, 0.05);

    // Should have run without crashing
    REQUIRE(app.data()->time > 0.05);
}
