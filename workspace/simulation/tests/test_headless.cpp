#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <simulation/app/app_shell.hpp>
#include <simulation/pipeline/module_pipeline.hpp>
#include <simulation/bridge/state_adapter.hpp>
#include <simulation/bridge/model_adapter.hpp>
#include <common/kinematics/differential_drive.hpp>

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

TEST_CASE("Scenario loader parses MJCF correctly", "[simulation][headless]") {
    auto scenarioPath = findScenario();
    REQUIRE(std::filesystem::exists(scenarioPath));

    char error[1000] = "";
    mjModel* m = mj_loadXML(scenarioPath.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    // Verify scenario has expected structure
    REQUIRE(m->nbody > 1);   // world + at least robot
    REQUIRE(m->nu >= 2);     // at least 2 actuators (left + right wheel)
    REQUIRE(m->ngeom > 0);   // has geometry

    // Verify robot body exists
    int robotId = mj_name2id(m, mjOBJ_BODY, "robot_chassis");
    REQUIRE(robotId >= 0);

    // Verify actuators exist
    int leftMotor = mj_name2id(m, mjOBJ_ACTUATOR, "left_motor");
    int rightMotor = mj_name2id(m, mjOBJ_ACTUATOR, "right_motor");
    REQUIRE(leftMotor >= 0);
    REQUIRE(rightMotor >= 0);

    mj_deleteModel(m);
}

TEST_CASE("Robot kinematic step via DifferentialDrive", "[simulation][headless]") {
    // Verify DifferentialDrive model works as expected by the simulation
    DifferentialDrive dd(0.05, 0.24);  // match flat_ground.xml wheel params

    Pose2D start{0.0, 0.0, 0.0};

    // Forward 1 second at 0.5 m/s
    auto result = dd.step(start, {0.5, 0.0}, 1.0);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.0, 1e-6));

    // Twist round-trip
    Twist cmd{0.3, 0.5};
    auto wheels = dd.fromTwist(cmd);
    auto recovered = dd.toTwist(wheels);
    REQUIRE_THAT(recovered.linear, Catch::Matchers::WithinAbs(cmd.linear, 1e-9));
    REQUIRE_THAT(recovered.angular, Catch::Matchers::WithinAbs(cmd.angular, 1e-9));
}

TEST_CASE("Pipeline module wiring", "[simulation][headless]") {
    ModulePipeline pipeline;

    // Should not crash without any modules set
    OccupancyGrid grid;
    LaserScan scan;
    auto cmd = pipeline.tick({0, 0, 0}, {0, 0}, scan, grid, 0.02);
    REQUIRE_THAT(cmd.linear, Catch::Matchers::WithinAbs(0.0, 1e-9));

    // Set modules and verify goal tracking
    PIDConfig hCfg{1.0, 0.0, 0.0, 3.0, 2.0};
    PIDConfig sCfg{1.0, 0.0, 0.0, 1.0, 1.0};
    pipeline.setController(std::make_unique<HeadingSpeedController>(hCfg, sCfg));
    pipeline.setEstimator(std::make_unique<EKF2D>());

    REQUIRE_FALSE(pipeline.hasGoal());
    pipeline.setGoal({1.0, 0.0, 0.0});
    REQUIRE(pipeline.hasGoal());
}
