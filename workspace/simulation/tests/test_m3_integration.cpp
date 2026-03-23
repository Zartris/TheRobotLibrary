#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <simulation/app/app_shell.hpp>
#include <simulation/pipeline/module_pipeline.hpp>
#include <simulation/bridge/state_adapter.hpp>
#include <simulation/bridge/model_adapter.hpp>

#include <pid/pid_controller.hpp>
#include <pure_pursuit/pure_pursuit_controller.hpp>
#include <mpc/mpc_controller.hpp>
#include <cbf/cbf_safety_filter.hpp>
#include <ekf/ekf2d.hpp>
#include <astar/astar_planner.hpp>
#include <dwa/dwa_planner.hpp>

#include <mujoco/mujoco.h>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace robotlib;
using namespace robotlib::sim;

// RAII wrapper for MuJoCo (from M2 pattern)
struct MjScope {
    mjModel* m{nullptr};
    mjData* d{nullptr};
    MjScope(const std::string& path) {
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

TEST_CASE("Controller hot-swap: PID to PurePursuit mid-run", "[m3][integration][headless]") {
    auto path = findScenario("flat_ground.xml");
    REQUIRE(std::filesystem::exists(path));

    AppConfig config;
    config.scenarioPath = path;
    config.headless = true;

    AppShell app(config);
    REQUIRE(app.initialize());

    auto& pipeline = app.pipeline();

    // Start with PID
    pipeline.selectController("pid");
    pipeline.setEstimator(std::make_unique<EKF2D>());
    pipeline.setGoal({3.0, 3.0, 0.0});

    // Run for 2 seconds with PID
    app.runHeadless(2.0, 0.02);
    REQUIRE(app.data()->time > 1.5);

    // Swap to PurePursuit mid-run
    pipeline.selectController("pure_pursuit");

    // Continue for 2 more seconds
    app.runHeadless(4.0, 0.02);
    REQUIRE(app.data()->time > 3.5);
}

TEST_CASE("Controller hot-swap: PID to MPC mid-run", "[m3][integration][headless]") {
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

    app.runHeadless(2.0, 0.02);

    // Swap to MPC
    pipeline.selectController("mpc");
    app.runHeadless(4.0, 0.02);
    REQUIRE(app.data()->time > 3.5);
}

TEST_CASE("Controller hot-swap: PID to CBF-PID mid-run", "[m3][integration][headless]") {
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

    app.runHeadless(2.0, 0.02);

    // Swap to CBF-wrapped PID
    pipeline.selectController("cbf_pid");
    app.runHeadless(4.0, 0.02);
    REQUIRE(app.data()->time > 3.5);
}

TEST_CASE("All controllers available in factory", "[m3][integration]") {
    auto controllers = ModulePipeline::availableControllers();
    REQUIRE(controllers.size() >= 4);

    ModulePipeline pipeline;
    for (const auto& name : controllers) {
        REQUIRE(pipeline.selectController(name));
    }
}

TEST_CASE("All kinematic models available in factory", "[m3][integration]") {
    auto models = ModulePipeline::availableKinematicModels();
    REQUIRE(models.size() >= 4);

    ModulePipeline pipeline;
    for (const auto& name : models) {
        REQUIRE(pipeline.selectKinematicModel(name));
    }
}

TEST_CASE("Controller cycle through all types without crash", "[m3][integration][headless]") {
    auto path = findScenario("flat_ground.xml");
    REQUIRE(std::filesystem::exists(path));

    AppConfig config;
    config.scenarioPath = path;
    config.headless = true;

    AppShell app(config);
    REQUIRE(app.initialize());

    auto& pipeline = app.pipeline();
    pipeline.setEstimator(std::make_unique<EKF2D>());
    pipeline.setGoal({3.0, 3.0, 0.0});

    // Cycle through all controllers, running 1 second each
    for (const auto& name : ModulePipeline::availableControllers()) {
        pipeline.selectController(name);
        app.runHeadless(app.data()->time + 1.0, 0.02);
    }

    // If we got here, no controller caused a crash
    REQUIRE(app.data()->time > 3.0);
}
