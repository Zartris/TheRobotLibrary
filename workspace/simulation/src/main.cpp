#include <simulation/app/app_shell.hpp>
#include <simulation/pipeline/module_pipeline.hpp>

#include <pid/pid_controller.hpp>
#include <astar/astar_planner.hpp>
#include <dwa/dwa_planner.hpp>
#include <ekf/ekf2d.hpp>
#include <velocity_profiling/trapezoidal_profiler.hpp>

#include <iostream>
#include <string>

using namespace robotlib;
using namespace robotlib::sim;

int main(int argc, char* argv[]) {
    AppConfig config;
    config.headless = false;

    // Parse command line
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            config.headless = true;
        } else if (arg == "--scenario" && i + 1 < argc) {
            config.scenarioPath = argv[++i];
        }
    }

    if (config.scenarioPath.empty()) {
        // Default scenario
        config.scenarioPath = "scenarios/flat_ground.xml";
    }

    AppShell app(config);
    if (!app.initialize()) {
        std::cerr << "Failed to initialize simulation" << std::endl;
        return 1;
    }

    // Set up default pipeline modules
    auto& pipeline = app.pipeline();

    PIDConfig headingCfg{2.0, 0.0, 0.1, 3.0, 2.0};
    PIDConfig speedCfg{1.0, 0.1, 0.0, 1.0, 1.0};
    pipeline.setController(std::make_unique<HeadingSpeedController>(headingCfg, speedCfg));

    pipeline.setGlobalPlanner(std::make_unique<AStarPlanner>());
    pipeline.setLocalPlanner(std::make_unique<DWAPlanner>());
    pipeline.setEstimator(std::make_unique<EKF2D>());
    pipeline.setVelocityProfiler(std::make_unique<TrapezoidalProfiler>());

    // Set a goal
    pipeline.setGoal({3.0, 3.0, 0.0});

    app.run();
    return 0;
}
