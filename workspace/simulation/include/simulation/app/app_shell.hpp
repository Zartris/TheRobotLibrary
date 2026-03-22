#pragma once
#include <simulation/pipeline/module_pipeline.hpp>
#include <simulation/bridge/state_adapter.hpp>
#include <simulation/bridge/sensor_adapter.hpp>
#include <simulation/bridge/actuator_adapter.hpp>
#include <simulation/bridge/model_adapter.hpp>
#include <logging/get_logger.hpp>
#include <mujoco/mujoco.h>
#include <memory>
#include <string>
#include <atomic>
#include <functional>

// Forward declarations - GLFW and rendering only in non-headless mode
struct GLFWwindow;

namespace robotlib::sim {

struct AppConfig {
    std::string scenarioPath;
    bool headless{false};
    int windowWidth{1280};
    int windowHeight{720};
    double simTimestep{0.002};
};

class AppShell {
public:
    explicit AppShell(const AppConfig& config);
    ~AppShell();

    bool initialize();
    void run();
    void runHeadless(double duration, double pipelineDt = 0.02);
    void stop();

    ModulePipeline& pipeline() { return m_pipeline; }
    const mjModel* model() const { return m_model; }
    const mjData* data() const { return m_data; }

private:
    bool loadModel();
    void physicsStep();
    void pipelineTick(double dt);

    AppConfig m_config;
    SimVehicleParams m_vehicleParams;
    SimLidarConfig m_lidarConfig;

    mjModel* m_model{nullptr};
    mjData* m_data{nullptr};

    ModulePipeline m_pipeline;

    std::atomic<bool> m_running{false};
    bool m_paused{false};
    double m_simTime{0.0};

    std::shared_ptr<ILogger> m_logger;

    // Graphics (only used in non-headless mode)
    GLFWwindow* m_window{nullptr};
    mjvCamera m_camera;
    mjvOption m_opt;
    mjvScene m_scene;
    mjrContext m_context;
};

}  // namespace robotlib::sim
