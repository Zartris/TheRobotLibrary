#include <simulation/app/app_shell.hpp>
#include <common/geometry.hpp>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <sstream>

namespace robotlib::sim {

AppShell::AppShell(const AppConfig& config)
    : m_config(config), m_logger(robotlib::getLogger("simulation.app")) {
    m_logger->debug("AppShell created");
}

AppShell::~AppShell() {
    if (m_data) mj_deleteData(m_data);
    if (m_model) mj_deleteModel(m_model);
    if (m_window) {
        mjr_freeContext(&m_context);
        mjv_freeScene(&m_scene);
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }
}

bool AppShell::loadModel() {
    char error[1000] = "";
    m_model = mj_loadXML(m_config.scenarioPath.c_str(), nullptr, error, sizeof(error));
    if (!m_model) {
        std::ostringstream oss;
        oss << "Failed to load MJCF: " << error;
        m_logger->error(oss.str());
        return false;
    }

    m_model->opt.timestep = m_config.simTimestep;
    m_data = mj_makeData(m_model);

    m_vehicleParams = ModelAdapter::extractVehicleParams(m_model);

    std::ostringstream oss;
    oss << "Model loaded: " << m_config.scenarioPath
        << " bodies=" << m_model->nbody
        << " actuators=" << m_model->nu;
    m_logger->debug(oss.str());
    return true;
}

bool AppShell::initialize() {
    if (!loadModel()) return false;

    if (!m_config.headless) {
        if (!glfwInit()) {
            m_logger->error("GLFW init failed");
            return false;
        }

        m_window = glfwCreateWindow(m_config.windowWidth, m_config.windowHeight,
                                     "TheRobotLibrary Simulation", nullptr, nullptr);
        if (!m_window) {
            m_logger->error("Failed to create GLFW window");
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(m_window);
        glfwSwapInterval(1);

        mjv_defaultCamera(&m_camera);
        mjv_defaultOption(&m_opt);
        mjv_defaultScene(&m_scene);
        mjr_defaultContext(&m_context);

        mjv_makeScene(m_model, &m_scene, 2000);
        mjr_makeContext(m_model, &m_context, mjFONTSCALE_150);

        // Set camera to look from above
        m_camera.type = mjCAMERA_FREE;
        m_camera.lookat[0] = 0.0;
        m_camera.lookat[1] = 0.0;
        m_camera.lookat[2] = 0.0;
        m_camera.distance = 5.0;
        m_camera.azimuth = 0.0;
        m_camera.elevation = -45.0;
    }

    m_logger->info("AppShell initialized");
    return true;
}

void AppShell::physicsStep() {
    mj_step(m_model, m_data);
    m_simTime = m_data->time;
}

void AppShell::pipelineTick(double dt) {
    auto pose = StateAdapter::extractPose2D(m_model, m_data, m_vehicleParams.bodyId);
    auto twist = StateAdapter::extractTwist(m_model, m_data, m_vehicleParams.bodyId);
    auto scan = SensorAdapter::generateLidar(m_model, m_data,
                                              m_vehicleParams.bodyId, m_lidarConfig);

    // Provide a free-space grid covering the scenario area so A* can plan
    if (m_grid.width == 0) {
        constexpr double kArenaSize = 10.0;     // half-extent from flat_ground.xml
        constexpr double kResolution = 0.25;
        int cells = static_cast<int>(2.0 * kArenaSize / kResolution);
        m_grid = OccupancyGrid(cells, cells, kResolution, {-kArenaSize, -kArenaSize});
        // Mark all cells as free (negative log-odds)
        std::fill(m_grid.cells.begin(), m_grid.cells.end(), -1);
    }

    auto cmd = m_pipeline.tick(pose, twist, scan, m_grid, dt);
    ActuatorAdapter::applyTwist(m_model, m_data, cmd, m_vehicleParams);
}

void AppShell::runHeadless(double duration, double pipelineDt) {
    m_running = true;
    m_logger->info("Starting headless simulation");

    double lastPipelineTime = 0.0;

    while (m_running && m_simTime < duration) {
        physicsStep();

        if (m_simTime - lastPipelineTime >= pipelineDt) {
            pipelineTick(pipelineDt);
            lastPipelineTime = m_simTime;
        }
    }

    std::ostringstream oss;
    oss << "Headless simulation finished at t=" << m_simTime;
    m_logger->info(oss.str());
    m_running = false;
}

void AppShell::run() {
    if (m_config.headless) {
        runHeadless(60.0);
        return;
    }

    m_running = true;
    m_logger->info("Starting interactive simulation");

    double lastPipelineTime = 0.0;
    const double pipelineDt = 0.02;

    while (m_running && !glfwWindowShouldClose(m_window)) {
        if (!m_paused) {
            // Step physics multiple times per render frame
            const int physicsStepsPerFrame = 10;
            for (int i = 0; i < physicsStepsPerFrame; ++i) {
                physicsStep();
            }

            if (m_simTime - lastPipelineTime >= pipelineDt) {
                pipelineTick(pipelineDt);
                lastPipelineTime = m_simTime;
            }
        }

        // Render
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(m_window, &viewport.width, &viewport.height);

        mjv_updateScene(m_model, m_data, &m_opt, nullptr, &m_camera, mjCAT_ALL, &m_scene);
        mjr_render(viewport, &m_scene, &m_context);

        glfwSwapBuffers(m_window);
        glfwPollEvents();

        if (glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            m_running = false;
        }
    }

    m_running = false;
    m_logger->info("Interactive simulation stopped");
}

void AppShell::stop() {
    m_running = false;
}

}  // namespace robotlib::sim
