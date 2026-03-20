# Simulation

A standalone C++ application that runs the 3D simulated world using MuJoCo physics
and provides an integrated ImGui control panel for visualization and interaction.

**Important:** The simulation is the single source of truth for world state. It is a
single executable — there are no separate frontends or network APIs. Visualization
(MuJoCo 3D rendering) and controls (ImGui panels) are integrated into the same process.

---

## Responsibilities

- Loading MJCF model files (robots, terrain, sensors, world scenarios)
- Running MuJoCo physics at a configurable timestep (default 2ms / 500Hz)
- Bridging MuJoCo state to `common/` types for robotics modules
- Running the module pipeline (estimator -> planner -> controller) each physics tick
- Rendering the 3D scene via MuJoCo's OpenGL renderer
- Providing ImGui overlay panels for simulation control and module configuration
- Supporting headless mode (no window) for batch testing and CI

---

## Technology

- **Physics:** [MuJoCo](https://mujoco.org/) — fast, accurate rigid-body dynamics with contact
- **Windowing:** GLFW (comes transitively via MuJoCo)
- **3D Rendering:** MuJoCo's built-in OpenGL renderer (`mjr_render()`)
- **UI Overlay:** [Dear ImGui](https://github.com/ocornut/imgui) (docking branch, GLFW+OpenGL3 backend)
- **Build:** CMake, produces an executable `simulation_app`
- **Dependencies:** MuJoCo, ImGui, and the robotics modules from
  `workspace/robotics/` (linked directly)

---

## Threading Model

- **Physics thread:** runs `mj_step1()` -> module pipeline -> `mj_step2()` in a tight loop
- **Main thread (OpenGL):** copies `mjData` under mutex, renders 3D scene, draws ImGui panels
- **Async planning thread:** slow planners (RRT*, MPC) run asynchronously and deliver results
  to the pipeline via a thread-safe buffer

---

## Directory Layout

```
simulation/
+-- CMakeLists.txt
+-- README.md               <- This file
+-- include/simulation/
|   +-- bridge/             # SensorAdapter, StateAdapter, ActuatorAdapter, ModelAdapter
|   +-- pipeline/           # Module pipeline wiring + runtime switching
|   +-- app/                # GLFW window, ImGui panels, render loop
+-- src/
|   +-- main.cpp            # Entry point: load MJCF, create window, run
|   +-- bridge/             # Bridge implementations
|   +-- pipeline/           # Pipeline implementations
|   +-- app/                # App shell, ImGui panels, render
|   +-- scenario_loader/    # Scenario loading logic
+-- scenarios/              # MJCF model files
+-- tests/                  # Integration tests (headless MuJoCo)
+-- docs/
    +-- design.md
```

---

## Running

```bash
# Build from workspace root
cmake -B build -S workspace -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Run with default scenario
./build/simulation/simulation_app

# Run with a specific scenario
./build/simulation/simulation_app --scenario scenarios/diff_drive_flat.xml

# Run headless (no window, for CI/testing)
./build/simulation/simulation_app --headless --scenario scenarios/diff_drive_flat.xml
```
