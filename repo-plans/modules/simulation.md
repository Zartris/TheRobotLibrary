# Module: simulation

**Milestone:** M1
**Status:** Not Started
**Depends on:** common (types), MuJoCo (physics), ImGui (UI)

---

### Phase 1 — Scaffold

- [ ] `simulation/include/simulation/bridge/sensor_adapter.hpp`
- [ ] `simulation/include/simulation/bridge/state_adapter.hpp`
- [ ] `simulation/include/simulation/bridge/actuator_adapter.hpp`
- [ ] `simulation/include/simulation/bridge/model_adapter.hpp`
- [ ] `simulation/include/simulation/pipeline/module_pipeline.hpp`
- [ ] `simulation/include/simulation/app/app_shell.hpp`
- [ ] `simulation/include/simulation/app/imgui_panels.hpp`
- [ ] `simulation/src/main.cpp`
- [ ] `simulation/src/bridge/*.cpp`
- [ ] `simulation/src/pipeline/*.cpp`
- [ ] `simulation/src/app/*.cpp`
- [ ] `simulation/src/scenario_loader/scenario_loader.cpp`

### Phase 2 — Core Implementation

- [ ] MuJoCo model loading from MJCF (`mj_loadXML` / `mj_loadModel`)
- [ ] Split-step physics loop: `mj_step1` → control injection → `mj_step2`
- [ ] Bridge adapters: read `mjData` fields → `common::` types (Pose3D, Twist3D, etc.)
- [ ] Bridge adapters: write control outputs → `mjData::ctrl[]`
- [ ] Model adapter: extract `VehicleParams`, `WheelConfig`, etc. from `mjModel` at startup
- [ ] Headless mode support (no window, physics-only execution)

### Phase 3 — Module Pipeline

- [ ] Pluggable module slots: `IStateEstimator`, `IGlobalPlanner`, `ILocalPlanner`, `IController`
- [ ] Runtime module switching via ImGui dropdowns
- [ ] Async planning thread for slow planners (global planner runs off physics thread)
- [ ] Thread-safe result buffer between planning thread and physics loop

### Phase 4 — App Shell

- [ ] GLFW window creation + OpenGL context initialization
- [ ] MuJoCo 3D rendering via `mjr_render()`
- [ ] ImGui overlay using GLFW+OpenGL3 backend (docking branch)
- [ ] ImGui panel: sim control (play/pause/reset, timestep)
- [ ] ImGui panel: module selector (dropdown per pipeline slot)
- [ ] ImGui panel: parameter tuning (live editable module params)
- [ ] ImGui panel: telemetry plots (pose, velocity, control outputs over time)
- [ ] ImGui panel: scenario loader (list + load MJCF scenarios)
- [ ] Threading model: physics thread + main render thread, `mj_copyData()` under mutex

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected throughout bridge and pipeline via `common::getLogger("simulation")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target simulation_tests
cd build && ctest -R simulation --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Scenarios

- [ ] First MJCF scenario: differential drive robot on flat ground (`simulation/scenarios/flat_ground.xml`)
- [ ] Scenario directory structure: `simulation/scenarios/`
- [ ] Scenario loader reads MJCF path from config / CLI argument

### Phase 6 — Testing

- [ ] Headless integration tests: MuJoCo physics runs N steps without window (CI-safe)
- [ ] Bridge adapter unit tests: `mjData` mock → expected `common::` type values
- [ ] Pipeline wiring tests: modules registered, called in correct order, outputs propagated

### Phase 7 — Documentation

- [ ] Update simulation `README.md` with architecture overview and scenario format
- [ ] Update `design.md` to reflect final implementation decisions
- [ ] Move this file to `repo-plans/modules/done/simulation.md`
