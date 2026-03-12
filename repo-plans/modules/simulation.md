# Module: simulation

**Milestone:** M1 — Minimum Viable Robot (Sub-phases M1-J + M1-K)  
**Status:** Not Started  
**Depends on:** common, ray_casting, + all M1 robotics modules for pipeline integration

---

### Phase 1 — Interface Design (M1-J)

- [ ] `include/simulation/world.hpp` — `WorldModel` (grid map, robot, landmarks, dynamic obstacles)
- [ ] `include/simulation/robot.hpp` — `Robot` (pose, velocity, uses `IKinematicModel` for `step()`)
- [ ] `include/simulation/sim_loop.hpp` — `SimLoop` (fixed timestep tick)
- [ ] `include/simulation/api_server.hpp` — Crow routes + WebSocket broadcast
- [ ] `include/simulation/scenario_loader.hpp` — JSON → world config
- [ ] `include/simulation/robot_pipeline.hpp` — `RobotPipeline` with swappable interfaces

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_robot.cpp` — kinematic model step (twist via DifferentialDrive → expected pose)
- [ ] `tests/test_world.cpp` — load scenario, step, state consistency
- [ ] `tests/test_scenario_loader.cpp` — parse JSON → valid config
- [ ] `tests/test_pipeline_integration.cpp` — robot navigates from A to B (M1-K)

### Phase 3 — Implementation (Green)

- [ ] `src/main.cpp` — entry point, CLI args (port, scenario)
- [ ] `src/world.cpp`
- [ ] `src/robot.cpp` — delegates motion to `IKinematicModel` (DifferentialDrive default)
- [ ] `src/sim_loop.cpp` — fixed timestep, autonomous tick pipeline
- [ ] `src/api_server.cpp` — Crow WS state stream + REST endpoints
- [ ] `src/scenario_loader.cpp` — JSON parsing via nlohmann/json
- [ ] At least 1 scenario JSON: simple room + walls + start/goal + landmarks

### Phase 4 — Passing Tests

- [ ] Unit tests pass (robot, world, scenario_loader)
- [ ] Integration test: robot reaches goal within 60 sim seconds
- [ ] Module swap via REST doesn't crash

### Phase 5 — Smoke Test

```bash
./build/simulation_server --scenario default
curl http://localhost:8080/api/scenario/list
curl -X POST http://localhost:8080/api/sim/start
# Verify WS state stream in browser/wscat
```

### Phase 6 — REST API for Module Swapping (M1-K)

- [ ] `PUT /api/robot/controller {"type": "pid"}`
- [ ] `PUT /api/robot/global_planner {"type": "astar"}`
- [ ] `PUT /api/robot/local_planner {"type": "dwa"}`
- [ ] `PUT /api/robot/estimator {"type": "ekf"}`
- [ ] `GET /api/robot/pipeline` — current module selections

### Phase 7 — Docs Polish

- [ ] Update simulation README.md with API docs + scenario format
- [ ] Move this file to `repo-plans/modules/done/simulation.md`
