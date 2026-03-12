# TheRobotLibrary — Development Roadmap Design

**Date:** 2026-03-12  
**Status:** Approved  
**Design philosophy:** Educational completeness per domain, navigation stack ordering, visual feedback early  

---

## Milestone Sequence Overview

| # | Name | Focus |
|---|------|-------|
| **M0** | Dev Infrastructure | AI agent configs, devcontainer, CI/CD, linting |
| **M1** | Foundation | `common` library + simulation backend (world model, sim loop, WS + REST API) |
| **M2** | Motion Planning Basics | A*, Dijkstra, DWA, velocity profiling — first "robot moves" moment |
| **M3** | Frontends | Native ImGui + Web React — visual feedback for all subsequent work |
| **M4** | Perception | lidar_processing → occupancy_grid → ray_casting → obstacle_detection |
| **M5** | Control | PID → pure_pursuit |
| **M6** | State Estimation | EKF → particle_filter |
| **M7** | SLAM | EKF-SLAM → lidar_slam |
| **M8** | Advanced Control | MPC |
| **M9** | Advanced Planning | RRT, spline_fitting, TEB, time_optimal |
| **M10** | Multi-Robot | ORCA → priority_planning → CBS → DMPC → MADER |
| **M11** | Polish & Showcase | Documentation site, examples, packaging, full integration demo |

Each domain milestone (M2, M4–M10) follows the standard module task template: interface design → implementation → unit tests → simulation integration → frontend visualization → mini-demo + docs polish.

---

## Ordering Constraints

```
M0 (infra) → M1 (foundation) → M2 (planning basics) → M3 (frontends)
                                                          ↓
                                          M4 (perception) → M5 (control)
                                                              ↓
                                                M6 (estimation) → M7 (SLAM)
                                                                    ↓
                                                  M8 (MPC) → M9 (adv. planning) → M10 (multi-robot)
                                                                                        ↓
                                                                                   M11 (polish)
```

---

## M0 — Dev Infrastructure

**Rationale:** Every line of code should benefit from CI, linting, and AI agent context from the start. Zero code exists yet, so this is the cheapest time to establish conventions.

**Scope:**
- **AI agent instructions:** `.claude/CLAUDE.md`, `.github/copilot-instructions.md`, `.opencode` config — pointing agents at `architecture.md`, naming conventions, module pattern, dependency rules
- **devcontainer:** `.devcontainer/devcontainer.json` wiring up the existing `Dockerfile.dev` + `docker-compose.dev.yml`, VS Code extensions (C++, CMake, clangd)
- **CI/CD:** GitHub Actions workflow — CMake configure + build + `ctest` on Ubuntu (GCC 12+), triggered on push/PR
- **Code quality:** `.clang-format` (project style), `.clang-tidy` (basic checks), pre-commit hooks for format checking
- **Code coverage:** lcov/gcov setup in CI, coverage report upload (Codecov or similar)
- **Module task template:** A markdown template in `repo-plans/` defining the standard per-module workflow

**What it enables:** Any developer or AI agent can clone, open in devcontainer, push code, and get automated feedback immediately.

---

## M1 — Foundation

**Rationale:** Everything depends on `common` types and a simulation backend that can host and visualize algorithms. This is the single biggest blocker for all other work.

### `common` library
- `Pose2D` (x, y, theta), `Twist` (linear, angular), `Transform2D`
- 2D geometry utilities: point/line/polygon operations, angle wrapping, distance functions
- `OccupancyGrid` data structure (shared type, used by perception + planning)
- `LaserScan` data structure (shared sensor type)
- Interfaces / concepts: `Controller`, `Planner`, `StateEstimator` (optional thin concepts)
- Unit tests for all math utilities

### Simulation backend
- World model: static grid map, robot state (pose + velocity), simulated differential-drive kinematics
- Sim loop: fixed-timestep update at configurable rate (default 50 Hz)
- Simulated lidar sensor: ray casting against the grid map, configurable FOV/resolution/noise
- Crow HTTP + WebSocket server: state streaming at 30 Hz, REST command endpoints per architecture.md
- Scenario loader: load map + initial robot pose from JSON config files
- At least 1 test scenario (simple room with walls)
- Integration tests: server starts, accepts connections, streams valid state

**What it enables:** After M1, any module can be implemented, tested, and plugged into the sim loop. The simulated lidar provides sensor data for perception modules.

---

## M2 — Motion Planning Basics

**Rationale:** A* on a grid is the first "the robot goes somewhere" moment. Combined with DWA for local planning and velocity profiling, this creates a basic navigation pipeline.

| Module | Key deliverables |
|--------|-----------------|
| **`global_planning/astar`** | A* on 2D occupancy grid, octile heuristic, weighted A* variant, path output as waypoint list |
| **`global_planning/dijkstra`** | Dijkstra search, multi-source distance map (for costmap generation), shares grid interface with A* |
| **`local_planning/dwa`** | Dynamic Window Approach: velocity sampling in (v, ω) space, scoring against obstacles, trajectory rollout |
| **`trajectory_planning/velocity_profiling`** | Trapezoidal + S-curve velocity profiles along a path, curvature-aware speed limits |

**Mini-demo:** Robot in a room with obstacles. A* finds global path, DWA follows it locally with collision avoidance, velocity profiling smooths commands. Visible via sim WebSocket state stream.

**What it enables:** A working navigation pipeline. Perception (M4) later replaces "perfect knowledge" grid with sensor-built grids. Control (M5) replaces DWA velocity output with PID/pure_pursuit tracking.

---

## M3 — Frontends

**Rationale:** With a working sim backend and navigation pipeline, adding visual frontends gives immediate feedback for all subsequent domain work.

### Native frontend (ImGui / C++)
- SDL2 + OpenGL window with ImGui rendering
- WebSocket client (IXWebSocket) connects to sim, receives state stream
- REST client (cpp-httplib) sends commands (start/stop/reset, cmd_vel, scenario load)
- 2D canvas renderer: draws grid map, robot pose, lidar rays, planned path, DWA trajectory candidates
- Basic UI panels: sim controls, robot state readout, scenario selector
- Extensible rendering — later milestones add layers (particles, SLAM landmarks, multi-robot)

### Web frontend (TypeScript / React)
- React app with Canvas 2D renderer
- Native `WebSocket` API for state stream, `fetch()` for REST commands
- Same visual elements as native: grid, robot, path, lidar
- Deployable as a static site (portfolio/demo ready)

**What it enables:** Visual debugging and demo capability for every subsequent milestone.

---

## M4 — Perception

**Rationale:** Perception replaces perfect knowledge with sensor-derived data. First domain to get full educational treatment in the navigation stack.

| Module | Key deliverables |
|--------|-----------------|
| **`lidar_processing`** | Scan filtering (median, range clipping), DBSCAN clustering, RANSAC line extraction |
| **`occupancy_grid`** | Binary Bayes grid with log-odds update, sensor-model-based cell updates, inflation layer |
| **`ray_casting`** | Bresenham / DDA ray traversal, configurable noise injection |
| **`obstacle_detection`** | Detection pipeline: clustering + Kalman-filter-based obstacle tracking |

**Internal dependency chain:** `lidar_processing` and `ray_casting` are independent → `occupancy_grid` depends on both → `obstacle_detection` builds on clustering from `lidar_processing`.

**Mini-demo:** Robot navigates, but occupancy grid is built incrementally from lidar scans. Frontend shows grid filling in as robot explores. Detected obstacles highlighted.

**What it enables:** Realistic sensor-to-map pipeline. DWA and A* can now plan on live sensor data.

---

## M5 — Control

**Rationale:** Control replaces DWA's direct velocity output with proper closed-loop controllers that track paths precisely.

| Module | Key deliverables |
|--------|-----------------|
| **`pid`** | Discrete PID with anti-windup (clamping + back-calculation), derivative kick fix. Generic for speed/heading/any single-axis tracking. |
| **`pure_pursuit`** | Geometric path follower for diff-drive. Adaptive lookahead distance (speed-dependent). Outputs curvature → (v, ω). |

**Mini-demo:** Robot follows A*-planned path using pure_pursuit for steering and PID for speed. Frontend shows lookahead point, tracking error, controller outputs. Compare with DWA-only from M2.

**What it enables:** Proper path tracking. MPC (M8) builds on this foundation.

---

## M6 — State Estimation

**Rationale:** Introduces realistic uncertainty — the robot must estimate its own pose from noisy sensors.

| Module | Key deliverables |
|--------|-----------------|
| **`ekf`** | Extended Kalman Filter for diff-drive. Prediction: odometry motion model with noise. Update: range-bearing measurements to known landmarks. Pose estimate + covariance. Eigen-based. |
| **`particle_filter`** | Monte Carlo Localization (MCL). Motion model sampling, likelihood-field sensor model, systematic resampling. Adaptive particle count (AMCL). |

**Mini-demo:** Robot navigates using EKF-estimated pose (with uncertainty ellipse) instead of ground truth. Toggle EKF vs. particle filter. Particle cloud visualization. Show pose drift when estimation disabled.

**What it enables:** Realistic localization. SLAM (M7) extends EKF to joint pose + map estimation.

---

## M7 — SLAM

**Rationale:** Combines localization (M6) and perception (M4) — the robot simultaneously builds the map and localizes within it. Capstone of single-robot autonomy.

| Module | Key deliverables |
|--------|-----------------|
| **`ekf_slam`** | Augmented-state EKF-SLAM. Joint state: robot pose + landmarks. Data association (nearest-neighbor + Mahalanobis gating). Scales to ~100 landmarks. |
| **`lidar_slam`** | Scan-matching SLAM. ICP and/or NDT for frame-to-frame alignment. Simple pose-graph backend. Loop closure via scan similarity. |

**Mini-demo:** Robot explores unknown environment. EKF-SLAM builds landmark map; lidar_slam builds occupancy map from scan matching. Both overlaid with ground truth for comparison.

**What it enables:** Full autonomous exploration. Completes the core "navigation stack" story.

---

## M8 — Advanced Control (MPC)

**Rationale:** Most sophisticated single-robot controller. Builds on PID/pure_pursuit concepts, uses OSQP.

| Module | Key deliverables |
|--------|-----------------|
| **`mpc`** | Receding-horizon MPC for diff-drive. Linearized kinematic model, QP formulation (tracking + input constraints + obstacle avoidance). OSQP integration. Configurable horizon, weights, constraints. |

**Mini-demo:** MPC path following with visible prediction horizon. Compare tracking: PID → pure_pursuit → MPC.

**What it enables:** Optimal control baseline. DMPC (M10) extends to multi-robot. MPC concepts reused in TEB (M9).

---

## M9 — Advanced Planning

**Rationale:** Adds sampling-based and optimization-based planning approaches for continuous spaces, smooth trajectories, and joint space-time optimization.

| Module | Key deliverables |
|--------|-----------------|
| **`global_planning/rrt`** | RRT and RRT* in continuous 2D C-space. Collision checking against occupancy grid, path shortcutting. |
| **`trajectory_planning/spline_fitting`** | Cubic spline, Catmull-Rom, B-spline interpolation. Smooth, differentiable trajectories with curvature continuity. |
| **`trajectory_planning/teb`** | Timed Elastic Band. Joint path + timing optimization via g2o. Obstacle/kinematic/temporal constraints. |
| **`trajectory_planning/time_optimal`** | TOPP-RA, minimum-snap QP, bang-bang profiles. Fastest feasible trajectory under velocity/acceleration limits. |

**Internal ordering:** RRT and spline_fitting independent → TEB depends on spline + g2o → time_optimal independent but benefits from splines.

**Mini-demo:** Compare path quality: A* (grid, jagged) → RRT* (smooth, continuous) → spline-smoothed → TEB-optimized. Time-optimal profiling applied to each. Curvature and velocity profile visualization.

**What it enables:** Full trajectory planning toolkit for sophisticated single-robot motion.

---

## M10 — Multi-Robot

**Rationale:** Most advanced domain. Requires all prior single-robot capabilities. Algorithms progress from reactive to optimal to decentralized.

| Module | Key deliverables |
|--------|-----------------|
| **`orca`** | VO → RVO → ORCA. Reactive pairwise collision avoidance. Linear program for safe velocity. Real-time, no communication. |
| **`priority_planning`** | Sequential priority-based planning. Later robots treat earlier trajectories as moving obstacles. |
| **`cbs`** | Conflict-Based Search. Two-level: low-level A* per agent, high-level conflict tree. Provably optimal MAPF. |
| **`dmpc`** | Distributed MPC. Per-robot MPC with inter-robot trajectory constraints. Iterative negotiation. OSQP. |
| **`mader`** | MADER: decentralized asynchronous replanning. Shared committed trajectory segments. Handles async communication. |

**Module ordering:** ORCA (reactive) → priority_planning (centralized sequential) → CBS (centralized optimal) → DMPC (distributed optimization) → MADER (decentralized async).

**Mini-demo:** 3–5 robots in shared environment. Toggle between algorithms to see behavioral differences. All robots, paths, and collision avoidance visible.

**What it enables:** Complete multi-robot planning toolkit, simple to state-of-the-art.

---

## M11 — Polish & Showcase

**Rationale:** Turn "working codebase" into "presentable project."

- **Documentation site:** Doxygen or mkdocs from READMEs, theory.md, and API headers. GitHub Pages.
- **Example programs:** Standalone binaries per module showing usage. Independently buildable.
- **Capstone demo:** "Full autonomy" — robot explores unknown environment with SLAM, plans to goal with MPC, avoids dynamic obstacles with ORCA. Both frontends.
- **Packaging:** CMake install targets, `find_package(TheRobotLibrary COMPONENTS pid ekf astar)`.
- **README polish:** CI/coverage badges, demo GIFs, clear getting-started instructions.
- **Web deployment:** Build pipeline for React app as live demo on personal site.

---

## Cross-Cutting Concerns

| Concern | When |
|---------|------|
| **Unit testing** | Every module from M1 onward. Written alongside implementation (TDD encouraged). |
| **Integration testing** | First in M1 (sim server). Each milestone adds sim-integration tests. |
| **CI/CD** | Established M0. Extended as needed (web build in M3, coverage thresholds in M4+). |
| **Frontend rendering layers** | Each domain milestone adds a visualization layer to both frontends. |
| **Documentation** | Theory docs exist (scaffolded). Polished during each module's milestone. API docs in M11. |
| **Performance benchmarks** | Optional. Recommended for: A* (node expansion), DWA (throughput), particle_filter (scaling), CBS (agent scaling). |

---

## Standard Module Task Template

Applied to every module in every domain milestone:

```
## Module: <name>

### Phase 1 — Interface Design
- [ ] Define public API headers in include/<module>/
- [ ] Document expected inputs, outputs, and error conditions
- [ ] Define configuration struct(s) with sensible defaults
- [ ] Review: does the interface match architecture.md conventions?

### Phase 2 — Implementation
- [ ] Implement core algorithm in src/
- [ ] Only depend on `common` (and Eigen if needed)
- [ ] Use std::expected<T,E> for recoverable errors
- [ ] Follow naming conventions (PascalCase types, camelCase methods, m_camelCase members)

### Phase 3 — Unit Tests
- [ ] Test with known-answer inputs (hand-computed or reference values)
- [ ] Test edge cases (empty input, degenerate configurations, numerical limits)
- [ ] Test configuration variations
- [ ] All tests pass in CI

### Phase 4 — Simulation Integration
- [ ] Wire module into simulation loop (if applicable)
- [ ] Add module output to WebSocket state message
- [ ] Add REST endpoints for module-specific configuration (if needed)
- [ ] Integration test: sim runs with module active, state is valid

### Phase 5 — Frontend Visualization
- [ ] Add rendering layer to native frontend (ImGui ImDrawList)
- [ ] Add rendering layer to web frontend (Canvas 2D)
- [ ] Toggle visibility from UI

### Phase 6 — Mini-Demo & Docs
- [ ] Create demo scenario exercising this module
- [ ] Polish theory.md with implementation notes and lessons learned
- [ ] Update module README with usage examples
```

---

## Handoff

This design is ready for implementation planning via **superpower-plan**.
