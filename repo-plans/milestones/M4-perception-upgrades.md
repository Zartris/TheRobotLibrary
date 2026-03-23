# M4 — Perception Upgrades

**Status:** Not Started  
**Dependencies:** M2 (stable perception interfaces)  
**Scope:** Obstacle detection, RANSAC line extraction, occupancy grid inflation, dynamic obstacles.

---

## Goal

Upgrade the perception pipeline from "basic lidar → grid" to a richer understanding of the environment: detected and tracked obstacles, extracted line features, and an inflated costmap for safer planning.

---

## Modules

### obstacle_detection

DBSCAN clustering on lidar scan points + simple Kalman-filter-based obstacle tracking.

- [ ] `include/obstacle_detection/obstacle_detector.hpp` — `ObstacleDetector`
- [ ] `src/obstacle_detector.cpp`
- [ ] `tests/test_obstacle_detector.cpp`:
  - Cluster 3 distinct groups of points → 3 obstacles
  - Track obstacle across 5 frames → stable ID + velocity estimate
  - No clusters in empty scan → empty output
  - Noise rejection (single-point clusters filtered)

### lidar_processing — RANSAC enhancement

Add RANSAC line extraction to the existing scan filter.

- [ ] `include/lidar_processing/line_extractor.hpp` — `extractLines()`
- [ ] `src/line_extractor.cpp`
- [ ] `tests/test_line_extractor.cpp`:
  - Scan against straight wall → single line with correct parameters
  - L-shaped wall → two lines
  - Noisy scan → lines extracted within tolerance
  - No structure → empty result

### occupancy_grid — Inflation layer

> Note: `occupancy_grid` moves from `perception/` to `mapping/` in M9.

Expand obstacles by robot radius for collision-safe planning.

- [ ] Add `inflate()` method to `OccupancyGridMap`
- [ ] `tests/test_inflation.cpp`:
  - Single occupied cell + radius 3 → circle of inflated cells
  - Inflation doesn't exceed grid bounds
  - Re-inflation after map update works correctly

### Simulation — Dynamic obstacles

- [ ] Add moving obstacle entities to world model (circles/rectangles, linear/circular motion)
- [ ] Configurable in scenario JSON (`"dynamic_obstacles": [...]`)
- [ ] Dynamic obstacles appear in lidar scans (via ray_casting)
- [ ] Simulation app: render dynamic obstacles, detected obstacle bounding boxes, extracted lines, inflated grid overlay in MuJoCo 3D scene / ImGui telemetry panel

---

## Deliverables

- [ ] obstacle_detection module: implementation + tests
- [ ] RANSAC line extraction: implementation + tests
- [ ] Inflation layer: implementation + tests
- [ ] Dynamic obstacles in sim
- [ ] Simulation app visualization for all new features

## Exit Criteria

1. Robot navigates among dynamic obstacles without collision
2. Detected obstacles tracked across frames with stable IDs
3. Inflation layer prevents close-to-wall path planning
4. All unit tests pass, CI green
5. All modules pass Phase 4.5 — Observability gate (state transitions logged at DEBUG, hot-loop metrics at TRACE)

## NOT IN

SLAM, new planners, new controllers.
