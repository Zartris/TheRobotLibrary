# obstacle_detection

Unified obstacle detection — detects and tracks obstacles from sensor data (lidar, camera,
mono camera) through a common interface.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.
Extend by adding a new detector class implementing `IObstacleDetector`.

---

## What's in this module

- `IObstacleDetector` — abstract interface all detectors implement
- `LidarObstacleDetector` — cluster-based detection from 2D lidar scans
- `MonoObstacleDetector` — monocular camera obstacle detection stub (depth estimation based)
- `ObstacleTracker` — multi-object tracker (simple nearest-neighbour association + Kalman filter)
- `Obstacle` — type representing a detected obstacle: position, extent, velocity estimate, id

---

## Extending

To add a new sensor modality (e.g. stereo camera, radar):
1. Create `include/obstacle_detection/<new_detector>.hpp` implementing `IObstacleDetector`
2. Add the implementation in `src/`
3. Register it in your application — no changes to existing code needed

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(obstacle_detection)
target_link_libraries(your_target PRIVATE obstacle_detection)
```

```cpp
#include <obstacle_detection/lidar_obstacle_detector.hpp>
#include <obstacle_detection/obstacle_tracker.hpp>

obstacle_detection::LidarObstacleDetector detector{};
auto obstacles = detector.detect(lidar_scan, robot_pose);

obstacle_detection::ObstacleTracker tracker{};
auto tracked = tracker.update(obstacles, dt);  // returns obstacles with persistent IDs
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for clustering algorithms, the detection–tracking
pipeline, and the Kalman filter used for velocity estimation.
