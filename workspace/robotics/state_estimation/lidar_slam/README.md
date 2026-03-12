# lidar_slam

Lidar-based SLAM using scan matching and pose graph optimization with loop closure.
Suitable for large environments where EKF-SLAM's $O(N^2)$ complexity is prohibitive.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- **Scan matching** — ICP (Iterative Closest Point) and NDT (Normal Distributions Transform)
  for estimating odometry from consecutive lidar scans
- **Pose graph** — sparse graph of robot poses connected by relative transform edges
- **Loop closure detection** — recognize previously visited places and add constraints
- **Graph optimization** — minimize the accumulated drift by solving the pose graph (Gauss-Seidel / Levenberg-Marquardt)

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(lidar_slam)
target_link_libraries(your_target PRIVATE lidar_slam)
```

```cpp
#include <lidar_slam/lidar_slam.hpp>

lidar_slam::LidarSLAM slam{};

// In the sensor loop:
slam.addScan(lidar_scan, odometry_delta);

// Query current best estimate:
common::Pose2D pose = slam.currentPose();
auto occupancy_map  = slam.getMap();
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for scan matching algorithms (ICP, NDT), pose
graph construction, and loop closure and nonlinear graph optimization.
