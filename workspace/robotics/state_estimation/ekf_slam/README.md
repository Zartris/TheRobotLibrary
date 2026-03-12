# ekf_slam

EKF-SLAM — simultaneous localization and mapping with an Extended Kalman Filter.
The robot builds a map of landmarks while localizing itself within it, with no prior map.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `EKFSLAM` — joint estimator over robot pose + landmark positions
- `LandmarkAssociator` — data association (nearest-neighbour + Mahalanobis gating)
- State vector is grown dynamically as new landmarks are observed

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(ekf_slam)
target_link_libraries(your_target PRIVATE ekf_slam)
```

```cpp
#include <ekf_slam/ekf_slam.hpp>

ekf_slam::EKFSLAM slam{initial_pose};

// In the control loop:
slam.predict(cmd_vel, dt);
slam.update(landmark_observations);   // range-bearing observations to detected landmarks

common::Pose2D  robot_pose = slam.robotPose();
auto      landmarks  = slam.landmarks();   // std::vector<common::Point2D>
```

---

## Limitations and When to Use

EKF-SLAM scales **quadratically** $O(N^2)$ in the number of landmarks — it is suitable
for small structured environments (tens to low hundreds of landmarks). For large-scale
environments, prefer `lidar_slam/` (graph-based, $O(N)$ per step).

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the augmented state formulation, the full
Jacobian derivation, and data association strategies.
