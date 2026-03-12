# ekf

Extended Kalman Filter (EKF) for nonlinear robot localization.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `EKF` — generic nonlinear EKF with pluggable motion and observation models
- `DifferentialDriveMotionModel` — odometry-based motion model for diff-drive robots
- `LandmarkObservationModel` — range-bearing observation model for known-map localization

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(ekf)
target_link_libraries(your_target PRIVATE ekf)
```

```cpp
#include <ekf/ekf.hpp>
#include <ekf/diff_drive_motion_model.hpp>
#include <ekf/landmark_observation_model.hpp>

ekf::EKF estimator{initial_pose, initial_covariance};
estimator.predict(cmd_vel, dt);
estimator.update(landmark_observations);

common::Pose2D pose = estimator.mean();
Eigen::Matrix3d cov = estimator.covariance();
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the EKF equations, Jacobian derivations,
and worked example for a differential-drive robot.
