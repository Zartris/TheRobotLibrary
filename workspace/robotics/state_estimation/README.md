# state_estimation

Probabilistic robot localization and mapping algorithms.

Each sub-folder is a **standalone C++ library** — copy only what you need. All sub-modules
depend only on `common`.

---

## Sub-modules

| Sub-module | Description |
|---|---|
| [`ekf/`](ekf/) | Extended Kalman Filter — nonlinear Gaussian localization |
| [`particle_filter/`](particle_filter/) | Monte Carlo Localization — handles multi-modal distributions |
| [`ekf_slam/`](ekf_slam/) | EKF-SLAM — joint localization and landmark mapping |
| [`lidar_slam/`](lidar_slam/) | Lidar-based SLAM — scan matching + loop closure |

---

## Extending this domain

State estimation grows naturally with the rest of the perception pipeline:
- `visual_odometry/` — camera-based pose estimation
- `graph_slam/` — pose graph optimization with loop closure (iSAM2 / g2o style)
- `inertial_fusion/` — IMU pre-integration + EKF fusion

Follow the standard sub-module layout and add to this table.

---

## Domain overview

See [`docs/theory.md`](docs/theory.md) for the Bayes filter framework that all
estimators in this domain are built on.

---

## Dependency rule

All sub-modules depend only on `common`. No sub-module may depend on another
state_estimation sub-module.
