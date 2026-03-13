# common/transforms

Header-only rigid-body transform types for 2D and 3D robotics: SE(2), SE(3), and SO(3).

**Standalone module.** Depends only on `common` (which pulls in Eigen transitively).
Copy this folder to any C++ project that already has Eigen available.

---

## What's in this module

| Type | Description |
|------|-------------|
| `transforms::SE2` | 2D rigid-body pose `(x, y, θ)` — ground-robot odometry, planner frames |
| `transforms::SE3` | 3D rigid-body pose `(translation, quaternion)` — sensor extrinsics, 3D state |
| `transforms::SO3` | 3D pure rotation `(quaternion)` — IMU orientation, camera attitude |

All types are pure value types (trivially copyable, no heap allocation) and support:
- `compose()` — group multiplication (chain transforms)
- `inverse()` — group inverse (invert a transform)
- `transformPoint()` — apply the transform to a point vector
- `toMatrix()` / `fromMatrix()` — convert to/from homogeneous matrix form

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(common/transforms)
target_link_libraries(your_target PRIVATE robotlib::transforms)
```

Or, when building the full workspace, `transforms` is included automatically via
`common/CMakeLists.txt`.

---

## Usage examples

### SE2 — 2D pose composition (robot odometry)

```cpp
#include <transforms/transforms.hpp>
using transforms::SE2;

// Robot starts at origin facing along +X
SE2 pose = SE2::identity();

// Drive forward 1 m
SE2 delta1{1.0, 0.0, 0.0};
pose = pose.compose(delta1);   // pose = (1, 0, 0)

// Turn 90 degrees left and drive forward 0.5 m
SE2 delta2{0.5, 0.0, std::numbers::pi / 2.0};
pose = pose.compose(delta2);   // pose = (1, 0.5, π/2)

// Convert to homogeneous matrix (e.g. for visualisation)
Eigen::Matrix3d T = pose.toMatrix();

// Project a landmark at (0.2, 0.0) in the robot frame to world frame
Eigen::Vector2d landmark_world = pose.transformPoint({0.2, 0.0});

// Compute relative transform between two poses
SE2 pose_a{1.0, 0.0, 0.0};
SE2 pose_b{2.0, 1.0, 0.5};
SE2 T_a_b = pose_a.inverse().compose(pose_b);  // b expressed in frame a
```

### SE3 — 3D camera-to-world transform (sensor extrinsics)

```cpp
#include <transforms/transforms.hpp>
using transforms::SE3;

// Camera mounted 0.3 m above base, rotated 15 degrees downward (pitch)
Eigen::Vector3d t_base_camera{0.0, 0.0, 0.3};
Eigen::Quaterniond R_base_camera{
    Eigen::AngleAxisd{-0.26, Eigen::Vector3d::UnitY()}  // ~15 deg pitch down
};
SE3 T_base_camera{t_base_camera, R_base_camera};

// Robot base pose in world frame (from state estimator)
SE3 T_world_base = SE3::fromTranslation(Eigen::Vector3d{5.0, 2.0, 0.0});

// Camera pose in world frame
SE3 T_world_camera = T_world_base.compose(T_base_camera);

// Transform a 3D point detected by the camera into world coordinates
Eigen::Vector3d p_camera{0.0, 0.0, 1.5};  // 1.5 m in front of camera
Eigen::Vector3d p_world = T_world_camera.transformPoint(p_camera);

// Project to 2D for a ground-plane planner
transforms::SE2 pose2d = T_world_camera.toSE2();
```

### SO3 — orientation from IMU

```cpp
#include <transforms/transforms.hpp>
using transforms::SO3;

// Build orientation from roll-pitch-yaw (ZYX convention)
SO3 orientation = SO3::fromRPY(0.05, -0.02, 1.57);  // ~90 deg yaw

// Rotate a gravity vector from body frame to world frame
Eigen::Vector3d g_body{0.0, 0.0, -9.81};
Eigen::Vector3d g_world = orientation.rotate(g_body);

// Extract angles for logging / display
Eigen::Vector3d rpy = orientation.toRPY();  // {roll, pitch, yaw}
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the full mathematical treatment:
SE(2)/SE(3)/SO(3) definitions, homogeneous matrix derivations, group operations,
quaternion representation, interpolation, and robotics applications.

**Key reference:** Lynch & Park, *Modern Robotics*, Chapter 3.
