# Visual-Inertial Odometry Theory

## 1. Motivation

Visual odometry (VO) drifts over time because it relies only on camera observations.
IMU integration accumulates error from uncorrected bias. Fusing both modalities exploits
their complementary strengths:
- IMU: high-rate, captures fast motion, bias-corrupted long-term
- VO: absolute scale (with stereo) or scale-ambiguous (monocular), corrects IMU drift

## 2. Loosely-Coupled Architecture

The loosely-coupled design treats the VO output (a relative pose estimate) as a
measurement in the estimator — without re-processing raw image features. This avoids
tight coupling complexity while still achieving significant drift reduction.

**Pipeline:**
```
IMU measurements → ImuPreintegrator → VIOState predict (high rate)
                                          ↓
VO keyframe pose  → OdometryResult → VIOState update (keyframe rate)
```

## 3. State Vector

$$x = \begin{bmatrix} p \\ q \\ v \\ b_a \\ b_g \end{bmatrix}$$

where $p \in \mathbb{R}^3$ is position, $q \in SO(3)$ is orientation (unit quaternion),
$v \in \mathbb{R}^3$ is velocity, $b_a$ and $b_g$ are accelerometer and gyroscope biases.

## 4. Predict Step (IMU)

Using the pre-integration result $(\Delta R, \Delta v, \Delta p)$ from `imu_processing`:
$$p_{k+1} = p_k + v_k \Delta t + R_k \Delta p$$
$$v_{k+1} = v_k + R_k \Delta v - g \Delta t$$
$$R_{k+1} = R_k \Delta R$$

Covariance is propagated via the pre-integration Jacobians w.r.t. bias.

## 5. Update Step (VO)

The VO relative pose $T_{ij}$ provides a measurement constraint. The innovation
$e = T_{ij} \ominus \hat{T}_{ij}$ drives the EKF/UKF update, correcting both pose and
bias estimates.

## 6. Bias Observability

IMU biases are not directly observable from IMU alone. They become observable through
the VO updates: a consistent pattern of VO correcting the IMU prediction in a particular
direction reveals the bias. After sufficient VO updates in varied motion, bias estimates
converge toward their true values.
