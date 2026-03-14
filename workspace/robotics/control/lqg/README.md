# lqg

Linear Quadratic Gaussian (LQG) controller. Combines an `LQRController` (optimal state feedback, from M13) with an internal EKF state observer to enable optimal output-feedback control. Demonstrates the separation principle: LQR gain and Kalman filter gain are designed independently and composed without loss of optimality.

**Milestone:** M22 — Optimal Output Feedback & Automotive Perception  
**Status:** Scaffold only — awaiting implementation

## Features

- `LQGController : IController` — accepts only output measurements `y = Cx + v`
- Internal EKF estimates full state `x̂` from partial observations
- Separation principle: design LQR `(A, B, Q, R)` and Kalman filter `(A, C, Q_w, R_v)` independently
- Reduces to pure LQR when C = I (perfect state knowledge)

## Dependencies

- `common` (logging, types)
- `lqr` (M13 — `LQRController`)
- `ekf` (M1 — state observer)
- Eigen3 (matrix math)
