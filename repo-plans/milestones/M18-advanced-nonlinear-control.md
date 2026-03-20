# M18 — Advanced Nonlinear Control

**Status:** Not Started  
**Dependencies:** M3 (stable `IController` interface + kinematic models). M13 recommended but not required.  
**Scope:** Two advanced nonlinear control algorithms extending the M3/M13 control chapter. MPPI provides stochastic trajectory optimisation via Monte Carlo rollouts; feedback linearization provides exact input-output linearization for differential-drive systems. Both are independently implementable within M18.

---

## Goal

M18 pushes control beyond deterministic-optimal (LQR/MPC) and geometric (pure_pursuit/Stanley) into the two dominant approaches for nonlinear systems: randomized optimization (MPPI) and algebraic state transformation (feedback linearization). MPPI is increasingly used in aggressive autonomy (legged locomotion, racing); feedback linearization is the textbook gateway to Lie-group control theory. Both sit naturally after M13's LQR/Stanley.

---

## Modules

### control/mppi

Model Predictive Path Integral controller. Samples N control trajectories as Gaussian perturbations of a nominal sequence, propagates each through the robot's motion model, scores by a configurable cost function, and computes the importance-weighted update to the nominal control.

- [ ] `include/mppi/mppi_controller.hpp` — `MPPIController : IController`
- [ ] `include/mppi/mppi_config.hpp` — `MPPIConfig` (N rollouts, horizon H, temperature λ, noise covariance Σ, dt)
- [ ] `include/mppi/mppi_cost.hpp` — `MPPICostFn = std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>` (state, control → cost scalar)
- [ ] `src/mppi_controller.cpp` — Monte Carlo rollout loop, importance weight normalization, control update: `u* = Σ (w_k · ε_k)` where `w_k = exp(−J_k / λ)`
- [ ] Configurable motion model via `std::function<VectorXd(VectorXd, VectorXd)>` — allows testing on diff-drive and double-integrator
- [ ] `tests/test_mppi_controller.cpp`:
  - Double integrator: MPPI drives state to target; final error < tolerance
  - Obstacle avoidance: high-cost obstacle region → sampled distribution clusters around valid trajectories
  - Weight normalization: sum of `w_k` over N rollouts = 1 for any λ > 0
  - Temperature λ: high λ → uniform weights (exploration); low λ → peaked on best rollout (exploitation)
  - N = 1 (degenerate case): no crash; produces finite control output
- [ ] Phase 4.5: `ILogger`, rollout cost distribution (min/max/mean) at `DEBUG`, per-iteration solve time at `TRACE`
- [ ] Sim: selectable via ImGui controller dropdown
- [ ] ImGui panel: visualise rollout fan (N sampled trajectories) + selected trajectory (highlighted)

### control/feedback_linearization

Exact input-output linearization for a differential-drive robot in chained form. Transforms the nonlinear system `(x, y, θ, v)` to a pair of decoupled linear integrators by introducing a virtual output point ahead of the robot's axle. Includes a numerical Lie derivative checker for test verification.

- [ ] `include/feedback_linearization/feedback_linearization_controller.hpp` — `FeedbackLinearizationController : IController`
- [ ] `include/feedback_linearization/feedback_linearization_config.hpp` — `FeedbackLinearizationConfig` (lookahead `l`, inner controller gains, velocity limits)
- [ ] `src/feedback_linearization_controller.cpp` — diffeomorphism: virtual output `ξ = (x + l·cos θ, y + l·sin θ)`; linearizing control law → `(v, ω)`; inverse kinematics back to robot frame
- [ ] `src/lie_derivative_check.cpp` — numerical check: verify `L_f h(x)` via finite-difference matches analytic expression
- [ ] `tests/test_feedback_linearization_controller.cpp`:
  - Straight-line tracking: virtual output tracks reference line; `(x, y, θ)` converges within tolerance
  - Circular arc: nonlinear system tracks circular reference; CTE bounded
  - Lookahead `l`: larger `l` → smoother but lagged response; smaller `l` → tighter but noisier
  - Lie derivative check: numerical `L_f h(x)` matches analytic expression (< 1e-5 relative error)
  - Singularity: `l = 0` → documented error via `std::expected`; no division by zero
- [ ] Phase 4.5: `ILogger`, virtual output error + Lie derivative residual at `DEBUG`, computation time at `TRACE`
- [ ] Sim: selectable via ImGui controller dropdown
- [ ] ImGui panel: render virtual output point trajectory overlay

---

## Deliverables

- [ ] `control/mppi` module: interface, implementation, tests
- [ ] `control/feedback_linearization` module: interface, implementation, tests
- [ ] Both controllers hot-swappable via ImGui mid-run without crash
- [ ] MPPI: importance weight normalization and temperature behaviour verified in tests
- [ ] Feedback linearization: Lie derivative numerical check passes
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. MPPI drives double integrator to target within tolerance; importance weights sum to 1
2. MPPI obstacle avoidance: obstacle-colliding rollouts receive near-zero weight
3. Feedback linearization: straight-line and circular-arc tracking within CTE tolerance
4. Lie derivative numerical check passes (< 1e-5 relative error vs. analytic)
5. Both controllers hot-swappable via ImGui mid-run without crash
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

GPU-accelerated rollouts (CUDA MPPI), gradient-based trajectory optimisation (iLQR/DDP), full SE3 diffeomorphism, Lie group Kalman filters, feedback linearization for non-diff-drive kinematics (Ackermann, holonomic).
