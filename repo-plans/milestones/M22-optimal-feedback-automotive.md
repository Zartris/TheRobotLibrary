# M22 — Optimal Output Feedback & Automotive Perception

**Status:** Not Started  
**Dependencies:** M13 (`LQRController`, `IController` interface) for `lqg`; M6 (`common/camera.hpp`, image types) + M4 for `lane_detection`. M1 (EKF) implicitly used inside LQG.  
**Scope:** Two P8 modules from separate domains. `lqg` completes the optimal control sequence by adding a Kalman state observer to LQR, enabling output-only feedback. `lane_detection` adds structured-environment perception for automotive scenarios. Both are independently implementable within M22.

---

## Goal

LQR (M13) is a state-feedback controller — it requires full state observation. LQG adds the observer half: an internal EKF that estimates the full state from output measurements only. This "separation principle" combination — design LQR and Kalman filter independently and combine — is one of the key theoretical results in modern control. `lane_detection` provides a complementary perception capability for structured-environment / automotive scenarios where LQG output feedback is most relevant.

---

## Modules

### control/lqg

Linear Quadratic Gaussian controller. Wraps an `LQRController` (from M13) for state feedback and an internal `EKF` instance as the state observer. Accepts only output measurements `y = Cx + v` (not full state `x`). Demonstrates the separation principle.

- [ ] `include/lqg/lqg_controller.hpp` — `LQGController : IController`
- [ ] `include/lqg/lqg_config.hpp` — `LQGConfig` (system: `A, B, C`; LQR weights: `Q, R`; noise: process `Q_w`, measurement `R_v`; discretization `dt`)
- [ ] `src/lqg_controller.cpp` — initialize `LQRController` with `(A, B, Q, R)` → gain `K`; initialize internal `EKF` with `(A, B, C, Q_w, R_v)` → Kalman gain; `update(y)` → EKF predict + update; `computeControl(x̂)` → `LQR u = −K x̂`
- [ ] `tests/test_lqg_controller.cpp`:
  - Double integrator, output = position only (C = [1 0]): LQG drives position and velocity to zero; velocity estimate converges even though not measured
  - Separation principle: LQR gain `K` matches `LQRController::computeGain()` independently; Kalman gain `L` matches standalone EKF construction
  - AWGN measurement noise: steady-state estimation covariance matches theoretical DARE solution
  - LQR + perfect state observer (C = I): LQG reduces to pure LQR (verify numerically)
  - Wrong noise covariance (R_v underestimated by 10×): controller still stabilizes; documents behaviour
- [ ] Phase 4.5: `ILogger`, covariance trace + control effort + EKF residual norm at `DEBUG`, EKF compute time at `TRACE`
- [ ] Sim: selectable via ImGui controller dropdown
- [ ] ImGui panel: render estimated state covariance ellipse; control effort vs. time; estimation residual panel

### perception/lane_detection

Lane marking extraction from binary edge images (e.g., Canny output). Applies Hough transform to detect line candidates, fits a polynomial to each lane cluster via RANSAC, and assigns left/right lane identity based on slope and image position.

- [ ] `include/lane_detection/lane_detector.hpp` — `LaneDetector`
- [ ] `include/lane_detection/lane_detection_config.hpp` — `LaneDetectionConfig` (Hough: rho resolution, theta resolution, threshold, min_line_length, max_line_gap; polynomial degree; RANSAC iterations)
- [ ] `include/lane_detection/lane_types.hpp` — `LanePolynomial` (coefficients `std::vector<double>`, degree, validity flag), `LaneResult` (left: `LanePolynomial`, right: `LanePolynomial`)
- [ ] `src/lane_detector.cpp` — Hough transform (Eigen-based accumulator; no OpenCV dependency); left/right split: negative slope + left half → left lane, positive slope + right half → right lane; polynomial fit via RANSAC least-squares on Hough line sample points
- [ ] `tests/test_lane_detection.cpp`:
  - Synthetic edge image (two known straight lanes): detected polynomials within 5% of ground truth coefficients
  - Curved lanes (known quadratic): degree-2 polynomial fit within tolerance; curvature extracted correctly
  - Only left lane present: `right.validity = false`; left lane detected correctly; no crash
  - Pure noise edge image: both lanes report `validity = false`; no false-positive lines
  - Very narrow lane gap: both lanes still separated correctly if Hough threshold met
- [ ] Phase 4.5: `ILogger`, detected line count + polynomial fit residual at `DEBUG`, Hough accumulator build time at `TRACE`
- [ ] Sim note: lane_detection operates on pre-computed binary edge images; sim edge rendering deferred to post-M22 extension
- [ ] ImGui panel: render left/right lane polynomial curves overlaid on camera image

---

## Deliverables

- [ ] `control/lqg` module: interface, implementation, tests
- [ ] `perception/lane_detection` module: interface, implementation, tests
- [ ] LQG separation principle verified numerically in tests
- [ ] Lane detection: straight + curved synthetic lanes detected within tolerance
- [ ] Both modules hot-swappable in sim via ImGui
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. LQG drives double integrator to zero using position-only measurements; velocity estimate converges
2. Separation principle verified: independently-computed K and L match joint LQG construction
3. Lane detection: straight synthetic lanes detected with coefficient error < 5%
4. Curved lane: polynomial fit within tolerance; curvature correctly extracted
5. Missing lane: reports `validity = false` cleanly
6. Both modules hot-swappable in sim via ImGui
7. All unit tests pass, CI green
8. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

LQR servo tracking (integral action), output-constrained LQG, H∞ robust control, lane change maneuver planning, lane-keeping PID wrapper, deep-learning lane detection (→ M23), OpenCV dependency.
