# lane_detection

Lane marking extraction from binary edge images (e.g., Canny output). Applies a Hough transform to detect line candidates, fits a polynomial to each lane cluster via RANSAC, and assigns left/right lane identity based on slope and image position. No OpenCV dependency.

**Milestone:** M22 — Optimal Output Feedback & Automotive Perception  
**Status:** Scaffold only — awaiting implementation

## Features

- `LaneDetector` — Hough accumulator (Eigen-based, no OpenCV)
- Left/right lane split by slope sign + image half
- Polynomial fitting (configurable degree) via RANSAC on Hough line sample points
- `LaneResult` with `LanePolynomial` (coefficients + validity flag) for each lane

## Dependencies

- `common` (logging, image types from `common/camera.hpp`)
- Eigen3 (accumulator and least-squares fitting)
