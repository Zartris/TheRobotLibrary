# noise_models

Seeded, reproducible noise injection primitives for testing. All types are templated on scalar type `T` and accept an explicit RNG seed for deterministic test sequences. Header-only INTERFACE library — no compilation step required.

**Milestone:** M21 — Estimation & Test Foundations  
**Status:** Scaffold only — awaiting implementation

## Features

- `GaussianNoise<T>` — zero-mean Gaussian with configurable σ, seeded `std::mt19937`
- `UniformNoise<T>` — uniform distribution over configurable `[lo, hi]`
- `OutlierInjector<T>` — wraps any base model; injects large outliers with probability `p`
- Batch API: `sampleN(n)` → `std::vector<T>`
- In-place injection: `addTo(Eigen::VectorXd&)` — dimension-wise noise addition

## Usage

```cpp
#include <common/noise_models.hpp>

GaussianNoise<double> noise{sigma, /*seed=*/42};
auto samples = noise.sampleN(100);  // deterministic with same seed

Eigen::VectorXd measurement = groundTruth;
noise.addTo(measurement);  // in-place noise injection
```

## Dependencies

- `common` (logging)
- Eigen3 (`VectorXd` in-place injection)
