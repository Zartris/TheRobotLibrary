# Module: noise_models

**Milestone:** M21 — Estimation & Test Foundations  
**Status:** Not Started  
**Depends on:** common (header-only; zero external dependencies beyond common)

---

### Phase 1 — Interface Design

- [ ] `include/common/noise_models.hpp` — `GaussianNoise<T>` (configurable `σ`, `std::mt19937` seeded RNG); `UniformNoise<T>` (configurable `[lo, hi]`); `OutlierInjector<T>` (base noise model + outlier probability `p` + outlier range)
- [ ] Batch API: `sampleN(n)` → `std::vector<T>`; `addTo(Eigen::VectorXd&)` → in-place noise injection
- [ ] **Header-only INTERFACE library** — all implementation in the header

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_noise_models.cpp`:
  - Gaussian mean: 10000 samples → `|mean| < 0.01·σ`; variance: `|var − σ²| < 0.05·σ²`
  - Uniform coverage: bucket histogram of 1000 samples → all buckets within 20% of uniform expectation
  - Seeded reproducibility: two `GaussianNoise<double>` with same seed → identical `sampleN(100)` sequence
  - Different seeds → sequences differ at first differing sample
  - `OutlierInjector`: 1000 samples with `p = 0.1` → outlier count in `[70, 130]`
  - `addTo(Eigen::VectorXd&)`: adds correct noise dimension-wise; does not modify vector size

### Phase 3 — Implementation (Green)

- [ ] All implementation in `include/common/noise_models.hpp`:
  - `GaussianNoise::sample()` → `std::normal_distribution<T>`
  - `UniformNoise::sample()` → `std::uniform_real_distribution<T>`
  - `OutlierInjector::sample()` → Bernoulli trial selects base model or uniform outlier

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected via `common::getLogger("noise_models")` (mockable in tests)
- [ ] Seed value logged at `DEBUG` level on construction; not logged per-sample (too hot)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target noise_models_tests
cd build && ctest -R noise_models --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] No sim integration required; `noise_models` is used by other modules' test suites

### Phase 6 — Frontend Visualization

- [ ] No visualization required for this utility module

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/noise_models.md`
