# Module: `feature_extraction`

**Milestone:** M6 — Visual Perception Building Blocks
**Status:** Not Started (pre-scaffolded)
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/feature_extraction/feature_extraction.hpp`:
  - `Keypoint` (x, y, response, octave)
  - `Descriptor` (32-byte array for BRIEF)
  - `Feature` (keypoint + descriptor)
  - `Match` (queryIdx, trainIdx, distance)
  - `FeatureError` enum
  - `GrayscaleImage` (width, height, pixel buffer)
  - `FeatureExtractorConfig` (max_features, threshold)
  - `FeatureExtractor` — `detectAndCompute(GrayscaleImage) → std::expected<std::vector<Feature>, FeatureError>`
  - `MatcherConfig` (ratio_threshold, cross_check)
  - `DescriptorMatcher` — `match(descriptors_a, descriptors_b) → std::vector<Match>`
- [ ] All public API errors returned via `std::expected<T, FeatureError>`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt` with Catch2 target `feature_extraction_tests`
- [ ] `tests/feature_extraction_tests.cpp`:
  - `hammingDistance` on identical descriptors → 0
  - `hammingDistance` on all-flipped descriptors → 256
  - `hammingDistance` on single-bit-flip descriptor → 1
  - `DescriptorMatcher` cross-check: A→B and B→A must agree
  - `DescriptorMatcher` ratio test: close-second match filtered out
  - Empty-input guard: zero descriptors returns empty match list
- [ ] Verify tests fail before implementation

```bash
cmake --build build --target feature_extraction_tests
cd build && ctest -R feature_extraction --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] `src/feature_extraction.cpp`:
  - `hammingDistance` via `__builtin_popcount` XOR over 8×4-byte chunks
  - `DescriptorMatcher::match` — forward pass + optional cross-check + Lowe's ratio test + sort by distance
  - `FeatureExtractor::detectAndCompute` — FAST corner detection + rBRIEF descriptor sampling
    - FAST stub with TODO for full non-max suppression acceptable at this milestone
- [ ] Only depends on `common`; no OpenCV
- [ ] Use `common::getLogger("feature_extraction")`

### Phase 4 — Passing Tests

- [ ] All `feature_extraction_tests` pass

```bash
cmake --build build --target feature_extraction_tests
cd build && ctest -R feature_extraction --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.**

- [ ] `ILogger` injected via `common::getLogger("feature_extraction")`
- [ ] State transitions at `DEBUG` (detect entry, match entry, error paths)
- [ ] Hot-loop metrics at `TRACE` (per-descriptor distance, iteration count)
- [ ] At least one test asserts expected log lines appear
- [ ] Zero `ERROR`-level entries during nominal test runs

```bash
cmake --build build --target feature_extraction_tests
cd build && ctest -R feature_extraction --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] `FeatureExtractor` used by sim integration test `test_vo_integration` (in `workspace/simulation/tests/`)
- [ ] Module itself does not link against simulation — sim links the module

### Phase 6 — Frontend Visualization

- [ ] (M6.5 / M9) Feature tracks overlay on camera view panel

### Phase 7 — Docs Polish

- [ ] Update `workspace/robotics/perception/feature_extraction/README.md` with usage examples
- [ ] Add FAST + BRIEF theory notes to `docs/theory.md`
- [ ] Move this file to `repo-plans/modules/done/feature_extraction.md`
