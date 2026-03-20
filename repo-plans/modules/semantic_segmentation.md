# Module: semantic_segmentation

**Milestone:** M23 — Lattice Planning & Semantic Vision  
**Status:** Not Started  
**Depends on:** common, M6 (`common/camera.hpp` `RgbImage` type)

> **Note:** This is a stub-only module. No DL inference in M23 scope. `StubSemanticSegmenter`
> and `PluginSemanticSegmenter` define the interface and plugin injection point for a future
> ONNX/TensorRT backend. See `docs/theory.md` for DL Backend Integration guide.

---

### Phase 1 — Interface Design

- [ ] `include/semantic_segmentation/semantic_segmenter.hpp` — `ISemanticSegmenter` (pure virtual: `std::expected<SemanticMap, std::string> segment(const RgbImage&)`); `StubSemanticSegmenter : ISemanticSegmenter` (returns all-`ROAD` map for valid image)
- [ ] `include/semantic_segmentation/semantic_types.hpp` — `SemanticClass` enum (`ROAD, SIDEWALK, OBSTACLE, PERSON, VEHICLE, BUILDING, SKY, UNKNOWN`); `SemanticMap` (2D grid same dimensions as input image)
- [ ] `include/semantic_segmentation/segmenter_plugin.hpp` — `SegmenterPlugin = std::function<std::expected<SemanticMap, std::string>(const RgbImage&)>`; `PluginSemanticSegmenter : ISemanticSegmenter`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_semantic_segmentation.cpp`:
  - Stub: output `SemanticMap` has same dimensions as input image; all labels are valid `SemanticClass` values
  - Stub: empty image (0×0) → returns `std::expected` error; no crash
  - Plugin: custom lambda returning checkerboard `ROAD/OBSTACLE` map → invoked correctly; output matches lambda
  - Plugin: lambda throwing exception → error propagated as `std::expected` error string; no unhandled exception
  - Interface: both `StubSemanticSegmenter` and `PluginSemanticSegmenter` assignable to `ISemanticSegmenter&`

### Phase 3 — Implementation (Green)

- [ ] `src/stub_semantic_segmenter.cpp` — returns all-`ROAD` `SemanticMap` for valid input; `std::expected` error for invalid
- [ ] `src/plugin_semantic_segmenter.cpp` — wraps `SegmenterPlugin` callable; propagates exceptions as `std::expected` errors

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("semantic_segmentation")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, image dimensions + inference mode)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target semantic_segmentation_tests
cd build && ctest -R semantic_segmentation --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Sim note: semantic overlay requires sim rendering extension; M23 scope covers offline/batch processing only

### Phase 6 — Frontend Visualization

- [ ] No live frontend visualization in M23 scope (requires sim rendering extension)

### Phase 7 — Docs Polish

- [ ] `docs/theory.md` must include "DL Backend Integration" section: how to implement a `SegmenterPlugin` wrapping ONNX Runtime, expected model input format (HWC float RGB, normalized), output format (class index map)
- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/semantic_segmentation.md`
