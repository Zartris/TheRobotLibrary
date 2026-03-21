# Module: place_recognition

**Milestone:** M17 — Camera Perception II  
**Status:** Not Started  
**Depends on:** common, M6 (`feature_extraction` descriptors)

---

### Phase 1 — Interface Design

- [ ] `include/place_recognition/place_recognizer.hpp` — `PlaceRecognizer`
- [ ] `include/place_recognition/place_database.hpp` — `PlaceDatabase` (indexed by frame ID)
- [ ] `include/place_recognition/place_types.hpp` — `PlaceDescriptor` (generic `std::vector<float>`), `PlaceCandidate` (frame_id + score)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_place_recognizer.cpp`:
  - 10-frame database: query with same descriptor → correct frame ID returned (score ≥ threshold)
  - Query with perturbed descriptor (5% noise): top-1 still correct
  - False positive rate: random descriptor → no match above threshold (100 random queries, < 2 false positives)
  - Lidar descriptor: scan histogram passed as `PlaceDescriptor`; visual descriptor: aggregated BRIEF as `PlaceDescriptor` — same API handles both
  - Large database (1000 frames): query completes in < 10 ms (linear scan acceptable at this scale)

### Phase 3 — Implementation (Green)

- [ ] `src/place_recognizer.cpp` — L2 / cosine similarity search; nearest-neighbour with score threshold

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("place_recognition")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, database size, query time)
- [ ] Hot-loop performance metrics logged at `TRACE` level (candidate scores per query in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target place_recognition_tests
cd build && ctest -R place_recognition --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Integration note: output `PlaceCandidate` list can be routed to `pose_graph (M14)` as loop closure edges
- [ ] No standalone ImGui entry; consumed by SLAM pipeline assembler

### Phase 6 — Visualization

- [ ] Rendered indirectly via SLAM loop closure visualization (post-M17 wiring)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/place_recognition.md`
