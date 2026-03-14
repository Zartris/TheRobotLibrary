# M23 — Lattice Planning & Semantic Vision

**Status:** Not Started  
**Dependencies:** M7 (`IGlobalPlanner` interface, planning infrastructure) for `lattice_planner`; M6 (`common/camera.hpp` image types) for `semantic_segmentation`.  
**Scope:** Two P9 modules. Lattice planner uses pre-computed motion primitives over a discrete state lattice for structured-environment planning. Semantic segmentation defines a clean stub interface for pixel-level scene understanding, with an explicit plugin point for a future DL backend. **No DL framework dependency in M23.**

---

## Goal

Lattice planning completes the "structured-environment planning" chapter: where RRT/PRM search continuous C-space, the lattice planner exploits the regularity of structured environments (roads, corridors) by restricting motion to pre-computed kinematically-feasible primitives. Semantic segmentation addresses the known DL-dependency challenge via deliberate stub-first design: M23 ships `ISemanticSegmenter` + `StubSemanticSegmenter` with a documented plugin interface for future ONNX/TensorRT. The interface remains durable regardless of which DL backend is eventually adopted.

---

## Modules

### motion_planning/global_planning/lattice_planner

State lattice planner over a discrete `(x, y, θ)` grid with pre-computed kinematically-feasible motion primitives. Planning is a graph search (Dijkstra or A* with 2D Euclidean heuristic) over nodes connected by primitives.

- [ ] `include/lattice_planner/lattice_planner.hpp` — `LatticePlanner : IGlobalPlanner`
- [ ] `include/lattice_planner/lattice_planner_config.hpp` — `LatticePlannerConfig` (lattice_resolution, num_headings, num_primitives_per_heading, max_primitive_length, use_astar flag)
- [ ] `include/lattice_planner/lattice_types.hpp` — `LatticeNode` (grid_x, grid_y, heading_idx), `MotionPrimitive` (sequence of `Pose2D` waypoints, cost), `LatticeGraph` (adjacency list)
- [ ] `src/lattice_planner.cpp` — primitive generation: for each heading, generate arcs (straight + left/right curves of varying radius) connecting to neighbouring lattice nodes; graph construction: check each primitive for `OccupancyGrid` collision; A* / Dijkstra search on `LatticeGraph`
- [ ] `tests/test_lattice_planner.cpp`:
  - Straight corridor: planner finds straight path using zero-curvature primitives
  - 90° turn required: planner selects turning primitive; output path is kinematically feasible (heading continuity at each node)
  - Blocked corridor: planner routes around via alternate primitives
  - No path exists (all primitives blocked): returns `std::nullopt`; no crash
  - Lattice resolution: coarser grid → faster planning (measured); path within 20% of fine-grid result
  - Heading continuity: consecutive primitive start/end headings align within heading discretization tolerance
- [ ] Phase 4.5: `ILogger`, graph node/edge counts + A* expansion count at `DEBUG`, primitive generation time at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/global_planner {"type":"lattice"}`
- [ ] Frontend: render lattice graph (grey edges) + planned path (highlighted) + heading arrows at each node

### perception/semantic_segmentation

Pixel-level scene classification with a stub implementation and explicit DL plugin interface. Ships `ISemanticSegmenter`, `StubSemanticSegmenter`, and a `SegmenterPlugin` (`std::function` injection point). **No DL inference in M23 scope.**

- [ ] `include/semantic_segmentation/semantic_segmenter.hpp` — `ISemanticSegmenter` (pure virtual: `std::expected<SemanticMap, std::string> segment(const RgbImage&)`); `StubSemanticSegmenter : ISemanticSegmenter` (returns all-`ROAD` map for valid image)
- [ ] `include/semantic_segmentation/semantic_types.hpp` — `SemanticClass` enum (`ROAD, SIDEWALK, OBSTACLE, PERSON, VEHICLE, BUILDING, SKY, UNKNOWN`); `SemanticMap` (2D grid same dimensions as input image)
- [ ] `include/semantic_segmentation/segmenter_plugin.hpp` — `SegmenterPlugin = std::function<std::expected<SemanticMap, std::string>(const RgbImage&)>`; `PluginSemanticSegmenter : ISemanticSegmenter` (wraps a plugin for runtime DL backend injection)
- [ ] `src/stub_semantic_segmenter.cpp`
- [ ] `src/plugin_semantic_segmenter.cpp`
- [ ] `tests/test_semantic_segmentation.cpp`:
  - Stub: output `SemanticMap` has same dimensions as input image; all labels are valid `SemanticClass` values
  - Stub: empty image (0×0) → returns `std::expected` error; no crash
  - Plugin: custom lambda returning checkerboard `ROAD/OBSTACLE` map → invoked correctly
  - Plugin: lambda throwing exception → error propagated as `std::expected` error string; no unhandled exception
  - Interface: both `StubSemanticSegmenter` and `PluginSemanticSegmenter` assignable to `ISemanticSegmenter&`
- [ ] Phase 4.5: `ILogger`, image dimensions + inference mode at `DEBUG`
- [ ] Future-DL documentation: `docs/theory.md` must include a "DL Backend Integration" section documenting how to implement a `SegmenterPlugin` wrapping ONNX Runtime
- [ ] Sim note: semantic overlay requires sim rendering extension; M23 scope covers offline/batch processing only

---

## Deliverables

- [ ] `motion_planning/global_planning/lattice_planner` module: interface, implementation, tests
- [ ] `perception/semantic_segmentation` stub module: interface + stub + plugin injection, tests
- [ ] Lattice planner: heading continuity and path feasibility verified in tests
- [ ] Semantic segmentation: plugin injection and interface contract verified
- [ ] DL backend integration guide documented in `docs/theory.md`
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. Lattice planner finds straight path; finds turning path around 90° turn
2. Heading continuity: consecutive primitives align at joining node (within heading discretization)
3. Lattice planner returns `std::nullopt` cleanly when no path exists
4. `ISemanticSegmenter` interface: both `StubSemanticSegmenter` and `PluginSemanticSegmenter` implement it correctly
5. Plugin injection: custom lambda invoked and result returned correctly
6. Invalid input (empty image): returns `std::expected` error; no crash
7. DL backend integration guide documented in `docs/theory.md`
8. All unit tests pass, CI green
9. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

Any DL framework (ONNX Runtime, TensorRT, LibTorch, OpenVINO) — all deferred to a post-M23 DL backend milestone. Real semantic inference (all inference is stub), lattice planner in 3D C-space, clothoid motion primitives, GPU-accelerated A* on lattice graph.
