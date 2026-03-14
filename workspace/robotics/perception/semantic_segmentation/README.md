# semantic_segmentation

Pixel-level scene classification with a stub implementation and explicit DL plugin interface. Ships `ISemanticSegmenter`, `StubSemanticSegmenter` (all-ROAD output), and `PluginSemanticSegmenter` (wraps a `std::function` for future ONNX/TensorRT backend injection).

**Milestone:** M23 — Lattice Planning & Semantic Vision  
**Status:** Scaffold only — awaiting implementation  
**Note:** Stub-only in M23. No DL inference. See `docs/theory.md` for the DL Backend Integration guide.

## Features

- `ISemanticSegmenter` — interface: `std::expected<SemanticMap, std::string> segment(const RgbImage&)`
- `StubSemanticSegmenter` — returns all-`ROAD` map; validates input dimensions
- `PluginSemanticSegmenter` — wraps a `SegmenterPlugin = std::function<...>` for runtime DL backend injection without recompilation
- `SemanticClass` enum: `ROAD, SIDEWALK, OBSTACLE, PERSON, VEHICLE, BUILDING, SKY, UNKNOWN`

## DL Backend Integration

See `docs/theory.md` section "DL Backend Integration" for how to implement a `SegmenterPlugin` wrapping ONNX Runtime.

## Dependencies

- `common` (logging, `RgbImage` from `common/camera.hpp`)
- Eigen3 (SemanticMap grid)
