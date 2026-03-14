# object_detection_3d

3D object detector from point clouds. Runs 3D DBSCAN to find spatial clusters, fits a PCA-based oriented bounding box to each cluster, and applies a size-based classification stub (person / car / unknown).

**Milestone:** M19 — Depth Perception & 3D Understanding  
**Status:** Scaffold only — awaiting implementation

## Features

- `ObjectDetector3D` — full pipeline: cluster → box → classify
- 3D DBSCAN with configurable ε and min_points
- PCA oriented bounding box via `SelfAdjointEigenSolver` (eigenvectors = box axes)
- Classification: `PERSON` (height > 1.0 m, footprint < 0.5×0.5 m), `CAR` (footprint > 1.5×1.5 m)
- `Detection3DList` with centroid, oriented box dimensions, and class label

## Dependencies

- `common` (logging, types)
- Eigen3 (PCA via `SelfAdjointEigenSolver`)
