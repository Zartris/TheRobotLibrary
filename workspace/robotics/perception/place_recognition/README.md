# place_recognition

Descriptor-based place recognition module. Maintains an indexed `PlaceDatabase` of
`PlaceDescriptor` vectors (generic `std::vector<float>`, supporting both lidar scan
histograms and aggregated visual feature descriptors). Query returns `PlaceCandidate`
matches ranked by similarity score.

## Components

- **`PlaceRecognizer`** — query engine with L2 / cosine similarity search
- **`PlaceDatabase`** — frame-ID-indexed descriptor store
- **`PlaceTypes`** — `PlaceDescriptor`, `PlaceCandidate`

## Dependencies

- `common` (logging)
- `Eigen3`

## Usage

```cpp
#include <place_recognition/place_recognizer.hpp>
#include <place_recognition/place_database.hpp>

PlaceRecognizer recognizer(threshold, common::getLogger("place_recognition"));
recognizer.addFrame(frame_id, descriptor);  // build database

auto candidates = recognizer.query(query_descriptor);
// candidates[0].frame_id, candidates[0].score
```

## Integration

Output `PlaceCandidate` list can be routed to `pose_graph` (M14) as loop closure edges.
Works identically for lidar scan descriptors and visual feature aggregates — same
`PlaceDescriptor` type handles both.

## Milestone

Part of **M17 — Camera Perception II**.  
See `repo-plans/modules/place_recognition.md` for full task checklist.
