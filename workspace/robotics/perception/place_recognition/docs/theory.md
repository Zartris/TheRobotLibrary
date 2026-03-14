# Place Recognition Theory

## 1. Problem Statement

Given a current scene descriptor and a database of previously seen scene descriptors,
determine whether the robot has visited this place before and, if so, retrieve the
candidate frame IDs. This is *loop closure detection* at the descriptor level.

## 2. Place Descriptor

A `PlaceDescriptor` is a generic `std::vector<float>` encoding a compact, appearance-based
scene fingerprint. Two supported descriptor types:

- **Lidar scan histogram** — angular bins of range values aggregated into a fixed-length
  histogram (scan-context-style). Rotation-variant but fast to compute.
- **Visual feature aggregate** — aggregated BRIEF descriptor statistics (mean, histogram
  of bit patterns) from a keyframe. Compact, lighting-sensitive but workable indoors.

Both types use the same `PlaceDescriptor` type, enabling a single `PlaceRecognizer` to
serve either modality.

## 3. Similarity Metrics

**L2 distance:** $d(a, b) = \|a - b\|_2$. Lower is more similar. Simple, fast.

**Cosine similarity:** $s(a, b) = \frac{a \cdot b}{\|a\|\|b\|}$. Range $[-1, 1]$;
1 = identical direction. Suitable when descriptor magnitude varies.

A configurable similarity threshold filters false positives: only candidates with
$s \geq \tau$ (or $d \leq \tau$ for L2) are returned.

## 4. Nearest-Neighbour Search

At the scale targeted here (up to ~1000 frames), linear scan over all database entries
is sufficient and achieves < 10 ms query time. For larger databases, KD-tree or HNSW
indexing can replace the linear scan without changing the API.

## 5. False Positive Rate

Random descriptors should rarely trigger a match above threshold. The threshold $\tau$ is
tuned to achieve < 2% false positive rate on random query descriptors, verified by the
test suite with 100 random queries against a 10-frame database.

## 6. Integration with Pose Graph

When the `PlaceRecognizer` returns a `PlaceCandidate` pair $(i, j)$ with high confidence,
the caller verifies the geometric consistency (via `visual_odometry` or ICP) and adds
a loop closure edge $(i, j, T_{ij}, \Omega_{ij})$ to the `pose_graph` optimizer.
