# feature_extraction

ORB-style keypoint detection and binary descriptor extraction for TheRobotLibrary.
Provides FAST corner detection, BRIEF descriptors, and a brute-force Hamming-distance
matcher — **no OpenCV dependency**.

Intended use cases: visual odometry front-end, visual SLAM map initialisation,
place recognition, and loop closure detection.

---

## API Summary

### Core types

| Type | Description |
|------|-------------|
| `GrayscaleImage` | Minimal 8-bit row-major image wrapper |
| `Keypoint` | 2D pixel location, response, orientation, octave |
| `Descriptor` | 256-bit binary descriptor (`std::array<uint8_t, 32>`) |
| `Feature` | Keypoint + Descriptor pair |
| `Match` | query/train index pair + Hamming distance |
| `FeatureError` | Error result with `Code` enum + message string |

### FeatureExtractor

```cpp
FeatureExtractorConfig cfg;
cfg.fastThreshold  = 20;   // FAST corner threshold
cfg.maxFeatures    = 500;  // max retained keypoints
cfg.nOctaves       = 4;    // scale pyramid depth
cfg.scaleFactor    = 1.2f; // scale between pyramid levels
cfg.useOrientation = true; // compute dominant orientation

feature_extraction::FeatureExtractor extractor{cfg};

// Detect keypoints AND compute descriptors
auto features = extractor.detectAndCompute(image);

// Detect keypoints only (cheaper — skip descriptor computation)
auto keypoints = extractor.detect(image);
```

### DescriptorMatcher

```cpp
feature_extraction::MatcherConfig mcfg;
mcfg.maxHammingDistance = 64.f;  // reject matches above this distance
mcfg.crossCheck         = true;  // mutual best-match filter
mcfg.ratioThreshold     = 0.75f; // Lowe's ratio test (0 = disabled)

feature_extraction::DescriptorMatcher matcher{mcfg};

auto matches = matcher.match(queryFeatures, trainFeatures);
// matches are sorted by ascending Hamming distance (best first)
```

### Hamming distance utility

```cpp
auto d = feature_extraction::DescriptorMatcher::hammingDistance(descA, descB);
// 0 = identical, 256 = all bits differ
```

---

## CMake Integration

Add `feature_extraction` as a subdirectory or integrate it through the workspace
aggregator.  The module requires the `common` target (provided by the workspace).

```cmake
# In your module or application CMakeLists.txt:
target_link_libraries(my_target PRIVATE robotlib::feature_extraction)
```

To build as a standalone module (for rapid iteration):

```bash
cmake -B build -S workspace/robotics/perception/feature_extraction
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

---

## Full Pipeline Example

```cpp
#include <feature_extraction/feature_extraction.hpp>

// --- Populate two images (replace with real data) ---
feature_extraction::GrayscaleImage imgA = loadGrayscale("frame_000.pgm");
feature_extraction::GrayscaleImage imgB = loadGrayscale("frame_001.pgm");

// --- Extract features from both frames ---
feature_extraction::FeatureExtractor extractor;

auto resultA = extractor.detectAndCompute(imgA);
auto resultB = extractor.detectAndCompute(imgB);

if (!resultA || !resultB) {
    // handle error: resultA.error().message
    return;
}

// --- Match descriptors ---
feature_extraction::MatcherConfig mcfg;
mcfg.crossCheck = true;
feature_extraction::DescriptorMatcher matcher{mcfg};

auto matches = matcher.match(*resultA, *resultB);
if (!matches) {
    // handle error
    return;
}

for (const auto& m : *matches) {
    const auto& kpA = (*resultA)[m.queryIdx].keypoint;
    const auto& kpB = (*resultB)[m.trainIdx].keypoint;
    // Use (kpA.x, kpA.y) <-> (kpB.x, kpB.y) correspondences
    // for e.g. essential matrix estimation via RANSAC
}
```

---

## Implementation Status

| Component | Status |
|-----------|--------|
| `GrayscaleImage` wrapper | Complete |
| `FeatureExtractorConfig` / `MatcherConfig` | Complete |
| `DescriptorMatcher::hammingDistance` | Complete |
| `DescriptorMatcher::match` (brute-force) | Complete |
| `FeatureExtractor::detect` (FAST) | Stub — TODO |
| `FeatureExtractor::detectAndCompute` (FAST + rBRIEF) | Stub — TODO |

See [docs/theory.md](docs/theory.md) for the full algorithmic background.
