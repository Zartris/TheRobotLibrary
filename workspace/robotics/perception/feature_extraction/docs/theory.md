# Feature Extraction — Theory

This document covers the algorithms behind the `feature_extraction` module:
FAST corner detection, scale pyramids, BRIEF descriptors, ORB (oFAST + rBRIEF),
Hamming-distance matching, Lowe's ratio test, and the practical rationale for
choosing binary descriptors over floating-point alternatives.

---

## 1. FAST Corner Detector

**FAST** (Features from Accelerated Segment Test) detects corners by examining
a Bresenham circle of 16 pixels at radius 3 around a candidate pixel `p`.

### Accelerated Segment Test

A pixel `p` is classified as a corner if there exists a contiguous arc of `n`
pixels on the circle that are **all** brighter than `I(p) + t` or **all** darker
than `I(p) - t`, where `t` is the threshold parameter.

For FAST-9 (n = 9), the test is accelerated using a four-pixel pre-rejection:
pixels 1, 5, 9, and 13 (the four cardinal points) are tested first.  At least
three of these four must satisfy the brightness condition before the full 16-pixel
test runs; otherwise the candidate is trivially rejected.  This reduces the
average per-pixel cost to roughly one comparison in non-corner regions.

### Threshold Parameter

A high threshold detects only strong, unambiguous corners.  A low threshold is
sensitive but produces more false positives.  The module exposes
`FeatureExtractorConfig::fastThreshold` (default: 20).

### Non-Maximum Suppression

Raw FAST often produces clusters of corner responses around a single salient
point.  Non-maximum suppression (NMS) retains only the local maximum in a 3×3
neighbourhood, measured by a corner response score.  The **Harris score** is
preferred over the raw segment-count score because it is more stable under small
image perturbations:

```
H = det(M) - k * trace(M)^2,  k ≈ 0.04
```

where `M` is the 2×2 structure tensor computed over a patch around the keypoint.

---

## 2. Scale Pyramids

A single-scale detector misses features that appear at different resolutions.
An **image pyramid** solves this by detecting features on `nOctaves` downsampled
versions of the image, each scaled by `1 / scaleFactor` relative to the previous.

For each octave `o`:
- Image is smoothed with a Gaussian filter (σ proportional to scale level)
- FAST is run on the smoothed image
- Detected keypoints are scaled back to original-image coordinates by multiplying
  by `scaleFactor^o`
- The octave index is stored in `Keypoint::octave` for downstream consumers

This provides **scale invariance**: a feature visible only at a coarser scale
is still detected and normalised to a consistent representation.

---

## 3. Dominant Orientation (Intensity Centroid)

To achieve rotation invariance, each keypoint is assigned a **dominant orientation**
computed via the intensity centroid method (Rosin 1999, adopted for ORB):

```
m_{p,q} = sum_{x,y in patch} x^p * y^q * I(x, y)

angle = atan2(m_{0,1}, m_{1,0})
```

The summation is performed over a circular patch of radius 15 pixels centred on
the keypoint.  The resulting angle is stored in `Keypoint::angle` and used to
rotate the BRIEF sampling pattern, yielding **rotation-invariant descriptors**.

---

## 4. BRIEF Descriptor

**BRIEF** (Binary Robust Independent Elementary Features) encodes a patch as a
256-bit binary string derived from intensity comparisons of pixel pairs:

```
tau(p; x, y) = 1  if I(p + x) < I(p + y)
               0  otherwise
```

256 such tests are performed using a pre-determined set of (x, y) sample-pair
offsets.  The offsets are sampled from a Gaussian distribution centred on the
keypoint and empirically selected to maximise variance while minimising
inter-descriptor correlation (Calonder et al. 2010).

The resulting descriptor is 256 bits = 32 bytes, stored as `std::array<uint8_t, 32>`.

### Why Binary Descriptors?

| Property           | Binary (BRIEF/ORB) | Float (SIFT/SURF) |
|--------------------|-------------------|-------------------|
| Descriptor size    | 32 bytes           | 512 bytes (SIFT)  |
| Matching speed     | O(N·M/64) popcount | O(N·M) FP mult    |
| Patent status      | Free               | SIFT/SURF patented (expired but historically restricted) |
| Accuracy           | Slightly lower     | Higher            |
| Suitable for       | Real-time SLAM     | Offline structure-from-motion |

For embedded robotics applications, BRIEF/ORB descriptors are 16× smaller and
can be matched using the hardware `POPCNT` instruction, making them orders of
magnitude faster than SIFT/SURF at similar recall rates for camera motion
estimation tasks.

---

## 5. ORB = oFAST + rBRIEF

**ORB** (Oriented FAST and Rotated BRIEF, Rublee et al. 2011) combines:

1. **oFAST** — FAST detector with Harris-score NMS and intensity-centroid orientation
2. **rBRIEF** — BRIEF with sample pairs rotated by the keypoint's dominant orientation

### rBRIEF Rotation

The sampling pattern is defined in a lookup table of 256 pre-learnt pair offsets
`{(x_i, y_i), (x_i', y_i')}`.  For a keypoint with orientation `θ`, each offset
is rotated:

```
x_rot =  cos(θ) * x - sin(θ) * y
y_rot =  sin(θ) * x + cos(θ) * y
```

The rotated offsets are rounded to integer pixel positions and the comparisons
proceed as standard BRIEF.

### Steered BRIEF vs. Learned BRIEF

Standard steered BRIEF suffers from high descriptor correlation under rotation.
ORB uses a **greedy search** offline to select 256 pairs from a pool of 300,000
candidates, maximising variance and minimising correlation between bits.  This
pre-learnt table is embedded as a compile-time constant array.

---

## 6. Hamming Distance

Binary descriptors are compared using **Hamming distance** — the number of
bit positions where the two descriptors differ:

```
hamming(a, b) = popcount(a XOR b)
```

For 32-byte descriptors this is computed as 8 iterations of `popcount` over
32-bit XOR words, exploiting the x86 `POPCNT` instruction via
`__builtin_popcount`.  The maximum possible distance is 256 (all bits differ);
a distance of 0 means identical descriptors.

---

## 7. Brute-Force Matching

The module provides a brute-force matcher that computes the Hamming distance
between every (query, train) pair in O(N · M) time.  For N = M = 500 features,
this is 250,000 distance computations, each taking roughly 8 `popcount`
operations — well within real-time budget at typical frame rates.

### Cross-Check Filter

With `MatcherConfig::crossCheck = true`, a match (q, t) is accepted only if:
- t is the best match for q **and**
- q is the best match for t

This eliminates one-to-many matches and significantly reduces false positives
without requiring a ratio test.

### Lowe's Ratio Test

Proposed by Lowe (2004) for SIFT but equally applicable to binary descriptors:
a match is accepted only if

```
d_best / d_second_best < ratioThreshold
```

A ratio threshold of 0.75 is the standard default.  A low ratio (≪ 1) means
the best match is much better than the second-best, indicating an unambiguous
correspondence.  Set `ratioThreshold = 0` to disable.

---

## 8. References

1. Rublee, E., Rabaud, V., Konolige, K., & Bradski, G. (2011). **ORB: An efficient
   alternative to SIFT or SURF.** ICCV 2011.
   https://doi.org/10.1109/ICCV.2011.6126544

2. Calonder, M., Lepetit, V., Strecha, C., & Fua, P. (2010). **BRIEF: Binary
   Robust Independent Elementary Features.** ECCV 2010.
   https://doi.org/10.1007/978-3-642-15561-1_56

3. Rosten, E., & Drummond, T. (2006). **Machine learning for high-speed corner
   detection.** ECCV 2006.
   https://doi.org/10.1007/11744023_34

4. Lowe, D. G. (2004). **Distinctive image features from scale-invariant keypoints.**
   IJCV 60(2), 91–110.
   https://doi.org/10.1023/B:VISI.0000029664.99615.94

5. Harris, C., & Stephens, M. (1988). **A combined corner and edge detector.**
   Proceedings of the 4th Alvey Vision Conference.
