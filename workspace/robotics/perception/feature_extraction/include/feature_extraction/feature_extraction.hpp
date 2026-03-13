#pragma once
#include <array>
#include <cstdint>
#include <expected>
#include <string>
#include <vector>

namespace feature_extraction {

// ── Types ─────────────────────────────────────────────────────────────────

/// A 2D image keypoint detected by FAST corner detector.
struct Keypoint {
    float x{0.f};         ///< Column (pixel)
    float y{0.f};         ///< Row (pixel)
    float response{0.f};  ///< Corner response score (higher = more distinctive)
    float angle{0.f};     ///< Dominant orientation in radians (for rotation-invariant descriptors)
    int   octave{0};      ///< Scale pyramid level (0 = original resolution)
};

/// 256-bit binary descriptor (ORB/BRIEF style) stored as 32 bytes.
using Descriptor = std::array<uint8_t, 32>;

/// Keypoint paired with its descriptor.
struct Feature {
    Keypoint   keypoint;
    Descriptor descriptor{};
};

/// A descriptor match between two feature sets.
struct Match {
    int    queryIdx{-1};    ///< Index into query feature list
    int    trainIdx{-1};    ///< Index into train feature list
    float  distance{0.f};   ///< Hamming distance (lower = better)
};

/// Error type for feature extraction operations.
struct FeatureError {
    enum class Code { InvalidImage, NoFeaturesFound, MatcherError };
    Code        code;
    std::string message;
};

// ── FeatureExtractorConfig ─────────────────────────────────────────────────

struct FeatureExtractorConfig {
    int   fastThreshold{20};      ///< FAST corner detection threshold
    int   maxFeatures{500};       ///< Maximum keypoints to retain per image
    int   nOctaves{4};            ///< Scale pyramid levels
    float scaleFactor{1.2f};      ///< Scale between pyramid levels
    bool  useOrientation{true};   ///< Compute dominant orientation for rotation invariance
};

// ── SimpleImage ────────────────────────────────────────────────────────────

/// Minimal grayscale image wrapper (row-major, 8-bit unsigned).
struct GrayscaleImage {
    int                  width{0};
    int                  height{0};
    std::vector<uint8_t> data;   ///< Row-major pixel data, size = width * height

    bool valid() const noexcept {
        return width > 0 && height > 0 &&
               static_cast<int>(data.size()) == width * height;
    }
    uint8_t at(int row, int col) const noexcept {
        return data[static_cast<size_t>(row * width + col)];
    }
};

// ── FeatureExtractor ───────────────────────────────────────────────────────

/// ORB-style feature extractor: FAST keypoints + BRIEF descriptors.
/// Pure CPU implementation, no OpenCV dependency.
class FeatureExtractor {
public:
    explicit FeatureExtractor(const FeatureExtractorConfig& config = {});
    ~FeatureExtractor();

    /// Detect keypoints and compute descriptors from a grayscale image.
    std::expected<std::vector<Feature>, FeatureError>
    detectAndCompute(const GrayscaleImage& image) const;

    /// Detect keypoints only (no descriptors).
    std::expected<std::vector<Keypoint>, FeatureError>
    detect(const GrayscaleImage& image) const;

    const FeatureExtractorConfig& config() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// ── DescriptorMatcher ──────────────────────────────────────────────────────

struct MatcherConfig {
    float maxHammingDistance{64.f};  ///< Reject matches with Hamming distance above this
    bool  crossCheck{true};          ///< Only return mutually best matches
    float ratioThreshold{0.75f};     ///< Lowe's ratio test threshold (0 = disabled)
};

/// Brute-force Hamming-distance matcher for binary descriptors.
class DescriptorMatcher {
public:
    explicit DescriptorMatcher(const MatcherConfig& config = {});
    ~DescriptorMatcher();

    /// Match query features against train features.
    /// Returns list of matches sorted by distance (best first).
    std::expected<std::vector<Match>, FeatureError>
    match(const std::vector<Feature>& query,
          const std::vector<Feature>& train) const;

    /// Compute Hamming distance between two 256-bit descriptors.
    static int hammingDistance(const Descriptor& a, const Descriptor& b) noexcept;

    const MatcherConfig& config() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace feature_extraction
