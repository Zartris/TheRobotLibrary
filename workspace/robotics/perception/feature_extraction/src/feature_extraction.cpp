#include <feature_extraction/feature_extraction.hpp>
#include <common/logging/logger.hpp>

#include <algorithm>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <vector>

namespace feature_extraction {

// ── FeatureExtractor::Impl ─────────────────────────────────────────────────

struct FeatureExtractor::Impl {
    FeatureExtractorConfig config;
    std::shared_ptr<common::ILogger> logger;

    explicit Impl(const FeatureExtractorConfig& cfg)
        : config{cfg}
        , logger{common::getLogger("feature_extraction")}
    {}
};

// ── FeatureExtractor ───────────────────────────────────────────────────────

FeatureExtractor::FeatureExtractor(const FeatureExtractorConfig& config)
    : m_impl{std::make_unique<Impl>(config)}
{}

FeatureExtractor::~FeatureExtractor() = default;

const FeatureExtractorConfig& FeatureExtractor::config() const noexcept {
    return m_impl->config;
}

std::expected<std::vector<Feature>, FeatureError>
FeatureExtractor::detectAndCompute(const GrayscaleImage& image) const {
    m_impl->logger->debug(
        std::format("detecting features width={} height={}", image.width, image.height));

    if (!image.valid()) {
        return std::unexpected(FeatureError{
            FeatureError::Code::InvalidImage,
            "GrayscaleImage is invalid: check width, height, and data size"
        });
    }

    // TODO: Implement FAST corner detection across scale pyramid
    //   1. For each octave, downsample the image by m_impl->config.scaleFactor
    //   2. Run FAST-9/12 accelerated segment test at m_impl->config.fastThreshold
    //   3. Apply non-maximum suppression within a 3x3 neighbourhood
    //   4. Retain top m_impl->config.maxFeatures by response score (Harris score)
    //   5. Compute dominant orientation via intensity centroid in a 31-px patch
    //      (only when m_impl->config.useOrientation == true)

    // TODO: Implement rBRIEF descriptor computation
    //   1. For each keypoint, rotate the 256 pre-defined sample-pair offsets
    //      by the keypoint's orientation angle
    //   2. For each rotated pair (p, q), compare image intensities:
    //      bit = (I(p) < I(q)) ? 1 : 0
    //   3. Pack 256 bits into a Descriptor (32 bytes)

    m_impl->logger->debug("detectAndCompute stub returning 0 features (FAST+BRIEF not yet implemented)");

    std::vector<Feature> features;
    return features;
}

std::expected<std::vector<Keypoint>, FeatureError>
FeatureExtractor::detect(const GrayscaleImage& image) const {
    m_impl->logger->debug(
        std::format("detecting keypoints only width={} height={}", image.width, image.height));

    if (!image.valid()) {
        return std::unexpected(FeatureError{
            FeatureError::Code::InvalidImage,
            "GrayscaleImage is invalid: check width, height, and data size"
        });
    }

    // TODO: Implement FAST corner detection (same pipeline as detectAndCompute
    //       but skip descriptor computation step).

    m_impl->logger->debug("detect stub returning 0 keypoints (FAST not yet implemented)");

    std::vector<Keypoint> keypoints;
    return keypoints;
}

// ── DescriptorMatcher::Impl ────────────────────────────────────────────────

struct DescriptorMatcher::Impl {
    MatcherConfig config;
    std::shared_ptr<common::ILogger> logger;

    explicit Impl(const MatcherConfig& cfg)
        : config{cfg}
        , logger{common::getLogger("feature_extraction")}
    {}
};

// ── DescriptorMatcher ──────────────────────────────────────────────────────

DescriptorMatcher::DescriptorMatcher(const MatcherConfig& config)
    : m_impl{std::make_unique<Impl>(config)}
{}

DescriptorMatcher::~DescriptorMatcher() = default;

const MatcherConfig& DescriptorMatcher::config() const noexcept {
    return m_impl->config;
}

// hammingDistance: XOR each of the 8 × 4-byte words and count set bits.
// __builtin_popcount operates on unsigned int (32-bit); accumulate over all 8 chunks.
int DescriptorMatcher::hammingDistance(const Descriptor& a, const Descriptor& b) noexcept {
    int dist = 0;
    // Interpret the 32-byte arrays as 8 × uint32_t for efficient popcount.
    for (int chunk = 0; chunk < 8; ++chunk) {
        const int base = chunk * 4;
        uint32_t wordA = 0, wordB = 0;
        // Load 4 bytes in little-endian order without breaking strict aliasing.
        for (int byte = 0; byte < 4; ++byte) {
            wordA |= static_cast<uint32_t>(a[static_cast<size_t>(base + byte)]) << (byte * 8);
            wordB |= static_cast<uint32_t>(b[static_cast<size_t>(base + byte)]) << (byte * 8);
        }
        dist += __builtin_popcount(wordA ^ wordB);
    }
    return dist;
}

std::expected<std::vector<Match>, FeatureError>
DescriptorMatcher::match(const std::vector<Feature>& query,
                          const std::vector<Feature>& train) const {
    m_impl->logger->debug(
        std::format("matching query={} train={}", query.size(), train.size()));

    if (query.empty() || train.empty()) {
        m_impl->logger->debug("match: one or both feature sets are empty, returning no matches");
        return std::vector<Match>{};
    }

    const auto& cfg = m_impl->config;

    // ── Forward pass: for each query find best (and second-best) in train ──

    // best_for_query[qi] = {distance, train_index}
    struct BestPair { int dist; int idx; };
    constexpr int INF = std::numeric_limits<int>::max();

    std::vector<BestPair> best_for_query(query.size(), {INF, -1});
    std::vector<BestPair> second_for_query(query.size(), {INF, -1});

    for (int qi = 0; qi < static_cast<int>(query.size()); ++qi) {
        for (int ti = 0; ti < static_cast<int>(train.size()); ++ti) {
            const int d = hammingDistance(query[static_cast<size_t>(qi)].descriptor,
                                          train[static_cast<size_t>(ti)].descriptor);
            m_impl->logger->trace(
                std::format("  hamming q={} t={} dist={}", qi, ti, d));

            if (d < best_for_query[static_cast<size_t>(qi)].dist) {
                second_for_query[static_cast<size_t>(qi)] =
                    best_for_query[static_cast<size_t>(qi)];
                best_for_query[static_cast<size_t>(qi)] = {d, ti};
            } else if (d < second_for_query[static_cast<size_t>(qi)].dist) {
                second_for_query[static_cast<size_t>(qi)] = {d, ti};
            }
        }
    }

    // ── Reverse pass for cross-check: for each train find best in query ──

    std::vector<int> best_query_for_train(train.size(), -1);
    if (cfg.crossCheck) {
        std::vector<int> best_dist_for_train(train.size(), INF);
        for (int qi = 0; qi < static_cast<int>(query.size()); ++qi) {
            const int ti  = best_for_query[static_cast<size_t>(qi)].idx;
            const int d   = best_for_query[static_cast<size_t>(qi)].dist;
            if (ti >= 0 && d < best_dist_for_train[static_cast<size_t>(ti)]) {
                best_dist_for_train[static_cast<size_t>(ti)] = d;
                best_query_for_train[static_cast<size_t>(ti)] = qi;
            }
        }
    }

    // ── Build match list applying all filters ──

    std::vector<Match> matches;
    matches.reserve(query.size());

    for (int qi = 0; qi < static_cast<int>(query.size()); ++qi) {
        const auto& best = best_for_query[static_cast<size_t>(qi)];
        if (best.idx < 0) continue;

        // Threshold filter
        if (static_cast<float>(best.dist) > cfg.maxHammingDistance) {
            m_impl->logger->debug(
                std::format("  q={} rejected by maxHammingDistance ({} > {})",
                            qi, best.dist, cfg.maxHammingDistance));
            continue;
        }

        // Lowe's ratio test (disabled when ratioThreshold == 0)
        if (cfg.ratioThreshold > 0.f) {
            const auto& second = second_for_query[static_cast<size_t>(qi)];
            if (second.dist != INF) {
                const float ratio =
                    static_cast<float>(best.dist) / static_cast<float>(second.dist);
                if (ratio >= cfg.ratioThreshold) {
                    m_impl->logger->debug(
                        std::format("  q={} rejected by ratio test ({:.3f} >= {:.3f})",
                                    qi, ratio, cfg.ratioThreshold));
                    continue;
                }
            }
        }

        // Cross-check filter
        if (cfg.crossCheck) {
            const int ti = best.idx;
            if (best_query_for_train[static_cast<size_t>(ti)] != qi) {
                m_impl->logger->debug(
                    std::format("  q={} rejected by cross-check", qi));
                continue;
            }
        }

        matches.push_back(Match{qi, best.idx, static_cast<float>(best.dist)});
    }

    // Sort by ascending Hamming distance (best match first).
    std::sort(matches.begin(), matches.end(),
              [](const Match& lhs, const Match& rhs) {
                  return lhs.distance < rhs.distance;
              });

    m_impl->logger->debug(std::format("match: {} matches retained", matches.size()));
    return matches;
}

} // namespace feature_extraction
