#include <catch2/catch_test_macros.hpp>
#include <feature_extraction/feature_extraction.hpp>

using namespace feature_extraction;

// ── Helper utilities ───────────────────────────────────────────────────────

/// Build a small valid grayscale image filled with a constant value.
static GrayscaleImage makeSolidImage(int width, int height, uint8_t fill = 128) {
    GrayscaleImage img;
    img.width  = width;
    img.height = height;
    img.data.assign(static_cast<size_t>(width * height), fill);
    return img;
}

/// Build a descriptor with all bytes set to a given value.
static Descriptor makeDescriptor(uint8_t fill) {
    Descriptor d{};
    d.fill(fill);
    return d;
}

// ── FeatureExtractor tests ─────────────────────────────────────────────────

TEST_CASE("FeatureExtractor basic construction", "[feature_extraction]") {
    // Default construction must not throw and must expose the default config.
    FeatureExtractor extractor;
    CHECK(extractor.config().fastThreshold == 20);
    CHECK(extractor.config().maxFeatures   == 500);
    CHECK(extractor.config().nOctaves      == 4);
    CHECK(extractor.config().scaleFactor   == 1.2f);
    CHECK(extractor.config().useOrientation == true);
}

TEST_CASE("FeatureExtractor construction with custom config", "[feature_extraction]") {
    FeatureExtractorConfig cfg;
    cfg.fastThreshold  = 10;
    cfg.maxFeatures    = 200;
    cfg.nOctaves       = 2;
    cfg.scaleFactor    = 1.5f;
    cfg.useOrientation = false;

    FeatureExtractor extractor{cfg};
    CHECK(extractor.config().fastThreshold  == 10);
    CHECK(extractor.config().maxFeatures    == 200);
    CHECK(extractor.config().nOctaves       == 2);
    CHECK(extractor.config().scaleFactor    == 1.5f);
    CHECK(extractor.config().useOrientation == false);
}

TEST_CASE("FeatureExtractor detect on invalid image — empty", "[feature_extraction]") {
    FeatureExtractor extractor;
    GrayscaleImage empty;  // width=0, height=0, data empty

    auto result = extractor.detectAndCompute(empty);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == FeatureError::Code::InvalidImage);
}

TEST_CASE("FeatureExtractor detect on invalid image — mismatched data size", "[feature_extraction]") {
    FeatureExtractor extractor;
    GrayscaleImage bad;
    bad.width  = 10;
    bad.height = 10;
    bad.data.assign(5, 0);  // deliberately wrong size

    auto result = extractor.detectAndCompute(bad);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == FeatureError::Code::InvalidImage);
}

TEST_CASE("FeatureExtractor detect (keypoints only) on invalid image", "[feature_extraction]") {
    FeatureExtractor extractor;
    GrayscaleImage empty;

    auto result = extractor.detect(empty);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == FeatureError::Code::InvalidImage);
}

TEST_CASE("FeatureExtractor detectAndCompute on valid image returns success", "[feature_extraction]") {
    FeatureExtractor extractor;
    auto img = makeSolidImage(64, 64);

    auto result = extractor.detectAndCompute(img);
    // Stub returns an empty feature list — important: it must NOT return an error.
    REQUIRE(result.has_value());
    // Stub returns empty; the real implementation will populate this.
    CHECK(result->size() == 0);
}

TEST_CASE("FeatureExtractor detect on valid image returns success", "[feature_extraction]") {
    FeatureExtractor extractor;
    auto img = makeSolidImage(32, 32);

    auto result = extractor.detect(img);
    REQUIRE(result.has_value());
    CHECK(result->size() == 0);  // stub
}

// ── DescriptorMatcher::hammingDistance tests ───────────────────────────────

TEST_CASE("DescriptorMatcher hammingDistance identical descriptors = 0", "[feature_extraction]") {
    Descriptor a = makeDescriptor(0xAB);
    Descriptor b = makeDescriptor(0xAB);
    CHECK(DescriptorMatcher::hammingDistance(a, b) == 0);
}

TEST_CASE("DescriptorMatcher hammingDistance all bits flipped = 256", "[feature_extraction]") {
    Descriptor a = makeDescriptor(0x00);
    Descriptor b = makeDescriptor(0xFF);
    CHECK(DescriptorMatcher::hammingDistance(a, b) == 256);
}

TEST_CASE("DescriptorMatcher hammingDistance single bit difference = 1", "[feature_extraction]") {
    Descriptor a{};
    Descriptor b{};
    a.fill(0x00);
    b.fill(0x00);
    b[0] = 0x01;  // flip exactly one bit in the first byte
    CHECK(DescriptorMatcher::hammingDistance(a, b) == 1);
}

TEST_CASE("DescriptorMatcher hammingDistance known value", "[feature_extraction]") {
    // 0xF0 XOR 0x0F = 0xFF => 8 bits set; if all 32 bytes differ this way => 256.
    // Use only first byte to get a known count of 8.
    Descriptor a{};
    Descriptor b{};
    a.fill(0x00);
    b.fill(0x00);
    a[0] = 0xF0;
    b[0] = 0x0F;  // XOR = 0xFF, popcount = 8
    CHECK(DescriptorMatcher::hammingDistance(a, b) == 8);
}

// ── DescriptorMatcher match tests ─────────────────────────────────────────

TEST_CASE("DescriptorMatcher match empty inputs returns empty list", "[feature_extraction]") {
    DescriptorMatcher matcher;
    std::vector<Feature> empty;

    auto result = matcher.match(empty, empty);
    REQUIRE(result.has_value());
    CHECK(result->empty());
}

TEST_CASE("DescriptorMatcher match empty query returns empty list", "[feature_extraction]") {
    DescriptorMatcher matcher;
    std::vector<Feature> empty;
    std::vector<Feature> train(3);

    auto result = matcher.match(empty, train);
    REQUIRE(result.has_value());
    CHECK(result->empty());
}

TEST_CASE("DescriptorMatcher match empty train returns empty list", "[feature_extraction]") {
    DescriptorMatcher matcher;
    std::vector<Feature> query(2);
    std::vector<Feature> empty;

    auto result = matcher.match(query, empty);
    REQUIRE(result.has_value());
    CHECK(result->empty());
}

TEST_CASE("DescriptorMatcher match identical features returns matches", "[feature_extraction]") {
    // Disable cross-check and ratio test so a perfect 1-to-1 match is straightforward.
    MatcherConfig cfg;
    cfg.crossCheck       = false;
    cfg.ratioThreshold   = 0.f;
    cfg.maxHammingDistance = 256.f;

    DescriptorMatcher matcher{cfg};

    Feature f;
    f.descriptor = makeDescriptor(0x55);

    std::vector<Feature> query = {f};
    std::vector<Feature> train = {f};

    auto result = matcher.match(query, train);
    REQUIRE(result.has_value());
    REQUIRE(result->size() == 1);
    CHECK(result->at(0).queryIdx == 0);
    CHECK(result->at(0).trainIdx == 0);
    CHECK(result->at(0).distance == 0.f);
}

TEST_CASE("DescriptorMatcher crossCheck filters asymmetric matches", "[feature_extraction]") {
    // Set up two query features and two train features where:
    //   query[0] is closest to train[0] (distance 0)
    //   query[1] is closest to train[0] as well (distance 4)
    //   train[0] is closest to query[0] (distance 0)
    //   train[1] has no close query
    //
    // With crossCheck ON:
    //   query[0] <-> train[0] is mutual => accepted
    //   query[1] -> train[0] but train[0] -> query[0], not query[1] => rejected

    MatcherConfig cfg;
    cfg.crossCheck         = true;
    cfg.ratioThreshold     = 0.f;
    cfg.maxHammingDistance = 256.f;

    DescriptorMatcher matcher{cfg};

    Feature q0, q1, t0, t1;
    q0.descriptor = makeDescriptor(0x00);  // all zeros
    t0.descriptor = makeDescriptor(0x00);  // identical to q0  => distance 0
    q1.descriptor = makeDescriptor(0x0F);  // distance to t0 = 4 (four 1-bits per byte? no: 0x0F = 4 set bits, only byte 0)
    // Make q1 closer to t0 than to t1 to ensure q1's best match is t0.
    t1.descriptor = makeDescriptor(0xFF);  // distance from q1 = 252 (32*8 - 4 = 252)

    std::vector<Feature> query = {q0, q1};
    std::vector<Feature> train = {t0, t1};

    auto result = matcher.match(query, train);
    REQUIRE(result.has_value());

    // Only q0 <-> t0 should survive cross-check.
    REQUIRE(result->size() == 1);
    CHECK(result->at(0).queryIdx == 0);
    CHECK(result->at(0).trainIdx == 0);
}

TEST_CASE("DescriptorMatcher results sorted by distance ascending", "[feature_extraction]") {
    MatcherConfig cfg;
    cfg.crossCheck         = false;
    cfg.ratioThreshold     = 0.f;
    cfg.maxHammingDistance = 256.f;

    DescriptorMatcher matcher{cfg};

    // Three query features of varying similarity to the single train feature.
    Feature t;
    t.descriptor = makeDescriptor(0x00);

    Feature q0, q1, q2;
    q0.descriptor = makeDescriptor(0x00);  // dist = 0
    q1.descriptor = makeDescriptor(0xFF);  // dist = 256
    q2.descriptor = makeDescriptor(0x0F);  // dist = 32*4 = 128? No: 0x0F has 4 bits, so 32 bytes * 4 = 128

    std::vector<Feature> query = {q1, q2, q0};  // deliberately unsorted
    std::vector<Feature> train = {t};

    auto result = matcher.match(query, train);
    REQUIRE(result.has_value());
    REQUIRE(result->size() == 3);

    // Verify ascending distance order.
    for (size_t i = 1; i < result->size(); ++i) {
        CHECK((*result)[i - 1].distance <= (*result)[i].distance);
    }
}
