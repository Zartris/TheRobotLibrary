#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/geometry.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("normalizeAngle wraps pi", "[common][geometry]") {
    REQUIRE_THAT(normalizeAngle(std::numbers::pi),
                 Catch::Matchers::WithinAbs(std::numbers::pi, 1e-9));
}

TEST_CASE("normalizeAngle wraps -pi", "[common][geometry]") {
    auto result = normalizeAngle(-std::numbers::pi);
    // -pi wraps to pi
    REQUIRE_THAT(std::abs(result), Catch::Matchers::WithinAbs(std::numbers::pi, 1e-9));
}

TEST_CASE("normalizeAngle wraps >2pi", "[common][geometry]") {
    REQUIRE_THAT(normalizeAngle(3 * std::numbers::pi),
                 Catch::Matchers::WithinAbs(std::numbers::pi, 1e-9));
}

TEST_CASE("angleWrap is normalizeAngle alias", "[common][geometry]") {
    REQUIRE_THAT(angleWrap(4.0), Catch::Matchers::WithinAbs(normalizeAngle(4.0), 1e-9));
}

TEST_CASE("distance between points", "[common][geometry]") {
    REQUIRE_THAT(distance(Point2D{0, 0}, Point2D{3, 4}),
                 Catch::Matchers::WithinRel(5.0, 1e-9));
}

TEST_CASE("distance from coordinates", "[common][geometry]") {
    REQUIRE_THAT(distance(0.0, 0.0, 3.0, 4.0),
                 Catch::Matchers::WithinRel(5.0, 1e-9));
}

TEST_CASE("lerp basic", "[common][geometry]") {
    REQUIRE_THAT(lerp(0.0, 10.0, 0.5), Catch::Matchers::WithinRel(5.0, 1e-9));
    REQUIRE_THAT(lerp(0.0, 10.0, 0.0), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(lerp(0.0, 10.0, 1.0), Catch::Matchers::WithinRel(10.0, 1e-9));
}
