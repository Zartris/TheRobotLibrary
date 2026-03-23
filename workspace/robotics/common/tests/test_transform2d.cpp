#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/transform2d.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("Transform2D identity", "[common][transform2d]") {
    auto t = Transform2D::identity();
    REQUIRE_THAT(t.pose().x, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(t.pose().y, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(t.pose().theta, Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("Transform2D compose", "[common][transform2d]") {
    Transform2D t1(1.0, 0.0, std::numbers::pi / 2);
    Transform2D t2(1.0, 0.0, 0.0);
    auto result = t1.compose(t2);
    REQUIRE_THAT(result.pose().x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.pose().y, Catch::Matchers::WithinAbs(1.0, 1e-6));
}

TEST_CASE("Transform2D inverse", "[common][transform2d]") {
    Transform2D t(1.0, 2.0, 0.5);
    auto inv = t.inverse();
    auto identity = t.compose(inv);
    REQUIRE_THAT(identity.pose().x, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(identity.pose().y, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(identity.pose().theta, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Transform2D transform point", "[common][transform2d]") {
    Transform2D t(1.0, 2.0, std::numbers::pi / 2);
    Point2D p{1.0, 0.0};
    auto result = t.transformPoint(p);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(3.0, 1e-6));
}
