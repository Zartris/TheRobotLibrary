#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/types.hpp>

using namespace robotlib;

TEST_CASE("Point2D construction and equality", "[common][types]") {
    Point2D p1{1.0, 2.0};
    Point2D p2{1.0, 2.0};
    Point2D p3{3.0, 4.0};

    REQUIRE(p1 == p2);
    REQUIRE_FALSE(p1 == p3);
}

TEST_CASE("Point2D distance", "[common][types]") {
    Point2D a{0.0, 0.0};
    Point2D b{3.0, 4.0};
    REQUIRE_THAT(a.distanceTo(b), Catch::Matchers::WithinRel(5.0, 1e-9));
}

TEST_CASE("Pose2D construction", "[common][types]") {
    Pose2D p{1.0, 2.0, 0.5};
    REQUIRE_THAT(p.x, Catch::Matchers::WithinRel(1.0, 1e-9));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinRel(2.0, 1e-9));
    REQUIRE_THAT(p.theta, Catch::Matchers::WithinRel(0.5, 1e-9));
    auto pt = p.position();
    REQUIRE(pt == Point2D{1.0, 2.0});
}

TEST_CASE("Twist construction and equality", "[common][types]") {
    Twist t1{1.0, 0.5};
    Twist t2{1.0, 0.5};
    REQUIRE(t1 == t2);
}

TEST_CASE("Path is a vector of Pose2D", "[common][types]") {
    Path path;
    path.push_back({0.0, 0.0, 0.0});
    path.push_back({1.0, 0.0, 0.0});
    REQUIRE(path.size() == 2);
}
