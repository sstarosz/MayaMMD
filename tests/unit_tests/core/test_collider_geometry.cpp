/*
 * SPDX-License-Identifier: MIT
 *
 * test_collider_geometry.cpp
 *
 * Unit tests for the Maya-free collider ray-cast math in
 * mmd/core/collider_geometry.hpp — the primitive derivation from the PMX
 * shape_size (full size -> engine radius/extents/length) and the
 * ray-vs-sphere / ray-vs-box / ray-vs-capsule hit tests the viewport draw
 * override's userSelect() uses for per-body picking.
 *
 * The math is pure (no Maya SDK, no Bullet), so it runs in the plain C++
 * mmd_core_tests target.
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "collider_geometry.hpp"
#include "physics_math.hpp"

#include <cmath>

using Catch::Approx;
using namespace mmd::core::collider_geometry;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::Simulation;

namespace
{

// A body centered at the origin with a unit quaternion (identity rotation).
RayBody centeredBody(Simulation::ColliderType type, const Double3& shapeSize)
{
    RayBody b;
    b.pos = Double3(0.0, 0.0, 0.0);
    b.quat = Double4(0.0, 0.0, 0.0, 1.0);
    b.colliderType = type;
    b.shapeSize = shapeSize;
    return b;
}

} // namespace

// ---------------------------------------------------------------------------
// Primitive derivation (PMX shape_size verbatim -> engine primitives)
// ---------------------------------------------------------------------------

TEST_CASE("Sphere primitive derives radius from shape_size[0]", "[collider_geometry]")
{
    const PrimitiveParams p = primitiveFromShapeSize(Simulation::ColliderType::eSphere,
                                                     Double3(0.8, 9.0, 9.0));
    CHECK(p.radius == Approx(0.8));
}

TEST_CASE("Box primitive halves the full shape_size into half extents", "[collider_geometry]")
{
    const PrimitiveParams p = primitiveFromShapeSize(Simulation::ColliderType::eBox,
                                                     Double3(2.0, 4.0, 6.0));
    CHECK(p.halfExtents.x == Approx(1.0));
    CHECK(p.halfExtents.y == Approx(2.0));
    CHECK(p.halfExtents.z == Approx(3.0));
}

TEST_CASE("Capsule primitive uses shape_size[0] radius and shape_size[1] length",
          "[collider_geometry]")
{
    const PrimitiveParams p = primitiveFromShapeSize(Simulation::ColliderType::eCapsule,
                                                     Double3(0.5, 4.0, 9.0));
    CHECK(p.radius == Approx(0.5));
    CHECK(p.length == Approx(4.0));
}

// ---------------------------------------------------------------------------
// Ray / sphere
// ---------------------------------------------------------------------------

TEST_CASE("Ray through the sphere center hits at t = radius", "[collider_geometry]")
{
    double t = -1.0;
    // Ray from (-5, 0, 0) toward +X hits a unit sphere at origin at t = 4.
    REQUIRE(raySphere(Double3(-5.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0), 1.0, t));
    CHECK(t == Approx(4.0));
}

TEST_CASE("Ray starting inside the sphere hits the far surface", "[collider_geometry]")
{
    double t = -1.0;
    REQUIRE(raySphere(Double3(0.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0), 1.0, t));
    CHECK(t == Approx(1.0));
}

TEST_CASE("Ray missing the sphere does not hit", "[collider_geometry]")
{
    double t = -1.0;
    // Passes 2 units off-center; sphere radius 1.
    CHECK_FALSE(raySphere(Double3(-5.0, 2.0, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0), 1.0, t));
}

// ---------------------------------------------------------------------------
// Ray / box (OBB — quaternion orientation)
// ---------------------------------------------------------------------------

TEST_CASE("Ray through the box center hits at the near face", "[collider_geometry]")
{
    double t = -1.0;
    // Axis-aligned box, half extents (1, 1, 1), ray along +X from (-5, 0, 0).
    REQUIRE(rayBox(Double3(-5.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0),
                   Double4(0, 0, 0, 1), Double3(1, 1, 1), t));
    CHECK(t == Approx(4.0));
}

TEST_CASE("Ray missing the box does not hit", "[collider_geometry]")
{
    double t = -1.0;
    CHECK_FALSE(rayBox(Double3(-5.0, 2.5, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0),
                       Double4(0, 0, 0, 1), Double3(1, 1, 1), t));
}

TEST_CASE("Ray hits a 90-degree-rotated box on its rotated face", "[collider_geometry]")
{
    // Box half extents (2, 1, 1), rotated 90° about Z: its 2-unit extent now
    // points along +Y.  A ray along +Y from (0, -5, 0) hits the near face at
    // t = 3 (from y = -5 to the face at y = -2).  q = euler 90° about Z.
    const double s = std::sin(mmd::core::physics_math::kPi / 4.0);
    const double c = std::cos(mmd::core::physics_math::kPi / 4.0);
    const Double4 q(0.0, 0.0, s, c); // 90° about Z: (0, 0, sin45, cos45)
    double t = -1.0;
    REQUIRE(rayBox(Double3(0.0, -5.0, 0.0), Double3(0.0, 1.0, 0.0), Double3(0, 0, 0), q,
                   Double3(2, 1, 1), t));
    CHECK(t == Approx(3.0));
}

// ---------------------------------------------------------------------------
// Ray / capsule
// ---------------------------------------------------------------------------

TEST_CASE("Ray through the capsule side hits at the cylinder entry", "[collider_geometry]")
{
    double t = -1.0;
    // Capsule radius 1, length 4 (axis +Y, from -2 to 2).  Ray along +X from
    // (-5, 0, 0) hits the cylinder at t = 4.
    REQUIRE(rayCapsule(Double3(-5.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0),
                       Double4(0, 0, 0, 1), 1.0, 4.0, t));
    CHECK(t == Approx(4.0));
}

TEST_CASE("Ray through the capsule end cap hits the hemisphere", "[collider_geometry]")
{
    double t = -1.0;
    // Capsule radius 1, length 4 (ends at y = +-2).  Ray along +Y from
    // (0, -6, 0) hits the bottom hemisphere surface at t = 3 (center at
    // y = -2, distance 4 from -6, minus the radius 1).
    REQUIRE(rayCapsule(Double3(0.0, -6.0, 0.0), Double3(0.0, 1.0, 0.0), Double3(0, 0, 0),
                       Double4(0, 0, 0, 1), 1.0, 4.0, t));
    CHECK(t == Approx(3.0));
}

TEST_CASE("Ray past the capsule side but outside the end caps does not hit",
          "[collider_geometry]")
{
    double t = -1.0;
    // Passes at y = 3.5 — above the top hemisphere (top center y = 2, radius
    // 1, so the cap reaches y = 3); y = 3 would be exactly tangent -> hit.
    CHECK_FALSE(rayCapsule(Double3(-5.0, 3.5, 0.0), Double3(1.0, 0.0, 0.0), Double3(0, 0, 0),
                           Double4(0, 0, 0, 1), 1.0, 4.0, t));
}

// ---------------------------------------------------------------------------
// raycastBodies — nearest-hit ordering
// ---------------------------------------------------------------------------

TEST_CASE("raycastBodies returns the nearest body and its index", "[collider_geometry]")
{
    // Two spheres along +X: body 0 at x = -2 (radius 1), body 1 at x = +3
    // (radius 1).  Ray from (-5, 0, 0) toward +X hits body 0 first.
    RayBody bodies[2];
    bodies[0] = centeredBody(Simulation::ColliderType::eSphere, Double3(1.0, 1.0, 1.0));
    bodies[0].pos = Double3(-2.0, 0.0, 0.0);
    bodies[1] = centeredBody(Simulation::ColliderType::eSphere, Double3(1.0, 1.0, 1.0));
    bodies[1].pos = Double3(3.0, 0.0, 0.0);

    double t = 0.0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const int hit = raycastBodies(Double3(-5.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), bodies, 2, t);
    CHECK(hit == 0);
    CHECK(t == Approx(2.0)); // -5 to sphere surface at -3 is 2
}

TEST_CASE("raycastBodies returns -1 when nothing is hit", "[collider_geometry]")
{
    RayBody bodies[1];
    bodies[0] = centeredBody(Simulation::ColliderType::eSphere, Double3(1.0, 1.0, 1.0));
    bodies[0].pos = Double3(0.0, 10.0, 0.0);

    double t = 0.0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const int hit = raycastBodies(Double3(-5.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), bodies, 1, t);
    CHECK(hit == -1);
}

TEST_CASE("raycastBodies respects body orientation for boxes", "[collider_geometry]")
{
    // A thin box (half extents 0.1 x 2 x 2), identity rotation, centered at
    // origin.  A ray along +X from (-5, 0, 0) hits it (thin face); a ray
    // along +Y from (0, -5, 0) also hits (wide face).
    RayBody bodies[1];
    bodies[0] = centeredBody(Simulation::ColliderType::eBox, Double3(0.2, 4.0, 4.0));

    double t = 0.0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    CHECK(raycastBodies(Double3(-5.0, 0.0, 0.0), Double3(1.0, 0.0, 0.0), bodies, 1, t) == 0);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    CHECK(raycastBodies(Double3(0.0, -5.0, 0.0), Double3(0.0, 1.0, 0.0), bodies, 1, t) == 0);
}
