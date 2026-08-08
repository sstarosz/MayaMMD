/*
 * SPDX-License-Identifier: MIT
 *
 * test_mmd_physics_masks.cpp
 *
 * Unit tests for the collision-mask resolver in
 * mmd/maya/nodes/mmd_physics_masks.h.  These lock in the exact behavior of the
 * ported proximity + cloth-on-cloth corrections (previously
 * rigid_body_builder._compute_collision_masks) with synthetic geometries.
 *
 * Build + run (BUILD_TESTS=ON):
 *   cmake --preset maya2026-release -DBUILD_TESTS=ON
 *   cmake --build out/build/maya2026-release --config Release --target mmd_tools_tests
 *   out/build/maya2026-release/tests/Release/mmd_tools_tests.exe [test-filter]
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "mmd/maya/nodes/mmd_physics_masks.h"

#include <vector>

using namespace mmd_physics_masks;

namespace
{

BodyInput sphere(double x, double y, double z, double r, short groupId, bool kinematic,
                 long nonCollisionGroup = 0xFFFF)
{
    BodyInput b;
    b.pos[0] = x;
    b.pos[1] = y;
    b.pos[2] = z;
    b.colliderType = kColliderSphere;
    b.radius = r;
    b.groupId = groupId;
    b.kinematic = kinematic;
    b.nonCollisionGroup = nonCollisionGroup;
    return b;
}

// Append a chain of `count` overlapping spheres (r=0.5, spacing 0.8 along z at
// (x, y)) as one connected cloth chain.  Returns the index of the first body.
int addChain(std::vector<BodyInput>& bodies, std::vector<JointInput>& joints, double x, double y,
             double z0, int count, short groupId)
{
    const int base = static_cast<int>(bodies.size());
    for (int i = 0; i < count; ++i)
    {
        bodies.push_back(sphere(x, y, z0 + i * 0.8, 0.5, groupId, false));
        if (i > 0)
            joints.push_back({base + i - 1, base + i});
    }
    return base;
}

bool hasBit(long mask, int group)
{
    return (mask & (1L << group)) != 0;
}

} // namespace

TEST_CASE("proximity correction + own-group clearing + kinematic keeps own group", "[masks]")
{
    // Body 0: kinematic sphere group 2 at origin.  Body 1: dynamic sphere
    // group 5 overlapping it.  Body 2: dynamic sphere group 5 far away.
    std::vector<BodyInput> bodies = {
        sphere(0, 0, 0, 1, 2, true),
        sphere(1, 0, 0, 1, 5, false),
        sphere(100, 0, 0, 1, 5, false),
    };
    std::vector<JointInput> joints;
    std::vector<long> masks;
    computeEffectiveMasks(bodies, joints, masks);

    // Kinematic body gains the overlapping dynamic group, own group NOT cleared.
    REQUIRE(masks[0] == (1L << 5));
    // Dynamic body gains the overlapping kinematic group, own group cleared.
    REQUIRE(masks[1] == (1L << 2));
    // Far dynamic body: nothing, own group cleared.
    REQUIRE(masks[2] == 0);
}

TEST_CASE("raw PMX non_collision_group feeds the base mask", "[masks]")
{
    // Kinematic body group 2 that explicitly excludes group 3
    // (nonCollisionGroup = ~(1<<3)).
    std::vector<BodyInput> bodies = {
        sphere(0, 0, 0, 1, 2, true, ~(1L << 3)),
    };
    std::vector<JointInput> joints;
    std::vector<long> masks;
    computeEffectiveMasks(bodies, joints, masks);

    // Base = ~nonCollisionGroup & 0xFFFF = 1<<3 (group 3 allowed).
    REQUIRE(masks[0] == (1L << 3));
}

TEST_CASE("cloth-on-cloth: small chain draping a large sheet gains its group", "[masks]")
{
    std::vector<BodyInput> bodies;
    std::vector<JointInput> joints;

    // Kinematic anchor (group 7), far away so it never couples by proximity.
    bodies.push_back(sphere(10, 0, 0, 0.5, 7, true));

    // Large sheet: 60 overlapping spheres along z at x=0 (one cloth chain).
    const int largeBase = static_cast<int>(bodies.size());
    const int largeCount = 60;
    for (int i = 0; i < largeCount; ++i)
    {
        bodies.push_back(sphere(0, 0, i * 0.8, 0.5, 3, false));
        if (i > 0)
            joints.push_back({largeBase + i - 1, largeBase + i});
    }

    // Small chain: 8 spheres draping the sheet near z=24 (one cloth chain).
    const int smallBase = static_cast<int>(bodies.size());
    const int smallCount = 8;
    for (int i = 0; i < smallCount; ++i)
    {
        bodies.push_back(sphere(0.5, -0.2, 24.0 + i * 0.8, 0.5, 4, false));
        if (i > 0)
            joints.push_back({smallBase + i - 1, smallBase + i});
    }

    // Both chains are anchored to the SAME kinematic body (same body part).
    joints.push_back({0, largeBase + 30}); // z = 24.0
    joints.push_back({0, smallBase});      // z = 24.0

    std::vector<long> masks;
    computeEffectiveMasks(bodies, joints, masks);

    // The small chain's overlapping bodies rest ON the large sheet.
    bool smallGainedLarge = false;
    for (int i = 0; i < smallCount; ++i)
        smallGainedLarge = smallGainedLarge || hasBit(masks[smallBase + i], 3);
    REQUIRE(smallGainedLarge);

    // The large sheet's overlapping bodies support the small chain.
    bool largeGainedSmall = false;
    for (int i = 0; i < largeCount; ++i)
        largeGainedSmall = largeGainedSmall || hasBit(masks[largeBase + i], 4);
    REQUIRE(largeGainedSmall);

    // A sheet body far away from the small chain gains nothing extra.
    REQUIRE(!hasBit(masks[largeBase], 4));
    // The kinematic anchor itself is untouched by the cloth rule.
    REQUIRE(!hasBit(masks[0], 3));
    REQUIRE(!hasBit(masks[0], 4));
}

TEST_CASE("cloth-on-cloth: a mere touch does NOT qualify as draping", "[masks]")
{
    std::vector<BodyInput> bodies;
    std::vector<JointInput> joints;

    bodies.push_back(sphere(10, 0, 0, 0.5, 7, true)); // anchor
    const int largeBase = static_cast<int>(bodies.size());
    for (int i = 0; i < 60; ++i)
    {
        bodies.push_back(sphere(0, 0, i * 0.8, 0.5, 3, false));
        if (i > 0)
            joints.push_back({largeBase + i - 1, largeBase + i});
    }
    const int smallBase = static_cast<int>(bodies.size());
    for (int i = 0; i < 8; ++i)
    {
        // x=0.9 -> centre distance 0.9: bodies TOUCH (0.9 < 1.0) but do NOT
        // interpenetrate enough (needs < 1.0 - 0.15 = 0.85) to drape.
        bodies.push_back(sphere(0.9, 0, 24.0 + i * 0.8, 0.5, 4, false));
        if (i > 0)
            joints.push_back({smallBase + i - 1, smallBase + i});
    }
    joints.push_back({0, largeBase + 30});
    joints.push_back({0, smallBase});

    std::vector<long> masks;
    computeEffectiveMasks(bodies, joints, masks);

    bool smallGainedLarge = false;
    for (int i = 0; i < 8; ++i)
        smallGainedLarge = smallGainedLarge || hasBit(masks[smallBase + i], 3);
    REQUIRE_FALSE(smallGainedLarge);
    REQUIRE_FALSE(hasBit(masks[largeBase + 30], 4));
}

TEST_CASE("cloth-on-cloth: different kinematic anchors never couple", "[masks]")
{
    std::vector<BodyInput> bodies;
    std::vector<JointInput> joints;

    const int anchorLarge = static_cast<int>(bodies.size());
    bodies.push_back(sphere(10, 0, 0, 0.5, 7, true));
    const int anchorSmall = static_cast<int>(bodies.size());
    bodies.push_back(sphere(20, 0, 0, 0.5, 9, true)); // different body part

    const int largeBase = static_cast<int>(bodies.size());
    for (int i = 0; i < 60; ++i)
    {
        bodies.push_back(sphere(0, 0, i * 0.8, 0.5, 3, false));
        if (i > 0)
            joints.push_back({largeBase + i - 1, largeBase + i});
    }
    const int smallBase = static_cast<int>(bodies.size());
    for (int i = 0; i < 8; ++i)
    {
        bodies.push_back(sphere(0.5, -0.2, 24.0 + i * 0.8, 0.5, 4, false));
        if (i > 0)
            joints.push_back({smallBase + i - 1, smallBase + i});
    }
    // Large sheet anchored to group-7 body; small chain to the group-9 body.
    joints.push_back({anchorLarge, largeBase + 30});
    joints.push_back({anchorSmall, smallBase});

    std::vector<long> masks;
    computeEffectiveMasks(bodies, joints, masks);

    bool smallGainedLarge = false;
    for (int i = 0; i < 8; ++i)
        smallGainedLarge = smallGainedLarge || hasBit(masks[smallBase + i], 3);
    REQUIRE_FALSE(smallGainedLarge);
    REQUIRE_FALSE(hasBit(masks[largeBase + 30], 4));
}

TEST_CASE("cloth-on-cloth: same chain / own group never self-collides", "[masks]")
{
    std::vector<BodyInput> bodies;
    std::vector<JointInput> joints;

    // A single dense chain of 8 overlapping spheres, all group 4.
    const int base = static_cast<int>(bodies.size());
    for (int i = 0; i < 8; ++i)
    {
        bodies.push_back(sphere(0, 0, i * 0.8, 0.5, 4, false));
        if (i > 0)
            joints.push_back({base + i - 1, base + i});
    }
    std::vector<long> masks;
    computeEffectiveMasks(bodies, joints, masks);

    for (int i = 0; i < 8; ++i)
    {
        // Own group stays cleared (hair <-> hair pass-through) and jointed
        // chain bodies never gain each other's group.
        REQUIRE(masks[base + i] == 0);
    }
}
