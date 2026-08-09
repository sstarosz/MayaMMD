/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_physics_masks.h
 *
 * Collision-group mask resolution for the MMD physics node — Maya-free and
 * Bullet-free, so it can be unit-tested with a plain C++ target.
 *
 * This is an EXACT port of the proximity + cloth-on-cloth mask corrections
 * that used to live in rigid_body_builder._compute_collision_masks (plus the
 * mask combination in _create_rigid_body_guide).  Converted game models ship
 * degenerate PMX non_collision_group values (every body = "own group only",
 * e.g. Tololo skirt 0x0004, legs 0x0002), which would let the skirt pass
 * straight through the legs.  The effective mask is therefore corrected from
 * the REST-POSE geometry:
 *
 *   - proximity: a DYNAMIC body gains the KINEMATIC groups whose colliders
 *     overlap its rest collider (skirt blocks on the legs/hips it wraps, but
 *     bangs keep colliding only with the head they rest on — not the huge
 *     torso capsule), and a KINEMATIC body gains the DYNAMIC groups that
 *     overlap it;
 *   - own-group bit cleared for dynamic bodies (hair/skirt spheres are stored
 *     DEEPLY OVERLAPPING; self-collision would push the chains apart);
 *   - cloth-on-cloth: a SHORT cloth chain (<= 10 bodies) that genuinely DRAPES
 *     a LARGE sheet (>= 50 bodies) sharing the same kinematic anchor (bangs
 *     resting on the skirt) gains the sheet's group (and vice versa).
 *
 * Do NOT change the constants or the logic without re-running the rigidbody
 * suite — the corrections are validated behaviorally across 17 models.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <set>
#include <utility>
#include <vector>

namespace mmd_physics_masks
{

// Collider shape tags for the mask resolver.  This module is standalone
// (Maya-free/Bullet-free), so it declares its own enum — values align with
// MMDPhysicsNode::ColliderType (1 box, 2 sphere, 3 capsule).
enum ColliderType : short
{
    kColliderBox = 1,
    kColliderSphere = 2,
    kColliderCapsule = 3,
};
// Cloth-on-cloth guards (see rigid_body_builder for the MMD-intent reasoning).
// A shallow touch (cape tips brushing the jacket back ~0.11 deep) must NOT
// qualify; a real drape (bangs penetrating the skirt 0.21-0.81) must.
constexpr double kClothOverlapPenetration = 0.15;
constexpr int kClothSmallChain = 10;
constexpr int kClothLargeSheet = 50;
// Proximity contact slack (extent sums are rough).
constexpr double kOverlapSlack = 0.2;

// Per-body input, index-aligned with the rigid-body array.
struct BodyInput
{
    double pos[3] = {0.0, 0.0, 0.0};
    ColliderType colliderType = kColliderBox;
    double radius = 0.5;
    double extents[3] = {1.0, 1.0, 1.0};
    double length = 1.0;
    short groupId = 0;               // PMX collision group 0..15
    bool kinematic = false;          // FOLLOW_BONE (anchor) vs dynamic
    long nonCollisionGroup = 0xFFFF; // raw PMX 16-bit non-collision mask
};

struct JointInput
{
    int bodyA = -1;
    int bodyB = -1;
};

// Rough bounding radius of a body's rest collider (mirror of _approx_extent):
// box -> largest half-extent; sphere -> radius; capsule -> max(radius, half+radius).
inline double approxExtent(const BodyInput& b)
{
    if (b.colliderType == kColliderBox)
        return std::max({b.extents[0], b.extents[1], b.extents[2]});
    if (b.colliderType == kColliderSphere)
        return b.radius;
    return std::max(b.radius, b.length / 2.0 + b.radius);
}

// Effective collision group bit (1 << groupId).
inline long groupBit(short groupId)
{
    return 1L << (groupId & 0x0F);
}

// Compute the effective collision mask for EVERY body (index-aligned with
// `bodies`).  Exact port of rigid_body_builder._compute_collision_masks plus
// the mask combination in _create_rigid_body_guide.  Distances use the rest
// positions — a Z-flip is an isometry, so it does not change the result.
inline void computeEffectiveMasks(const std::vector<BodyInput>& bodies,
                                  const std::vector<JointInput>& joints,
                                  std::vector<long>& outMasks)
{
    const size_t n = bodies.size();
    outMasks.assign(n, 0);

    std::vector<double> extents(n);
    for (size_t i = 0; i < n; ++i)
        extents[i] = approxExtent(bodies[i]);

    // --- PROXIMITY correction (kinematic <-> dynamic overlap at rest) ---
    // kinOverlap[dyn]  = kinematic group bits overlapping that dynamic body.
    // dynOverlap[kin]  = dynamic group bits overlapping that kinematic body.
    std::vector<long> kinOverlap(n, 0);
    std::vector<long> dynOverlap(n, 0);
    for (size_t i = 0; i < n; ++i)
    {
        long bits = 0;
        for (size_t j = 0; j < n; ++j)
        {
            if (bodies[i].kinematic == bodies[j].kinematic)
                continue; // only body <-> cloth pairs
            const double dx = bodies[i].pos[0] - bodies[j].pos[0];
            const double dy = bodies[i].pos[1] - bodies[j].pos[1];
            const double dz = bodies[i].pos[2] - bodies[j].pos[2];
            const double rr = extents[i] + extents[j] + kOverlapSlack;
            if (dx * dx + dy * dy + dz * dz < rr * rr)
                bits |= groupBit(bodies[j].groupId);
        }
        if (bodies[i].kinematic)
            dynOverlap[i] = bits;
        else
            kinOverlap[i] = bits;
    }

    // --- Cloth-on-cloth correction (hair/bangs draping the skirt/jacket) ---
    // Chains: dynamic bodies jointed together form one cloth chain (kinematic
    // bodies never merge chains).  The draped chains then get each other's
    // group, guarded by: same body part (shared kinematic-anchor group),
    // one chain small / the other large, and real interpenetration at rest.
    std::vector<size_t> parent(n);
    for (size_t i = 0; i < n; ++i)
        parent[i] = i;
    std::function<size_t(size_t)> find = [&parent](size_t x)
    {
        while (parent[x] != x)
        {
            parent[x] = parent[parent[x]]; // path halving
            x = parent[x];
        }
        return x;
    };
    auto unite = [&find, &parent](size_t a, size_t c)
    {
        const size_t ra = find(a), rc = find(c);
        if (ra != rc)
            parent[rc] = ra;
    };

    for (const JointInput& jn : joints)
    {
        if (jn.bodyA < 0 || jn.bodyB < 0)
            continue;
        const size_t a = static_cast<size_t>(jn.bodyA);
        const size_t b = static_cast<size_t>(jn.bodyB);
        if (a >= n || b >= n)
            continue;
        if (!bodies[a].kinematic && !bodies[b].kinematic)
            unite(a, b); // dynamic <-> dynamic joint = same cloth chain
    }

    std::vector<int> chainSize(n, 0);
    for (size_t i = 0; i < n; ++i)
    {
        if (bodies[i].kinematic)
            continue;
        chainSize[find(i)] += 1;
    }

    std::vector<long> anchorGroups(n, 0); // chain root -> kinematic group bits
    for (const JointInput& jn : joints)
    {
        if (jn.bodyA < 0 || jn.bodyB < 0)
            continue;
        const size_t a = static_cast<size_t>(jn.bodyA);
        const size_t b = static_cast<size_t>(jn.bodyB);
        if (a >= n || b >= n)
            continue;
        if (bodies[a].kinematic && !bodies[b].kinematic)
            anchorGroups[find(b)] |= groupBit(bodies[a].groupId);
        else if (bodies[b].kinematic && !bodies[a].kinematic)
            anchorGroups[find(a)] |= groupBit(bodies[b].groupId);
    }

    // Chain pairs where the small chain drapes the large one.
    std::set<std::pair<size_t, size_t>> draped;
    for (size_t i = 0; i < n; ++i)
    {
        if (bodies[i].kinematic)
            continue;
        const size_t ri = find(i);
        for (size_t j = i + 1; j < n; ++j)
        {
            if (bodies[j].kinematic)
                continue;
            const size_t rj = find(j);
            if (bodies[j].groupId == bodies[i].groupId || rj == ri)
                continue;
            // Guard: one chain small, the other large.
            const int si = chainSize[ri], sj = chainSize[rj];
            if (!((si <= kClothSmallChain && sj >= kClothLargeSheet) ||
                  (sj <= kClothSmallChain && si >= kClothLargeSheet)))
                continue;
            // Guard: same body part (same kinematic-anchor group).
            if ((anchorGroups[ri] & anchorGroups[rj]) == 0)
                continue;
            // Guard: real draping interpenetration, not a mere touch.
            const double dx = bodies[i].pos[0] - bodies[j].pos[0];
            const double dy = bodies[i].pos[1] - bodies[j].pos[1];
            const double dz = bodies[i].pos[2] - bodies[j].pos[2];
            const double rr = extents[i] + extents[j] - kClothOverlapPenetration;
            if (dx * dx + dy * dy + dz * dz < rr * rr)
            {
                const size_t lo = std::min(ri, rj);
                const size_t hi = std::max(ri, rj);
                draped.insert({lo, hi});
            }
        }
    }

    std::vector<long> clothOverlap(n, 0); // dynamic body -> dynamic group bits
    for (size_t i = 0; i < n; ++i)
    {
        if (bodies[i].kinematic)
            continue;
        const size_t ri = find(i);
        long bits = 0;
        for (size_t j = 0; j < n; ++j)
        {
            if (i == j || bodies[j].kinematic)
                continue;
            if (bodies[j].groupId == bodies[i].groupId)
                continue; // own group stays cleared (hair <-> hair pass-through)
            const size_t rj = find(j);
            if (rj == ri)
                continue; // same cloth chain — jointed bodies never collide
            if ((anchorGroups[ri] & anchorGroups[rj]) == 0)
                continue; // different body part (arm/head vs torso)
            const size_t lo = std::min(ri, rj);
            const size_t hi = std::max(ri, rj);
            if (draped.find({lo, hi}) == draped.end())
                continue; // chains only touch / wrong size mix, no drape
            const double dx = bodies[i].pos[0] - bodies[j].pos[0];
            const double dy = bodies[i].pos[1] - bodies[j].pos[1];
            const double dz = bodies[i].pos[2] - bodies[j].pos[2];
            const double rr = extents[i] + extents[j] + kOverlapSlack;
            if (dx * dx + dy * dy + dz * dz < rr * rr)
                bits |= groupBit(bodies[j].groupId);
        }
        clothOverlap[i] = bits;
    }

    // --- Combine into the effective mask (mirror of _create_rigid_body_guide) ---
    for (size_t i = 0; i < n; ++i)
    {
        long mask = (~bodies[i].nonCollisionGroup) & 0xFFFF;
        if (bodies[i].kinematic)
        {
            mask |= dynOverlap[i];
        }
        else
        {
            mask &= ~groupBit(bodies[i].groupId); // own group cleared
            mask |= kinOverlap[i];
            mask |= clothOverlap[i];
        }
        outMasks[i] = mask;
    }
}

} // namespace mmd_physics_masks
