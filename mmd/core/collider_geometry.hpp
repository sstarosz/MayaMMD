/**
 * @file collider_geometry.hpp
 * @brief Pure, Maya-free geometry for the rigid-body collider guides.
 *
 * The viewport draw override (physics_draw_override.cpp) needs two things the
 * node's DrawBody doesn't directly provide:
 *   1. the ENGINE primitive params (radius / box half-extents / capsule
 *      length) derived from the PMX shape_size stored VERBATIM on the node,
 *      and
 *   2. a ray-vs-collider hit test so a viewport pick can select the body.
 *
 * Both are pure math on the core Double3/Double4 types (no Maya SDK, no
 * Bullet), so they live here and are unit-tested with a plain C++ target.
 *
 * The collider frame follows the engine: a body is positioned at `pos` and
 * oriented by unit quaternion `quat` ({x, y, z, w}); the capsule's cylinder
 * axis is the body's +Y axis (PMX capsules are Y-aligned, as in Bullet).
 */

#pragma once

#include "common.hpp"
#include "simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mmd::core::collider_geometry
{

// One rigid body as seen by a ray: the pose + the PMX shape_size VERBATIM
// (full size — the ray functions derive the engine primitives internally).
struct RayBody
{
    Double3 pos;
    Double4 quat = Double4(0.0, 0.0, 0.0, 1.0); // unit {x, y, z, w}
    Simulation::ColliderType colliderType = Simulation::ColliderType::eBox;
    Double3 shapeSize; // PMX shape_size (full size)
};

// ---------------------------------------------------------------------------
// Primitive derivation — PMX shape_size (full) → engine radius/extents/length.
// Mirrors mmd::core::applyShapeSize (the node bakes the same numbers into the
// engine), but returns plain doubles so the draw code can use them directly.
// ---------------------------------------------------------------------------
struct PrimitiveParams
{
    double radius = 0.5;
    Double3 halfExtents = Double3(1.0, 1.0, 1.0); // box only
    double length = 1.0;                          // capsule only
};

inline PrimitiveParams primitiveFromShapeSize(Simulation::ColliderType type,
                                              const Double3& size)
{
    PrimitiveParams p;
    switch (type)
    {
    case Simulation::ColliderType::eSphere:
        p.radius = size.x; // sphere uses shape_size[0] as its radius
        break;
    case Simulation::ColliderType::eBox:
        // FULL size -> half extents (as btBoxShape expects).
        p.halfExtents = Double3(size.x * 0.5, size.y * 0.5, size.z * 0.5);
        break;
    case Simulation::ColliderType::eCapsule:
        p.radius = size.x; // capsule: shape_size[0] = radius
        p.length = size.y; //          shape_size[1] = cylinder length
        break;
    }
    return p;
}

// ---------------------------------------------------------------------------
// Vector helpers (Double3 has no operators — keep it minimal here).
// ---------------------------------------------------------------------------
inline Double3 sub(const Double3& a, const Double3& b)
{
    return Double3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline double dot(const Double3& a, const Double3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Double3 cross(const Double3& a, const Double3& b)
{
    return Double3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Rotate v by unit quaternion q ({x, y, z, w}) — standard q v q^-1.
// (Double3 has no operators; the Hamilton double-rotation is expanded here.)
inline Double3 rotatePoint(const Double4& q, const Double3& v)
{
    const Double3 u(q.x, q.y, q.z);
    const Double3 t = cross(u, v);
    const double s = 2.0 * q.w;
    const Double3 sv(t.x * s, t.y * s, t.z * s);
    const Double3 uxt = cross(u, t);
    return Double3(v.x + sv.x + uxt.x, v.y + sv.y + uxt.y, v.z + sv.z + uxt.z);
}

// ---------------------------------------------------------------------------
// Ray / collider intersection.
//
// `origin` and `dir` are in the same frame as the body pose (the node's
// object space = the physics group's local space).  `dir` need not be
// normalized; the returned t is in units of `dir`'s length (Maya's
// MSelectionInfo::getLocalRay returns a unit ray, so t is a distance).
// Returns true + `t` on a hit, false otherwise.
// ---------------------------------------------------------------------------

inline bool raySphere(const Double3& origin, const Double3& dir, const Double3& center,
                      double radius, double& t)
{
    const Double3 oc = sub(origin, center);
    const double b = dot(oc, dir);
    const double c = dot(oc, oc) - radius * radius;
    const double disc = b * b - c;
    if (disc < 0.0)
        return false;
    const double sq = std::sqrt(disc);
    const double t0 = -b - sq;
    const double t1 = -b + sq;
    t = (t0 >= 0.0) ? t0 : t1;
    return t >= 0.0;
}

inline bool rayBox(const Double3& origin, const Double3& dir, const Double3& center,
                   const Double4& quat, const Double3& halfExtents, double& t)
{
    // Transform the ray into the box's local frame (box is axis-aligned there).
    const Double3 oc = sub(origin, center);
    const Double4 invQ(-quat.x, -quat.y, -quat.z, quat.w); // conjugate = inverse (unit)
    const Double3 lo = rotatePoint(invQ, oc);
    const Double3 ld = rotatePoint(invQ, dir);

    double tmin = 0.0;
    double tmax = std::numeric_limits<double>::max();
    for (int i = 0; i < 3; ++i)
    {
        const double h = halfExtents[i];
        const double oi = lo[i];
        const double di = ld[i];
        if (std::abs(di) < 1e-12)
        {
            if (oi < -h || oi > h)
                return false; // ray parallel to this slab, outside it
            continue;
        }
        double t1 = (-h - oi) / di;
        double t2 = (h - oi) / di;
        if (t1 > t2)
            std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax)
            return false;
    }
    t = tmin;
    return t >= 0.0;
}

inline bool rayCapsule(const Double3& origin, const Double3& dir, const Double3& center,
                       const Double4& quat, double radius, double length, double& t)
{
    // Capsule = cylinder (axis +Y, radius r, from -h to +h) plus two end
    // hemispheres.  Transform the ray into the capsule's local frame first.
    const Double3 oc = sub(origin, center);
    const Double4 invQ(-quat.x, -quat.y, -quat.z, quat.w);
    const Double3 lo = rotatePoint(invQ, oc);
    const Double3 ld = rotatePoint(invQ, dir);
    const double h = length * 0.5;

    // Infinite cylinder around +Y: solve |lo + t*ld - (0, y, 0)|^2 = r^2 in
    // the XZ plane: A t^2 + 2 B t + C = 0 with
    //   A = ld.x^2 + ld.z^2
    //   B = lo.x*ld.x + lo.z*ld.z
    //   C = lo.x^2 + lo.z^2 - r^2
    const double a = ld.x * ld.x + ld.z * ld.z;
    const double b = lo.x * ld.x + lo.z * ld.z;
    const double c = lo.x * lo.x + lo.z * lo.z - radius * radius;
    double tBest = -1.0;
    if (std::abs(a) > 1e-12)
    {
        const double disc = b * b - a * c;
        if (disc >= 0.0)
        {
            const double sq = std::sqrt(disc);
            const double t0 = (-b - sq) / a;
            const double t1 = (-b + sq) / a;
            for (double tc : {t0, t1})
            {
                if (tc < 0.0)
                    continue;
                const double y = lo.y + tc * ld.y;
                if (y >= -h && y <= h)
                {
                    tBest = tc;
                    break; // t0 is the entry, use it
                }
            }
        }
    }

    // End spheres at (0, ±h, 0).
    double ts;
    if (raySphere(lo, ld, Double3(0.0, h, 0.0), radius, ts) && (tBest < 0.0 || ts < tBest))
        tBest = ts;
    if (raySphere(lo, ld, Double3(0.0, -h, 0.0), radius, ts) && (tBest < 0.0 || ts < tBest))
        tBest = ts;

    t = tBest;
    return t >= 0.0;
}

// Hit-test a ray against every body; returns the index of the nearest hit
// (lowest t), or -1.  `dir` may be unnormalized; the comparison is in units
// of `dir` so it is order-correct either way.
inline int raycastBodies(const Double3& origin, const Double3& dir, const RayBody* bodies,
                         size_t count, double& outT)
{
    int best = -1;
    double bestT = std::numeric_limits<double>::max();
    for (size_t i = 0; i < count; ++i)
    {
        const PrimitiveParams p =
            primitiveFromShapeSize(bodies[i].colliderType, bodies[i].shapeSize);
        double t = -1.0;
        bool hit = false;
        switch (bodies[i].colliderType)
        {
        case Simulation::ColliderType::eSphere:
            hit = raySphere(origin, dir, bodies[i].pos, p.radius, t);
            break;
        case Simulation::ColliderType::eBox:
            hit = rayBox(origin, dir, bodies[i].pos, bodies[i].quat, p.halfExtents, t);
            break;
        case Simulation::ColliderType::eCapsule:
            hit = rayCapsule(origin, dir, bodies[i].pos, bodies[i].quat, p.radius, p.length, t);
            break;
        }
        if (hit && t < bestT)
        {
            bestT = t;
            best = static_cast<int>(i);
        }
    }
    outT = bestT;
    return best;
}

} // namespace mmd::core::collider_geometry
