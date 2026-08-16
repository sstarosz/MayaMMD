/**
 * @file bullet_bridge.hpp
 * @brief Bullet-facing conversions for the Maya-free core.
 *
 * This is the ONLY core header that exposes Bullet types.  Consumers that
 * want to stay Bullet-free include common.hpp / physics_math.hpp /
 * rigid_body_simulation.hpp; this bridge is for the engine internals and for adapters
 * that already link Bullet (e.g. the Maya physics node).
 *
 * Conventions (each was verified empirically against Maya 2026):
 *   - Maya matrices are ROW-vector (p' = p * M); Bullet matrices are
 *     COLUMN-vector (v' = M * v).  doubleMatrixToBtTransform transposes.
 *   - Euler degrees / quaternions live in physics_math.hpp as core Double3 /
 *     Double4; the conversion to Bullet happens here.
 */

#pragma once

#include "common.hpp"
#include "physics_math.hpp"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

namespace mmd::core::physics_math
{

// Build a btTransform from a rest position + Maya XYZ euler degrees (both as
// Double3 value types — the conversion to Bullet happens here).
inline btTransform transformFromRest(const Double3& pos, const Double3& rotDeg)
{
    btTransform t;
    t.setIdentity();
    // Bullet stays float precision (see vcpkg.json); narrowing from the
    // core's Double3/Double4 value types is intentional.
    t.setOrigin(btVector3(
        static_cast<btScalar>(pos.x), static_cast<btScalar>(pos.y),
        static_cast<btScalar>(pos.z)));
    const Double4 q = eulerDegreesToQuat(rotDeg.x, rotDeg.y, rotDeg.z);
    t.setBasis(btMatrix3x3(btQuaternion(
        static_cast<btScalar>(q.x), static_cast<btScalar>(q.y),
        static_cast<btScalar>(q.z), static_cast<btScalar>(q.w))));
    return t;
}

// Convert a Matrix4 (ROW-vector, Maya convention: m(r, c) holds row r,
// column c) to a Bullet COLUMN-vector transform.
// Maya matrices are ROW-vector (p' = p * M): row r holds the image of the
// r-th basis vector and m[3][0..2] is the translation.  Bullet uses
// COLUMN-vector matrices (v' = M * v), so the same orientation's matrix is
// the TRANSPOSE of Maya's.  Copying the row matrix directly (as done
// before) gave every rotated anchor a transposed — i.e. wrong — basis,
// which yanked the attached rigid chains into a mess.
inline btTransform doubleMatrixToBtTransform(const Matrix4& m)
{
    btTransform t;
    btMatrix3x3 bm;
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            // transpose: Bullet column matrix = Maya row^T; narrowing from
            // the core's double Matrix4 to Bullet's float btScalar is intended.
            bm[c][r] = static_cast<btScalar>(m(r, c));
        }
    }
    t.setBasis(bm);
    t.setOrigin(btVector3(
        static_cast<btScalar>(m(3, 0)), static_cast<btScalar>(m(3, 1)),
        static_cast<btScalar>(m(3, 2))));
    return t;
}

// Store a Bullet transform as a Double3/Double4 pos+quat (no Bullet type
// escapes the core — the conversion to the value types happens here).
inline void storePose(Double3& pos, Double4& quat, const btTransform& t)
{
    const btVector3& o = t.getOrigin();
    const btQuaternion& q = t.getRotation();
    pos.x = o.x();
    pos.y = o.y();
    pos.z = o.z();
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
}

inline btTransform poseToTransform(const Double3& pos, const Double4& quat)
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(btScalar(pos.x), btScalar(pos.y), btScalar(pos.z)));
    t.setRotation(
        btQuaternion(btScalar(quat.x), btScalar(quat.y), btScalar(quat.z), btScalar(quat.w)));
    return t;
}

// Convert a Bullet COLUMN-vector transform to a Matrix4 (ROW-vector, Maya
// convention).  Exact inverse of doubleMatrixToBtTransform: m(r,c) = bm(c,r),
// and the translation lands in the last row (m[3][0..2]) as Maya expects.
inline void btTransformToRowMatrix(const btTransform& t, Matrix4& m)
{
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            m(r, c) = t.getBasis()[c][r]; // transpose back to row-vector
        }
    }
    m(0, 3) = m(1, 3) = m(2, 3) = 0.0;
    const btVector3& o = t.getOrigin();
    m(3, 0) = o.x();
    m(3, 1) = o.y();
    m(3, 2) = o.z();
    m(3, 3) = 1.0;
}

} // namespace mmd::core::physics_math
