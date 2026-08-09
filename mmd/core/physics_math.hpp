/**
 * @file physics_math.hpp
 * @brief Pure math shared by the core — Maya-free and Bullet-only, so it can
 * be unit-tested with a plain C++ target (no Maya SDK needed).
 *
 * Everything here is `inline` in namespace mmd::core::physics_math so the
 * same header serves the physics engine and, later, the draw override.
 *
 * Conventions (each was verified empirically against Maya 2026):
 *   - Attributes carry XYZ euler angles in DEGREES; the Bullet world works in
 *     radians and quaternions.  eulerDegreesToQuat / quatToEulerXYZDegrees are
 *     exact inverses for the Maya rotate-XYZ convention.
 *   - Maya matrices are ROW-vector (p' = p * M); Bullet matrices are
 *     COLUMN-vector (v' = M * v).  doubleMatrixToBtTransform transposes.
 */

#pragma once

#include "common.hpp"

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

#include <cmath>

namespace mmd
{
namespace core
{
namespace physics_math
{

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d)
{
    return d * kPi / 180.0;
}
inline double rad2deg(double r)
{
    return r * 180.0 / kPi;
}

// Build the Bullet (column-vector) rotation for Maya rotate-XYZ degrees.
// Maya's rotate-XYZ (rotateOrder 0) builds the row-vector matrix
//   M_row = (Rz * Ry * Rx)^T   (verified empirically against Maya 2026)
// so the equivalent Bullet (column-vector) matrix is M_col = Rz * Ry * Rx,
// whose quaternion is q = qz * qy * qx.
inline btQuaternion eulerDegreesToQuat(double rx, double ry, double rz)
{
    btQuaternion qx(btVector3(1, 0, 0), deg2rad(rx));
    btQuaternion qy(btVector3(0, 1, 0), deg2rad(ry));
    btQuaternion qz(btVector3(0, 0, 1), deg2rad(rz));
    return qz * qy * qx; // M_col = Rz * Ry * Rx
}

// Extract XYZ euler (degrees) from a quaternion in the Maya rotate convention.
// The Bullet matrix m = Rz * Ry * Rx (matches eulerDegreesToQuat = qz*qy*qx):
//   sin(ry) = -m[2][0];  rx = atan2(m[2][1], m[2][2]);  rz = atan2(m[1][0], m[0][0])
inline void quatToEulerXYZDegrees(const btQuaternion& q, Double3& out)
{
    btMatrix3x3 m(q);
    const double sy = -m[2][0]; // sin(ry)
    const double epsilon = 1e-6;
    if (sy < -1.0 + epsilon || sy > 1.0 - epsilon)
    {
        // Gimbal lock: ry = ±90°, rx/rz degenerate (only their combination is
        // well-defined).  Set rx = 0 and solve the combined term:
        //   ry=+90: m[0][1]=sin(rx-rz), m[0][2]=cos(rx-rz) -> rz = atan2(-m[0][1], m[0][2])
        //   ry=-90: m[0][1]=-sin(rx+rz), m[1][1]=cos(rx+rz) -> rz = atan2(-m[0][1], m[1][1])
        // (Earlier this extracted for M=Rx*Ry*Rz and flipped ry's sign, which
        // rotated every gimbal-locked body 180° and displaced the bones.)
        double ry;
        double rz;
        if (m[2][0] < 0.0)
        {
            ry = kPi / 2.0;
            rz = std::atan2(-m[0][1], m[0][2]);
        }
        else
        {
            ry = -kPi / 2.0;
            rz = std::atan2(-m[0][1], m[1][1]);
        }
        out.x = 0.0;
        out.y = rad2deg(ry);
        out.z = rad2deg(rz);
        return;
    }
    double rx = std::atan2(m[2][1], m[2][2]);
    double ry = std::asin(sy);
    double rz = std::atan2(m[1][0], m[0][0]);
    out.x = rad2deg(rx);
    out.y = rad2deg(ry);
    out.z = rad2deg(rz);
}

// Build a btTransform from a rest position + Maya XYZ euler degrees (both as
// Double3 value types — the conversion to Bullet happens here).
inline btTransform transformFromRest(const Double3& pos, const Double3& rotDeg)
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos.x, pos.y, pos.z));
    t.setBasis(btMatrix3x3(eulerDegreesToQuat(rotDeg.x, rotDeg.y, rotDeg.z)));
    return t;
}

// Convert a Matrix4 (ROW-vector, Maya convention: m[r][c] holds row r,
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
        for (int c = 0; c < 3; ++c)
            bm[c][r] = m[r][c]; // transpose: Bullet column matrix = Maya row^T
    t.setBasis(bm);
    t.setOrigin(btVector3(m[3][0], m[3][1], m[3][2]));
    return t;
}

// Store a Bullet transform as a Double3/Double4 pos+quat (no Bullet type
// escapes the core — the conversion to the value types happens here).
inline void storeAnchorPose(Double3& pos, Double4& quat, const btTransform& t)
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

inline btTransform anchorPoseToTransform(const Double3& pos, const Double4& quat)
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
        for (int c = 0; c < 3; ++c)
            m[r][c] = t.getBasis()[c][r]; // transpose back to row-vector
    m[0][3] = m[1][3] = m[2][3] = 0.0;
    const btVector3& o = t.getOrigin();
    m[3][0] = o.x();
    m[3][1] = o.y();
    m[3][2] = o.z();
    m[3][3] = 1.0;
}

// 4x4 ROW-vector matrix multiply: out = a * b (Maya convention, p' = p * M).
inline void rowMatrixMultiply(const Matrix4& a, const Matrix4& b, Matrix4& out)
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
        {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
                s += a[r][k] * b[k][c];
            out[r][c] = s;
        }
}

} // namespace physics_math
} // namespace core
} // namespace mmd
