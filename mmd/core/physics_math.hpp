/**
 * @file physics_math.hpp
 * @brief Pure math shared by the core — Bullet-free and Maya-free, so it can
 * be unit-tested with a plain C++ target (no Maya SDK, no Bullet needed).
 *
 * Everything here is `constexpr`/`inline` in namespace mmd::core::physics_math.
 * Quaternions are carried as the core Double4 type ({x, y, z, w}) so no Bullet
 * type escapes the core; the Bullet-facing conversions live in
 * bullet_bridge.hpp (the only header that exposes Bullet types).
 *
 * Conventions (each was verified empirically against Maya 2026):
 *   - Attributes carry XYZ euler angles in DEGREES; the Bullet world works in
 *     radians and quaternions.  eulerDegreesToQuat / quatToEulerXYZDegrees are
 *     exact inverses for the Maya rotate-XYZ convention.
 *   - Maya matrices are ROW-vector (p' = p * M).
 */

#pragma once

#include "common.hpp"

#include <cmath>

namespace mmd::core::physics_math
{

constexpr double kPi = 3.14159265358979323846;

constexpr double deg2rad(double d)
{
    return d * kPi / 180.0;
}
constexpr double rad2deg(double r)
{
    return r * 180.0 / kPi;
}

// Build the Maya rotate-XYZ (rotateOrder 0) quaternion as a core Double4
// {x, y, z, w}.  Maya's rotate-XYZ builds the row-vector matrix
//   M_row = (Rz * Ry * Rx)^T   (verified empirically against Maya 2026)
// so the equivalent column-vector rotation is M_col = Rz * Ry * Rx, whose
// quaternion is q = qz * qy * qx.  The Hamilton product is expanded here in
// plain doubles so no Bullet type is needed — the result matches Bullet's
// qz * qy * qx component for component.
inline Double4 eulerDegreesToQuat(double rx, double ry, double rz)
{
    const double hx = deg2rad(rx) * 0.5;
    const double hy = deg2rad(ry) * 0.5;
    const double hz = deg2rad(rz) * 0.5;
    const double cx = std::cos(hx);
    const double sx = std::sin(hx);
    const double cy = std::cos(hy);
    const double sy = std::sin(hy);
    const double cz = std::cos(hz);
    const double sz = std::sin(hz);

    // q = qz * qy * qx (Hamilton product):
    //   w = cz*cy*cx + sz*sy*sx
    //   x = cz*cy*sx - sz*sy*cx
    //   y = cz*sy*cx + sz*cy*sx
    //   z = sz*cy*cx - cz*sy*sx
    Double4 q;
    q.w = cz * cy * cx + sz * sy * sx;
    q.x = cz * cy * sx - sz * sy * cx;
    q.y = cz * sy * cx + sz * cy * sx;
    q.z = sz * cy * cx - cz * sy * sx;
    return q;
}

// Extract XYZ euler (degrees) from a unit quaternion in the Maya rotate
// convention.  Matches the Bullet-derived version exactly — same matrix
// elements as btMatrix3x3(q) (Bullet's setRotation convention), same
// extraction formulas:
//   sin(ry) = -m[2][0];  rx = atan2(m[2][1], m[2][2]);  rz = atan2(m[1][0], m[0][0])
inline void quatToEulerXYZDegrees(const Double4& qIn, Double3& out)
{
    // Normalize first — matches btMatrix3x3(q), which scales by 2/|q|^2.
    const double inv =
        1.0 / std::sqrt(qIn.x * qIn.x + qIn.y * qIn.y + qIn.z * qIn.z + qIn.w * qIn.w);
    const double x = qIn.x * inv;
    const double y = qIn.y * inv;
    const double z = qIn.z * inv;
    const double w = qIn.w * inv;

    // Rotation matrix elements (Bullet setRotation convention, unit q):
    //   m[2][0] = 2(xz - yw); m[2][1] = 2(yz + xw); m[2][2] = 1 - 2(x^2+y^2)
    //   m[1][0] = 2(xy + zw); m[1][1] = 1 - 2(x^2+z^2)
    //   m[0][0] = 1 - 2(y^2+z^2); m[0][1] = 2(xy - zw); m[0][2] = 2(xz + yw)
    const double sinRy = -(2.0 * (x * z - y * w)); // -m[2][0]
    constexpr double kGimbalEps = 1e-6;
    if (sinRy < -1.0 + kGimbalEps || sinRy > 1.0 - kGimbalEps)
    {
        // Gimbal lock: ry = ±90°, rx/rz degenerate (only their combination is
        // well-defined).  Set rx = 0 and solve the combined term:
        //   ry=+90: m[0][1]=sin(rx-rz), m[0][2]=cos(rx-rz) -> rz = atan2(-m[0][1], m[0][2])
        //   ry=-90: m[0][1]=-sin(rx+rz), m[1][1]=cos(rx+rz) -> rz = atan2(-m[0][1], m[1][1])
        // (Earlier this extracted for M=Rx*Ry*Rz and flipped ry's sign, which
        // rotated every gimbal-locked body 180° and displaced the bones.)
        double ry;
        double rz;
        if (sinRy > 0.0) // m[2][0] < 0 -> ry = +90
        {
            ry = kPi / 2.0;
            rz = std::atan2(-2.0 * (x * y - z * w), 2.0 * (x * z + y * w));
        }
        else
        {
            ry = -kPi / 2.0;
            rz = std::atan2(-2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z));
        }
        out.x = 0.0;
        out.y = rad2deg(ry);
        out.z = rad2deg(rz);
        return;
    }
    const double rx = std::atan2(2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y));
    const double ry = std::asin(sinRy);
    const double rz = std::atan2(2.0 * (x * y + z * w), 1.0 - 2.0 * (y * y + z * z));
    out.x = rad2deg(rx);
    out.y = rad2deg(ry);
    out.z = rad2deg(rz);
}

// 4x4 ROW-vector matrix multiply: out = a * b (Maya convention, p' = p * M).
// Pure double arithmetic, so it is constexpr.
constexpr void rowMatrixMultiply(const Matrix4& a, const Matrix4& b, Matrix4& out)
{
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
            {
                s += a(r, k) * b(k, c);
            }
            out(r, c) = s;
        }
    }
}

} // namespace mmd::core::physics_math
