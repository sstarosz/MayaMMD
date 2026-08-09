/**
 * @file common.hpp
 * @brief Shared value types for the Maya-free C++ core.
 *
 * The Maya API works with raw `double[3]` triples / `double[4]` quaternions,
 * but carrying those fixed C arrays through the code is error-prone —
 * Double3 / Double4 / Matrix4 give the tuples named fields, constexpr
 * construction, index access and a contiguous data() pointer for the API
 * boundary.
 *
 * PRECISION MODEL: the core's public value types are DOUBLE because the
 * consumer (Maya) is double throughout, and the euler/matrix math must not
 * lose precision.  The Bullet world inside Simulation is FLOAT (btScalar) —
 * MikuMikuDance itself runs Bullet in float, so float arithmetic is the
 * fidelity reference.  Conversions happen only at the core/Bullet boundary
 * (explicit btScalar() in simulation.cpp; storeAnchorPose /
 * anchorPoseToTransform in physics_math.hpp).  Do NOT switch vcpkg's bullet3
 * to double-precision — it would diverge from MMD's behavior.
 */

#pragma once

#include <cassert>

namespace mmd
{
namespace core
{

struct Double3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Double3() = default;
    constexpr Double3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // Index access for the flag-parsing loops (i in 0..2).  Out-of-range
    // indices are a programming error — assert before the default case.
    constexpr double& operator[](int i)
    {
        assert(i >= 0 && i < 3);
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        default:
            return z;
        }
    }
    constexpr const double& operator[](int i) const
    {
        assert(i >= 0 && i < 3);
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        default:
            return z;
        }
    }

    // Contiguous storage for the Maya API (setValue3Double, matrices, ...).
    double* data() { return &x; }
    const double* data() const { return &x; }
};

// A `double[4]` tuple — quaternion convention {x, y, z, w}.
struct Double4
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;

    constexpr Double4() = default;
    constexpr Double4(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}

    // Index access (i in 0..3).  Out-of-range indices are a programming
    // error — assert before the default case.
    constexpr double& operator[](int i)
    {
        assert(i >= 0 && i < 4);
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            return w;
        }
    }
    constexpr const double& operator[](int i) const
    {
        assert(i >= 0 && i < 4);
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            return w;
        }
    }

    // Contiguous storage for the Maya API / Bullet boundary.
    double* data() { return &x; }
    const double* data() const { return &x; }
};

// A 4x4 ROW-vector matrix (Maya convention: p' = p * M, row-major).  Wraps
// the raw double[4][4] so the math helpers never pass bare C arrays around.
// m[r][c] reads row r, column c; the storage is zero-initialized.
struct Matrix4
{
    double data[4][4] = {};

    // Row access — m[r][c] reads row r, column c.
    double* operator[](int r) { return data[r]; }
    const double* operator[](int r) const { return data[r]; }

    // The identity matrix.
    static Matrix4 identity()
    {
        Matrix4 m;
        m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0;
        return m;
    }
};

} // namespace core
} // namespace mmd
