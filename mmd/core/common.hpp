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
 * (explicit btScalar() in simulation.cpp; storePose /
 * poseToTransform in bullet_bridge.hpp).  Do NOT switch vcpkg's bullet3
 * to double-precision — it would diverge from MMD's behavior.
 */

#pragma once

#include <cassert>

namespace mmd::core
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
    constexpr double* data() { return &x; }
    constexpr const double* data() const { return &x; }
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
    constexpr double* data() { return &x; }
    constexpr const double* data() const { return &x; }
};

// A 4x4 ROW-vector matrix (Maya convention: p' = p * M, row-major).  Named
// members (mRC = row R, column C) keep the matrices readable; operator()(r, c)
// serves the index loops in the math helpers.
struct Matrix4
{
    double m00 = 0.0;
    double m01 = 0.0;
    double m02 = 0.0;
    double m03 = 0.0;
    double m10 = 0.0;
    double m11 = 0.0;
    double m12 = 0.0;
    double m13 = 0.0;
    double m20 = 0.0;
    double m21 = 0.0;
    double m22 = 0.0;
    double m23 = 0.0;
    double m30 = 0.0;
    double m31 = 0.0;
    double m32 = 0.0;
    double m33 = 0.0;

    // Index access — m(r, c) reads row r, column c.  A switch on the flat
    // index keeps this strictly well-defined (a pointer-into-the-members
    // shortcut would be out-of-bounds access to a single object).
    constexpr double& operator()(int r, int c)
    {
        assert(r >= 0 && r < 4 && c >= 0 && c < 4);
        switch (r * 4 + c)
        {
        case 0:
            return m00;
        case 1:
            return m01;
        case 2:
            return m02;
        case 3:
            return m03;
        case 4:
            return m10;
        case 5:
            return m11;
        case 6:
            return m12;
        case 7:
            return m13;
        case 8:
            return m20;
        case 9:
            return m21;
        case 10:
            return m22;
        case 11:
            return m23;
        case 12:
            return m30;
        case 13:
            return m31;
        case 14:
            return m32;
        default:
            return m33;
        }
    }
    constexpr const double& operator()(int r, int c) const
    {
        assert(r >= 0 && r < 4 && c >= 0 && c < 4);
        switch (r * 4 + c)
        {
        case 0:
            return m00;
        case 1:
            return m01;
        case 2:
            return m02;
        case 3:
            return m03;
        case 4:
            return m10;
        case 5:
            return m11;
        case 6:
            return m12;
        case 7:
            return m13;
        case 8:
            return m20;
        case 9:
            return m21;
        case 10:
            return m22;
        case 11:
            return m23;
        case 12:
            return m30;
        case 13:
            return m31;
        case 14:
            return m32;
        default:
            return m33;
        }
    }

    // The identity matrix.
    static constexpr Matrix4 identity()
    {
        Matrix4 m;
        m.m00 = m.m11 = m.m22 = m.m33 = 1.0;
        return m;
    }
};

} // namespace mmd::core
