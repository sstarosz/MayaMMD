/*
 * SPDX-License-Identifier: MIT
 *
 * common.hpp
 *
 * Shared value types for the Maya-free C++ core (and the native MMD commands
 * that use it).  The Maya API works with raw `double[3]` triples / `double[4]`
 * quaternions, but carrying those fixed C arrays through the code is
 * error-prone — Double3 / Double4 give the tuples named fields, constexpr
 * construction, index access and a contiguous data() pointer for the API
 * boundary.
 */

#pragma once

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

    // Index access for the flag-parsing loops (i in 0..2).
    constexpr double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr const double& operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

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

    // Index access (i in 0..3).
    constexpr double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w)); }
    constexpr const double& operator[](int i) const
    {
        return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
    }

    // Contiguous storage for the Maya API / Bullet boundary.
    double* data() { return &x; }
    const double* data() const { return &x; }
};

} // namespace core
} // namespace mmd
