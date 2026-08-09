/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_common.h
 *
 * Small shared value types for the native MMD commands.  The Maya API works
 * with raw `double[3]` triples (MFnNumericData, MTransformationMatrix, ...),
 * but carrying those fixed C arrays through the command implementations is
 * error-prone — Double3 gives the triples named fields, constexpr
 * construction, index access and a contiguous data() pointer for the API
 * boundary.
 */

#pragma once

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
