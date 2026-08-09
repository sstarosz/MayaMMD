/*
 * SPDX-License-Identifier: MIT
 *
 * test_physics_math.cpp
 *
 * Unit tests for the Bullet-free pure math in mmd/core/physics_math.hpp.
 * These lock in the hard-won Maya conventions (Euler order, row/column
 * matrix transpose) that previously caused "the anchor orientation mess" and
 * the gimbal-lock bone displacement.
 *
 * No Bullet types appear in this file — the Bullet-facing conversions are
 * tested separately in test_bullet_bridge.cpp.
 *
 * Build + run (BUILD_TESTS is OFF by default; enable it for a dev build):
 *   cmake --preset maya2026-release -DBUILD_TESTS=ON
 *   cmake --build out/build/maya2026-release --config Release
 *   ctest --preset default -E "maya-integration"
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "physics_math.hpp"

#include <cmath>
#include <random>

// Catch2 v3 keeps Approx in the Catch namespace (v2 had it global).
using Catch::Approx;
using namespace mmd::core::physics_math;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::Matrix4;

namespace
{

// Largest allowed error when round-tripping euler degrees through
// eulerDegreesToQuat -> quatToEulerXYZDegrees.
constexpr double kEulerToleranceDeg = 1e-3;
constexpr double kFullCircleDeg = 360.0; // angle-comparison wrap
constexpr double kHalfCircleDeg = 180.0; // +-180 wrap atan2 produces

// Compare two angles modulo 360 (handles the +-180 wrap atan2 produces).
bool approxEuler(double actual, double expected, double tolDeg = kEulerToleranceDeg)
{
    double d = std::fmod(actual - expected, kFullCircleDeg);
    if (d < -kHalfCircleDeg)
    {
        d += kFullCircleDeg;
    }
    if (d > kHalfCircleDeg)
    {
        d -= kFullCircleDeg;
    }
    return std::fabs(d) <= tolDeg;
}

// Two unit quaternions represent the same rotation when their 4D dot product
// is +-1 (q and -q encode the same rotation).
void requireSameRotation(const Double4& a, const Double4& b, double tol = 1e-6)
{
    const double dot = (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
    REQUIRE(std::fabs(std::fabs(dot) - 1.0) <= tol);
}

} // namespace

TEST_CASE("deg2rad / rad2deg are exact inverses", "[math]")
{
    for (double d : {0.0, 30.0, -45.0, 90.0, 180.0, 360.0, 123.456})
    {
        REQUIRE(rad2deg(deg2rad(d)) == Approx(d).margin(1e-9));
    }
}

TEST_CASE("eulerDegreesToQuat rotates about the expected single axes", "[math]")
{
    {
        Double4 q = eulerDegreesToQuat(30.0, 0.0, 0.0);
        REQUIRE(q.x == Approx(std::sin(deg2rad(15.0))).margin(1e-4));
        REQUIRE(q.w == Approx(std::cos(deg2rad(15.0))).margin(1e-4));
        REQUIRE(std::fabs(q.y) < 1e-4);
        REQUIRE(std::fabs(q.z) < 1e-4);
    }
    {
        Double4 q = eulerDegreesToQuat(0.0, 45.0, 0.0);
        REQUIRE(q.y == Approx(std::sin(deg2rad(22.5))).margin(1e-4));
        REQUIRE(q.w == Approx(std::cos(deg2rad(22.5))).margin(1e-4));
        REQUIRE(std::fabs(q.x) < 1e-4);
        REQUIRE(std::fabs(q.z) < 1e-4);
    }
    {
        Double4 q = eulerDegreesToQuat(0.0, 0.0, 90.0);
        REQUIRE(q.z == Approx(std::sin(deg2rad(45.0))).margin(1e-4));
        REQUIRE(q.w == Approx(std::cos(deg2rad(45.0))).margin(1e-4));
        REQUIRE(std::fabs(q.x) < 1e-4);
        REQUIRE(std::fabs(q.y) < 1e-4);
    }
}

TEST_CASE("euler round-trips through the Maya XYZ convention", "[math]")
{
    // ry must stay within [-90, 90] — that is the canonical range of the XYZ
    // extraction (asin).  rx/rz may be anywhere in [-180, 180] (atan2).
    const double angles[][3] = {
        {0.0, 0.0, 0.0},    {30.0, 0.0, 0.0},   {0.0, -45.0, 0.0},
        {0.0, 0.0, 90.0},   {10.0, 20.0, 30.0}, {-170.0, 15.0, 120.0},
        {89.0, -89.0, 1.0}, {45.0, 0.0, 180.0}, {-180.0, 45.0, -180.0},
    };
    for (const auto& a : angles)
    {
        Double3 out;
        quatToEulerXYZDegrees(eulerDegreesToQuat(a[0], a[1], a[2]), out);
        REQUIRE(approxEuler(out.x, a[0]));
        REQUIRE(approxEuler(out.y, a[1]));
        REQUIRE(approxEuler(out.z, a[2]));
    }
}

TEST_CASE("random euler round-trips (non-gimbal)", "[math]")
{
    // Fixed seed keeps the round-trip test deterministic across runs.
    std::mt19937 rng(12345); // NOLINT(bugprone-random-generator-seed)
    std::uniform_real_distribution<double> wide(-179.0, 179.0);
    std::uniform_real_distribution<double> narrow(-89.0, 89.0); // ry canonical range
    for (int i = 0; i < 500; ++i)
    {
        const double rx = wide(rng);
        const double ry = narrow(rng);
        const double rz = wide(rng);
        Double3 out;
        quatToEulerXYZDegrees(eulerDegreesToQuat(rx, ry, rz), out);
        REQUIRE(approxEuler(out.x, rx));
        REQUIRE(approxEuler(out.y, ry));
        REQUIRE(approxEuler(out.z, rz));
    }
}

TEST_CASE("gimbal lock (ry = +-90) extracts a stable representation", "[math]")
{
    for (double ry : {-90.0, 90.0})
    {
        for (double rx : {-30.0, 10.0, 75.0})
        {
            for (double rz : {-45.0, 20.0})
            {
                Double3 out;
                quatToEulerXYZDegrees(eulerDegreesToQuat(rx, ry, rz), out);
                // ry preserved exactly; rx forced to 0 by the convention.
                REQUIRE(out.y == Approx(ry).margin(1e-3));
                REQUIRE(std::fabs(out.x) < 1e-6);
                // Re-encoding the extracted angles must reproduce the SAME
                // rotation (rotation-equivalent, not euler-identical).
                requireSameRotation(eulerDegreesToQuat(rx, ry, rz),
                                    eulerDegreesToQuat(out.x, out.y, out.z));
            }
        }
    }
}

TEST_CASE("rowMatrixMultiply composes 4x4 row-vector matrices", "[math]")
{
    // T(1,2,3) * Rx(90) in row-vector convention: translation (1,2,3), basis
    // (Rx^T).  Verify against explicit multiplication.
    Matrix4 t;
    t(0, 0) = t(1, 1) = t(2, 2) = t(3, 3) = 1.0;
    t(3, 0) = 1.0;
    t(3, 1) = 2.0;
    t(3, 2) = 3.0;

    Matrix4 r;
    r(3, 3) = 1.0;
    r(0, 0) = 1.0;
    r(1, 2) = 1.0;
    r(2, 1) = -1.0; // Rx(90) as a Maya ROW matrix (transpose of the column form)

    Matrix4 out;
    rowMatrixMultiply(t, r, out);

    // T(1,2,3) * Rx(90): the translation is ROTATED by the rotation in the
    // row-vector convention, so it lands on (1, -3, 2), not (1, 2, 3).
    REQUIRE(out(0, 0) == Approx(1.0).margin(1e-6));
    REQUIRE(out(1, 2) == Approx(1.0).margin(1e-6));
    REQUIRE(out(2, 1) == Approx(-1.0).margin(1e-6));
    REQUIRE(out(3, 0) == Approx(1.0).margin(1e-6));
    REQUIRE(out(3, 1) == Approx(-3.0).margin(1e-6));
    REQUIRE(out(3, 2) == Approx(2.0).margin(1e-6));
}

TEST_CASE("rowMatrixMultiply inverse composes to identity", "[math]")
{
    Matrix4 a;
    a(0, 0) = 0.8;
    a(0, 1) = -0.6;
    a(1, 0) = 0.6;
    a(1, 1) = 0.8;
    a(2, 2) = 1.0;
    a(3, 3) = 1.0;
    a(3, 0) = 4.0;
    a(3, 1) = -7.0;
    a(3, 2) = 2.0;

    Matrix4 inv;
    inv(0, 0) = 0.8;
    inv(0, 1) = 0.6;
    inv(1, 0) = -0.6;
    inv(1, 1) = 0.8;
    inv(2, 2) = 1.0;
    inv(3, 3) = 1.0;
    // Row-vector inverse translation: -t * R^T = -((4,-7,2) * R^T)
    // = -(7.4, -3.2, 2) = (-7.4, 3.2, -2.0).
    inv(3, 0) = -7.4;
    inv(3, 1) = 3.2;
    inv(3, 2) = -2.0;

    Matrix4 out;
    rowMatrixMultiply(a, inv, out);
    for (int rc = 0; rc < 16; ++rc)
    {
        const int r = rc / 4;
        const int c = rc % 4;
        REQUIRE(out(r, c) == Approx(r == c ? 1.0 : 0.0).margin(1e-6));
    }
}
