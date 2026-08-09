/*
 * SPDX-License-Identifier: MIT
 *
 * test_physics_math.cpp
 *
 * Unit tests for the Maya-free physics math in
 * mmd/core/physics_math.hpp.  These lock in the hard-won Maya
 * conventions (Euler order, row/column matrix transpose) that previously
 * caused "the anchor orientation mess" and the gimbal-lock bone displacement.
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
// eulerDegreesToQuat -> quatToEulerXYZDegrees.  Bullet stores floats, so
// ~1e-3 degrees is plenty of headroom.
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

void requireVecClose(const btVector3& a, const btVector3& b, double tol = 1e-4)
{
    for (int i = 0; i < 3; ++i)
        REQUIRE(a[i] == Approx(b[i]).margin(tol));
}

void requireTransformClose(const btTransform& a, const btTransform& b, double tol = 1e-4)
{
    requireVecClose(a.getOrigin(), b.getOrigin(), tol);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            REQUIRE(a.getBasis()[r][c] == Approx(b.getBasis()[r][c]).margin(tol));
}

} // namespace

TEST_CASE("deg2rad / rad2deg are exact inverses", "[math]")
{
    for (double d : {0.0, 30.0, -45.0, 90.0, 180.0, 360.0, 123.456})
        REQUIRE(rad2deg(deg2rad(d)) == Approx(d).margin(1e-9));
}

TEST_CASE("eulerDegreesToQuat rotates about the expected single axes", "[math]")
{
    {
        btQuaternion q = eulerDegreesToQuat(30.0, 0.0, 0.0);
        REQUIRE(q.getAngle() == Approx(deg2rad(30.0)).margin(1e-4));
        btVector3 a = q.getAxis();
        REQUIRE(a.x() == Approx(1.0).margin(1e-4));
        REQUIRE(std::fabs(a.y()) < 1e-4);
        REQUIRE(std::fabs(a.z()) < 1e-4);
    }
    {
        btQuaternion q = eulerDegreesToQuat(0.0, 45.0, 0.0);
        REQUIRE(q.getAngle() == Approx(deg2rad(45.0)).margin(1e-4));
        btVector3 a = q.getAxis();
        REQUIRE(a.y() == Approx(1.0).margin(1e-4));
        REQUIRE(std::fabs(a.x()) < 1e-4);
        REQUIRE(std::fabs(a.z()) < 1e-4);
    }
    {
        btQuaternion q = eulerDegreesToQuat(0.0, 0.0, 90.0);
        REQUIRE(q.getAngle() == Approx(deg2rad(90.0)).margin(1e-4));
        btVector3 a = q.getAxis();
        REQUIRE(a.z() == Approx(1.0).margin(1e-4));
        REQUIRE(std::fabs(a.x()) < 1e-4);
        REQUIRE(std::fabs(a.y()) < 1e-4);
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
        double rx = wide(rng);
        double ry = narrow(rng);
        double rz = wide(rng);
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
                // matrix (rotation-equivalent, not euler-identical).
                btMatrix3x3 m1(eulerDegreesToQuat(rx, ry, rz));
                btMatrix3x3 m2(eulerDegreesToQuat(out.x, out.y, out.z));
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        REQUIRE(m1[r][c] == Approx(m2[r][c]).margin(1e-3));
            }
        }
    }
}

TEST_CASE("doubleMatrixToBtTransform converts identity", "[math]")
{
    Matrix4 m;
    m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0;
    btTransform t = doubleMatrixToBtTransform(m);
    requireVecClose(t.getOrigin(), btVector3(0, 0, 0));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            REQUIRE(t.getBasis()[r][c] == Approx(r == c ? 1.0 : 0.0).margin(1e-6));
}

TEST_CASE("doubleMatrixToBtTransform transposes Maya row matrices", "[math]")
{
    // A Maya ROW matrix for a 90 deg X rotation: M_row = (Rz*Ry*Rx)^T = Rx^T,
    // i.e. rows (1,0,0), (0,0,1), (0,-1,0).  Bullet's COLUMN matrix must come
    // out as Rx(90) — +90 deg about +X maps +Y -> +Z and +Z -> -Y, so the
    // columns are (1,0,0), (0,0,1), (0,-1,0).
    Matrix4 m;
    m[3][3] = 1.0;
    m[0][0] = 1.0;
    m[1][2] = 1.0;
    m[2][1] = -1.0;

    btTransform t = doubleMatrixToBtTransform(m);
    requireVecClose(t.getBasis().getColumn(0), btVector3(1, 0, 0));
    requireVecClose(t.getBasis().getColumn(1), btVector3(0, 0, 1));
    requireVecClose(t.getBasis().getColumn(2), btVector3(0, -1, 0));
    requireVecClose(t.getOrigin(), btVector3(0, 0, 0));
}

TEST_CASE("transformFromRest places the rest pose", "[math]")
{
    const Double3 pos(1.0, -2.0, 3.0);
    const Double3 rot(0.0, 0.0, 0.0);
    btTransform t = transformFromRest(pos, rot);
    requireVecClose(t.getOrigin(), btVector3(1.0, -2.0, 3.0));
    Double3 out;
    quatToEulerXYZDegrees(t.getRotation(), out);
    REQUIRE(approxEuler(out.x, 0.0));
    REQUIRE(approxEuler(out.y, 0.0));
    REQUIRE(approxEuler(out.z, 0.0));
}

TEST_CASE("anchor pose stores and rebuilds the same transform", "[math]")
{
    const Double3 pos(1.0, 2.0, 3.0);
    const Double3 rot(15.0, -30.0, 45.0);
    btTransform src = transformFromRest(pos, rot);

    Double3 sp;
    Double4 sq = Double4(0.0, 0.0, 0.0, 1.0);
    storeAnchorPose(sp, sq, src);
    btTransform rebuilt = anchorPoseToTransform(sp, sq);
    requireTransformClose(rebuilt, src);
}

TEST_CASE("btTransformToRowMatrix is the exact inverse of doubleMatrixToBtTransform", "[math]")
{
    // A rotated + translated transform; round-trip must be exact.
    const Double3 pos(1.5, -2.0, 3.25);
    const Double3 rot(30.0, -45.0, 60.0);
    btTransform src = transformFromRest(pos, rot);

    Matrix4 row;
    btTransformToRowMatrix(src, row);
    btTransform rebuilt = doubleMatrixToBtTransform(row);
    requireTransformClose(rebuilt, src, 1e-5);

    // The row-vector translation lands in the LAST row (Maya convention).
    REQUIRE(row[3][0] == Approx(1.5).margin(1e-5));
    REQUIRE(row[3][1] == Approx(-2.0).margin(1e-5));
    REQUIRE(row[3][2] == Approx(3.25).margin(1e-5));
    REQUIRE(row[0][3] == Approx(0.0).margin(1e-6));
    REQUIRE(row[1][3] == Approx(0.0).margin(1e-6));
    REQUIRE(row[2][3] == Approx(0.0).margin(1e-6));
    REQUIRE(row[3][3] == Approx(1.0).margin(1e-6));
}

TEST_CASE("rowMatrixMultiply composes 4x4 row-vector matrices", "[math]")
{
    // T(1,2,3) * Rx(90) in row-vector convention: translation (1,2,3), basis
    // (Rx^T).  Verify against explicit multiplication.
    Matrix4 t;
    t[0][0] = t[1][1] = t[2][2] = t[3][3] = 1.0;
    t[3][0] = 1.0;
    t[3][1] = 2.0;
    t[3][2] = 3.0;

    Matrix4 r;
    r[3][3] = 1.0;
    r[0][0] = 1.0;
    r[1][2] = 1.0;
    r[2][1] = -1.0; // Rx(90) as a Maya ROW matrix (transpose of the column form)

    Matrix4 out;
    rowMatrixMultiply(t, r, out);

    // T(1,2,3) * Rx(90): the translation is ROTATED by the rotation in the
    // row-vector convention, so it lands on (1, -3, 2), not (1, 2, 3).
    REQUIRE(out[0][0] == Approx(1.0).margin(1e-6));
    REQUIRE(out[1][2] == Approx(1.0).margin(1e-6));
    REQUIRE(out[2][1] == Approx(-1.0).margin(1e-6));
    REQUIRE(out[3][0] == Approx(1.0).margin(1e-6));
    REQUIRE(out[3][1] == Approx(-3.0).margin(1e-6));
    REQUIRE(out[3][2] == Approx(2.0).margin(1e-6));

    // Associative vs the equivalent Bullet compose (transposed): the row
    // product T*R transposed must equal the column product R^T * T^T.
    btTransform tt = doubleMatrixToBtTransform(t);
    btTransform rt = doubleMatrixToBtTransform(r);
    btTransform prod = rt * tt; // column-vector: R^T * T^T
    Matrix4 prodRow;
    btTransformToRowMatrix(prod, prodRow);
    for (int rc = 0; rc < 16; ++rc)
        REQUIRE(prodRow[rc / 4][rc % 4] == Approx(out[rc / 4][rc % 4]).margin(1e-5));
}

TEST_CASE("rowMatrixMultiply inverse composes to identity", "[math]")
{
    Matrix4 a;
    a[0][0] = 0.8;
    a[0][1] = -0.6;
    a[1][0] = 0.6;
    a[1][1] = 0.8;
    a[2][2] = 1.0;
    a[3][3] = 1.0;
    a[3][0] = 4.0;
    a[3][1] = -7.0;
    a[3][2] = 2.0;

    Matrix4 inv;
    inv[0][0] = 0.8;
    inv[0][1] = 0.6;
    inv[1][0] = -0.6;
    inv[1][1] = 0.8;
    inv[2][2] = 1.0;
    inv[3][3] = 1.0;
    // Row-vector inverse translation: -t * R^T = -((4,-7,2) * R^T)
    // = -(7.4, -3.2, 2) = (-7.4, 3.2, -2.0).
    inv[3][0] = -7.4;
    inv[3][1] = 3.2;
    inv[3][2] = -2.0;

    Matrix4 out;
    rowMatrixMultiply(a, inv, out);
    for (int rc = 0; rc < 16; ++rc)
    {
        const int r = rc / 4;
        const int c = rc % 4;
        REQUIRE(out[r][c] == Approx(r == c ? 1.0 : 0.0).margin(1e-6));
    }
}
