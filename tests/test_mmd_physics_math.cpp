/*
 * SPDX-License-Identifier: MIT
 *
 * test_mmd_physics_math.cpp
 *
 * Unit tests for the Maya-free physics math in
 * mmd/maya/nodes/mmd_physics_math.h.  These lock in the hard-won Maya
 * conventions (Euler order, row/column matrix transpose) that previously
 * caused "the anchor orientation mess" and the gimbal-lock bone displacement.
 *
 * Build + run (BUILD_TESTS is OFF by default; enable it for a dev build):
 *   cmake --preset maya2026-release -DBUILD_TESTS=ON
 *   cmake --build out/build/maya2026-release --config Release
 *   ctest --preset default -R mmd_tools_tests
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "mmd/maya/nodes/mmd_physics_math.h"

#include <cmath>
#include <random>

// Catch2 v3 keeps Approx in the Catch namespace (v2 had it global).
using Catch::Approx;
using namespace mmd_physics_math;

namespace
{

// Largest allowed error when round-tripping euler degrees through
// eulerDegreesToQuat -> quatToEulerXYZDegrees.  Bullet stores floats, so
// ~1e-3 degrees is plenty of headroom.
constexpr double kEulerToleranceDeg = 1e-3;

// Compare two angles modulo 360 (handles the +-180 wrap atan2 produces).
bool approxEuler(double a, double b, double tolDeg = kEulerToleranceDeg)
{
    double d = std::fmod(a - b, 360.0);
    if (d < -180.0)
        d += 360.0;
    if (d > 180.0)
        d -= 360.0;
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
        {0.0, 0.0, 0.0},
        {30.0, 0.0, 0.0},
        {0.0, -45.0, 0.0},
        {0.0, 0.0, 90.0},
        {10.0, 20.0, 30.0},
        {-170.0, 15.0, 120.0},
        {89.0, -89.0, 1.0},
        {45.0, 0.0, 180.0},
        {-180.0, 45.0, -180.0},
    };
    for (const auto& a : angles)
    {
        double out[3];
        quatToEulerXYZDegrees(eulerDegreesToQuat(a[0], a[1], a[2]), out);
        REQUIRE(approxEuler(out[0], a[0]));
        REQUIRE(approxEuler(out[1], a[1]));
        REQUIRE(approxEuler(out[2], a[2]));
    }
}

TEST_CASE("random euler round-trips (non-gimbal)", "[math]")
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> wide(-179.0, 179.0);
    std::uniform_real_distribution<double> narrow(-89.0, 89.0); // ry canonical range
    for (int i = 0; i < 500; ++i)
    {
        double rx = wide(rng);
        double ry = narrow(rng);
        double rz = wide(rng);
        double out[3];
        quatToEulerXYZDegrees(eulerDegreesToQuat(rx, ry, rz), out);
        REQUIRE(approxEuler(out[0], rx));
        REQUIRE(approxEuler(out[1], ry));
        REQUIRE(approxEuler(out[2], rz));
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
                double out[3];
                quatToEulerXYZDegrees(eulerDegreesToQuat(rx, ry, rz), out);
                // ry preserved exactly; rx forced to 0 by the convention.
                REQUIRE(out[1] == Approx(ry).margin(1e-3));
                REQUIRE(std::fabs(out[0]) < 1e-6);
                // Re-encoding the extracted angles must reproduce the SAME
                // matrix (rotation-equivalent, not euler-identical).
                btMatrix3x3 m1(eulerDegreesToQuat(rx, ry, rz));
                btMatrix3x3 m2(eulerDegreesToQuat(out[0], out[1], out[2]));
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        REQUIRE(m1[r][c] == Approx(m2[r][c]).margin(1e-3));
            }
        }
    }
}

TEST_CASE("doubleMatrixToBtTransform converts identity", "[math]")
{
    double m[4][4] = {};
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
    double m[4][4] = {};
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
    const double pos[3] = {1.0, -2.0, 3.0};
    const double rot[3] = {0.0, 0.0, 0.0};
    btTransform t = transformFromRest(pos, rot);
    requireVecClose(t.getOrigin(), btVector3(1.0, -2.0, 3.0));
    double out[3];
    quatToEulerXYZDegrees(t.getRotation(), out);
    REQUIRE(approxEuler(out[0], 0.0));
    REQUIRE(approxEuler(out[1], 0.0));
    REQUIRE(approxEuler(out[2], 0.0));
}

TEST_CASE("anchor pose stores and rebuilds the same transform", "[math]")
{
    const double pos[3] = {1.0, 2.0, 3.0};
    const double rot[3] = {15.0, -30.0, 45.0};
    btTransform src = transformFromRest(pos, rot);

    double sp[3] = {0, 0, 0};
    double sq[4] = {0, 0, 0, 1};
    storeAnchorPose(sp, sq, src);
    btTransform rebuilt = anchorPoseToTransform(sp, sq);
    requireTransformClose(rebuilt, src);
}
