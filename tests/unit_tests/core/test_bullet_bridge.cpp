/*
 * SPDX-License-Identifier: MIT
 *
 * test_bullet_bridge.cpp
 *
 * Unit tests for the Bullet-facing conversions in
 * mmd/core/bullet_bridge.hpp.  This is the ONE core header that exposes
 * Bullet types, so this file (and its target) needs Bullet — unlike
 * test_physics_math.cpp / test_rigid_body_simulation.cpp, which stay Bullet-free.
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "bullet_bridge.hpp"

#include <cmath>

using Catch::Approx;
using namespace mmd::core::physics_math;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::Matrix4;

namespace
{

void requireVecClose(const btVector3& a, const btVector3& b, double tol = 1e-4)
{
    for (int i = 0; i < 3; ++i)
    {
        REQUIRE(a[i] == Approx(b[i]).margin(tol));
    }
}

void requireTransformClose(const btTransform& a, const btTransform& b, double tol = 1e-4)
{
    requireVecClose(a.getOrigin(), b.getOrigin(), tol);
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(a.getBasis()[r][c] == Approx(b.getBasis()[r][c]).margin(tol));
        }
    }
}

} // namespace

TEST_CASE("transformFromRest places the rest pose", "[bridge]")
{
    const Double3 pos(1.0, -2.0, 3.0);
    const Double3 rot(0.0, 0.0, 0.0);
    btTransform t = transformFromRest(pos, rot);
    requireVecClose(t.getOrigin(), btVector3(1.0, -2.0, 3.0));
    Double3 out;
    quatToEulerXYZDegrees(eulerDegreesToQuat(0.0, 0.0, 0.0), out);
    REQUIRE(out.x == Approx(0.0).margin(1e-4));
    REQUIRE(out.y == Approx(0.0).margin(1e-4));
    REQUIRE(out.z == Approx(0.0).margin(1e-4));
}

TEST_CASE("doubleMatrixToBtTransform converts identity", "[bridge]")
{
    Matrix4 m;
    m(0, 0) = m(1, 1) = m(2, 2) = m(3, 3) = 1.0;
    btTransform t = doubleMatrixToBtTransform(m);
    requireVecClose(t.getOrigin(), btVector3(0, 0, 0));
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(t.getBasis()[r][c] == Approx(r == c ? 1.0 : 0.0).margin(1e-6));
        }
    }
}

TEST_CASE("doubleMatrixToBtTransform transposes Maya row matrices", "[bridge]")
{
    // A Maya ROW matrix for a 90 deg X rotation: M_row = (Rz*Ry*Rx)^T = Rx^T,
    // i.e. rows (1,0,0), (0,0,1), (0,-1,0).  Bullet's COLUMN matrix must come
    // out as Rx(90) — +90 deg about +X maps +Y -> +Z and +Z -> -Y, so the
    // columns are (1,0,0), (0,0,1), (0,-1,0).
    Matrix4 m;
    m(3, 3) = 1.0;
    m(0, 0) = 1.0;
    m(1, 2) = 1.0;
    m(2, 1) = -1.0;

    btTransform t = doubleMatrixToBtTransform(m);
    requireVecClose(t.getBasis().getColumn(0), btVector3(1, 0, 0));
    requireVecClose(t.getBasis().getColumn(1), btVector3(0, 0, 1));
    requireVecClose(t.getBasis().getColumn(2), btVector3(0, -1, 0));
    requireVecClose(t.getOrigin(), btVector3(0, 0, 0));
}

TEST_CASE("storePose / poseToTransform round-trip a transform", "[bridge]")
{
    const Double3 pos(1.0, 2.0, 3.0);
    const Double3 rot(15.0, -30.0, 45.0);
    btTransform src = transformFromRest(pos, rot);

    Double3 sp;
    Double4 sq = Double4(0.0, 0.0, 0.0, 1.0);
    storePose(sp, sq, src);
    btTransform rebuilt = poseToTransform(sp, sq);
    requireTransformClose(rebuilt, src);
}

TEST_CASE("btTransformToRowMatrix is the exact inverse of doubleMatrixToBtTransform", "[bridge]")
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
    REQUIRE(row(3, 0) == Approx(1.5).margin(1e-5));
    REQUIRE(row(3, 1) == Approx(-2.0).margin(1e-5));
    REQUIRE(row(3, 2) == Approx(3.25).margin(1e-5));
    REQUIRE(row(0, 3) == Approx(0.0).margin(1e-6));
    REQUIRE(row(1, 3) == Approx(0.0).margin(1e-6));
    REQUIRE(row(2, 3) == Approx(0.0).margin(1e-6));
    REQUIRE(row(3, 3) == Approx(1.0).margin(1e-6));
}

TEST_CASE("rowMatrixMultiply agrees with the Bullet compose", "[bridge]")
{
    // T(1,2,3) * Rx(90) — the row product transposed must equal the column
    // product R^T * T^T computed by Bullet.
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

    btTransform tt = doubleMatrixToBtTransform(t);
    btTransform rt = doubleMatrixToBtTransform(r);
    btTransform prod = rt * tt; // column-vector: R^T * T^T
    Matrix4 prodRow;
    btTransformToRowMatrix(prod, prodRow);
    for (int rc = 0; rc < 16; ++rc)
    {
        REQUIRE(prodRow(rc / 4, rc % 4) == Approx(out(rc / 4, rc % 4)).margin(1e-5));
    }
}
