/*
 * SPDX-License-Identifier: MIT
 *
 * joint_glyph_mesh.hpp
 *
 * Tiny CPU triangle-mesh builder for the joint hub (RigidBodyJointGeometryOverride).
 * The surface is LIT geometry with per-vertex normals — rendered with the same
 * stock surface shader as the per-body colliders (surface_shader.hpp).
 *
 * The mesh is built in the attachment frame and baked into the joint node's
 * LOCAL space on upload (MPxGeometryOverride::populateGeometry).
 *
 * Winding: triangles are emitted so that back-face culling shows the OUTSIDE of
 * the solid (see joint_glyph_mesh.cpp).  The same vertices also carry a line
 * list (the wireframe-mode hub).
 */

#pragma once

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <vector>

namespace mmd::maya::joint_glyph
{
// ---------------------------------------------------------------------------
// Mesh — xyz-interleaved positions + normals, a triangle index list and a
// line index list over the same vertices.
// ---------------------------------------------------------------------------
struct Mesh
{
    std::vector<float> positions; // xyz interleaved
    std::vector<float> normals;   // xyz interleaved
    std::vector<unsigned int> indices;
    std::vector<unsigned int> lines;

    // Append one vertex, returning its index.
    unsigned int vertex(const MPoint& p, const MVector& n);
    void tri(unsigned int a, unsigned int b, unsigned int c);
    void line(unsigned int a, unsigned int b);
};

// Sphere with smooth normals; its wire is the latitude and longitude lines.
void appendSphere(Mesh& mesh, const MPoint& center, double radius, int slices = 8, int stacks = 6);
} // namespace mmd::maya::joint_glyph
