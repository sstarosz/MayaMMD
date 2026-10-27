/*
 * SPDX-License-Identifier: MIT
 *
 * joint_glyph_mesh.cpp — see joint_glyph_mesh.hpp.
 */

#include "joint_glyph_mesh.hpp"

#include <cmath>

namespace mmd::maya::joint_glyph
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Stitch a grid of rows (each `columns + 1` vertices, the last repeating the
// first position) into quads.  Rows advance the way a UV sphere's rows do
// (+Y pole downwards on the outer side), which makes these triangles face
// outward — see appendSphere.
void stitch(Mesh& mesh, const std::vector<std::vector<unsigned int>>& rows, int columns)
{
    for (std::size_t i = 0; i + 1 < rows.size(); ++i)
    {
        for (int j = 0; j < columns; ++j)
        {
            const auto col = static_cast<std::size_t>(j);
            const unsigned int a = rows[i][col];
            const unsigned int d = rows[i][col + 1];
            const unsigned int b = rows[i + 1][col];
            const unsigned int c = rows[i + 1][col + 1];
            mesh.tri(a, d, b);
            mesh.tri(b, d, c);
        }
    }
}
} // namespace

unsigned int Mesh::vertex(const MPoint& p, const MVector& n)
{
    const unsigned int i = static_cast<unsigned int>(positions.size() / 3);
    positions.insert(positions.end(),
                     {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
    normals.insert(normals.end(),
                   {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)});
    return i;
}

void Mesh::tri(unsigned int a, unsigned int b, unsigned int c)
{
    indices.insert(indices.end(), {a, b, c});
}

void Mesh::line(unsigned int a, unsigned int b)
{
    lines.insert(lines.end(), {a, b});
}

void appendSphere(Mesh& mesh, const MPoint& center, double radius, int slices, int stacks)
{
    if (radius <= 0.0 || slices < 3 || stacks < 2)
        return;

    // Interior rings; the poles are single vertices, so a UV sphere emits no
    // degenerate (zero-area) triangles at the caps.  Winding: outward — the
    // collider override's sphere (RigidBodyShapeGeometryOverride::buildSphere)
    // emits the same triangles with the opposite winding, i.e. inward.
    const auto ringVertex = [&](int i, int j)
    {
        const double phi = kPi * i / stacks;
        const double y = std::cos(phi);
        const double ringR = std::sin(phi);
        const double theta = kTwoPi * j / slices;
        const MVector dir(ringR * std::cos(theta), y, ringR * std::sin(theta));
        return mesh.vertex(center + dir * radius, dir);
    };

    const unsigned int top = ringVertex(0, 0);         // +Y pole
    const unsigned int bottom = ringVertex(stacks, 0); // -Y pole
    std::vector<std::vector<unsigned int>> rows(static_cast<std::size_t>(stacks) - 1);
    for (int i = 1; i < stacks; ++i)
    {
        std::vector<unsigned int>& row = rows[static_cast<std::size_t>(i) - 1];
        row.reserve(static_cast<std::size_t>(slices) + 1);
        for (int j = 0; j <= slices; ++j)
            row.push_back(ringVertex(i, j));
    }

    for (int j = 0; j < slices; ++j)
    {
        const auto col = static_cast<std::size_t>(j);
        mesh.tri(top, rows[0][col + 1], rows[0][col]);
        const std::vector<unsigned int>& last = rows.back();
        mesh.tri(bottom, last[col], last[col + 1]);
    }
    stitch(mesh, rows, slices);

    // Wire: every latitude ring, and every meridian from pole to pole.
    for (int j = 0; j < slices; ++j)
    {
        const auto col = static_cast<std::size_t>(j);
        mesh.line(top, rows[0][col]);
        for (std::size_t i = 0; i < rows.size(); ++i)
        {
            mesh.line(rows[i][col], rows[i][col + 1]);
            if (i + 1 < rows.size())
                mesh.line(rows[i][col], rows[i + 1][col]);
        }
        mesh.line(rows.back()[col], bottom);
    }
}
} // namespace mmd::maya::joint_glyph
