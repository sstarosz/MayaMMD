/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_display.hpp
 *
 * Everything the rigid-body viewport overrides share, in one module:
 *
 *   - the palette: every colour the physics display uses, each hue belonging
 *     to ONE channel (joint state, A/B relationship, motion axes, neutral);
 *   - collider outlines (drawColliderWire) and their A/B role variant
 *     (drawRoleColliderWire), fed by the role bits bodyRoles reports;
 *   - the lit stock surface shader (OpenPBR on Maya 2026+, Blinn before);
 *   - session-wide selection state — selected joints, the A/B roles of their
 *     bodies, isolate-select changes — resolved once per event instead of once
 *     per node, and one shared stock shader per colour (implemented in
 *     rigid_body_display.cpp);
 *   - which selected joint owns the fallback outline of a shared hidden body
 *     (highlightOwner).
 *
 * The Maya-free geometry helpers (freedom model, guide segments, sizes) stay
 * in mmd/core/joint_display_* so they can be unit-tested without the Maya SDK.
 */

#pragma once

#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MFloatVector.h>
#include <maya/MGeometry.h>
#include <maya/MGeometryRequirements.h>
#include <maya/MHWGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>
#include <maya/MPoint.h>
#include <maya/MShaderManager.h>
#include <maya/MTypes.h>
#include <maya/MUIDrawManager.h>
#include <maya/MVector.h>

#include <vector>

#include "rigid_body_shape.hpp"

namespace mmd::maya::rigid_body_display
{
// Every colour the physics viewport display uses, in one place.  Each hue
// belongs to ONE channel:
//
//   - joint STATE (hubs): what kind of motion the joint permits.  A joint
//     that both rotates and translates shows BOTH hues, in alternating
//     wedges, so this channel needs only two hues;
//   - RELATIONSHIP (selected joint): body A / body B.  A body that is A for
//     one selected joint and B for another is striped A/B, not a third hue;
//   - AXES (selected joint's motion guides): local X / Y / Z, Maya's RGB;
//   - guide neutral: boundaries, zero references, equilibrium marks.
//
// The four state and relationship hues sit about 90 degrees apart (rose,
// gold, cyan, violet), clear of the axis colours and Maya's selection green:
// every pair of them is at least 28 CIEDE2000 apart, also under simulated
// deuteranopia and protanopia.  Selection itself uses Maya's own selection
// colours (lead green, white).
namespace palette
{
// --- state (hub colour) ---
inline const MColor kWelded(0.70f, 0.70f, 0.73f);     // grey: no freedom
inline const MColor kRotates(0.35f, 0.95f, 0.95f);    // cyan
inline const MColor kTranslates(1.00f, 0.45f, 0.56f); // rose
// Inactive (constrains nothing): a hollow dashed hub.  The FORM, not the
// colour, tells it from a welded hub; the darker grey only lets it recede.
inline const MColor kInactive(0.52f, 0.52f, 0.55f);

// --- relationship (selected joint; also the bodies' highlight wires) ---
inline const MColor kBodyA(0.70f, 0.45f, 1.00f); // violet
inline const MColor kBodyB(1.00f, 0.82f, 0.00f); // gold
// Role bits: 1 = body A, 2 = body B, 3 = A for one selected joint, B for
// another (bodyRoles reports them).
constexpr unsigned int kRoleA = 1u;
constexpr unsigned int kRoleB = 2u;

// --- axes (selected joint's motion guides) ---
inline const MColor kAxis[3] = {MColor(1.0f, 0.25f, 0.22f), MColor(0.35f, 1.0f, 0.25f),
                                MColor(0.30f, 0.65f, 1.0f)};
inline const MColor kGuide(0.95f, 0.95f, 0.95f);

// Fallback when Maya's selection colour is near-black.
inline const MColor kSelectionFallback(1.0f, 0.60f, 0.0f);
} // namespace palette

// Fallback outline ownership for a body shared by several selected joints:
// the first candidate whose body is drawable draws it, and every candidate's
// role bits are merged (so a body that is A and B is striped).  Candidates are
// offered lead-first, so the lead keeps its own bodies.
struct HighlightCandidate
{
    unsigned int roles;
    bool drawable;
    bool self;
};
struct HighlightOwner
{
    unsigned int roles = 0;
    bool self = false;
};
inline HighlightOwner highlightOwner(const std::vector<HighlightCandidate>& candidates)
{
    HighlightOwner result;
    bool assigned = false;
    for (const auto& candidate : candidates)
    {
        result.roles |= candidate.roles;
        if (!assigned && candidate.drawable)
        {
            result.self = candidate.self;
            assigned = true;
        }
    }
    return result;
}

// A selected joint as the viewport needs it: its connected body shapes (null
// handles when an end is unconnected).
struct SelectedJoint
{
    MDagPath path;
    MObjectHandle joint;
    MObjectHandle solver;
    MObjectHandle bodyA;
    MObjectHandle bodyB;
};

void install();
void uninstall();

// Selected, drawn joints in lead-first order (Maya's lead is the LAST entry of
// the active selection list).
const std::vector<SelectedJoint>& selectedJoints();
bool isSelectedJoint(const MObject& joint);

// Maya's selection state of a DAG path: the lead object or an active one.
bool isSelected(const MDagPath& path);

// Maya's wireframe colour for the path, with palette::kSelectionFallback when
// the colour would be too dark to see.
MColor selectionColor(const MDagPath& path);

// Roles of a body shape across the selected joints: bit 0 = body A, bit 1 =
// body B (drawRoleColliderWire draws them).
unsigned int bodyRoles(const MObject& bodyShape);

// Shared stock shaders with this colour (owned here): lit surface for shaded
// hubs, flat colour for wireframe hubs, dashed flat colour for inactive hubs.
MHWRender::MShaderInstance* solidShader(const MColor& color);
MHWRender::MShaderInstance* wireShader(const MColor& color);
MHWRender::MShaderInstance* dashedWireShader(const MColor& color);

// ===========================================================================
// Render-item plumbing (both geometry overrides)
// ===========================================================================
// Fills a render item's vertex buffers from xyz-interleaved positions and
// normals, using whatever the item's shader declares and falling back to
// position + normal + uv when the requirements come back empty.  A
// solid-colour item needs no real UVs, so any texture semantic gets a zeroed
// float2 stream.
void fillMeshBuffers(const MHWRender::MGeometryRequirements& requirements,
                     const std::vector<float>& positions, const std::vector<float>& normals,
                     MHWRender::MGeometry& data);

// A UV sphere at the origin with smooth normals and OUTWARD winding, appended
// to the given streams.  The poles are single vertices, so no degenerate
// triangles are produced.  Returns the index of its first vertex; when `lines`
// is given it also receives the latitude/longitude wire over the same vertices.
unsigned int appendSphereMesh(double radius, int slices, int stacks, std::vector<float>& positions,
                              std::vector<float>& normals, std::vector<unsigned int>& indices,
                              std::vector<unsigned int>* lines = nullptr);
} // namespace mmd::maya::rigid_body_display

namespace mmd::maya
{
// A new shader instance; the caller releases it with MShaderManager::releaseShader.
inline MHWRender::MShaderInstance* createSurfaceShader(const MHWRender::MShaderManager& manager)
{
#if MAYA_API_VERSION >= 20260000
    return manager.getStockShader(MHWRender::MShaderManager::k3dIsotropicOpenPBRSurfaceShader);
#else
    return manager.getStockShader(MHWRender::MShaderManager::k3dBlinnShader);
#endif
}

inline void setSurfaceColor(MHWRender::MShaderInstance& shader, const MColor& color)
{
#if MAYA_API_VERSION >= 20260000
    shader.setParameter("baseColor", MFloatVector(color.r, color.g, color.b)); // float3
#else
    const float diffuse[4] = {color.r, color.g, color.b, 1.0f};
    shader.setParameter("diffuseColor", diffuse); // float4
#endif
}

// Shared native primitives: capsule height is the cylinder length, box sizes
// are half-extents. The optional matrix maps body space into the drawing space.
inline void drawColliderWire(MHWRender::MUIDrawManager& draw, const MPoint& size, short type,
                             const MMatrix& matrix = MMatrix::identity)
{
    const MPoint center = MPoint::origin * matrix;
    const MVector up = MVector(0, 1, 0) * matrix;
    const MVector right = MVector(1, 0, 0) * matrix;
    const MVector front = MVector(0, 0, 1) * matrix;
    switch (type)
    {
    case RigidBodyShape::kColliderBox:
        draw.box(center, up.normal(), right.normal(), size.x * right.length(), size.y * up.length(),
                 size.z * front.length(), false);
        break;
    case RigidBodyShape::kColliderSphere:
        draw.sphere(center, size.x * right.length(), false);
        break;
    default:
        draw.capsule(center, up.normal(), size.x * right.length(), size.y * up.length(), 12, 8,
                     false);
        break;
    }
}

// A collider's wire in its A/B role colour (`roles` as
// rigid_body_display::bodyRoles reports them: bit 0 = body A, bit 1 = body B);
// a body with both roles is striped.  The body override draws it for the
// bodies of the selected joints.
inline void drawRoleColliderWire(MHWRender::MUIDrawManager& draw, const MPoint& size, short type,
                                 unsigned int roles, const MMatrix& matrix = MMatrix::identity)
{
    namespace palette = rigid_body_display::palette;
    draw.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    draw.setColor((roles & palette::kRoleA) ? palette::kBodyA : palette::kBodyB);
    drawColliderWire(draw, size, type, matrix);
    if (roles == (palette::kRoleA | palette::kRoleB))
    {
        draw.setLineStyle(MHWRender::MUIDrawManager::kDashed);
        draw.setColor(palette::kBodyB);
        drawColliderWire(draw, size, type, matrix);
        draw.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    }
}
} // namespace mmd::maya
