/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape_draw_override.cpp
 *
 * RigidBodyShapeDrawOverride — viewport draw override for one pmxRigidBodyShape.
 *
 * Draws the body's collider in the guide's local space (box / sphere /
 * capsule).  The guide transform is the body's rest pose, so the collider
 * sits at the local origin and follows the guide when the user moves it with
 * the Move tool.  Draw mode is the shape's `drawMode` attribute
 * (off / wire / solid / wire+solid); colour comes from the PMX collision
 * group (a 16-colour palette), kinematic bodies are dimmed, disabled bodies
 * are greyed, and the selected guide is highlighted.
 */

#include "rigid_body_shape_draw_override.hpp"

#include "rigid_body_shape.hpp"

#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFrameContext.h>
#include <maya/MGlobal.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MUIDrawManager.h>
#include <maya/MVector.h>

#include <array>

namespace
{
// PMX collision-group colour palette (group id -> RGB).  Group 0 is the
// default "red" the solver's bodies most often use; the rest follow a
// MMD/PMX-Editor-style hue wheel so neighbouring groups stay distinguishable.
constexpr std::array<std::array<float, 3>, 16> kGroupColors = {{
    {1.00f, 0.25f, 0.25f}, // 0 red
    {1.00f, 0.55f, 0.15f}, // 1 orange
    {1.00f, 0.85f, 0.15f}, // 2 yellow
    {0.45f, 1.00f, 0.25f}, // 3 lime
    // 4: PURPLE/ORCHID — a saturated hue chosen as the complement of the
    // (green) selection highlight, so it can never be confused with it, and
    // distinct from the palette's blue-violet (7) and pink (8/9/15) entries.
    // (Green was abandoned: both bright green and dark forest green still
    // read as "selected".)
    {0.70f, 0.25f, 0.70f}, // 4 orchid
    {0.20f, 0.90f, 1.00f}, // 5 cyan
    {0.30f, 0.45f, 1.00f}, // 6 blue
    {0.65f, 0.30f, 1.00f}, // 7 violet
    {1.00f, 0.30f, 0.85f}, // 8 magenta
    {0.90f, 0.20f, 0.50f}, // 9 pink
    {0.60f, 0.40f, 0.25f}, // 10 brown
    {0.55f, 0.55f, 0.55f}, // 11 grey
    {0.30f, 0.70f, 0.50f}, // 12 teal
    {0.90f, 0.75f, 0.40f}, // 13 tan
    {0.40f, 0.60f, 0.80f}, // 14 steel
    {0.80f, 0.50f, 0.60f}, // 15 rose
}};

// Fallback selection colour (only used if Maya's selection colour queries as
// near-black) — a bright orange-yellow that cannot collide with the palette.
constexpr float kSelectionR = 1.00f;
constexpr float kSelectionG = 0.60f;
constexpr float kSelectionB = 0.00f;

// Read a k3Double plug into an MPoint (x, y, z).
[[nodiscard]] MPoint readPoint3(const MPlug& plug)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = plug.asMDataHandle().asDouble3();
    return MPoint(v[0], v[1], v[2]);
}

// Draw one collider in the guide's local space.  Sizing mirrors the engine's
// applyShapeSize: box = PMX shape_size VERBATIM as the half-extent (full span
// = 2 × shape_size, matching MMD/btBoxShape), sphere = radius = size.x,
// capsule = radius size.x + cylindrical section length size.y.
// `solid` selects the filled vs. wireframe primitive (the same colour is
// applied by the caller either way).
void drawCollider(MHWRender::MUIDrawManager& drawMgr, bool solid, const MPoint& size,
                  short colliderType)
{
    switch (colliderType)
    {
    case RigidBodyShape::kColliderBox:
        // drawMgr.box's scale params are the box HALF-extents — pass the PMX
        // shape_size verbatim (the same convention the engine uses for
        // btBoxShape), so the drawn box matches MMD Editor.
        drawMgr.box(MPoint(0.0, 0.0, 0.0), MVector(0.0, 1.0, 0.0), MVector(1.0, 0.0, 0.0), size.x,
                    size.y, size.z, solid);
        break;
    case RigidBodyShape::kColliderSphere:
        drawMgr.sphere(MPoint(0.0, 0.0, 0.0), size.x, solid);
        break;
    default: // kColliderCapsule
        drawMgr.capsule(MPoint(0.0, 0.0, 0.0), MVector(0.0, 1.0, 0.0), size.x, size.y, 12, 8,
                        solid);
        break;
    }
}
} // namespace

// ===========================================================================
// Lifecycle / registration
// ===========================================================================
MHWRender::MPxDrawOverride* RigidBodyShapeDrawOverride::creator(const MObject& obj)
{
    return new RigidBodyShapeDrawOverride(obj);
}

RigidBodyShapeDrawOverride::RigidBodyShapeDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, nullptr)
{
}

RigidBodyShapeDrawOverride::~RigidBodyShapeDrawOverride() = default;

MHWRender::DrawAPI RigidBodyShapeDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile;
}

bool RigidBodyShapeDrawOverride::isBounded(const MDagPath& /*objPath*/,
                                           const MDagPath& /*cameraPath*/) const
{
    // Not a "real" bounded object — the collider's own box is small and the
    // override draws it directly; return false so VP2 does not cull it.
    return false;
}

bool RigidBodyShapeDrawOverride::hasUIDrawables() const
{
    return true;
}

MUserData* RigidBodyShapeDrawOverride::prepareForDraw(
    const MDagPath& /*objPath*/, const MDagPath& /*cameraPath*/,
    const MHWRender::MFrameContext& /*frameContext*/, MUserData* oldData)
{
    // All drawing state is read live in addUIDrawables, so there is no
    // per-frame data to (re)build; hand the old data back unchanged.
    return oldData;
}

// ===========================================================================
// Draw
// ===========================================================================
void RigidBodyShapeDrawOverride::addUIDrawables(const MDagPath& objPath,
                                                MHWRender::MUIDrawManager& drawMgr,
                                                const MHWRender::MFrameContext& /*frameContext*/,
                                                const MUserData* /*data*/)
{
    // Read the body data straight from the shape node.
    MStatus stat;
    MFnDependencyNode fn(objPath.node());
    const MPlug typePlug = fn.findPlug(RigidBodyShape::aBodyColliderType, true, &stat);
    const MPlug sizePlug = fn.findPlug(RigidBodyShape::aBodyShapeSize, true, &stat);
    const MPlug groupPlug = fn.findPlug(RigidBodyShape::aBodyGroupId, true, &stat);
    const MPlug modePlug = fn.findPlug(RigidBodyShape::aDrawMode, true, &stat);
    const MPlug pmPlug = fn.findPlug(RigidBodyShape::aBodyPhysicsMode, true, &stat);
    const MPlug enabledPlug = fn.findPlug(RigidBodyShape::aBodyEnabled, true, &stat);

    if (sizePlug.isNull() || modePlug.isNull())
        return;

    const short drawMode = static_cast<short>(modePlug.asShort());
    if (drawMode == RigidBodyShape::kDrawOff)
        return;
    const bool enabled = enabledPlug.isNull() ? true : enabledPlug.asBool();
    const short colliderType = static_cast<short>(typePlug.asShort());
    const MPoint size = readPoint3(sizePlug);

    // Colour: group palette, kinematic dimmed 0.6x, disabled greyed.
    const short group = static_cast<short>(groupPlug.asShort());
    const int g = (group >= 0 && group < 16) ? group : 0;
    const bool kinematic = pmPlug.isNull() ? false : (pmPlug.asShort() == 0); // FollowBone
    float cr = kGroupColors.at(g)[0];
    float cg = kGroupColors.at(g)[1];
    float cb = kGroupColors.at(g)[2];
    if (!enabled)
    {
        cr = 0.45f;
        cg = 0.45f;
        cb = 0.45f;
    }
    else if (kinematic)
    {
        cr *= 0.6f;
        cg *= 0.6f;
        cb *= 0.6f;
    }

    // Selection highlight (active / lead) — use Maya's STANDARD selection
    // colour (the current theme's selection highlight, via wireframeColor)
    // rather than a hardcoded colour.  The group palette is chosen so no
    // entry is confused with it (group 4 is a dark forest green for this
    // reason).
    const MHWRender::DisplayStatus status = MHWRender::MGeometryUtilities::displayStatus(objPath);
    const bool selected = (status == MHWRender::kLead || status == MHWRender::kActive);
    if (selected)
    {
        const MColor sel = MHWRender::MGeometryUtilities::wireframeColor(objPath);
        cr = sel.r;
        cg = sel.g;
        cb = sel.b;
        // Guard: a near-black selection colour would hide the guide — fall
        // back to a bright orange that cannot match the palette.
        if (cr + cg + cb < 0.01f)
        {
            cr = kSelectionR;
            cg = kSelectionG;
            cb = kSelectionB;
        }
    }

    const bool drawSolid =
        (drawMode == RigidBodyShape::kDrawSolid || drawMode == RigidBodyShape::kDrawWireSolid);
    const bool drawWire =
        (drawMode == RigidBodyShape::kDrawWire || drawMode == RigidBodyShape::kDrawWireSolid);

    if (drawSolid)
    {
        drawMgr.beginDrawable();
        drawMgr.setColor(MColor(cr, cg, cb));
        drawCollider(drawMgr, /*solid=*/true, size, colliderType);
        drawMgr.endDrawable();
    }
    if (drawWire)
    {
        drawMgr.beginDrawable();
        drawMgr.setColor(MColor(cr, cg, cb));
        drawCollider(drawMgr, /*solid=*/false, size, colliderType);
        drawMgr.endDrawable();
    }
}
