/*
 * SPDX-License-Identifier: MIT
 *
 * physics_draw_override.cpp
 *
 * See physics_draw_override.h — viewport drawing for pmxPhysicsNode.
 *
 * DRAW SPACE: the Bullet world runs in WORLD space (the node's header and
 * builder both say the solver's own location never matters), so
 * collectDrawData() returns WORLD-space poses.  addUIDrawables() draws in the
 * locator's OBJECT space, so prepareForDraw() captures the node's
 * world-inverse matrix and every body is transformed world -> object space
 * before drawing (world * objWorldInverse, row-vector convention).
 */

#include "physics_draw_override.h"

#include <maya/MColor.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/M3dView.h>
#include <maya/MUIDrawManager.h>

#include "collider_geometry.hpp"
#include "physics_math.hpp"
#include "physics_node.h"

#include <algorithm>
#include <cmath>
#include <vector>

using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::Simulation;

namespace
{

// Draw data cached by prepareForDraw and consumed by addUIDrawables / userSelect.
// Reused across frames via the oldData pointer; Maya manages its lifetime.
class MMDPhysicsDrawData : public MUserData
{
  public:
    std::vector<PhysicsNode::DrawBody> bodies;
    PhysicsNode::DrawMode drawMode = PhysicsNode::kDrawWireframe;
    float drawOpacity = 1.0F;
    int selectedBodyIndex = -1; // from uiSelectedBodyIndex (may exceed bodies when stale)
    MMatrix worldInverse;       // node's world-inverse: world -> object space
};

// Collision-group palette — matches the PMX group colors that used to be
// assigned to the (now removed) guide mesh shaders (rigid_body_builder's
// _RIGID_BODY_GROUP_COLORS).  Groups 0-7 classic rainbow; 8-15 extended.
const MColor kGroupColors[16] = {
    MColor(0.90F, 0.10F, 0.10F), // 0  red
    MColor(0.10F, 0.75F, 0.15F), // 1  green
    MColor(0.15F, 0.35F, 0.95F), // 2  blue
    MColor(1.00F, 0.90F, 0.10F), // 3  yellow
    MColor(0.95F, 0.15F, 0.65F), // 4  magenta
    MColor(0.00F, 0.85F, 0.90F), // 5  cyan
    MColor(1.00F, 0.55F, 0.10F), // 6  orange
    MColor(0.50F, 0.10F, 0.90F), // 7  purple
    MColor(0.60F, 0.90F, 0.10F), // 8  lime
    MColor(1.00F, 0.35F, 0.50F), // 9  rose
    MColor(0.40F, 0.65F, 0.95F), // 10 sky blue
    MColor(1.00F, 0.65F, 0.80F), // 11 pink
    MColor(0.55F, 0.35F, 0.15F), // 12 brown
    MColor(0.80F, 0.60F, 0.95F), // 13 lavender
    MColor(0.10F, 0.70F, 0.55F), // 14 teal
    MColor(0.10F, 0.15F, 0.50F), // 15 navy
};

// Body color: unique per collision group, dimmed for kinematic (bone-following)
// colliders.  The picked body is drawn in the Maya selection/hilite color.
// (groupId is clamped to 0..15 before indexing — the lookup is bounds-safe.)
MColor bodyColor(const PhysicsNode::DrawBody& b)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    MColor c = kGroupColors[b.groupId % 16];
    if (b.kinematic)
        c *= 0.6F; // kinematic (bone-following) colliders drawn dimmer
    return c;
}

// Unit axes in the body frame (the collider primitives are axis-aligned in
// the body's local space; the override rotates them by the body quaternion).
const float kXAxis[3] = {1.0F, 0.0F, 0.0F};
const float kYAxis[3] = {0.0F, 1.0F, 0.0F};
const float kZeroLocal[3] = {0.0F, 0.0F, 0.0F};

// The Maya selection/hilite color (user configurable in Preferences).
MColor selectionColor()
{
    MStatus stat;
    const MColor c = M3dView::hiliteColor(&stat);
    return stat ? c : MColor(1.0F, 1.0F, 1.0F);
}

// Rotate v by unit quaternion q (x, y, z, w) — standard q*v*q^-1.  The
// quaternion comes from the node as doubles; the vertex stays float.  (The C
// array parameters are the Maya-draw convention — decay is unavoidable here.)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
void rotatePoint(const double q[4], const float v[3], float out[3])
{
    const double tx = 2.0 * ((q[1] * v[2]) - (q[2] * v[1]));
    const double ty = 2.0 * ((q[2] * v[0]) - (q[0] * v[2]));
    const double tz = 2.0 * ((q[0] * v[1]) - (q[1] * v[0]));
    out[0] = static_cast<float>(v[0] + (q[3] * tx) + ((q[1] * tz) - (q[2] * ty)));
    out[1] = static_cast<float>(v[1] + (q[3] * ty) + ((q[2] * tx) - (q[0] * tz)));
    out[2] = static_cast<float>(v[2] + (q[3] * tz) + ((q[0] * ty) - (q[1] * tx)));
}
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// Body point in object space: the body pose is WORLD space, so rotate the
// local vertex by the body quaternion, translate to the body center, then
// transform world -> object via the node's world-inverse (row-vector: p * M).
MPoint bodyPoint(const PhysicsNode::DrawBody& b, const float local[3], const MMatrix& worldInverse)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    float o[3];
    rotatePoint(b.quat, local, o);
    // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double world[4] = {b.pos[0] + o[0], b.pos[1] + o[1], b.pos[2] + o[2], 1.0};
    double object[4];
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    // p_object = p_world * worldInverse (row-vector convention).
    for (int r = 0; r < 4; ++r)
        object[r] = 0.0;
    for (int c = 0; c < 4; ++c)
    {
        for (int r = 0; r < 4; ++r)
            object[r] += world[c] * worldInverse(c, r);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
    return MPoint(object[0], object[1], object[2]);
}

// Engine primitive params from the PMX shape_size VERBATIM (the draw contract
// reads shapeSize directly; the box extents are full, so halve them).
mmd::core::collider_geometry::PrimitiveParams primitiveFor(const PhysicsNode::DrawBody& b)
{
    using namespace mmd::core::collider_geometry;
    Simulation::ColliderType engineType = Simulation::ColliderType::eBox;
    switch (b.colliderType)
    {
    case PhysicsNode::kColliderSphere:
        engineType = Simulation::ColliderType::eSphere;
        break;
    case PhysicsNode::kColliderCapsule:
        engineType = Simulation::ColliderType::eCapsule;
        break;
    case PhysicsNode::kColliderBox:
    default:
        engineType = Simulation::ColliderType::eBox;
        break;
    }
    return primitiveFromShapeSize(engineType,
                                  Double3(b.shapeSize[0], b.shapeSize[1], b.shapeSize[2]));
}

// 12 box edges as line endpoints (wireframe).  The local vertex arrays are
// the draw code's own small buffers — the decay to bodyPoint's const float*
// is intentional and bounds-safe (indexed only by the kEdges constants).
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
void addBoxEdges(const PhysicsNode::DrawBody& b, MPointArray& pts, const MMatrix& worldInverse)
{
    const mmd::core::collider_geometry::PrimitiveParams p = primitiveFor(b);
    const float hx = static_cast<float>(p.halfExtents.x);
    const float hy = static_cast<float>(p.halfExtents.y);
    const float hz = static_cast<float>(p.halfExtents.z);
    const float corners[8][3] = {
        {-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, -hy, hz}, {-hx, -hy, hz},
        {-hx, hy, -hz},  {hx, hy, -hz},  {hx, hy, hz},  {-hx, hy, hz},
    };
    static const int kEdges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top
        {0, 4}, {1, 5}, {2, 6}, {3, 7}, // vertical
    };
    for (const auto& e : kEdges)
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        pts.append(bodyPoint(b, corners[e[0]], worldInverse));
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        pts.append(bodyPoint(b, corners[e[1]], worldInverse));
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// Capsule (Y axis): two circles at ±h/2 plus 4 connecting lines (wireframe).
// Same intentional local-array decay as addBoxEdges.
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
void addCapsuleLines(const PhysicsNode::DrawBody& b, MPointArray& pts, const MMatrix& worldInverse)
{
    const mmd::core::collider_geometry::PrimitiveParams p = primitiveFor(b);
    const float r = static_cast<float>(p.radius);
    const float half = static_cast<float>(p.length) * 0.5F;
    constexpr int kSegs = 12;
    constexpr int kSpokes = 4;
    for (int s = 0; s < kSegs; ++s)
    {
        const float a0 = static_cast<float>(s * 2.0 * mmd::core::physics_math::kPi / kSegs);
        const float a1 = static_cast<float>((s + 1) * 2.0 * mmd::core::physics_math::kPi / kSegs);
        const float v0[3] = {std::cos(a0) * r, half, std::sin(a0) * r};
        const float v1[3] = {std::cos(a1) * r, half, std::sin(a1) * r};
        const float v2[3] = {std::cos(a0) * r, -half, std::sin(a0) * r};
        const float v3[3] = {std::cos(a1) * r, -half, std::sin(a1) * r};
        pts.append(bodyPoint(b, v0, worldInverse));
        pts.append(bodyPoint(b, v1, worldInverse));
        pts.append(bodyPoint(b, v2, worldInverse));
        pts.append(bodyPoint(b, v3, worldInverse));
    }
    for (int s = 0; s < kSpokes; ++s)
    {
        const float a = static_cast<float>(s * 2.0 * mmd::core::physics_math::kPi / kSpokes);
        const float top[3] = {std::cos(a) * r, half, std::sin(a) * r};
        const float bot[3] = {std::cos(a) * r, -half, std::sin(a) * r};
        pts.append(bodyPoint(b, top, worldInverse));
        pts.append(bodyPoint(b, bot, worldInverse));
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// Read the node's view-only draw attributes.
void readDrawState(const MObject& node, MMDPhysicsDrawData& data)
{
    MFnDependencyNode fn(node);
    MStatus stat;
    MPlug modePlug = fn.findPlug(PhysicsNode::aDrawMode, true, &stat);
    if (!modePlug.isNull())
        data.drawMode = static_cast<PhysicsNode::DrawMode>(modePlug.asShort());
    MPlug opacityPlug = fn.findPlug(PhysicsNode::aDrawOpacity, true, &stat);
    if (!opacityPlug.isNull())
        data.drawOpacity = opacityPlug.asFloat();
    MPlug selPlug = fn.findPlug(PhysicsNode::aUiSelectedBodyIndex, true, &stat);
    if (!selPlug.isNull())
        data.selectedBodyIndex = selPlug.asInt();
}

} // namespace

// ===========================================================================
// PhysicsDrawOverride
// ===========================================================================

PhysicsDrawOverride::PhysicsDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, nullptr)
{
}

MHWRender::MPxDrawOverride* PhysicsDrawOverride::creator(const MObject& obj)
{
    return new PhysicsDrawOverride(obj);
}

MHWRender::DrawAPI PhysicsDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kAllDevices;
}

bool PhysicsDrawOverride::hasUIDrawables() const
{
    return true; // addUIDrawables() queues the guide primitives
}

bool PhysicsDrawOverride::isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    (void) objPath;
    (void) cameraPath;
    return true;
}

MBoundingBox PhysicsDrawOverride::boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const
{
    (void) cameraPath;
    MObject node = objPath.node();
    MFnDependencyNode fn(node);
    if (fn.typeId() == PhysicsNode::kTypeId)
    {
        auto* physics = dynamic_cast<PhysicsNode*>(fn.userNode());
        if (physics != nullptr)
            return physics->boundingBox();
    }
    return MBoundingBox(MPoint(-1.0, -1.0, -1.0), MPoint(1.0, 1.0, 1.0));
}

MUserData* PhysicsDrawOverride::prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath,
                                               const MFrameContext& frameContext,
                                               MUserData* oldData)
{
    (void) cameraPath;
    (void) frameContext;
    MMDPhysicsDrawData* data = dynamic_cast<MMDPhysicsDrawData*>(oldData);
    if (data == nullptr)
        data = new MMDPhysicsDrawData();
    data->bodies.clear();

    // World -> object space for the draw pass.
    data->worldInverse = objPath.inclusiveMatrix().inverse();

    MObject node = objPath.node();
    MFnDependencyNode fn(node);
    if (fn.typeId() == PhysicsNode::kTypeId)
    {
        auto* physics = dynamic_cast<PhysicsNode*>(fn.userNode());
        if (physics != nullptr)
        {
            physics->collectDrawData(data->bodies);
            readDrawState(node, *data);
        }
    }
    return data;
}

void PhysicsDrawOverride::addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawManager,
                                         const MFrameContext& frameContext,
                                         const MUserData* userData)
{
    (void) objPath;
    (void) frameContext;
    const auto* data = dynamic_cast<const MMDPhysicsDrawData*>(userData);
    if (data == nullptr)
        return;
    if (data->drawMode == PhysicsNode::kDrawOff)
        return;

    const bool wire = data->drawMode == PhysicsNode::kDrawWireframe ||
                      data->drawMode == PhysicsNode::kDrawWireframeAndSolid;
    const bool solid = data->drawMode == PhysicsNode::kDrawSolid ||
                       data->drawMode == PhysicsNode::kDrawWireframeAndSolid;
    // drawOpacity scales the ALPHA of the solid fills (wireframe stays crisp).
    const float opacity = std::clamp(data->drawOpacity, 0.0F, 1.0F);
    const MColor selectedColor = selectionColor();

    drawManager.beginDrawable();
    for (size_t i = 0; i < data->bodies.size(); ++i)
    {
        const PhysicsNode::DrawBody& b = data->bodies[i];
        const bool isSelected = (static_cast<int>(i) == data->selectedBodyIndex);
        MColor color = isSelected ? selectedColor : bodyColor(b);
        if (solid)
            color.a = opacity;
        drawManager.setColor(color);
        if (isSelected)
            drawManager.setLineWidth(2.0F);

        if (b.colliderType == PhysicsNode::kColliderSphere)
        {
            const mmd::core::collider_geometry::PrimitiveParams p = primitiveFor(b);
            const MPoint center =
                bodyPoint(b, &kZeroLocal[0], data->worldInverse);
            if (solid)
                drawManager.sphere(center, p.radius, 12, 8, true);
            if (wire)
                drawManager.sphere(center, p.radius, 12, 8, false);
        }
        else if (b.colliderType == PhysicsNode::kColliderBox)
        {
            const mmd::core::collider_geometry::PrimitiveParams p = primitiveFor(b);
            if (solid)
            {
                // Orientation axes from the body quaternion (in object space,
                // the world-inverse drops the rotation too).
                // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
                const double q[4] = {b.quat[0], b.quat[1], b.quat[2], b.quat[3]};
                float up[3];
                float right[3];
                rotatePoint(q, &kYAxis[0], up);
                rotatePoint(q, &kXAxis[0], right);
                // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
                const MVector upV(up[0], up[1], up[2]);
                const MVector rightV(right[0], right[1], right[2]);
                const MPoint center = bodyPoint(b, &kZeroLocal[0], data->worldInverse);
                drawManager.box(center, upV, rightV, p.halfExtents.x * 2.0,
                                p.halfExtents.y * 2.0, p.halfExtents.z * 2.0, true);
            }
            if (wire)
            {
                MPointArray pts;
                addBoxEdges(b, pts, data->worldInverse);
                drawManager.mesh(MUIDrawManager::kLines, pts);
            }
        }
        else // capsule
        {
            const mmd::core::collider_geometry::PrimitiveParams p = primitiveFor(b);
            if (solid)
            {
                // Cylinder body + two hemisphere caps (drawn as spheres).
                const MPoint center = bodyPoint(b, &kZeroLocal[0], data->worldInverse);
                drawManager.cylinder(center, MVector(0, 1, 0), p.radius, p.length, 12, true);
                const float half = static_cast<float>(p.length) * 0.5F;
                const float top[3] = {0.0F, half, 0.0F};
                const float bot[3] = {0.0F, -half, 0.0F};
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
                drawManager.sphere(bodyPoint(b, top, data->worldInverse), p.radius, 12, 8, true);
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
                drawManager.sphere(bodyPoint(b, bot, data->worldInverse), p.radius, 12, 8, true);
            }
            if (wire)
            {
                MPointArray pts;
                addCapsuleLines(b, pts, data->worldInverse);
                drawManager.mesh(MUIDrawManager::kLines, pts);
            }
        }
    }
    drawManager.endDrawable();
}

// ── Selection ──

void PhysicsDrawOverride::updateSelectionGranularity(const MDagPath& objPath,
                                                     MSelectionContext& selectionContext)
{
    (void) objPath;
    // Component level: individual bodies are pickable (not just the node).
    selectionContext.setSelectionLevel(MSelectionContext::kComponent);
}

bool PhysicsDrawOverride::wantUserSelection() const
{
    // Custom userSelect() below does the actual pick (ray-cast against the
    // colliders) and writes the hit body index into uiSelectedBodyIndex.
    return true;
}

bool PhysicsDrawOverride::userSelect(const MSelectionInfo& selectInfo, const MDrawContext& context,
                                     const MDagPath& objPath, const MUserData* data,
                                     MSelectionList& selectionList, MPointArray& worldSpaceHitPts)
{
    (void) context;
    (void) worldSpaceHitPts;
    const auto* drawData = dynamic_cast<const MMDPhysicsDrawData*>(data);
    if (drawData == nullptr || drawData->bodies.empty())
        return false;

    // getLocalRay returns the pick ray in the node's OBJECT space.  The draw
    // data (and the Bullet world) is WORLD space, so transform the ray into
    // world space and test against the bodies directly.
    MPoint pnt;
    MVector vec;
    if (selectInfo.getLocalRay(pnt, vec) != MS::kSuccess)
        return false;

    // object -> world = the node's world matrix (row-vector: p * M).  MPoint
    // (w=1) is translated; MVector (w=0) is rotated/scaled only — exactly the
    // ray origin vs. direction semantics.
    const MMatrix world = objPath.inclusiveMatrix();
    const MPoint origin = pnt * world;
    const MVector dir = vec * world;

    std::vector<mmd::core::collider_geometry::RayBody> bodies;
    bodies.reserve(drawData->bodies.size());
    for (const PhysicsNode::DrawBody& b : drawData->bodies)
    {
        mmd::core::collider_geometry::RayBody rb;
        rb.pos = Double3(b.pos[0], b.pos[1], b.pos[2]);
        rb.quat = Double4(b.quat[0], b.quat[1], b.quat[2], b.quat[3]);
        switch (b.colliderType)
        {
        case PhysicsNode::kColliderSphere:
            rb.colliderType = Simulation::ColliderType::eSphere;
            break;
        case PhysicsNode::kColliderCapsule:
            rb.colliderType = Simulation::ColliderType::eCapsule;
            break;
        case PhysicsNode::kColliderBox:
        default:
            rb.colliderType = Simulation::ColliderType::eBox;
            break;
        }
        rb.shapeSize = Double3(b.shapeSize[0], b.shapeSize[1], b.shapeSize[2]);
        bodies.push_back(rb);
    }

    double t = 0.0;
    const int hit = mmd::core::collider_geometry::raycastBodies(
        Double3(origin.x, origin.y, origin.z), Double3(dir.x, dir.y, dir.z), bodies.data(),
        bodies.size(), t);
    static_cast<void>(hit);  // hit is assigned by raycastBodies (out-param style)
    if (hit < 0)
        return false;

    // Select the node and record the picked body for the AE template.
    MSelectionList sel;
    sel.add(objPath);
    selectionList.merge(sel);

    MStatus plugStat;
    MFnDependencyNode fn(objPath.node());
    MPlug plug = fn.findPlug(PhysicsNode::aUiSelectedBodyIndex, true, &plugStat);
    if (!plug.isNull())
        plug.setInt(hit);

    return true;
}
