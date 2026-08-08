/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_physics_draw_override.cpp
 *
 * See mmd_physics_draw_override.h — viewport drawing for mmdPhysicsNode.
 *
 * The drawing happens in the locator's OBJECT space, which is the physics
 * group's local space (the locator sits at the group's origin), i.e. exactly
 * the space the Bullet world runs in — so the solved poses pulled via
 * MMDPhysicsNode::collectDrawData can be drawn as-is.
 */

#include "mmd_physics_draw_override.h"

#include <maya/MColor.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MUIDrawManager.h>

#include "mmd_physics_math.h"
#include "mmd_physics_node.h"

#include <cmath>
#include <vector>

namespace
{

// Draw data cached by prepareForDraw and consumed by addUIDrawables.  Reused
// across frames via the oldData pointer; Maya manages its lifetime.
class MMDPhysicsDrawData : public MUserData
{
  public:
    std::vector<MMDPhysicsNode::DrawBody> bodies;
};

// Collision-group palette — matches the PMX group colors that used to be
// assigned to the (now removed) guide mesh shaders (rigid_body_builder's
// _RIGID_BODY_GROUP_COLORS).  Groups 0-7 classic rainbow; 8-15 extended.
const MColor kGroupColors[16] = {
    MColor(0.90f, 0.10f, 0.10f), // 0  red
    MColor(0.10f, 0.75f, 0.15f), // 1  green
    MColor(0.15f, 0.35f, 0.95f), // 2  blue
    MColor(1.00f, 0.90f, 0.10f), // 3  yellow
    MColor(0.95f, 0.15f, 0.65f), // 4  magenta
    MColor(0.00f, 0.85f, 0.90f), // 5  cyan
    MColor(1.00f, 0.55f, 0.10f), // 6  orange
    MColor(0.50f, 0.10f, 0.90f), // 7  purple
    MColor(0.60f, 0.90f, 0.10f), // 8  lime
    MColor(1.00f, 0.35f, 0.50f), // 9  rose
    MColor(0.40f, 0.65f, 0.95f), // 10 sky blue
    MColor(1.00f, 0.65f, 0.80f), // 11 pink
    MColor(0.55f, 0.35f, 0.15f), // 12 brown
    MColor(0.80f, 0.60f, 0.95f), // 13 lavender
    MColor(0.10f, 0.70f, 0.55f), // 14 teal
    MColor(0.10f, 0.15f, 0.50f), // 15 navy
};

MColor bodyColor(int groupId, bool kinematic)
{
    MColor c = kGroupColors[groupId % 16];
    if (kinematic)
        c *= 0.6f; // kinematic (bone-following) colliders drawn dimmer
    return c;
}

// Rotate v by unit quaternion q (x, y, z, w) — standard q*v*q^-1.  The
// quaternion comes from the node as doubles; the vertex stays float.
void rotatePoint(const double q[4], const float v[3], float out[3])
{
    const double tx = 2.0 * (q[1] * v[2] - q[2] * v[1]);
    const double ty = 2.0 * (q[2] * v[0] - q[0] * v[2]);
    const double tz = 2.0 * (q[0] * v[1] - q[1] * v[0]);
    out[0] = static_cast<float>(v[0] + q[3] * tx + (q[1] * tz - q[2] * ty));
    out[1] = static_cast<float>(v[1] + q[3] * ty + (q[2] * tx - q[0] * tz));
    out[2] = static_cast<float>(v[2] + q[3] * tz + (q[0] * ty - q[1] * tx));
}

MPoint bodyPoint(const MMDPhysicsNode::DrawBody& b, const float local[3])
{
    float o[3];
    rotatePoint(b.quat, local, o);
    return MPoint(b.pos[0] + o[0], b.pos[1] + o[1], b.pos[2] + o[2]);
}

// 12 box edges (half extents) as line endpoints.
void addBoxEdges(const MMDPhysicsNode::DrawBody& b, MPointArray& pts)
{
    const float hx = static_cast<float>(b.extents[0]);
    const float hy = static_cast<float>(b.extents[1]);
    const float hz = static_cast<float>(b.extents[2]);
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
        pts.append(bodyPoint(b, corners[e[0]]));
        pts.append(bodyPoint(b, corners[e[1]]));
    }
}

// Capsule (Y axis): two circles at +-h/2 plus 4 connecting lines.
void addCapsuleLines(const MMDPhysicsNode::DrawBody& b, MPointArray& pts)
{
    const float r = static_cast<float>(b.radius);
    const float half = static_cast<float>(b.length) * 0.5f;
    constexpr int kSegs = 12;
    constexpr int kSpokes = 4;
    for (int s = 0; s < kSegs; ++s)
    {
        const float a0 = static_cast<float>(s * 2.0 * mmd_physics_math::kPi / kSegs);
        const float a1 = static_cast<float>((s + 1) * 2.0 * mmd_physics_math::kPi / kSegs);
        const float v0[3] = {std::cos(a0) * r, half, std::sin(a0) * r};
        const float v1[3] = {std::cos(a1) * r, half, std::sin(a1) * r};
        const float v2[3] = {std::cos(a0) * r, -half, std::sin(a0) * r};
        const float v3[3] = {std::cos(a1) * r, -half, std::sin(a1) * r};
        pts.append(bodyPoint(b, v0));
        pts.append(bodyPoint(b, v1));
        pts.append(bodyPoint(b, v2));
        pts.append(bodyPoint(b, v3));
    }
    for (int s = 0; s < kSpokes; ++s)
    {
        const float a = static_cast<float>(s * 2.0 * mmd_physics_math::kPi / kSpokes);
        const float top[3] = {std::cos(a) * r, half, std::sin(a) * r};
        const float bot[3] = {std::cos(a) * r, -half, std::sin(a) * r};
        pts.append(bodyPoint(b, top));
        pts.append(bodyPoint(b, bot));
    }
}

} // namespace

// ===========================================================================
// MMDPhysicsDrawOverride
// ===========================================================================

MMDPhysicsDrawOverride::MMDPhysicsDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, nullptr)
{
}

MMDPhysicsDrawOverride::~MMDPhysicsDrawOverride() = default;

MHWRender::MPxDrawOverride* MMDPhysicsDrawOverride::creator(const MObject& obj)
{
    return new MMDPhysicsDrawOverride(obj);
}

MHWRender::DrawAPI MMDPhysicsDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile;
}

bool MMDPhysicsDrawOverride::hasUIDrawables() const
{
    return true; // addUIDrawables() queues the wireframe guide primitives
}

bool MMDPhysicsDrawOverride::isBounded(const MDagPath&, const MDagPath&) const
{
    return true;
}

MBoundingBox MMDPhysicsDrawOverride::boundingBox(const MDagPath& objPath, const MDagPath&) const
{
    MObject node = objPath.node();
    MFnDependencyNode fn(node);
    if (fn.typeId() == MMDPhysicsNode::kTypeId)
    {
        auto* physics = static_cast<MMDPhysicsNode*>(fn.userNode());
        if (physics)
            return physics->boundingBox();
    }
    return MBoundingBox(MPoint(-1.0, -1.0, -1.0), MPoint(1.0, 1.0, 1.0));
}

MUserData* MMDPhysicsDrawOverride::prepareForDraw(const MDagPath& objPath, const MDagPath&,
                                                  const MFrameContext&, MUserData* oldData)
{
    MMDPhysicsDrawData* data = dynamic_cast<MMDPhysicsDrawData*>(oldData);
    if (data == nullptr)
        data = new MMDPhysicsDrawData();
    data->bodies.clear();

    MObject node = objPath.node();
    MFnDependencyNode fn(node);
    if (fn.typeId() == MMDPhysicsNode::kTypeId)
    {
        auto* physics = static_cast<MMDPhysicsNode*>(fn.userNode());
        if (physics)
            physics->collectDrawData(data->bodies);
    }
    return data;
}

void MMDPhysicsDrawOverride::addUIDrawables(const MDagPath&, MUIDrawManager& drawManager,
                                            const MFrameContext&, const MUserData* userData)
{
    const auto* data = dynamic_cast<const MMDPhysicsDrawData*>(userData);
    if (data == nullptr)
        return;

    drawManager.beginDrawable();
    for (const auto& b : data->bodies)
    {
        drawManager.setColor(bodyColor(b.groupId, b.kinematic));
        if (b.colliderType == mmd_physics_math::kColliderSphere)
        {
            drawManager.sphere(MPoint(b.pos[0], b.pos[1], b.pos[2]), b.radius, 8, 8);
        }
        else
        {
            MPointArray pts;
            if (b.colliderType == mmd_physics_math::kColliderBox)
                addBoxEdges(b, pts);
            else
                addCapsuleLines(b, pts);
            drawManager.mesh(MUIDrawManager::kLines, pts);
            if (b.colliderType == mmd_physics_math::kColliderCapsule)
            {
                // Cap hemispheres read as spheres at the two cylinder ends.
                const float half = static_cast<float>(b.length) * 0.5f;
                const float top[3] = {0.0f, half, 0.0f};
                const float bot[3] = {0.0f, -half, 0.0f};
                drawManager.sphere(bodyPoint(b, top), b.radius, 8, 8);
                drawManager.sphere(bodyPoint(b, bot), b.radius, 8, 8);
            }
        }
    }
    drawManager.endDrawable();
}
