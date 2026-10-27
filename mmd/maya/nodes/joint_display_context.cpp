/*
 * SPDX-License-Identifier: MIT
 *
 * joint_display_context.cpp — see joint_display_context.hpp.
 */

#include "joint_display_context.hpp"

#include "rigid_body_joint.hpp"
#include "rigid_body_shape.hpp"

#include "surface_shader.hpp"

#include <maya/MEventMessage.h>
#include <maya/MFn.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MMessage.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MSelectionList.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>

namespace mmd::maya::joint_display
{
namespace
{
struct BodyRole
{
    MObjectHandle shape;
    unsigned int roles = 0;
};
struct State
{
    MCallbackId selectionCallback = 0;
    MCallbackId editorCallback = 0;
    std::vector<SelectedJoint> selected;
    std::vector<BodyRole> roles;
    std::vector<std::pair<std::uint32_t, MHWRender::MShaderInstance*>> shaders;
};

// Heap-allocated and destroyed only in uninstall(): Maya objects must never be
// touched from a static destructor at DLL-exit time.
State* gState = nullptr;

bool alive(const MObjectHandle& handle)
{
    return handle.isValid() && handle.isAlive();
}

void dirty(const MObject& node)
{
    const MObjectHandle handle(node);
    if (alive(handle))
        MHWRender::MRenderer::setGeometryDrawDirty(node, false);
}

// The solver a joint is registered on: joint.solver -> solver.jointShapes[n].
MObject solverOf(const MObject& joint)
{
    MPlugArray destinations;
    MPlug(joint, RigidBodyJoint::aSolver).connectedTo(destinations, false, true);
    return destinations.length() > 0 ? destinations[0].node() : MObject::kNullObj;
}

void resolveSelection(State& s)
{
    s.selected.clear();
    s.roles.clear();

    const auto add = [&](const MDagPath& path)
    {
        const MObject node = path.node();
        if (MFnDependencyNode(node).typeId() != RigidBodyJoint::kTypeId ||
            MPlug(node, RigidBodyJoint::aDrawMode).asShort() == RigidBodyJoint::kDrawOff)
            return;
        if (std::any_of(s.selected.begin(), s.selected.end(),
                        [&](const SelectedJoint& j) { return j.joint.object() == node; }))
            return;
        const MObject solver = solverOf(node);
        if (solver.isNull())
            return;
        SelectedJoint joint;
        joint.path = path;
        joint.joint = MObjectHandle(node);
        joint.solver = MObjectHandle(solver);
        const MObject shapes[2] = {RigidBodyJoint::bodyShape(node, RigidBodyJoint::aJointBodyA),
                                   RigidBodyJoint::bodyShape(node, RigidBodyJoint::aJointBodyB)};
        joint.bodyA = MObjectHandle(shapes[0]);
        joint.bodyB = MObjectHandle(shapes[1]);
        s.selected.push_back(joint);
        for (int end = 0; end < 2; ++end)
        {
            const MObject& shape = shapes[end];
            if (shape.isNull())
                continue;
            auto it = std::find_if(s.roles.begin(), s.roles.end(),
                                   [&](const BodyRole& r) { return r.shape.object() == shape; });
            if (it == s.roles.end())
                it = s.roles.insert(s.roles.end(), BodyRole{MObjectHandle(shape), 0u});
            it->roles |= 1u << end;
        }
    };

    // Lead first: Maya's lead is the LAST entry of the active selection list,
    // and a hidden body shared by several selected joints is outlined by the
    // first drawable one (highlightOwner), so the lead keeps its own bodies.
    MSelectionList selection;
    MGlobal::getActiveSelectionList(selection);
    for (unsigned int k = selection.length(); k > 0; --k)
    {
        MDagPath path;
        if (!selection.getDagPath(k - 1, path))
            continue;
        if (path.node().hasFn(MFn::kTransform))
        {
            MFnDagNode transform(path);
            for (unsigned int j = 0; j < transform.childCount(); ++j)
            {
                MDagPath child = path;
                child.push(transform.child(j));
                add(child);
            }
        }
        else
            add(path);
    }
}

void dirtySelectedJoints(const State& s)
{
    for (const auto& joint : s.selected)
        if (alive(joint.joint))
            dirty(joint.joint.object());
}

void selectionChanged(void*)
{
    if (!gState)
        return;
    State& s = *gState;
    const auto oldSelected = s.selected;
    const auto oldRoles = s.roles;
    resolveSelection(s);

    // Joints entering or leaving the selection change their guides/outlines.
    for (const auto& joint : oldSelected)
        if (alive(joint.joint))
            dirty(joint.joint.object());
    dirtySelectedJoints(s);

    // Bodies whose A/B role changed.
    const auto rolesOf = [](const std::vector<BodyRole>& roles, const MObject& shape)
    {
        for (const auto& r : roles)
            if (alive(r.shape) && r.shape.object() == shape)
                return r.roles;
        return 0u;
    };
    for (const auto& list : {std::cref(oldRoles), std::cref(s.roles)})
        for (const auto& r : list.get())
            if (alive(r.shape) &&
                rolesOf(oldRoles, r.shape.object()) != rolesOf(s.roles, r.shape.object()))
                dirty(r.shape.object());
}

// A model-editor change (isolate select on or off, among others) redraws
// every rigid-body shape and joint.  Leaving isolate select could leave
// shapes that had been isolated out undrawn until something else refreshed
// them, such as selecting them or a parent.  It also updates what the
// selected joints draw (fallback outlines for isolated-out bodies).
void editorChanged(void*)
{
    if (!gState)
        return;
    for (MItDependencyNodes it(MFn::kPluginLocatorNode); !it.isDone(); it.next())
    {
        const MObject node = it.thisNode();
        const MTypeId type = MFnDependencyNode(node).typeId();
        if (type == RigidBodyShape::kTypeId || type == RigidBodyJoint::kTypeId)
            dirty(node);
    }
}

} // namespace

void install()
{
    if (gState)
        return;
    gState = new State();
    gState->selectionCallback =
        MEventMessage::addEventCallback("SelectionChanged", selectionChanged);
    gState->editorCallback = MEventMessage::addEventCallback("modelEditorChanged", editorChanged);
    resolveSelection(*gState);
}

void uninstall()
{
    if (!gState)
        return;
    State& s = *gState;
    for (MCallbackId id : {s.selectionCallback, s.editorCallback})
        if (id)
            MMessage::removeCallback(id);
    // Safe here (not at DLL exit): the overrides using these were destroyed
    // when their creators were deregistered.
    if (MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer())
        if (const MHWRender::MShaderManager* manager = renderer->getShaderManager())
            for (const auto& entry : s.shaders)
                manager->releaseShader(entry.second);
    delete gState;
    gState = nullptr;
}

const std::vector<SelectedJoint>& selectedJoints()
{
    static const std::vector<SelectedJoint> kNone;
    return gState ? gState->selected : kNone;
}

bool isSelectedJoint(const MObject& joint)
{
    const auto& joints = selectedJoints();
    return std::any_of(joints.begin(), joints.end(), [&](const SelectedJoint& j)
                       { return alive(j.joint) && j.joint.object() == joint; });
}

unsigned int bodyRoles(const MObject& bodyShape)
{
    if (!gState)
        return 0;
    for (const auto& r : gState->roles)
        if (alive(r.shape) && r.shape.object() == bodyShape)
            return r.roles;
    return 0;
}

namespace
{
enum class ShaderKind : std::uint32_t
{
    Wire,
    Lit,
    DashedWire,
};
// One shared shader per colour and kind: lit surface (shaded hubs), flat
// colour (wireframe hubs) or dashed flat colour (inactive hubs).
MHWRender::MShaderInstance* cachedShader(const MColor& color, ShaderKind kind)
{
    if (!gState)
        return nullptr;
    const auto channel = [](float v)
    { return static_cast<std::uint32_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
    const std::uint32_t key = static_cast<std::uint32_t>(kind) << 24 | channel(color.r) << 16 |
                              channel(color.g) << 8 | channel(color.b);
    for (const auto& entry : gState->shaders)
        if (entry.first == key)
            return entry.second;
    MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
    const MHWRender::MShaderManager* manager = renderer ? renderer->getShaderManager() : nullptr;
    if (manager == nullptr)
        return nullptr;
    const bool lit = kind == ShaderKind::Lit;
    MHWRender::MShaderInstance* shader =
        lit ? mmd::maya::createSurfaceShader(*manager)
            : manager->getStockShader(kind == ShaderKind::DashedWire
                                          ? MHWRender::MShaderManager::k3dDashLineShader
                                          : MHWRender::MShaderManager::k3dSolidShader);
    if (shader == nullptr)
        return nullptr;
    if (lit)
        mmd::maya::setSurfaceColor(*shader, color);
    else
    {
        const float solid[4] = {color.r, color.g, color.b, 1.0f};
        shader->setParameter("solidColor", solid);
    }
    gState->shaders.emplace_back(key, shader);
    return shader;
}
} // namespace

MHWRender::MShaderInstance* solidShader(const MColor& color)
{
    return cachedShader(color, ShaderKind::Lit);
}

MHWRender::MShaderInstance* wireShader(const MColor& color)
{
    return cachedShader(color, ShaderKind::Wire);
}

MHWRender::MShaderInstance* dashedWireShader(const MColor& color)
{
    return cachedShader(color, ShaderKind::DashedWire);
}
} // namespace mmd::maya::joint_display
