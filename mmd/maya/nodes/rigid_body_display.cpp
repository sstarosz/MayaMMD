/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_display.cpp — see rigid_body_display.hpp.
 *
 * Session-wide state shared by every joint and body viewport override, so
 * that work which depends on the SELECTION (not on one node) happens once per
 * event instead of once per node:
 *
 *   - the selected joints (lead first) and the A/B roles of their bodies,
 *     resolved on SelectionChanged; only the joints and bodies whose display
 *     actually changes are dirtied;
 *   - model-editor changes (isolate select on or off, among others): every
 *     rigid-body shape and joint is redrawn.  VP2 could leave plug-in shapes
 *     that had been isolated out undrawn until something else refreshed them
 *     (re-selecting them, or a parent); this also updates the selected
 *     joints' fallback outlines;
 *   - one shared stock shader per colour (lit surface and flat wire).
 *
 * Simulation steps need no callback: every joint's solverStep input is
 * connected to its solver's outStep, and the joint node marks its drawing
 * dirty after each step (RigidBodyJoint::postEvaluation).
 *
 * Everything here runs on the main thread (DG preparation and event
 * callbacks).  install()/uninstall() are called from the plugin entry points.
 */

#include "rigid_body_display.hpp"

#include "rigid_body_joint.hpp"
#include "rigid_body_shape.hpp"

#include <maya/MEventMessage.h>
#include <maya/MFn.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGeometry.h>
#include <maya/MGeometryRequirements.h>
#include <maya/MGlobal.h>
#include <maya/MHWGeometry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MMessage.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MSelectionList.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <utility>

namespace mmd::maya::rigid_body_display
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
            it->roles |= (end == 0 ? palette::kRoleA : palette::kRoleB);
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

bool isSelected(const MDagPath& path)
{
    const MHWRender::DisplayStatus status = MHWRender::MGeometryUtilities::displayStatus(path);
    return status == MHWRender::kLead || status == MHWRender::kActive;
}

MColor selectionColor(const MDagPath& path)
{
    const MColor sel = MHWRender::MGeometryUtilities::wireframeColor(path);
    // Guard: a near-black selection colour would hide the wire.
    return sel.r + sel.g + sel.b < 0.01f ? palette::kSelectionFallback : sel;
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

// ===========================================================================
// Render-item plumbing
// ===========================================================================
namespace
{
// Fill one vertex buffer (`dimension` floats per vertex) with `numVerts`
// elements.  NOTE: acquire()'s size is the ELEMENT count, not bytes.
void fillVertexBuffer(MHWRender::MVertexBuffer* buffer, const std::vector<float>& data,
                      unsigned int numVerts, int dimension,
                      const MHWRender::MVertexBufferDescriptor& desc)
{
    if (buffer == nullptr || numVerts == 0 || data.size() < numVerts * dimension)
        return;
    if (desc.dataType() != MHWRender::MGeometry::kFloat || desc.dimension() != dimension)
        return;
    void* ptr = buffer->acquire(numVerts, /*writeOnly=*/true);
    if (ptr == nullptr)
        return;
    std::memcpy(ptr, data.data(), numVerts * dimension * sizeof(float));
    buffer->commit(ptr);
}
} // namespace

void fillMeshBuffers(const MHWRender::MGeometryRequirements& requirements,
                     const std::vector<float>& positions, const std::vector<float>& normals,
                     MHWRender::MGeometry& data)
{
    const unsigned int numVerts = static_cast<unsigned int>(positions.size() / 3);
    if (numVerts == 0)
        return;
    // A solid-colour item needs no real UVs — a zeroed float2 stream satisfies
    // any texture semantic the shader declares.
    const std::vector<float> dummyUvs(static_cast<std::size_t>(numVerts) * 2, 0.0f);
    const auto fill = [&](const MHWRender::MVertexBufferDescriptor& desc)
    {
        switch (desc.semantic())
        {
        case MHWRender::MGeometry::kPosition:
            fillVertexBuffer(data.createVertexBuffer(desc), positions, numVerts, 3, desc);
            break;
        case MHWRender::MGeometry::kNormal:
            fillVertexBuffer(data.createVertexBuffer(desc), normals, numVerts, 3, desc);
            break;
        case MHWRender::MGeometry::kTexture:
            fillVertexBuffer(data.createVertexBuffer(desc), dummyUvs, numVerts, 2, desc);
            break;
        default:
            break;
        }
    };

    // Use the shader's OWN requirements so the stream names and semantics
    // always match what the item expects; fall back to manual descriptors when
    // they come back empty (possible for custom render items).
    const MHWRender::MVertexBufferDescriptorList& descList = requirements.vertexRequirements();
    if (descList.length() > 0)
    {
        MHWRender::MVertexBufferDescriptor desc;
        for (int i = 0; i < descList.length(); ++i)
            if (descList.getDescriptor(i, desc))
                fill(desc);
        return;
    }
    fill(MHWRender::MVertexBufferDescriptor("position", MHWRender::MGeometry::kPosition,
                                            MHWRender::MGeometry::kFloat, 3));
    fill(MHWRender::MVertexBufferDescriptor("normal", MHWRender::MGeometry::kNormal,
                                            MHWRender::MGeometry::kFloat, 3));
    fill(MHWRender::MVertexBufferDescriptor("uv", MHWRender::MGeometry::kTexture,
                                            MHWRender::MGeometry::kFloat, 2));
}

unsigned int appendSphereMesh(double radius, int slices, int stacks, std::vector<float>& positions,
                              std::vector<float>& normals, std::vector<unsigned int>& indices,
                              std::vector<unsigned int>* lines)
{
    if (radius <= 0.0 || slices < 3 || stacks < 2)
        return static_cast<unsigned int>(positions.size() / 3);

    const unsigned int base = static_cast<unsigned int>(positions.size() / 3);
    // Append one vertex, returning its index.
    const auto vertex = [&](const MVector& dir)
    {
        positions.insert(positions.end(),
                         {static_cast<float>(dir.x * radius), static_cast<float>(dir.y * radius),
                          static_cast<float>(dir.z * radius)});
        normals.insert(normals.end(), {static_cast<float>(dir.x), static_cast<float>(dir.y),
                                       static_cast<float>(dir.z)});
        return static_cast<unsigned int>(positions.size() / 3) - 1;
    };
    const auto tri = [&](unsigned int a, unsigned int b, unsigned int c)
    { indices.insert(indices.end(), {a, b, c}); };
    const auto line = [&](unsigned int a, unsigned int b)
    {
        if (lines != nullptr)
            lines->insert(lines->end(), {a, b});
    };

    // Interior rings; the poles are single vertices, so the caps have no
    // degenerate (zero-area) triangles.
    const auto ringVertex = [&](int i, int j)
    {
        const double phi = M_PI * i / stacks;
        const double y = std::cos(phi);
        const double ringR = std::sin(phi);
        const double theta = 2.0 * M_PI * j / slices;
        return vertex(MVector(ringR * std::cos(theta), y, ringR * std::sin(theta)));
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
        tri(top, rows[0][col + 1], rows[0][col]);
        const std::vector<unsigned int>& last = rows.back();
        tri(bottom, last[col], last[col + 1]);
    }
    // Stitch the rows (each `slices + 1` vertices, the last repeating the
    // first position) into quads.  Rows advance the way a UV sphere's rows
    // do (+Y pole downwards on the outer side), so these face outward.
    for (std::size_t i = 0; i + 1 < rows.size(); ++i)
    {
        for (int j = 0; j < slices; ++j)
        {
            const auto col = static_cast<std::size_t>(j);
            const unsigned int a = rows[i][col];
            const unsigned int d = rows[i][col + 1];
            const unsigned int b = rows[i + 1][col];
            const unsigned int c = rows[i + 1][col + 1];
            tri(a, d, b);
            tri(b, d, c);
        }
    }

    // Wire: every latitude ring, and every meridian from pole to pole.
    for (int j = 0; j < slices; ++j)
    {
        const auto col = static_cast<std::size_t>(j);
        line(top, rows[0][col]);
        for (std::size_t i = 0; i < rows.size(); ++i)
        {
            line(rows[i][col], rows[i][col + 1]);
            if (i + 1 < rows.size())
                line(rows[i][col], rows[i + 1][col]);
        }
        line(rows.back()[col], bottom);
    }
    return base;
}
} // namespace mmd::maya::rigid_body_display
