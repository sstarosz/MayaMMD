/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint_geometry_override.cpp
 *
 * RigidBodyJointGeometryOverride — lit viewport renderer for one
 * pmxRigidBodyJoint (see the header for the design summary).
 */

#include "rigid_body_joint_geometry_override.hpp"

#include "joint_display.hpp"
#include "maya_utils.hpp"
#include "rigid_body_display.hpp"
#include "rigid_body_joint.hpp"
#include "rigid_body_node.hpp"
#include "rigid_body_shape.hpp"

#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MNodeMessage.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MQuaternion.h>
#include <maya/MShaderManager.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MTypes.h>
#include <maya/MUIDrawManager.h>
#include <maya/MVector.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <vector>

namespace
{
// Every colour lives in rigid_body_display.hpp (one hue per channel); the
// alias keeps the drawing code below reading as palette::kXxx.
namespace palette = mmd::maya::rigid_body_display::palette;

// The hub is the only render item geometry; everything else is UI drawables.
// The hub's wedges alternate between two items (like a beach ball), each with
// a shaded and a wireframe variant, so a joint that rotates AND translates
// shows both state colours from every direction, and the hub stays visible
// and pickable in wireframe mode.
struct HubItem
{
    const char* name;
    bool primary; // even wedges (primary colour) or odd wedges
    bool wire;    // wireframe-mode lines or shaded-mode triangles
};
constexpr std::array<HubItem, 4> kHubItems = {
    {
        {"mmd_rigid_body_joint_hub_even", true, false},
        {"mmd_rigid_body_joint_hub_odd", false, false},
        {"mmd_rigid_body_joint_hub_even_wire", true, true},
        {"mmd_rigid_body_joint_hub_odd_wire", false, true},
    },
};
// Wedges around the hub sphere (its slices); half of them per colour.
constexpr int kHubWedges = 8;
// Opacity of the X-ray pass that keeps a selected joint's overlay readable
// through the geometry in front of it (see addUIDrawables).
constexpr float kGhostAlpha = 0.3F;

// Overlay styling (line widths are in pixels; kPressedWidth for the guides'
// pressed limits lives in joint_display.hpp).
constexpr float kSelectionWireWidth = 1.75F; // hub wire in Maya's selection colour
constexpr float kBodyLinkWidth = 3.0F;       // body centre -> its attachment
constexpr float kInactiveLinkWidth = 2.5F;   // dashed link of an inactive joint
constexpr float kMarkerWidth = 2.5F;         // body-centre / attachment markers
constexpr float kOutlineWidth = 4.0F;        // fallback collider outline
constexpr float kCoincidentPointSize = 5.0F; // the shared ring's centre dot
// Attachment frames closer than this (a fraction of the hub radius, but never
// below the absolute floor) read as one point: they share one ring instead of
// drawing two overlapping ones.
constexpr double kSeparateMinDistance = 1e-5;
constexpr double kSeparateHubFraction = 0.08;

// A mesh: xyz-interleaved positions + normals, a triangle index list, and a
// line index list over the same vertices (the wireframe-mode hub).
struct Mesh
{
    std::vector<float> positions; // xyz interleaved
    std::vector<float> normals;   // xyz interleaved
    std::vector<unsigned int> indices;
    std::vector<unsigned int> lines;
};
// The hub at UNIT radius in its attachment-A frame, split into the
// alternating wedges of kHubItems (one index list per item).  It is the same
// for every joint, so it is built once; populateGeometry scales it by the
// joint's hub radius as it bakes it into the joint's LOCAL space.
struct Hub
{
    Mesh mesh;
    std::array<std::vector<unsigned int>, 4> indices;
};
const Hub& unitHub()
{
    static const Hub hub = []
    {
        Hub out;
        // A compact pickable hub at attachment A.  Springs and limits are
        // drawn per axis only for selected joints (drawOverlay's guide
        // segments): a single marker here could not say WHICH axis is sprung.
        mmd::maya::rigid_body_display::appendSphereMesh(1.0, kHubWedges, 6, out.mesh.positions,
                                                        out.mesh.normals, out.mesh.indices,
                                                        &out.mesh.lines);

        // A primitive belongs to an even wedge when its centroid's azimuth
        // about the sphere's axis falls in one: wedges alternate all around
        // the hub, so every view direction sees both colours (a top/bottom
        // split would show one colour from above and the other from below).
        const auto evenWedge =
            [&](const std::vector<unsigned int>& list, std::size_t first, std::size_t count)
        {
            double x = 0.0;
            double z = 0.0;
            for (std::size_t k = 0; k < count; ++k)
            {
                const auto vertex = static_cast<std::size_t>(list[first + k]) * 3;
                x += out.mesh.positions[vertex];
                z += out.mesh.positions[vertex + 2];
            }
            const double azimuth = std::atan2(z, x) + (z < 0.0 ? 2.0 * M_PI : 0.0);
            return static_cast<int>(azimuth / (2.0 * M_PI / kHubWedges)) % 2 == 0;
        };
        for (std::size_t item = 0; item < 4; ++item)
        {
            const HubItem& spec = kHubItems.at(item);
            const std::vector<unsigned int>& source = spec.wire ? out.mesh.lines : out.mesh.indices;
            const std::size_t stride = spec.wire ? 2 : 3;
            for (std::size_t i = 0; i + stride <= source.size(); i += stride)
            {
                if (evenWedge(source, i, stride) == spec.primary)
                {
                    out.indices.at(item).insert(
                        out.indices.at(item).end(), source.begin() + static_cast<std::ptrdiff_t>(i),
                        source.begin() + static_cast<std::ptrdiff_t>(i + stride));
                }
            }
        }
        return out;
    }();
    return hub;
}

// The hub mesh, scaled to the joint's rendered hub radius.  The vertices are
// LOCAL to the joint node: the solver drives the node's transform to the live
// attachment on body A, so VP2 applies the pose (and every animated parent) and
// nothing has to be rebuilt per frame.  A uniform scale leaves the normals
// untouched, so only the positions are written.
void bakeHub(const Mesh& hub, double radius, std::vector<float>& positions,
             std::vector<float>& normals)
{
    positions.resize(hub.positions.size());
    normals = hub.normals;
    for (std::size_t i = 0; i < hub.positions.size(); ++i)
    {
        positions[i] = static_cast<float>(hub.positions[i] * radius);
    }
}

// Read a k3Double (or angle3) plug; a missing plug reads as zero.
[[nodiscard]] mmd::core::Double3 readDouble3(const MPlug& plug)
{
    return plug.isNull() ? mmd::core::Double3{} : mmd::maya::readPlugDouble3(plug);
}

[[nodiscard]] MPoint toPoint(const mmd::core::Double3& d)
{
    return {d.x, d.y, d.z};
}

// The unit circle as (cos, sin) pairs, closed (segments + 1 points), built once
// per segment count: the marker rings are drawn every frame and every pass, and
// only their centre, radius and plane change.
const std::vector<MVector>& unitRing(int segments)
{
    // std::map, not unordered_map: the reference handed out must survive later
    // insertions.
    static std::map<int, std::vector<MVector>> rings;
    const int count = std::max(1, segments);
    const auto it = rings.find(count);
    if (it != rings.end())
    {
        return it->second;
    }

    std::vector<MVector> points;
    points.reserve(static_cast<std::size_t>(count) + 1);
    for (int i = 0; i <= count; ++i)
    {
        const double angle = 2.0 * M_PI * i / count;
        points.emplace_back(std::cos(angle), std::sin(angle), 0.0);
    }
    return rings.emplace(count, std::move(points)).first->second;
}
} // namespace

// ===========================================================================
// Lifecycle / registration
// ===========================================================================
MHWRender::MPxGeometryOverride* RigidBodyJointGeometryOverride::creator(const MObject& obj)
{
    return new RigidBodyJointGeometryOverride(obj);
}

void RigidBodyJointGeometryOverride::inputChanged(const MObject& node, const MPlug& plug)
{
    MPlug root = plug;
    while (root.isChild())
    {
        root = root.parent();
    }
    if (root.attribute() == RigidBodyJoint::aSolverStep)
    {
        poseChanged(node);
    }
    else if (root.attribute() == RigidBodyJoint::aJoint ||
             root.attribute() == RigidBodyJoint::aJointDisplay ||
             root.attribute() == RigidBodyJoint::aJointBodyA ||
             root.attribute() == RigidBodyJoint::aJointBodyB ||
             root.attribute() == RigidBodyJoint::aSolver ||
             root.attribute() == RigidBodyJoint::aDrawMode)
    {
        MHWRender::MRenderer::setGeometryDrawDirty(node);
    }
}

void RigidBodyJointGeometryOverride::inputsEvaluated(const MObject& node,
                                                     const MEvaluationNode& evaluationNode)
{
    // EM does not call setDependentsDirty during playback. Only configuration
    // dirt prepares all hubs; postEvaluation handles a picked joint's pose.
    for (const MObject& attribute : {
             RigidBodyJoint::aJoint,
             RigidBodyJoint::aJointDisplay,
             RigidBodyJoint::aJointBodyA,
             RigidBodyJoint::aJointBodyB,
             RigidBodyJoint::aSolver,
             RigidBodyJoint::aDrawMode,
         })
    {
        if (evaluationNode.dirtyPlugExists(attribute))
        {
            MHWRender::MRenderer::setGeometryDrawDirty(node);
            return;
        }
    }
}

void RigidBodyJointGeometryOverride::poseChanged(const MObject& node)
{
    if (mmd::maya::rigid_body_display::isSelectedJoint(node))
    {
        MHWRender::MRenderer::setGeometryDrawDirty(node, false);
    }
}

RigidBodyJointGeometryOverride::RigidBodyJointGeometryOverride(const MObject& obj)
    : MHWRender::MPxGeometryOverride(obj), fJoint(obj)
{
    fPlugs.drawMode = MPlug(obj, RigidBodyJoint::aDrawMode);
    fPlugs.solver = MPlug(obj, RigidBodyJoint::aSolver);
    fPlugs.solverStep = MPlug(obj, RigidBodyJoint::aSolverStep);
    fPlugs.hubRadius = MPlug(obj, RigidBodyJoint::aJointHubRadius);
    fPlugs.active = MPlug(obj, RigidBodyJoint::aJointActive);
    // Sensible before the first updateRenderItems() caches the real ones.
    fWorldAxisLocal[0] = MVector::xAxis;
    fWorldAxisLocal[1] = MVector::yAxis;
    fWorldAxisLocal[2] = MVector::zAxis;
}

RigidBodyJointGeometryOverride::~RigidBodyJointGeometryOverride()
{
    for (const auto& watch : fOutlineWatches)
    {
        if (watch.callback != 0U)
        {
            MMessage::removeCallback(watch.callback);
        }
    }
    // The shaders are shared and owned by rigid_body_display.
}

MHWRender::DrawAPI RigidBodyJointGeometryOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile;
}

void RigidBodyJointGeometryOverride::invalidateDrawables()
{
    const MObjectHandle joint(fJoint);
    if (joint.isValid() && joint.isAlive())
    {
        MHWRender::MRenderer::setGeometryDrawDirty(fJoint, false);
    }
}

void RigidBodyJointGeometryOverride::dependencyDirty(MObject& /*unused*/, MPlug& /*unused*/,
                                                     void* clientData)
{
    static_cast<RigidBodyJointGeometryOverride*>(clientData)->invalidateDrawables();
}

void RigidBodyJointGeometryOverride::updateOutlineWatches()
{
    std::vector<MObject> nodes;
    const auto appendPath = [&](MDagPath path)
    {
        while (path.isValid() && path.length() > 0)
        {
            const MObject node = path.node();
            if (std::find(nodes.begin(), nodes.end(), node) == nodes.end())
            {
                nodes.push_back(node);
            }
            path.pop();
        }
    };
    for (const auto& outline : fBodyOutlines)
    {
        if (outline.valid && !outline.candidates.empty())
        {
            appendPath(outline.path);
            for (const auto& candidate : outline.candidates)
            {
                appendPath(candidate.path);
            }
        }
    }
    for (auto it = fOutlineWatches.begin(); it != fOutlineWatches.end();)
    {
        if (!it->node.isValid() || !it->node.isAlive() ||
            std::find(nodes.begin(), nodes.end(), it->node.object()) == nodes.end())
        {
            if (it->callback != 0U)
            {
                MMessage::removeCallback(it->callback);
            }
            it = fOutlineWatches.erase(it);
        }
        else
        {
            ++it;
        }
    }
    for (MObject node : nodes)
    {
        const bool watched =
            std::any_of(fOutlineWatches.begin(), fOutlineWatches.end(),
                        [&](const DependencyWatch& watch) { return watch.node.object() == node; });
        if (!watched)
        {
            fOutlineWatches.push_back({
                MObjectHandle(node),
                MNodeMessage::addNodeDirtyPlugCallback(node, dependencyDirty, this),
            });
        }
    }
}

bool RigidBodyJointGeometryOverride::requiresGeometryUpdate() const
{
    // True would force a full update (solver queries + buffer upload) on
    // EVERY viewport refresh, camera tumbles included.  Everything the joint
    // draws from outside its node dirties it instead: solver steps and edits
    // through RigidBodyJoint (setDependentsDirty / postEvaluation), selection
    // and isolate changes through rigid_body_display, hidden bodies through the
    // outline watches.
    return false;
}

bool RigidBodyJointGeometryOverride::requiresUpdateRenderItems(const MDagPath& /*dagPath*/) const
{
    // False: the item state (enable / draw mode / shader) only changes on
    // edits, which mark the drawing dirty.  Returning true does NOT make VP2
    // re-prepare the object on its own; the pose needs no item state at all
    // (the solver-driven DAG transform carries it).
    return false;
}

// ===========================================================================
// Data preparation
// ===========================================================================
bool RigidBodyJointGeometryOverride::isIndexingDirty(const MHWRender::MRenderItem& /*item*/)
{
    return false; // The hub topology is constant, including after a radius edit.
}

bool RigidBodyJointGeometryOverride::isStreamDirty(
    const MHWRender::MVertexBufferDescriptor& /*desc*/)
{
    return mGeometryDirty;
}

void RigidBodyJointGeometryOverride::updateDG()
{
    if (fPlugs.drawMode.isNull())
    {
        return;
    }

    fState.drawMode = fPlugs.drawMode.asShort();
    // "picked" = the joint itself is selected (not just one of its ancestors),
    // which is what the overlay ownership rules key off.
    fState.picked = mmd::maya::rigid_body_display::isSelectedJoint(fJoint);
    fState.definition = {};
    if (!RigidBodyJoint::readJointDefinition(fJoint, fState.definition))
    {
        return;
    }

    const double radius = fPlugs.hubRadius.asDouble();
    mGeometryDirty = mGeometryDirty || radius != fState.hubRadius;
    fState.hubRadius = radius;
    fState.active = fPlugs.active.asBool();
    fState.live = false;
    fSolver = MObject::kNullObj;
    // The hub's colour, the marker sizes and the guide ranges all derive from
    // the limits and the activity, and this only runs when the drawing was
    // marked dirty (an edit, a solver display change, or a picked joint's
    // per-frame pose refresh) — so it is always recomputed.
    classifyState();

    // Only a picked joint outlines its bodies: resolving the two body
    // connections costs DG queries, and the overlays are the only consumer.
    if (fState.picked)
    {
        resolveSolver();
        const std::array<MObject, 2> bodyShapes{
            RigidBodyJoint::bodyShape(fJoint, RigidBodyJoint::aJointBodyA),
            RigidBodyJoint::bodyShape(fJoint, RigidBodyJoint::aJointBodyB),
        };
        cacheBodyOutlines(bodyShapes);
    }
    else
    {
        for (auto& outline : fBodyOutlines)
        {
            outline.valid = false;
            outline.candidates.clear();
        }
    }
    updateOutlineWatches();
}

// Only a picked overlay needs the solver's live attachment / body frames.
// Ordinary hubs read their own display inputs and move with the DAG.
void RigidBodyJointGeometryOverride::resolveSolver()
{
    MPlugArray destinations;
    fPlugs.solver.connectedTo(destinations, false, true);
    if (destinations.length() == 0)
    {
        // Not wired to a solver: draw the joint's own (unconstrained) data.
        return;
    }
    fSolver = destinations[0].node();
    const RigidBodyNode* const solver =
        dynamic_cast<RigidBodyNode*>(MFnDependencyNode(fSolver).userNode());
    if (solver == nullptr)
    {
        return;
    }
    // Pulling solverStep evaluates the solver, so the frames a picked joint
    // reads below are the current frame's.
    fPlugs.solverStep.asDouble();
    fState.definition.bodyA =
        solver->bodyIndex(RigidBodyJoint::bodyShape(fJoint, RigidBodyJoint::aJointBodyA));
    fState.definition.bodyB =
        solver->bodyIndex(RigidBodyJoint::bodyShape(fJoint, RigidBodyJoint::aJointBodyB));
    MMatrix bodyA;
    MMatrix bodyB;
    fState.live = solver->jointViewportFrames(fState.definition, bodyA, bodyB,
                                              fState.attachmentAWorld, fState.attachmentBWorld);
    if (fState.live)
    {
        fState.bodyAWorld = MPoint::origin * bodyA;
        fState.bodyBWorld = MPoint::origin * bodyB;
    }
}

// Selected joints only: cache the connected colliders (settings, DAG pose and
// visibility) so a body hidden or isolated out of a view can still be outlined,
// and decide with the other selected joints which one owns that fallback
// outline.  Unselected joints draw no overlays at all.
void RigidBodyJointGeometryOverride::cacheBodyOutlines(const std::array<MObject, 2>& bodyShapes)
{
    for (auto& outline : fBodyOutlines)
    {
        outline.valid = false;
        outline.candidates.clear();
    }
    if (!fState.picked || fSolver.isNull())
    {
        return;
    }

    for (int end = 0; end < 2; ++end)
    {
        const auto index = static_cast<std::size_t>(end);
        const MObject& shape = bodyShapes.at(index);
        auto& outline = fBodyOutlines.at(index);
        if (shape.isNull() || !MDagPath::getAPathTo(shape, outline.path))
        {
            continue;
        }
        outline.size = toPoint(readDouble3(MPlug(shape, RigidBodyShape::aBodyShapeSize)));
        const MPlug typePlug(shape, RigidBodyShape::aBodyColliderType);
        outline.type = typePlug.isNull() ? 0 : typePlug.asShort();
        outline.world = outline.path.inclusiveMatrix();
        outline.visible = outline.path.isVisible();
        outline.valid = true;
    }
    // Candidates arrive lead-first from the context, so a hidden body shared by
    // several selected joints is owned by the lead.
    for (const auto& joint : mmd::maya::rigid_body_display::selectedJoints())
    {
        if (joint.solver.object() != fSolver || !joint.path.isValid())
        {
            continue;
        }
        for (auto& outline : fBodyOutlines)
        {
            if (!outline.valid)
            {
                continue;
            }
            const MObject shape = outline.path.node();
            const unsigned int roles = (joint.bodyA.object() == shape ? palette::kRoleA : 0U) |
                                       (joint.bodyB.object() == shape ? palette::kRoleB : 0U);
            if (roles != 0U)
            {
                outline.candidates.push_back({joint.path, roles, joint.path.isVisible()});
            }
        }
    }
}

// The hub's colour channel is the core freedom model (joint_display.hpp), so
// the summary and the selected motion guides can never disagree: both read the
// Ranges resolved here.
void RigidBodyJointGeometryOverride::classifyState()
{
    fState.sizes = mmd::core::joint_display::guideSizes(fState.hubRadius);
    if (!fState.active)
    {
        fState.ranges = {};
        fState.freedom = {};
        return;
    }
    fState.ranges = mmd::core::joint_display::describe(fState.definition);
    fState.freedom = mmd::core::joint_display::summarize(fState.ranges);
}

void RigidBodyJointGeometryOverride::stateColors(MColor& primary, MColor& secondary) const
{
    if (!fState.active)
    {
        primary = secondary = palette::kInactive;
        return;
    }
    primary = palette::kWelded;
    if (fState.freedom.rotates)
    {
        primary = palette::kRotates;
    }
    else if (fState.freedom.translates)
    {
        primary = palette::kTranslates;
    }
    // Both hues only for a joint that does both; the wedges alternate them.
    secondary =
        fState.freedom.rotates && fState.freedom.translates ? palette::kTranslates : primary;
}

bool RigidBodyJointGeometryOverride::selected(const MDagPath& dagPath)
{
    const std::uint64_t statusSerial = mmd::maya::rigid_body_display::displayStatusSerial();
    if (!fSelectedValid || statusSerial != fSelectedSerial)
    {
        fSelected = mmd::maya::rigid_body_display::isSelected(dagPath);
        fSelectedSerial = statusSerial;
        fSelectedValid = true;
    }
    return fSelected;
}

void RigidBodyJointGeometryOverride::updateRenderItems(const MDagPath& dagPath,
                                                       MRenderItemList& renderItems)
{
    const bool visible = fState.drawMode != RigidBodyJoint::kDrawOff;

    // The hub is LOCAL geometry at the node's origin: the solver drives the
    // node's transform to the live attachment on body A, so VP2 carries the
    // pose (and every animated parent) and the items have no matrix of their
    // own.  This runs on an edit, on a solver display change and for a picked
    // joint's per-frame overlay refresh — never for an ordinary playback frame.
    const bool inactive = !fState.active;
    const auto shadedModes =
        static_cast<MGeometry::DrawMode>(static_cast<unsigned int>(MGeometry::kShaded) |
                                         static_cast<unsigned int>(MGeometry::kTextured));
    MColor primary;
    MColor secondary;
    // The hub's colour is a few comparisons on the classified freedom state.
    // The SOLID keeps its STATE colour even when selected: the selection
    // highlight is a native-coloured wire drawn in addUIDrawables(), not a tint
    // of the whole hub — so the lead hub is outlined green and the rest of a
    // multiple selection white, instead of flooding the hubs.
    stateColors(primary, secondary);
    for (const HubItem& spec : kHubItems)
    {
        MRenderItem* item = nullptr;
        const int index = renderItems.indexOf(spec.name);
        if (index >= 0)
        {
            item = renderItems.itemAt(index);
        }
        else
        {
            item = MRenderItem::Create(spec.name, MRenderItem::NonMaterialSceneItem,
                                       spec.wire ? MGeometry::kLines : MGeometry::kTriangles);
            if (item == nullptr || !renderItems.append(item))
            {
                continue;
            }
            // The hub respects scene depth.
            item->depthPriority(MRenderItem::sDormantWireDepthPriority);
            // Pick under the joint's own selection type (priority), not "locator".
            item->setSelectionMask(MSelectionMask(RigidBodyJoint::kSelectionType));
            // VP2 applies the solver-driven DAG transform to local geometry.
        }
        // Must match the viewport's display modes or the item never draws.
        MGeometry::DrawMode drawMode = MGeometry::kWireframe;
        if (!spec.wire)
        {
            drawMode = shadedModes;
        }
        else if (inactive)
        {
            drawMode =
                static_cast<MGeometry::DrawMode>(static_cast<unsigned int>(shadedModes) |
                                                 static_cast<unsigned int>(MGeometry::kWireframe));
        }
        item->setDrawMode(drawMode);
        item->enable(visible && (spec.wire || !inactive));
        // Shared shaders per colour (rigid_body_display).
        namespace display = mmd::maya::rigid_body_display;
        const MColor& color = spec.primary ? primary : secondary;
        MHWRender::MShaderInstance* shader = display::solidShader(color);
        if (spec.wire)
        {
            shader = inactive ? display::dashedWireShader(color) : display::wireShader(color);
        }
        if (shader != nullptr)
        {
            item->setShader(shader);
        }
    }

    // A picked joint's overlay is world-space UI drawables: resolve its other
    // attachment, the body centres and the world axes in the node's LOCAL space
    // (the attachment-A frame the solver drove the transform to) while the DAG
    // matrix is at hand, so the draw phase reads no DAG state.
    if (!fState.picked)
    {
        return;
    }
    fLocalToWorld = dagPath.inclusiveMatrix();
    fWorldToLocal = fLocalToWorld.inverse();
    fAttachmentB =
        fState.live ? MPoint::origin * fState.attachmentBWorld * fWorldToLocal : MPoint::origin;
    if (fState.live)
    {
        fLocalA = fState.bodyAWorld * fWorldToLocal;
        fLocalB = fState.bodyBWorld * fWorldToLocal;
    }
    // World X/Y/Z as unit directions in the joint's local space: the markers
    // (drawRings) are axis-aligned rings built in local space, so they keep
    // the hub's proportions even when the model root is scaled.
    const std::array<MVector, 3> worldAxes{MVector::xAxis, MVector::yAxis, MVector::zAxis};
    for (int a = 0; a < 3; ++a)
    {
        fWorldAxisLocal.at(static_cast<std::size_t>(a)) =
            (worldAxes.at(static_cast<std::size_t>(a)) * fWorldToLocal).normal();
    }
}

bool RigidBodyJointGeometryOverride::hasUIDrawables() const
{
    return true;
}

void RigidBodyJointGeometryOverride::addUIDrawables(const MDagPath& dagPath,
                                                    MHWRender::MUIDrawManager& drawManager,
                                                    const MHWRender::MFrameContext& frameContext)
{
    if (fState.drawMode == RigidBodyJoint::kDrawOff)
    {
        return;
    }
    // Asked again rather than taken from updateRenderItems(): a selection change
    // redraws every joint, but it does not have to go through
    // updateRenderItems() first.  selected() caches the viewport query per
    // selection / isolate change, so this is a serial compare.
    if (!selected(dagPath))
    {
        return;
    }

    // Native selection outline: the solid keeps its state colour and the pick
    // is marked with Maya's OWN selection colour drawn as a wire around the
    // hub — green for the LEAD item, white for the other members of a multiple
    // selection — mirroring how RigidBodyShapeGeometryOverride outlines a
    // selected collider. Drawn before the attachment overlay so it shows even
    // when no solver pose is available.  Drawables are in the node's local
    // space (VP2 applies the DAG matrix), where the hub sits at the origin.
    {
        drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
        drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
        drawManager.setLineWidth(kSelectionWireWidth);
        drawManager.setColor(mmd::maya::rigid_body_display::selectionColor(dagPath));
        mmd::maya::drawColliderWire(drawManager, MPoint(fState.hubRadius, 0.0, 0.0),
                                    RigidBodyShape::kColliderSphere);
        drawManager.endDrawable();
    }

    // A joint drawn as selected only because a parent group is (the whole
    // {model}_Joints group, say) keeps the wire but draws no overlay.
    if (!fState.picked)
    {
        return;
    }

    // Built once and drawn twice (X-ray ghost + depth-tested pass), so the two
    // passes cannot disagree and the guides are not computed twice.
    const OverlayGeom geom = buildOverlayGeom();

    // Joints usually sit inside cloth and hair, so a depth-tested diagram is
    // clipped exactly where it matters.  A faint X-ray ghost keeps the hidden
    // parts readable; the full-strength depth-tested pass reads as "in front".
    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.beginDrawInXray();
    drawOverlay(drawManager, kGhostAlpha, geom);
    drawManager.endDrawInXray();
    drawManager.endDrawable();

    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    drawIsolatedBodyOutlines(drawManager, frameContext);
    drawOverlay(drawManager, 1.0F, geom);
    drawManager.endDrawable();
}

RigidBodyJointGeometryOverride::OverlayGeom RigidBodyJointGeometryOverride::buildOverlayGeom() const
{
    // Cached in updateRenderItems(): the draw phase reads no DAG state.
    OverlayGeom geom;
    // Attachment A is the node's origin (the solver drove the transform there);
    // fLocalToWorld only scales the distance to the coincident-attachment test.
    const double separation = ((MPoint::origin - fAttachmentB) * fLocalToWorld).length();
    geom.separate =
        separation > std::max(kSeparateMinDistance, fState.hubRadius * kSeparateHubFraction);
    if (fState.active && fState.live && fState.drawMode != RigidBodyJoint::kDrawHub)
    {
        geom.guides = mmd::core::joint_display::motionGuides(motionGuideInput());
    }
    return geom;
}

// What the core guide builder needs, in the attachment-A frame.
mmd::core::joint_display::GuideInput RigidBodyJointGeometryOverride::motionGuideInput() const
{
    using namespace mmd::core::joint_display;
    // The Ranges were resolved with the hub's colour (classifyState), so the
    // guides and the colour cannot disagree.
    GuideInput input;
    input.ranges = fState.ranges;
    input.sizes = fState.sizes;
    const MPoint currentB =
        MPoint::origin * fState.attachmentBWorld * fState.attachmentAWorld.inverse();
    input.currentB = {currentB.x, currentB.y, currentB.z};
    const auto quat = [](const MMatrix& m)
    {
        const MQuaternion q = MTransformationMatrix(m).rotation();
        return mmd::core::Double4(q.x, q.y, q.z, q.w);
    };
    input.hasCurrentAngles = true;
    input.currentAngles =
        currentAngles(quat(fState.attachmentAWorld), quat(fState.attachmentBWorld));
    return input;
}

// The joint's overlay for one pass: body links and markers, or — for a joint
// that constrains nothing — dashed links to the bodies it names.
void RigidBodyJointGeometryOverride::drawOverlay(MHWRender::MUIDrawManager& drawManager,
                                                 float alpha, const OverlayGeom& geom) const
{
    using namespace mmd::core::joint_display;
    const GuideSizes& sizes = fState.sizes;
    // View-independent markers, sized from the hub: every BODY CENTRE is a
    // small wire diamond (an octahedron from any view) and every ATTACHMENT a
    // round ring, so a body's centre can never be mistaken for a separated
    // attachment (they used to be identical diamonds for body B).
    const auto bodyMark = [&](const MPoint& local)
    { drawRings(drawManager, local, sizes.bodyMarker, 4); };
    const auto attachmentMark = [&](const MPoint& local)
    { drawRings(drawManager, local, sizes.attachmentMarker, 24); };

    drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    if (!fState.active || !fState.live)
    {
        for (int end = 0; end < 2; ++end)
        {
            const BodyOutline& outline = fBodyOutlines.at(static_cast<std::size_t>(end));
            if (!outline.valid)
            {
                continue;
            }
            MColor color = end == 0 ? palette::kBodyA : palette::kBodyB;
            color.a *= alpha;
            drawManager.setColor(color);
            drawManager.setLineWidth(kInactiveLinkWidth);
            const MPoint body = MPoint::origin * outline.world * fWorldToLocal;
            drawManager.setLineStyle(MHWRender::MUIDrawManager::kDashed);
            drawManager.line(body, MPoint::origin); // the hub sits at the node's origin
            drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
            bodyMark(body);
        }
        return;
    }

    // One stroke per body link: a second, wider black stroke at the same depth
    // can obscure the coloured one depending on VP2 draw ordering.
    const auto endpoint =
        [&](const MPoint& body, const MPoint& attachment, MColor color, bool showAttachment)
    {
        color.a *= alpha;
        drawManager.setColor(color);
        drawManager.setLineWidth(kBodyLinkWidth);
        drawManager.line(body, attachment);
        drawManager.setLineWidth(kMarkerWidth);
        bodyMark(body);
        if (showAttachment)
        {
            attachmentMark(attachment);
        }
    };
    endpoint(fLocalA, MPoint::origin, palette::kBodyA, true);
    endpoint(fLocalB, fAttachmentB, palette::kBodyB, geom.separate);
    if (!geom.separate)
    {
        // Coincident endpoints share one ring-and-centre mark.
        drawManager.setPointSize(kCoincidentPointSize);
        drawManager.point(MPoint::origin);
    }
    if (!geom.guides.empty())
    {
        drawGuideSegments(drawManager, geom.guides, alpha);
    }
}

void RigidBodyJointGeometryOverride::drawIsolatedBodyOutlines(
    MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext) const
{
    if (std::none_of(std::begin(fBodyOutlines), std::end(fBodyOutlines),
                     [](const BodyOutline& outline) { return outline.valid; }))
    {
        return;
    }
    // Resolved once per viewport and shared by every joint drawn in it.
    const mmd::maya::rigid_body_display::ViewFilter& view =
        mmd::maya::rigid_body_display::viewFilter(frameContext);
    for (const auto& outline : fBodyOutlines)
    {
        if (!outline.valid || !outline.path.isValid())
        {
            continue;
        }
        if (outline.visible && view.shows(outline.path))
        {
            continue;
        }
        std::vector<mmd::maya::rigid_body_display::HighlightCandidate> candidates;
        candidates.reserve(outline.candidates.size());
        for (const auto& candidate : outline.candidates)
        {
            candidates.push_back({
                candidate.roles,
                candidate.visible && view.shows(candidate.path),
                candidate.path.node() == fJoint,
            });
        }
        // Candidates were offered lead-first by the context, so a body shared by
        // the lead and an earlier-selected joint is owned by the lead here.
        const auto owner = mmd::maya::rigid_body_display::highlightOwner(candidates);
        if (!owner.self)
        {
            continue;
        }
        drawManager.setLineWidth(kOutlineWidth);
        mmd::maya::drawRoleColliderWire(drawManager, outline.size, outline.type, owner.roles,
                                        outline.world * fWorldToLocal);
    }
}

void RigidBodyJointGeometryOverride::drawRings(MHWRender::MUIDrawManager& drawManager,
                                               const MPoint& local, double radius,
                                               int segments) const
{
    // Built in the joint's local space like every other guide, so the markers
    // keep the hub's proportions whatever scale the model root carries.  The
    // planes stay the WORLD axis planes (fWorldAxisLocal), so the marks read
    // the same from every joint orientation.  The unit circle is shared.
    const std::vector<MVector>& ring = unitRing(segments);
    for (int a = 0; a < 3; ++a)
    {
        const MVector& u = fWorldAxisLocal.at(static_cast<std::size_t>((a + 1) % 3));
        const MVector& v = fWorldAxisLocal.at(static_cast<std::size_t>((a + 2) % 3));
        MPointArray strip;
        strip.setLength(static_cast<unsigned int>(ring.size()));
        for (std::size_t i = 0; i < ring.size(); ++i)
        {
            strip[static_cast<unsigned int>(i)] = local + (u * ring[i].x + v * ring[i].y) * radius;
        }
        drawManager.mesh(MHWRender::MUIDrawManager::kLineStrip, strip);
    }
}

// Maps the core's guide segments (attachment-A frame = the node's local space)
// to draw calls.
void RigidBodyJointGeometryOverride::drawGuideSegments(
    MHWRender::MUIDrawManager& drawManager,
    const std::vector<mmd::core::joint_display::GuideSegment>& segments, float alpha)
{
    using namespace mmd::core::joint_display;
    const auto colorOf = [&](GuideColor guide)
    {
        MColor color = palette::kGuide;
        if (guide == GuideColor::AxisX)
        {
            color = palette::kAxis.at(0);
        }
        else if (guide == GuideColor::AxisY)
        {
            color = palette::kAxis.at(1);
        }
        else if (guide == GuideColor::AxisZ)
        {
            color = palette::kAxis.at(2);
        }
        color.a *= alpha;
        return color;
    };

    // Equally styled segments that join end to end (an arc, a zigzag) go out
    // as ONE line strip: a dash pattern then runs along the whole arc instead
    // of restarting on every short piece (which can make a dashed arc look
    // solid), and the overlay needs far fewer draw calls.
    MPointArray strip;
    const GuideSegment* style = nullptr;
    const auto flush = [&]
    {
        if (style != nullptr && strip.length() >= 2)
        {
            drawManager.setColor(colorOf(style->color));
            drawManager.setLineWidth(style->width);
            drawManager.setLineStyle(style->dashed ? MHWRender::MUIDrawManager::kDashed
                                                   : MHWRender::MUIDrawManager::kSolid);
            drawManager.mesh(MHWRender::MUIDrawManager::kLineStrip, strip);
        }
        strip.clear();
    };
    for (const GuideSegment& segment : segments)
    {
        const bool continues = style != nullptr && segment.color == style->color &&
                               segment.width == style->width && segment.dashed == style->dashed &&
                               segment.a.x == style->b.x && segment.a.y == style->b.y &&
                               segment.a.z == style->b.z;
        if (!continues)
        {
            flush();
            strip.append(toPoint(segment.a));
        }
        strip.append(toPoint(segment.b));
        style = &segment;
    }
    flush();
    drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
}

void RigidBodyJointGeometryOverride::populateGeometry(const MGeometryRequirements& requirements,
                                                      const MRenderItemList& renderItems,
                                                      MGeometry& data)
{
    if (fState.drawMode == RigidBodyJoint::kDrawOff)
    {
        return;
    }

    // The shared unit hub scaled to this joint's rendered radius, in the
    // node's LOCAL space: the pose is the solver-driven DAG transform (see
    // bakeHub), so this runs only when something was re-baked (first prepare,
    // an edit, or a solver display change) and never per playback frame.
    const Hub& hub = unitHub();
    bakeHub(hub.mesh, fState.hubRadius, fScratchPositions, fScratchNormals);
    mmd::maya::rigid_body_display::fillMeshBuffers(requirements, fScratchPositions, fScratchNormals,
                                                   data);

    // Each hub item indexes its own half of the shared vertices.
    const int itemCount = renderItems.length();
    for (int i = 0; i < itemCount; ++i)
    {
        const MRenderItem* item = renderItems.itemAt(i);
        if (item == nullptr)
        {
            continue;
        }
        const MString& name = item->name();
        for (std::size_t k = 0; k < 4; ++k)
        {
            const auto& indices = hub.indices.at(k);
            if (indices.empty() || name != kHubItems.at(k).name)
            {
                continue;
            }
            MIndexBuffer* indexBuf = data.createIndexBuffer(MGeometry::kUnsignedInt32);
            if (indexBuf == nullptr)
            {
                break;
            }
            const auto numIndices = static_cast<unsigned int>(indices.size());
            void* idxPtr = indexBuf->acquire(numIndices, /*writeOnly=*/true);
            if (idxPtr == nullptr)
            {
                break;
            }
            std::memcpy(idxPtr, indices.data(), numIndices * sizeof(unsigned int));
            indexBuf->commit(idxPtr);
            item->associateWithIndexBuffer(indexBuf);
            break;
        }
    }
    mGeometryDirty = false;
}

// MPxGeometryOverride::cleanUp() is pure virtual, so it must be overridden even
// though it does nothing: render items live in VP2's persistent list and are
// owned by VP2 — there is no per-frame cache to release.
void RigidBodyJointGeometryOverride::cleanUp() {}

// ===========================================================================
// Selection — object-level picking of the hub render items.
// ===========================================================================
bool RigidBodyJointGeometryOverride::refineSelectionPath(const MSelectionInfo& /*selectInfo*/,
                                                         const MRenderItem& /*hitItem*/,
                                                         MDagPath& dagPath,
                                                         MObject& /*geomComponents*/,
                                                         MSelectionMask& objectMask)
{
    MDagPath jointPath;
    if (!MDagPath::getAPathTo(fJoint, jointPath))
    {
        return false;
    }
    dagPath = jointPath;
    objectMask = MSelectionMask(RigidBodyJoint::kSelectionType);
    return true;
}

void RigidBodyJointGeometryOverride::updateSelectionGranularity(const MDagPath& /*dagPath*/,
                                                                MSelectionContext& selectionContext)
{
    selectionContext.setSelectionLevel(MSelectionContext::kObject);
}
