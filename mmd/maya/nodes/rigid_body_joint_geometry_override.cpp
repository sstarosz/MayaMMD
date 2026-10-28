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

#include <maya/M3dView.h>
#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MMatrix.h>
#include <maya/MNodeMessage.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MQuaternion.h>
#include <maya/MSelectionList.h>
#include <maya/MShaderManager.h>
#include <maya/MStringArray.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MTypes.h>
#include <maya/MUIDrawManager.h>
#include <maya/MVector.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
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
constexpr HubItem kHubItems[4] = {
    {"mmd_rigid_body_joint_hub_even", true, false},
    {"mmd_rigid_body_joint_hub_odd", false, false},
    {"mmd_rigid_body_joint_hub_even_wire", true, true},
    {"mmd_rigid_body_joint_hub_odd_wire", false, true},
};
// Wedges around the hub sphere (its slices); half of them per colour.
constexpr int kHubWedges = 8;
// Opacity of the selected joint's X-ray ghost pass (see addUIDrawables).
constexpr float kGhostAlpha = 0.3f;

// A mesh: xyz-interleaved positions + normals, a triangle index list, and a
// line index list over the same vertices (the wireframe-mode hub).
struct Mesh
{
    std::vector<float> positions; // xyz interleaved
    std::vector<float> normals;   // xyz interleaved
    std::vector<unsigned int> indices;
    std::vector<unsigned int> lines;

    // Append one vertex, returning its index.
    unsigned int vertex(const MPoint& p, const MVector& n)
    {
        const unsigned int i = static_cast<unsigned int>(positions.size() / 3);
        positions.insert(positions.end(), {static_cast<float>(p.x), static_cast<float>(p.y),
                                           static_cast<float>(p.z)});
        normals.insert(normals.end(), {static_cast<float>(n.x), static_cast<float>(n.y),
                                       static_cast<float>(n.z)});
        return i;
    }
    void tri(unsigned int a, unsigned int b, unsigned int c)
    {
        indices.insert(indices.end(), {a, b, c});
    }
    void line(unsigned int a, unsigned int b) { lines.insert(lines.end(), {a, b}); }

    // A UV sphere with smooth normals; its wire is the latitude and longitude
    // lines.  Winding: outward, so back-face culling shows its outside — the
    // collider override's sphere (RigidBodyShapeGeometryOverride::buildSphere)
    // emits the same triangles with the opposite winding, i.e. inward.
    void appendSphere(const MPoint& center, double radius, int slices, int stacks)
    {
        if (radius <= 0.0 || slices < 3 || stacks < 2)
            return;

        // Interior rings; the poles are single vertices, so the caps have no
        // degenerate (zero-area) triangles.
        const auto ringVertex = [&](int i, int j)
        {
            const double phi = M_PI * i / stacks;
            const double y = std::cos(phi);
            const double ringR = std::sin(phi);
            const double theta = 2.0 * M_PI * j / slices;
            const MVector dir(ringR * std::cos(theta), y, ringR * std::sin(theta));
            return vertex(center + dir * radius, dir);
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
    }
};

// The hub at UNIT radius in its attachment-A frame, split into the
// alternating wedges of kHubItems (one index list per item).  It is the same
// for every joint, so it is built once; populateGeometry scales it by the
// joint's hub radius as it bakes it into the joint's LOCAL space.
struct Hub
{
    Mesh mesh;
    std::vector<unsigned int> indices[4];
};
const Hub& unitHub()
{
    static const Hub hub = []
    {
        Hub out;
        // A compact pickable hub at attachment A.  Springs and limits are
        // drawn per axis only for selected joints (drawMotionRanges): a single
        // marker here could not say WHICH axis is sprung.
        out.mesh.appendSphere(MPoint::origin, 1.0, kHubWedges, 6);

        // A primitive belongs to an even wedge when its centroid's azimuth
        // about the sphere's axis falls in one: wedges alternate all around
        // the hub, so every view direction sees both colours (a top/bottom
        // split would show one colour from above and the other from below).
        const auto evenWedge =
            [&](const std::vector<unsigned int>& list, std::size_t first, std::size_t count)
        {
            double x = 0.0, z = 0.0;
            for (std::size_t k = 0; k < count; ++k)
            {
                x += out.mesh.positions[list[first + k] * 3];
                z += out.mesh.positions[list[first + k] * 3 + 2];
            }
            const double azimuth = std::atan2(z, x) + (z < 0.0 ? 2.0 * M_PI : 0.0);
            return static_cast<int>(azimuth / (2.0 * M_PI / kHubWedges)) % 2 == 0;
        };
        for (std::size_t item = 0; item < 4; ++item)
        {
            const HubItem& spec = kHubItems[item];
            const std::vector<unsigned int>& source = spec.wire ? out.mesh.lines : out.mesh.indices;
            const std::size_t stride = spec.wire ? 2 : 3;
            for (std::size_t i = 0; i + stride <= source.size(); i += stride)
                if (evenWedge(source, i, stride) == spec.primary)
                    out.indices[item].insert(
                        out.indices[item].end(), source.begin() + static_cast<std::ptrdiff_t>(i),
                        source.begin() + static_cast<std::ptrdiff_t>(i + stride));
        }
        return out;
    }();
    return hub;
}

// Read a k3Double (or angle3) plug; a missing plug reads as zero.
[[nodiscard]] mmd::core::Double3 readDouble3(const MPlug& plug)
{
    return plug.isNull() ? mmd::core::Double3{} : mmd::maya::readPlugDouble3(plug);
}

[[nodiscard]] MPoint toPoint(const mmd::core::Double3& d)
{
    return MPoint(d.x, d.y, d.z);
}

// Fill one vertex buffer (`dimension` floats per vertex) with `numVerts`
// elements.  NOTE: acquire()'s size is the ELEMENT count, not bytes.
void fillVertexBuffer(MVertexBuffer* buffer, const std::vector<float>& data, unsigned int numVerts,
                      int dimension, const MVertexBufferDescriptor& desc)
{
    if (buffer == nullptr || numVerts == 0 || data.size() < numVerts * dimension)
        return;
    if (desc.dataType() != MGeometry::kFloat || desc.dimension() != dimension)
        return;
    void* ptr = buffer->acquire(numVerts, /*writeOnly=*/true);
    if (ptr == nullptr)
        return;
    std::memcpy(ptr, data.data(), numVerts * dimension * sizeof(float));
    buffer->commit(ptr);
}

// Whole-joint summary for the unselected state display: which kinds of
// motion the joint permits.  Same freedom model as the selected motion
// guides (joint_display.hpp); a joint that constrains nothing has no
// freedom to show.
struct Freedom
{
    bool translates = false; // some linear axis is limited or free
    bool rotates = false;    // some angular axis is limited or free
};
Freedom summarize(const mmd::core::joint_display::Ranges& ranges)
{
    Freedom out;
    for (int i = 0; i < 3; ++i)
    {
        out.translates =
            out.translates || ranges.linear[i].kind() != mmd::core::joint_display::RangeKind::Fixed;
        out.rotates =
            out.rotates || ranges.angular[i].kind() != mmd::core::joint_display::RangeKind::Fixed;
    }
    return out;
}

// Isolate-select state of the viewport being drawn.  Maya < 2026 cannot query
// it through M3dView, so there every view counts as showing everything and
// only hidden bodies get a fallback outline.
class ViewFilter
{
  public:
    explicit ViewFilter(const MHWRender::MFrameContext& frameContext)
    {
#if MAYA_API_VERSION >= 20260000
        MString destination;
        if (frameContext.renderingDestination(destination) != MFrameContext::k3dViewport)
            return;
        M3dView view;
        if (!M3dView::getM3dViewFromModelPanel(destination, view) &&
            !M3dView::getM3dViewFromModelEditor(destination, view))
            return;
        fIsolated = view.viewSelected();
        fChildren = view.showViewSelectedChildren(nullptr);
        for (unsigned int i = 0; fIsolated && i < view.numViewSelectedObjects(); ++i)
        {
            MStringArray names;
            if (view.viewSelectedObject(i, names))
                for (unsigned int j = 0; j < names.length(); ++j)
                    fIncluded.add(names[j]);
        }
#else
        (void) frameContext;
#endif
    }

    bool shows(const MDagPath& path) const
    {
        if (!fIsolated)
            return true;
        for (unsigned int i = 0; i < fIncluded.length(); ++i)
        {
            MDagPath member;
            if (!fIncluded.getDagPath(i, member))
                continue;
            MDagPath ancestor = path;
            while (ancestor.length() > 0)
            {
                if (!fChildren && ancestor.length() + 1 < path.length())
                    break;
                if (ancestor == member)
                    return true;
                ancestor.pop();
            }
        }
        return false;
    }

  private:
    bool fIsolated = false;
    bool fChildren = true;
    MSelectionList fIncluded;
};
} // namespace

// ===========================================================================
// Lifecycle / registration
// ===========================================================================
MHWRender::MPxGeometryOverride* RigidBodyJointGeometryOverride::creator(const MObject& obj)
{
    return new RigidBodyJointGeometryOverride(obj);
}

RigidBodyJointGeometryOverride::RigidBodyJointGeometryOverride(const MObject& obj)
    : MHWRender::MPxGeometryOverride(obj), fJoint(obj)
{
    fPlugs.drawMode = MPlug(obj, RigidBodyJoint::aDrawMode);
    fPlugs.type = MPlug(obj, RigidBodyJoint::aJointType);
    fPlugs.linearMin = MPlug(obj, RigidBodyJoint::aJointLinearMin);
    fPlugs.linearMax = MPlug(obj, RigidBodyJoint::aJointLinearMax);
    fPlugs.angularMin = MPlug(obj, RigidBodyJoint::aJointAngularMin);
    fPlugs.angularMax = MPlug(obj, RigidBodyJoint::aJointAngularMax);
    fPlugs.linearStiffness = MPlug(obj, RigidBodyJoint::aJointLinearStiffness);
    fPlugs.angularStiffness = MPlug(obj, RigidBodyJoint::aJointAngularStiffness);
    fPlugs.solver = MPlug(obj, RigidBodyJoint::aSolver);
    fPlugs.solverStep = MPlug(obj, RigidBodyJoint::aSolverStep);
}

RigidBodyJointGeometryOverride::~RigidBodyJointGeometryOverride()
{
    for (const auto& watch : fOutlineWatches)
        if (watch.callback)
            MMessage::removeCallback(watch.callback);
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
        MHWRender::MRenderer::setGeometryDrawDirty(fJoint, false);
}

void RigidBodyJointGeometryOverride::dependencyDirty(MObject&, MPlug&, void* clientData)
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
                nodes.push_back(node);
            path.pop();
        }
    };
    for (const auto& outline : fBodyOutlines)
        if (outline.valid && !outline.candidates.empty())
        {
            appendPath(outline.path);
            for (const auto& candidate : outline.candidates)
                appendPath(candidate.path);
        }
    for (auto it = fOutlineWatches.begin(); it != fOutlineWatches.end();)
    {
        if (!it->node.isValid() || !it->node.isAlive() ||
            std::find(nodes.begin(), nodes.end(), it->node.object()) == nodes.end())
        {
            if (it->callback)
                MMessage::removeCallback(it->callback);
            it = fOutlineWatches.erase(it);
        }
        else
            ++it;
    }
    for (MObject node : nodes)
    {
        const bool watched =
            std::any_of(fOutlineWatches.begin(), fOutlineWatches.end(),
                        [&](const DependencyWatch& watch) { return watch.node.object() == node; });
        if (!watched)
            fOutlineWatches.push_back({MObjectHandle(node), MNodeMessage::addNodeDirtyPlugCallback(
                                                                node, dependencyDirty, this)});
    }
}

bool RigidBodyJointGeometryOverride::requiresGeometryUpdate() const
{
    // True would force a full update (solver queries + buffer upload) on
    // EVERY viewport refresh, camera tumbles included.  Everything the joint
    // draws from outside its node dirties it instead: solver steps and
    // edits through RigidBodyJoint (setDependentsDirty / postEvaluation),
    // selection and isolate changes through rigid_body_display, hidden
    // bodies through the outline watches.
    return false;
}

bool RigidBodyJointGeometryOverride::requiresUpdateRenderItems(const MDagPath& /*dagPath*/) const
{
    return true;
}

// ===========================================================================
// Data preparation
// ===========================================================================
void RigidBodyJointGeometryOverride::updateDG()
{
    if (fPlugs.drawMode.isNull())
        return;

    fState.drawMode = fPlugs.drawMode.asShort();
    auto& def = fState.definition;
    def.type = fPlugs.type.isNull() ? 0 : fPlugs.type.asShort();
    // The bodies are connections; the solver turns them into its indices.
    const MObject bodyShapes[2] = {RigidBodyJoint::bodyShape(fJoint, RigidBodyJoint::aJointBodyA),
                                   RigidBodyJoint::bodyShape(fJoint, RigidBodyJoint::aJointBodyB)};
    def.bodyA = -1;
    def.bodyB = -1;
    def.linearMin = readDouble3(fPlugs.linearMin);
    def.linearMax = readDouble3(fPlugs.linearMax);
    def.angularMin = readDouble3(fPlugs.angularMin);
    def.angularMax = readDouble3(fPlugs.angularMax);
    def.linearStiffness = readDouble3(fPlugs.linearStiffness);
    def.angularStiffness = readDouble3(fPlugs.angularStiffness);

    // The bodies' CURRENT poses and attachment frames come from the owning
    // solver (world space; mapped to the node's LOCAL space in
    // updateRenderItems()).
    fState.hasAttachments = false;
    fState.active = false;
    for (auto& outline : fBodyOutlines)
    {
        outline.valid = false;
        outline.candidates.clear();
    }
    fSolver = MObject::kNullObj;
    fState.modelHubRadius = 0.0;
    fState.hubScale = 1.0;
    MPlugArray solverDestinations;
    if (!fPlugs.solver.isNull())
        // The command registers joint.solver -> solver.jointShapes[index].
        // The solver is a DESTINATION of this message, not its source.
        fPlugs.solver.connectedTo(solverDestinations, false, true);
    const bool selected = mmd::maya::rigid_body_display::isSelectedJoint(fJoint);
    fState.picked = selected;
    if (solverDestinations.length() > 0)
    {
        fSolver = solverDestinations[0].node();
        // solver.outStep -> solverStep: reading it evaluates the solver for
        // the current time before its world is queried below.
        if (!fPlugs.solverStep.isNull())
            fPlugs.solverStep.asDouble();

        MFnDependencyNode solverFn(fSolver);
        if (auto* node = dynamic_cast<RigidBodyNode*>(solverFn.userNode()))
        {
            def.bodyA = node->bodyIndex(bodyShapes[0]);
            def.bodyB = node->bodyIndex(bodyShapes[1]);
            fState.active =
                node->jointStatus(def) == mmd::core::RigidBodySimulation::JointStatus::eActive;
            MMatrix bodyMatrixA, bodyMatrixB;
            fState.hasAttachments =
                RigidBodyJoint::readJointFrame(fJoint, RigidBodyNode::groupWorldMatrix(fSolver),
                                               def.frameT, def.frameR) &&
                node->jointViewportFrames(def, bodyMatrixA, bodyMatrixB, fState.attachmentAWorld,
                                          fState.attachmentBWorld);
            fState.bodyAWorld = MPoint::origin * bodyMatrixA;
            fState.bodyBWorld = MPoint::origin * bodyMatrixB;
            fState.modelHubRadius = node->jointHubRadius();
        }

        // The hub size multiplier lives on the solver (one knob per model).
        const MPlug scalePlug(fSolver, RigidBodyNode::aJointDisplayScale);
        if (!scalePlug.isNull())
            fState.hubScale = std::max(0.05, scalePlug.asDouble());
    }

    // Selected joints only: cache the connected colliders (settings, DAG pose
    // and visibility) so a body hidden or isolated out of a view can still be
    // outlined, and decide with the other selected joints which one owns
    // that fallback outline.  Unselected joints draw no overlays at all.
    if (selected && !fSolver.isNull())
    {
        for (int end = 0; end < 2; ++end)
        {
            const MObject& shape = bodyShapes[end];
            auto& outline = fBodyOutlines[end];
            if (shape.isNull() || !MDagPath::getAPathTo(shape, outline.path))
                continue;
            outline.size = toPoint(readDouble3(MPlug(shape, RigidBodyShape::aBodyShapeSize)));
            const MPlug typePlug(shape, RigidBodyShape::aBodyColliderType);
            outline.type = typePlug.isNull() ? 0 : typePlug.asShort();
            outline.world = outline.path.inclusiveMatrix();
            outline.visible = outline.path.isVisible();
            outline.valid = true;
        }
        // Candidates arrive lead-first from the context, so a hidden body
        // shared by several selected joints is owned by the lead.
        for (const auto& joint : mmd::maya::rigid_body_display::selectedJoints())
        {
            if (joint.solver.object() != fSolver || !joint.path.isValid())
                continue;
            for (auto& outline : fBodyOutlines)
            {
                if (!outline.valid)
                    continue;
                const MObject shape = outline.path.node();
                const unsigned int roles =
                    (joint.bodyA.object() == shape ? palette::kRoleA : 0u) |
                    (joint.bodyB.object() == shape ? palette::kRoleB : 0u);
                if (roles)
                    outline.candidates.push_back({joint.path, roles, joint.path.isVisible()});
            }
        }
    }
    updateOutlineWatches();
    classifyState();
    deriveSizes();
}

// Same freedom model as the selected motion guides (joint_display.hpp);
// a joint that constrains nothing has no freedom to show.
void RigidBodyJointGeometryOverride::classifyState()
{
    if (!fState.active)
    {
        fState.state = kStateInactive;
        return;
    }
    const Freedom freedom = summarize(mmd::core::joint_display::describe(fState.definition));
    fState.state = freedom.translates
                       ? (freedom.rotates ? kStateRotatesTranslates : kStateTranslates)
                       : (freedom.rotates ? kStateRotates : kStateWelded);
}

// Every joint of a model shares one hub size, derived by the solver from the
// model's rest poses (RigidBodyNode's modelHubRadius), so the hubs look alike
// and never change with the simulation.  The solver's jointDisplayScale
// stays live.
void RigidBodyJointGeometryOverride::deriveSizes()
{
    const double base = fState.modelHubRadius > 0.0 ? fState.modelHubRadius
                                                    : mmd::core::joint_display::kFallbackHubRadius;
    fState.hubRadius = base * fState.hubScale;
}

void RigidBodyJointGeometryOverride::stateColors(MColor& primary, MColor& secondary) const
{
    switch (fState.state)
    {
    case kStateRotates:
        primary = secondary = palette::kRotates;
        break;
    case kStateTranslates:
        primary = secondary = palette::kTranslates;
        break;
    case kStateRotatesTranslates:
        primary = palette::kRotates;
        secondary = palette::kTranslates;
        break;
    case kStateInactive:
        primary = secondary = palette::kInactive;
        break;
    default:
        primary = secondary = palette::kWelded;
        break;
    }
}

bool RigidBodyJointGeometryOverride::isSelected(const MDagPath& path)
{
    const MHWRender::DisplayStatus status = MHWRender::MGeometryUtilities::displayStatus(path);
    return status == MHWRender::kLead || status == MHWRender::kActive;
}

MColor RigidBodyJointGeometryOverride::selectionColor(const MDagPath& path)
{
    const MColor sel = MHWRender::MGeometryUtilities::wireframeColor(path);
    // Guard: a near-black selection colour would hide the wire.
    return sel.r + sel.g + sel.b < 0.01f ? palette::kSelectionFallback : sel;
}

void RigidBodyJointGeometryOverride::updateRenderItems(const MDagPath& dagPath,
                                                       MRenderItemList& renderItems)
{
    const bool visible = fState.drawMode != RigidBodyJoint::kDrawOff;

    // Attachment and body positions, in the joint node's LOCAL space (the space
    // the render item's geometry is built in — VP2 applies the joint's DAG matrix).
    // Without a live solver snapshot nothing but the hub is drawn, so the hub
    // simply sits at the joint frame.
    fLocalToWorld = dagPath.inclusiveMatrix();
    const MMatrix worldToLocal = fLocalToWorld.inverse();
    const MMatrix hubWorld = fState.hasAttachments ? fState.attachmentAWorld : fLocalToWorld;
    fHubToLocal = hubWorld * worldToLocal;
    fAttachmentA = MPoint::origin * hubWorld * worldToLocal;
    fAttachmentB = fState.hasAttachments ? MPoint::origin * fState.attachmentBWorld * worldToLocal
                                         : fAttachmentA;
    if (fState.hasAttachments)
    {
        fLocalA = fState.bodyAWorld * worldToLocal;
        fLocalB = fState.bodyBWorld * worldToLocal;
    }

    // The SOLID keeps its STATE colour even when selected. The selection
    // highlight is a native-coloured wire drawn in addUIDrawables(), not a
    // tint of the whole hub — so the lead hub is outlined green and the rest
    // of a multiple selection white, instead of flooding the hubs.
    MColor primary, secondary;
    stateColors(primary, secondary);
    // An inactive hub is hollow: no solid, only its dashed wire, in every
    // display mode (so it stays visible and pickable while shaded).
    const bool inactive = fState.state == kStateInactive;
    const auto shadedModes =
        static_cast<MGeometry::DrawMode>(MGeometry::kShaded | MGeometry::kTextured);
    for (const HubItem& spec : kHubItems)
    {
        MRenderItem* item = nullptr;
        const int index = renderItems.indexOf(spec.name);
        if (index >= 0)
            item = renderItems.itemAt(index);
        else
        {
            item = MRenderItem::Create(spec.name, MRenderItem::NonMaterialSceneItem,
                                       spec.wire ? MGeometry::kLines : MGeometry::kTriangles);
            if (item == nullptr)
                continue;
            // The hub respects scene depth.
            item->depthPriority(MRenderItem::sDormantWireDepthPriority);
            // Pick under the joint's own selection type (priority), not "locator".
            item->setSelectionMask(MSelectionMask(RigidBodyJoint::kSelectionType));
            renderItems.append(item);
        }
        // Must match the viewport's display modes or the item never draws.
        item->setDrawMode(!spec.wire  ? shadedModes
                          : inactive ? static_cast<MGeometry::DrawMode>(shadedModes |
                                                                        MGeometry::kWireframe)
                                     : MGeometry::kWireframe);
        // Vertices are joint-local: populateGeometry bakes the attachment-to-
        // local transform in, so VP2 applies only the joint's DAG transform
        // (including its animated parents).  Supplying a world matrix here
        // is unreliable in the geometry-override draw path.
        item->setMatrix(nullptr);
        item->enable(visible && (spec.wire || !inactive));
        // Shared shaders per colour (rigid_body_display).
        namespace display = mmd::maya::rigid_body_display;
        const MColor& color = spec.primary ? primary : secondary;
        MHWRender::MShaderInstance* shader = !spec.wire ? display::solidShader(color)
                                             : inactive ? display::dashedWireShader(color)
                                                        : display::wireShader(color);
        if (shader != nullptr)
            item->setShader(shader);
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
    if (fState.drawMode == RigidBodyJoint::kDrawOff || !isSelected(dagPath))
        return;

    // Native selection outline: the solid keeps its state colour and the pick
    // is marked with Maya's OWN selection colour drawn as a wire around the
    // hub — green for the LEAD item, white for the other members of a multiple
    // selection — mirroring how RigidBodyShapeGeometryOverride outlines a
    // selected collider. Drawn before the attachment overlay so it shows even
    // when no solver pose is available.
    {
        drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
        drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
        drawManager.setLineWidth(1.75f);
        drawManager.setColor(selectionColor(dagPath));
        mmd::maya::drawColliderWire(drawManager, MPoint(fState.hubRadius, 0.0, 0.0),
                                    RigidBodyShape::kColliderSphere, fHubToLocal);
        drawManager.endDrawable();
    }

    // A joint drawn as selected only because a parent group is (the whole
    // {model}_Joints group, say) keeps the wire but draws no overlay.
    if (!fState.picked)
        return;

    // Cached in updateRenderItems(): the draw phase reads no DAG state.
    const MMatrix& localToWorld = fLocalToWorld;
    const MMatrix worldToLocal = localToWorld.inverse();
    const auto sizes = mmd::core::joint_display::guideSizes(fState.hubRadius);

    // View-independent markers, sized from the hub: A is a ring in each axis
    // plane (round from any view), B a wire octahedron (a diamond from any view).
    const auto marker = [&](const MPoint& local, bool diamond, double radius)
    { drawRings(drawManager, local, radius, diamond ? 4 : 24); };
    const auto endpoint = [&](const MPoint& body, const MPoint& attachment, MColor color,
                              bool diamond, bool showAttachment, float alpha)
    {
        MPointArray line;
        line.append(body);
        line.append(attachment);
        // One stroke. A second, wider black stroke at the same depth can
        // obscure the colored stroke depending on VP2 draw ordering.
        color.a *= alpha;
        drawManager.setColor(color);
        drawManager.setLineWidth(3.0f);
        drawManager.mesh(MHWRender::MUIDrawManager::kLines, line);
        drawManager.setLineWidth(2.5f);
        marker(body, diamond, sizes.bodyMarker);
        if (showAttachment)
            marker(attachment, diamond, diamond ? sizes.bodyMarker : sizes.attachmentMarker);
    };
    // An inactive joint holds neither body: dashed links from its hub to the
    // bodies it names (whatever is connected), and no motion ranges.
    const auto inactiveLinks = [&](float alpha)
    {
        for (int end = 0; end < 2; ++end)
        {
            const BodyOutline& outline = fBodyOutlines[end];
            if (!outline.valid)
                continue;
            MColor color = end == 0 ? palette::kBodyA
                                    : palette::kBodyB;
            color.a *= alpha;
            drawManager.setColor(color);
            drawManager.setLineWidth(2.5f);
            const MPoint body = MPoint::origin * outline.world * worldToLocal;
            MPointArray line;
            line.append(body);
            line.append(fAttachmentA);
            drawManager.setLineStyle(MHWRender::MUIDrawManager::kDashed);
            drawManager.mesh(MHWRender::MUIDrawManager::kLines, line);
            drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
            marker(body, end == 1, sizes.bodyMarker);
        }
    };
    // The frame is the joint's transform, where the Move manipulator sits,
    // but the hub rides on body A.  Once they part, a dashed ghost of the hub
    // at the frame, tethered to the hub, shows where the frame is.
    const bool frameApart =
        ((fAttachmentA - MPoint::origin) * localToWorld).length() > fState.hubRadius * 1.5;
    const auto restFrame = [&](float alpha)
    {
        if (!frameApart)
            return;
        MColor color = palette::kGuide;
        color.a *= alpha;
        drawManager.setColor(color);
        drawManager.setLineWidth(1.5f);
        drawManager.setLineStyle(MHWRender::MUIDrawManager::kDashed);
        drawRings(drawManager, MPoint::origin, fState.hubRadius, 24);
        MPointArray tether;
        tether.append(MPoint::origin);
        tether.append(fAttachmentA);
        drawManager.mesh(MHWRender::MUIDrawManager::kLines, tether);
        drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    };
    const double separation = ((fAttachmentA - fAttachmentB) * localToWorld).length();
    const bool separate = separation > std::max(1e-5, fState.hubRadius * 0.08);
    const auto overlay = [&](float alpha)
    {
        drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
        restFrame(alpha);
        if (!fState.active || !fState.hasAttachments)
        {
            inactiveLinks(alpha);
            return;
        }
        endpoint(fLocalA, fAttachmentA, palette::kBodyA, false, true, alpha);
        endpoint(fLocalB, fAttachmentB, palette::kBodyB, true, separate, alpha);
        // Coincident endpoints use one ring-and-center mark, avoiding overlapping outlines.
        if (!separate)
        {
            drawManager.setPointSize(5.0f);
            drawManager.point(fAttachmentA);
        }
        if (fState.drawMode != RigidBodyJoint::kDrawHub)
            drawMotionRanges(drawManager, alpha);
    };

    // Joints usually sit inside cloth and hair, so a depth-tested diagram is
    // clipped exactly where it matters.  A faint X-ray ghost keeps the hidden
    // parts readable; the full-strength depth-tested pass reads as "in front".
    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.beginDrawInXray();
    overlay(kGhostAlpha);
    drawManager.endDrawInXray();
    drawManager.endDrawable();

    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
    drawIsolatedBodyOutlines(drawManager, frameContext, worldToLocal);
    overlay(1.0f);
    drawManager.endDrawable();
}

void RigidBodyJointGeometryOverride::drawIsolatedBodyOutlines(
    MHWRender::MUIDrawManager& drawManager, const MHWRender::MFrameContext& frameContext,
    const MMatrix& worldToLocal) const
{
    if (std::none_of(std::begin(fBodyOutlines), std::end(fBodyOutlines),
                     [](const BodyOutline& outline) { return outline.valid; }))
        return;
    const ViewFilter view(frameContext);
    for (const auto& outline : fBodyOutlines)
    {
        if (!outline.valid || !outline.path.isValid())
            continue;
        if (outline.visible && view.shows(outline.path))
            continue;
        std::vector<mmd::maya::rigid_body_display::HighlightCandidate> candidates;
        for (const auto& candidate : outline.candidates)
            candidates.push_back({candidate.roles, candidate.visible && view.shows(candidate.path),
                                  candidate.path.node() == fJoint});
        // Candidates were offered lead-first by the context, so a body shared by
        // the lead and an earlier-selected joint is owned by the lead here.
        const auto owner = mmd::maya::rigid_body_display::highlightOwner(candidates);
        if (!owner.self)
            continue;
        drawManager.setLineWidth(4.0f);
        mmd::maya::drawRoleColliderWire(drawManager, outline.size, outline.type, owner.roles,
                                        outline.world * worldToLocal);
    }
}

void RigidBodyJointGeometryOverride::drawRings(MHWRender::MUIDrawManager& drawManager,
                                               const MPoint& local, double radius,
                                               int segments) const
{
    const MMatrix worldToLocal = fLocalToWorld.inverse();
    const MPoint world = local * fLocalToWorld;
    const MVector axes[3] = {MVector::xAxis, MVector::yAxis, MVector::zAxis};
    for (int a = 0; a < 3; ++a)
    {
        const MVector& u = axes[(a + 1) % 3];
        const MVector& v = axes[(a + 2) % 3];
        MPointArray strip;
        for (int i = 0; i <= segments; ++i)
        {
            const double angle = 2.0 * M_PI * i / segments;
            strip.append((world + (u * std::cos(angle) + v * std::sin(angle)) * radius) *
                         worldToLocal);
        }
        drawManager.mesh(MHWRender::MUIDrawManager::kLineStrip, strip);
    }
}

void RigidBodyJointGeometryOverride::drawMotionRanges(MHWRender::MUIDrawManager& drawManager,
                                                      float alpha) const
{
    using namespace mmd::core::joint_display;
    // The geometry lives in core (joint_display.hpp); this maps it from the
    // attachment-A frame into the viewport.
    GuideInput input;
    input.ranges = describe(fState.definition);
    input.sizes = guideSizes(fState.hubRadius);
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

    const auto colorOf = [&](GuideColor guide)
    {
        MColor color = guide == GuideColor::AxisX   ? palette::kAxis[0]
                       : guide == GuideColor::AxisY ? palette::kAxis[1]
                       : guide == GuideColor::AxisZ ? palette::kAxis[2]
                                                    : palette::kGuide;
        color.a *= alpha;
        return color;
    };

    // Equally styled segments that join end to end (an arc, a zigzag) go out
    // as ONE line strip: a dash pattern then runs along the whole arc instead
    // of restarting on every short piece (which can make a dashed arc look
    // solid), and the overlay needs far fewer draw calls.
    const std::vector<GuideSegment> segments = motionGuides(input);
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
            strip.append(toPoint(segment.a) * fHubToLocal);
        }
        strip.append(toPoint(segment.b) * fHubToLocal);
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
        return;

    // Scale a COPY of the shared unit hub to this joint's radius and bake it
    // into the joint's LOCAL space. VP2 applies the DAG matrix once:
    // vertex * attachmentWorld * inverse(jointWorld) * jointWorld.
    const Hub& hub = unitHub();
    Mesh mesh = hub.mesh;
    const double radius = fState.hubRadius;
    const MMatrix normalToLocal = fHubToLocal.inverse().transpose();
    for (std::size_t i = 0; i < mesh.positions.size(); i += 3)
    {
        const MPoint p = MPoint(mesh.positions[i] * radius, mesh.positions[i + 1] * radius,
                                mesh.positions[i + 2] * radius) *
                         fHubToLocal;
        const MVector n =
            (MVector(mesh.normals[i], mesh.normals[i + 1], mesh.normals[i + 2]) * normalToLocal)
                .normal();
        mesh.positions[i] = static_cast<float>(p.x);
        mesh.positions[i + 1] = static_cast<float>(p.y);
        mesh.positions[i + 2] = static_cast<float>(p.z);
        mesh.normals[i] = static_cast<float>(n.x);
        mesh.normals[i + 1] = static_cast<float>(n.y);
        mesh.normals[i + 2] = static_cast<float>(n.z);
    }

    const unsigned int numVerts = static_cast<unsigned int>(mesh.positions.size() / 3);

    // Create vertex buffers from the shaders' OWN requirements (position +
    // normal [+ uv] for the stock surface shader).  Fall back to manual
    // descriptors if the requirements come back empty.
    const MVertexBufferDescriptorList& descList = requirements.vertexRequirements();
    if (descList.length() > 0)
    {
        MVertexBufferDescriptor desc;
        for (int i = 0; i < descList.length(); ++i)
        {
            if (!descList.getDescriptor(i, desc))
                continue;
            switch (desc.semantic())
            {
            case MGeometry::kPosition:
                fillVertexBuffer(data.createVertexBuffer(desc), mesh.positions, numVerts, 3, desc);
                break;
            case MGeometry::kNormal:
                fillVertexBuffer(data.createVertexBuffer(desc), mesh.normals, numVerts, 3, desc);
                break;
            case MGeometry::kTexture:
            {
                const std::vector<float> dummyUvs(numVerts * 2, 0.0f);
                fillVertexBuffer(data.createVertexBuffer(desc), dummyUvs, numVerts, 2, desc);
                break;
            }
            default:
                break;
            }
        }
    }
    else
    {
        const MVertexBufferDescriptor posDesc("position", MGeometry::kPosition, MGeometry::kFloat,
                                              3);
        const MVertexBufferDescriptor nrmDesc("normal", MGeometry::kNormal, MGeometry::kFloat, 3);
        const MVertexBufferDescriptor uvDesc("uv", MGeometry::kTexture, MGeometry::kFloat, 2);
        fillVertexBuffer(data.createVertexBuffer(posDesc), mesh.positions, numVerts, 3, posDesc);
        fillVertexBuffer(data.createVertexBuffer(nrmDesc), mesh.normals, numVerts, 3, nrmDesc);
        const std::vector<float> dummyUvs(numVerts * 2, 0.0f);
        fillVertexBuffer(data.createVertexBuffer(uvDesc), dummyUvs, numVerts, 2, uvDesc);
    }

    // Each hub item indexes its own half of the shared vertices.
    for (int i = 0; i < renderItems.length(); ++i)
    {
        const MRenderItem* item = renderItems.itemAt(i);
        if (item == nullptr)
            continue;
        for (std::size_t k = 0; k < 4; ++k)
        {
            if (item->name() != kHubItems[k].name || hub.indices[k].empty())
                continue;
            const auto& indices = hub.indices[k];
            MIndexBuffer* indexBuf = data.createIndexBuffer(MGeometry::kUnsignedInt32);
            if (indexBuf == nullptr)
                break;
            const unsigned int numIndices = static_cast<unsigned int>(indices.size());
            void* idxPtr = indexBuf->acquire(numIndices, /*writeOnly=*/true);
            if (idxPtr == nullptr)
                break;
            std::memcpy(idxPtr, indices.data(), numIndices * sizeof(unsigned int));
            indexBuf->commit(idxPtr);
            item->associateWithIndexBuffer(indexBuf);
            break;
        }
    }
}

void RigidBodyJointGeometryOverride::cleanUp()
{
    // Render items live in VP2's persistent list and are owned by VP2 —
    // nothing to release per frame.
}

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
        return false;
    dagPath = jointPath;
    objectMask = MSelectionMask(RigidBodyJoint::kSelectionType);
    return true;
}

void RigidBodyJointGeometryOverride::updateSelectionGranularity(const MDagPath& /*dagPath*/,
                                                                MSelectionContext& selectionContext)
{
    selectionContext.setSelectionLevel(MSelectionContext::kObject);
}
