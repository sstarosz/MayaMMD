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
// Opacity of the X-ray pass that keeps a selected joint's overlay readable
// through the geometry in front of it (see addUIDrawables).
constexpr float kGhostAlpha = 0.3f;

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
                // With "show children" off only the selected node itself is
                // drawn, so an ancestor selection must not count as shown.
                if (!fChildren && ancestor.length() < path.length())
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
    // updateRenderItems()).  resolveSolverView also evaluates the solver, so
    // the world it reports is the current one.
    fState.hasAttachments = false;
    fState.active = false;
    for (auto& outline : fBodyOutlines)
    {
        outline.valid = false;
        outline.candidates.clear();
    }
    fSolver = MObject::kNullObj;
    // "picked" = the joint itself is selected (not just one of its ancestors),
    // which is what the overlay ownership rules key off.
    fState.picked = mmd::maya::rigid_body_display::isSelectedJoint(fJoint);
    const RigidBodyJoint::SolverView view = RigidBodyJoint::resolveSolverView(fJoint);
    if (view.solver != nullptr)
    {
        fSolver = view.solverObject;
        def.bodyA = view.definition.bodyA;
        def.bodyB = view.definition.bodyB;
        def.frameT = view.definition.frameT;
        def.frameR = view.definition.frameR;
        fState.active = view.active;
        fState.hasAttachments = view.live;
        fState.attachmentAWorld = view.attachmentA;
        fState.attachmentBWorld = view.attachmentB;
        fState.bodyAWorld = MPoint::origin * view.bodyAWorld;
        fState.bodyBWorld = MPoint::origin * view.bodyBWorld;
        fState.hubRadius = view.hubRadius;
    }

    // Selected joints only: cache the connected colliders (settings, DAG pose
    // and visibility) so a body hidden or isolated out of a view can still be
    // outlined, and decide with the other selected joints which one owns
    // that fallback outline.  Unselected joints draw no overlays at all.
    if (fState.picked && !fSolver.isNull())
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
}

// The hub's colour channel is the core freedom model (joint_display.hpp), so
// the summary and the selected motion guides can never disagree.
void RigidBodyJointGeometryOverride::classifyState()
{
    if (!fState.active)
    {
        fState.freedom = {};
        return;
    }
    fState.freedom =
        mmd::core::joint_display::summarize(mmd::core::joint_display::describe(fState.definition));
}

void RigidBodyJointGeometryOverride::stateColors(MColor& primary, MColor& secondary) const
{
    if (!fState.active)
    {
        primary = secondary = palette::kInactive;
        return;
    }
    primary = fState.freedom.rotates      ? palette::kRotates
              : fState.freedom.translates ? palette::kTranslates
                                          : palette::kWelded;
    // Both hues only for a joint that does both; the wedges alternate them.
    secondary =
        fState.freedom.rotates && fState.freedom.translates ? palette::kTranslates : primary;
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
    const bool inactive = !fState.active;
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
    if (fState.drawMode == RigidBodyJoint::kDrawOff ||
        !mmd::maya::rigid_body_display::isSelected(dagPath))
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
        drawManager.setColor(mmd::maya::rigid_body_display::selectionColor(dagPath));
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

    // View-independent markers, sized from the hub: every BODY CENTRE is a
    // small wire diamond (an octahedron from any view) and every ATTACHMENT a
    // round ring, so a body's centre can never be mistaken for a separated
    // attachment (they used to be identical diamonds for body B).
    const auto bodyMark = [&](const MPoint& local)
    { drawRings(drawManager, local, sizes.bodyMarker, 4); };
    const auto attachmentMark = [&](const MPoint& local)
    { drawRings(drawManager, local, sizes.attachmentMarker, 24); };
    const auto endpoint = [&](const MPoint& body, const MPoint& attachment, MColor color,
                              bool showAttachment, float alpha)
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
        bodyMark(body);
        if (showAttachment)
            attachmentMark(attachment);
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
            bodyMark(body);
        }
    };
    const double separation = ((fAttachmentA - fAttachmentB) * localToWorld).length();
    const bool separate = separation > std::max(1e-5, fState.hubRadius * 0.08);
    const auto overlay = [&](float alpha)
    {
        drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
        if (!fState.active || !fState.hasAttachments)
        {
            inactiveLinks(alpha);
            return;
        }
        endpoint(fLocalA, fAttachmentA, palette::kBodyA, true, alpha);
        endpoint(fLocalB, fAttachmentB, palette::kBodyB, separate, alpha);
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

    mmd::maya::rigid_body_display::fillMeshBuffers(requirements, mesh.positions, mesh.normals,
                                                   data);

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
