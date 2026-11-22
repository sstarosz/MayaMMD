/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint_geometry_override.hpp
 *
 * RigidBodyJointGeometryOverride — viewport renderer for one pmxRigidBodyJoint,
 * parallel to RigidBodyShapeGeometryOverride for the per-body colliders.
 *
 * Every joint draws a compact, pickable HUB at its live attachment A: lit in
 * shaded modes, a wire sphere in wireframe mode.  The hub's colour is the
 * joint's freedom state (welded / rotates / translates; a joint that does both
 * shows both colours in alternating wedges, visible from every direction; the
 * palette is at the top of the .cpp), from the shared freedom model
 * (joint_display.hpp).
 *
 * A joint that constrains nothing (a missing, disabled or shared body, or
 * two bone-driven bodies) has a hollow dashed hub instead.
 *
 * A selected joint gets a selection-coloured wire around the hub.  A joint
 * picked DIRECTLY (itself or its transform, not a parent group: selecting
 * {model}_Joints would flood the view with ~400 overlays) also adds, as UI
 * drawables: links from each body's centre to its own attachment (A rings, B
 * diamonds); in Ranges mode the motion-range guides with each rotation axis's
 * current angle (joint_display.hpp); and fallback outlines for connected
 * bodies hidden or isolated out of the view.  An inactive joint draws only
 * dashed links to its bodies.  The overlay is drawn twice — a faint X-ray
 * ghost and a depth-tested pass — so joints inside cloth and hair stay
 * readable.  Nothing depends on the camera: every viewport shows the same
 * overlay, sized relative to the hub.
 *
 * The hub is LOCAL geometry at the node's origin, baked at the hub radius: the
 * solver drives the joint's transform to the joint's live attachment on body A
 * (outJointTranslate/outJointRotate), so a playback frame costs no override
 * work at all — VP2 moves the drawing with the ordinary DAG transform, exactly
 * like a collider under its solver-driven guide.  Only a picked joint is
 * re-prepared per frame, because its overlay is world-space UI drawables.
 *
 * Selection events and the shared surface shaders live in
 * rigid_body_display.  Simulation steps reach the joint through its
 * solverStep input, which refreshes selected overlays only. The solver's
 * configuration-only jointDisplay input supplies hub size and activity.
 * VP2 owns the render items.
 *
 * Registered by MayaMMD.mll under the pmxRigidBodyJoint classification.
 */

#pragma once

#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MEvaluationNode.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MMessage.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MPxGeometryOverride.h>
#include <maya/MVector.h>

#include <array>
#include <cstdint>
#include <vector>

#include "joint_display.hpp"
#include "rigid_body_simulation.hpp"

// ===========================================================================
// RigidBodyJointGeometryOverride
// ===========================================================================
class RigidBodyJointGeometryOverride : public MHWRender::MPxGeometryOverride
{
  public:
    static MHWRender::MPxGeometryOverride* creator(const MObject& obj);

    // Stateless notification bridge from the data node. Display decisions
    // belong here; the node keeps no renderer or solver cache.
    static void inputChanged(const MObject& node, const MPlug& plug);
    static void inputsEvaluated(const MObject& node, const MEvaluationNode& evaluationNode);
    static void poseChanged(const MObject& node);

    explicit RigidBodyJointGeometryOverride(const MObject& obj);
    ~RigidBodyJointGeometryOverride() override;

    MHWRender::DrawAPI supportedDrawAPIs() const override;

    // False: the hub is local geometry under the node's solver-driven
    // transform, so a playback frame moves it through the DAG without any
    // override work.  populateGeometry() runs on the first prepare and after an
    // radius change or new VP2 stream requirements.
    bool requiresGeometryUpdate() const override;
    // False: the item state changes only on edits, which mark the drawing
    // dirty (a hard `true` does not by itself make VP2 re-prepare the object).
    bool requiresUpdateRenderItems(const MDagPath& dagPath) const override;
    bool isIndexingDirty(const MHWRender::MRenderItem& item) override;
    bool isStreamDirty(const MHWRender::MVertexBufferDescriptor& desc) override;

    void updateDG() override;
    void updateRenderItems(const MDagPath& dagPath, MRenderItemList& renderItems) override;
    bool hasUIDrawables() const override;
    void addUIDrawables(const MDagPath& dagPath, MHWRender::MUIDrawManager& drawManager,
                        const MHWRender::MFrameContext& frameContext) override;
    void populateGeometry(const MGeometryRequirements& requirements,
                          const MRenderItemList& renderItems, MGeometry& data) override;
    // Pure virtual in MPxGeometryOverride; nothing to release (see the .cpp).
    void cleanUp() override;

    // Selection — the hub is the only pickable item; the overlays are
    // unselectable. Granularity is object-level (the joint node is selected).
    bool refineSelectionPath(const MSelectionInfo& selectInfo, const MRenderItem& hitItem,
                             MDagPath& dagPath, MObject& geomComponents,
                             MSelectionMask& objectMask) override;
    void updateSelectionGranularity(const MDagPath& dagPath,
                                    MSelectionContext& selectionContext) override;

  private:
    // A selected joint watches its own bodies' DAG ancestors (hide/show).
    static void dependencyDirty(MObject&, MPlug&, void* clientData);
    void invalidateDrawables();
    struct OutlineCandidate
    {
        MDagPath path;
        unsigned int roles = 0;
        bool visible = false; // resolved in updateDG, not while drawing
    };
    struct DependencyWatch
    {
        MObjectHandle node;
        MCallbackId callback = 0;
    };
    std::vector<DependencyWatch> fOutlineWatches;
    void updateOutlineWatches();
    struct BodyOutline
    {
        MDagPath path; // the connected pmxRigidBodyShape
        MPoint size;
        short type = 0;
        MMatrix world;
        bool visible = false; // resolved in updateDG, not while drawing
        bool valid = false;
        std::vector<OutlineCandidate> candidates;
    };
    std::array<BodyOutline, 2> fBodyOutlines;
    void drawIsolatedBodyOutlines(MHWRender::MUIDrawManager& drawManager,
                                  const MHWRender::MFrameContext& frameContext) const;

    // The joint's own plugs, resolved once (the attributes never change).  The
    // bodies are connections, read with RigidBodyJoint::bodyShape; the solver
    // is the `solver` message destination.
    struct Plugs
    {
        MPlug drawMode;
        MPlug solver;
        MPlug solverStep;
        MPlug hubRadius;
        MPlug active;
    };

    // Per-update data pulled from the joint node in updateDG().
    struct DrawState
    {
        short drawMode = 0;
        // Picked directly (rigid_body_display), not merely drawn as
        // selected because a parent group is: only a direct pick draws the
        // overlay.
        bool picked = false;
        // Limits, stiffness and bodies as the solver reads them (angles in
        // radians); the frame is the joint's rest frame, used to place the
        // attachment frames of a picked joint.
        mmd::core::RigidBodySimulation::JointDefinition definition;
        // The joint constrains something (JointStatus::eActive); an inactive
        // joint draws a hollow hub and, when picked, only dashed links to
        // the bodies it names.
        bool active = false;

        // The two constrained bodies' CURRENT poses and attachment frames
        // (world space), from the solver's live simulation — picked joints
        // only, because the hub's own pose is carried by the DAG transform.
        bool live = false;
        MMatrix attachmentAWorld;
        MMatrix attachmentBWorld;
        MPoint bodyAWorld;
        MPoint bodyBWorld;

        // What the joint permits and the guide sizes, resolved here: the hub
        // colour, the markers and the motion guides all read them, so they
        // cannot disagree.
        mmd::core::joint_display::Ranges ranges;
        mmd::core::joint_display::GuideSizes sizes{};
        // What the joint permits — the core freedom model (joint_display.hpp),
        // which the hub's colour encodes.  An inactive joint draws a hollow
        // hub instead of a colour.
        mmd::core::joint_display::Freedom freedom;
        // The rendered sphere radius: the model's hub size x the user's scale.
        double hubRadius = mmd::core::joint_display::kFallbackHubRadius;
    };

    // Resolve live frames and connected colliders for a picked overlay only.
    void resolveSolver();
    void cacheBodyOutlines(const std::array<MObject, 2>& bodyShapes);

    // What the joint permits, from the limits/springs the solver reads
    // (fills DrawState::ranges / sizes / freedom).
    void classifyState();
    // Hub colour of the even and odd wedges; they differ only for a joint
    // that rotates AND translates.
    void stateColors(MColor& primary, MColor& secondary) const;
    // Cached isSelected() answer: a viewport query every joint asks for on every
    // redraw, valid while the selection and the isolate state are unchanged
    // (rigid_body_display::displayStatusSerial).
    [[nodiscard]] bool selected(const MDagPath& dagPath);

    // The joint's own drawing, in the attachment-A frame, shared by the X-ray
    // and depth-tested passes: the guide segments (built once, so both passes
    // draw the same geometry) and whether the two attachments read as separate.
    struct OverlayGeom
    {
        std::vector<mmd::core::joint_display::GuideSegment> guides; // empty = none
        bool separate = false;
    };
    [[nodiscard]] OverlayGeom buildOverlayGeom() const;
    // The core guide builder's input, in the attachment-A frame (ranges and
    // sizes come from classifyState()).
    [[nodiscard]] mmd::core::joint_display::GuideInput motionGuideInput() const;
    void drawOverlay(MHWRender::MUIDrawManager& drawManager, float alpha,
                     const OverlayGeom& geom) const;
    static void
    drawGuideSegments(MHWRender::MUIDrawManager& drawManager,
                      const std::vector<mmd::core::joint_display::GuideSegment>& segments,
                      float alpha);
    // Three rings of `radius` (joint-LOCAL units) around `local`, in the
    // world's axis planes, each one line strip so a dash pattern runs round it.
    void drawRings(MHWRender::MUIDrawManager& drawManager, const MPoint& local, double radius,
                   int segments) const;

    MObject fJoint;
    Plugs fPlugs;
    MObject fSolver; // owning pmxRigidBodyNode (resolved in updateDG)
    DrawState fState;
    // Cached isSelected() answer, resolved in selected() from the draw phase
    // and valid while the selection and isolate state are unchanged
    // (rigid_body_display::displayStatusSerial).
    bool fSelected = false;
    bool fSelectedValid = false;
    std::uint64_t fSelectedSerial = 0;

    // A PICKED joint's overlay, in the node's LOCAL space (the frame the solver
    // drives to attachment A), resolved when the render items are updated so
    // the draw phase reads no DAG state.  Unpicked joints never fill them.
    MPoint fLocalA;
    MPoint fLocalB;
    MPoint fAttachmentB;
    // The joint's DAG matrix and its inverse: the picked overlay maps the live
    // frames through them, and it sizes the coincident-attachment test.
    MMatrix fLocalToWorld;
    MMatrix fWorldToLocal;
    // World X/Y/Z as unit directions in the joint's LOCAL space, cached with
    // fLocalToWorld: the markers are axis-aligned rings, built in local space so
    // they keep the hub's proportions even when the model root is scaled.
    std::array<MVector, 3> fWorldAxisLocal{};

    // populateGeometry()'s scratch streams: the shared unit hub scaled to the
    // joint's radius, reused across re-populates (no per-frame allocation).
    std::vector<float> fScratchPositions;
    std::vector<float> fScratchNormals;
    bool mGeometryDirty = true;
};
