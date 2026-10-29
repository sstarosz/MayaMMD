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
 * Selection events and the shared surface shaders live in
 * rigid_body_display.  Simulation steps reach the joint through its
 * solverStep input, and RigidBodyJoint marks the drawing dirty.  VP2 owns the
 * render items.
 *
 * Registered by MayaMMD.mll under the pmxRigidBodyJoint classification.
 */

#pragma once

#include <maya/MColor.h>
#include <maya/MDagPath.h>
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

    explicit RigidBodyJointGeometryOverride(const MObject& obj);
    ~RigidBodyJointGeometryOverride() override;

    MHWRender::DrawAPI supportedDrawAPIs() const override;

    // Update only when the joint is dirtied (see the .cpp): the hub follows
    // body A because each solver step dirties the joint, not by polling
    // every refresh.
    bool requiresGeometryUpdate() const override;
    bool requiresUpdateRenderItems(const MDagPath& dagPath) const override;

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
    BodyOutline fBodyOutlines[2];
    void drawIsolatedBodyOutlines(MHWRender::MUIDrawManager& drawManager,
                                  const MHWRender::MFrameContext& frameContext) const;

    // The joint's plugs, resolved once (the attributes never change).  The
    // bodies are connections, read with RigidBodyJoint::bodyShape.
    struct Plugs
    {
        MPlug drawMode;
        MPlug linearMin, linearMax, angularMin, angularMax;
        MPlug linearStiffness, angularStiffness;
        MPlug solver, solverStep;
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
        // radians); the frame is only filled for the solver query.
        mmd::core::RigidBodySimulation::JointDefinition definition;
        // The joint constrains something (JointStatus::eActive); an inactive
        // joint draws a hollow hub and, when picked, only dashed links to
        // the bodies it names.
        bool active = false;

        // The two constrained bodies' CURRENT poses and attachment frames
        // (world space), from the solver's live simulation.
        bool hasAttachments = false;
        MMatrix attachmentAWorld;
        MMatrix attachmentBWorld;
        MPoint bodyAWorld;
        MPoint bodyBWorld;

        // What the joint permits and the guide sizes, resolved once here: the
        // hub colour, the markers and the motion guides all read them, so they
        // cannot disagree and are not recomputed per overlay pass.
        mmd::core::joint_display::Ranges ranges;
        mmd::core::joint_display::GuideSizes sizes{};
        // What the joint permits — the core freedom model (joint_display.hpp),
        // which the hub's colour encodes.  An inactive joint draws a hollow
        // hub instead of a colour.
        mmd::core::joint_display::Freedom freedom;
        // The rendered sphere radius: the model's hub size x the user's scale.
        double hubRadius = mmd::core::joint_display::kFallbackHubRadius;
    };

    // updateDG() steps: read the node's own inputs, resolve the owning solver's
    // live view of this joint, then cache the connected colliders (selected
    // joints only).
    void readJointInputs();
    void resolveSolver();
    void cacheBodyOutlines(const MObject (&bodyShapes)[2]);

    // What the joint permits, from the limits/springs the solver reads
    // (fills DrawState::ranges / sizes / freedom).
    void classifyState();
    // Hub colour of the even and odd wedges; they differ only for a joint
    // that rotates AND translates.
    void stateColors(MColor& primary, MColor& secondary) const;

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
    void drawGuideSegments(MHWRender::MUIDrawManager& drawManager,
                           const std::vector<mmd::core::joint_display::GuideSegment>& segments,
                           float alpha) const;
    // Three rings of `radius` (joint-LOCAL units) around `local`, in the
    // world's axis planes, each one line strip so a dash pattern runs round it.
    void drawRings(MHWRender::MUIDrawManager& drawManager, const MPoint& local, double radius,
                   int segments) const;

    MObject fJoint;
    Plugs fPlugs;
    MObject fSolver; // owning pmxRigidBodyNode (resolved in updateDG)
    DrawState fState;

    // Body centres and attachment points in the node's LOCAL space (the space
    // the render item's geometry is built in — VP2 applies the joint's DAG matrix).
    MPoint fLocalA;
    MPoint fLocalB;
    MPoint fAttachmentA;
    MPoint fAttachmentB;
    MMatrix fHubToLocal;
    // The joint's DAG matrix and its inverse, both cached in
    // updateRenderItems(): the draw phase reads no DAG state.
    MMatrix fLocalToWorld;
    MMatrix fWorldToLocal;
    // World X/Y/Z as unit directions in the joint's LOCAL space, cached with
    // fLocalToWorld: the markers are axis-aligned rings, built in local space so
    // they keep the hub's proportions even when the model root is scaled.
    MVector fWorldAxisLocal[3];
};
