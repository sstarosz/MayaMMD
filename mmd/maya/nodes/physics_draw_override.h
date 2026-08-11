/*
 * SPDX-License-Identifier: MIT
 *
 * physics_draw_override.h
 *
 * PhysicsDrawOverride — MPxDrawOverride that renders the rigid-body guide
 * visualization for pmxPhysicsNode.
 *
 * The node (an MPxLocatorNode) owns the Bullet world and its body state; this
 * override draws a box / sphere / capsule per body, colored by collision
 * group (the palette that used to shade Python-side guide meshes).  The
 * geometry is pulled in prepareForDraw() from the node's CURRENT solver state
 * (solved world poses if the Bullet world is built, rest poses otherwise), so
 * the viewport always shows exactly what the simulation has — the node is the
 * single source of truth and no guide meshes/shaders are needed in the scene.
 *
 * DRAW SPACE: the Bullet world runs in WORLD space, so collectDrawData()
 * returns world-space body poses.  addUIDrawables() draws in the locator's
 * OBJECT space, so prepareForDraw() captures the node's world-inverse matrix
 * and the draw code transforms every body into object space (world * inverse).
 * This keeps the guides glued to the colliders even when the model root or
 * physics group is moved/scaled.
 *
 * DRAW STYLE is a per-node view attribute (`drawMode`:
 * off/wireframe/solid/wireframeAndSolid) plus `drawOpacity`.  Both are
 * deliberately NOT part of the node's config comparison, so changing them
 * never rebuilds the Bullet world.
 *
 * SELECTION: each body is pickable.  updateSelectionGranularity() declares
 * component-level picking and wantUserSelection() opts into the custom
 * userSelect() path, which ray-casts the pick against the collider geometry
 * (mmd::core::collider_geometry) and writes the hit body's index into the
 * node's `uiSelectedBodyIndex` attribute — the bridge the AE template reads to
 * show that body's properties.  The picked body is drawn in the Maya
 * selection/hilite color.
 *
 * Registered by MayaMMD.mll's initializePlugin alongside the node.
 */

#pragma once

#include <maya/MBoundingBox.h>
#include <maya/MDagPath.h>
#include <maya/MFrameContext.h>
#include <maya/MObject.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>

class PhysicsDrawOverride : public MHWRender::MPxDrawOverride
{
  public:
    static MHWRender::MPxDrawOverride* creator(const MObject& obj);

    ~PhysicsDrawOverride() override = default;

    MHWRender::DrawAPI supportedDrawAPIs() const override;
    bool hasUIDrawables() const override;
    bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const override;
    MBoundingBox boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const override;

    MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath,
                              const MFrameContext& frameContext, MUserData* oldData) override;
    void addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawManager,
                        const MFrameContext& frameContext, const MUserData* data) override;

    // ── Selection support ──
    // Declare component-level picking so individual bodies are selectable.
    void updateSelectionGranularity(const MDagPath& objPath,
                                    MSelectionContext& selectionContext) override;
    // Opt into the custom selection path (see the class comment).
    bool wantUserSelection() const override;
    // Ray-cast the pick against the colliders, select the node and store the
    // hit body index in uiSelectedBodyIndex.
    bool userSelect(const MSelectionInfo& selectInfo, const MDrawContext& context,
                    const MDagPath& objPath, const MUserData* data,
                    MSelectionList& selectionList, MPointArray& worldSpaceHitPts) override;

  private:
    explicit PhysicsDrawOverride(const MObject& obj);
};
