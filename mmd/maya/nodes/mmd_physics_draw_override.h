/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_physics_draw_override.h
 *
 * MMDPhysicsDrawOverride — MPxDrawOverride that renders the rigid-body guide
 * visualization for mmdPhysicsNode.
 *
 * The node (an MPxLocatorNode) owns the Bullet world and its body state; this
 * override draws a wireframe box / sphere / capsule per body, colored by
 * collision group (the palette that used to shade Python-side guide meshes).
 * The geometry is pulled in prepareForDraw() from the node's CURRENT solver
 * state (solved world poses if the Bullet world is built, rest poses
 * otherwise), so the viewport always shows exactly what the simulation has —
 * the node is the single source of truth and no guide meshes/shaders are
 * needed in the scene.
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

class MMDPhysicsDrawOverride : public MHWRender::MPxDrawOverride
{
  public:
    static MHWRender::MPxDrawOverride* creator(const MObject& obj);

    ~MMDPhysicsDrawOverride() override;

    MHWRender::DrawAPI supportedDrawAPIs() const override;
    bool hasUIDrawables() const override;
    bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const override;
    MBoundingBox boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const override;

    MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath,
                              const MFrameContext& frameContext, MUserData* oldData) override;
    void addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawManager,
                        const MFrameContext& frameContext, const MUserData* data) override;

  private:
    explicit MMDPhysicsDrawOverride(const MObject& obj);
};
