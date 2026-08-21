/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape_draw_override.hpp
 *
 * RigidBodyShapeDrawOverride — viewport draw override for one pmxRigidBodyShape.
 *
 * Draws the body's collider (box / sphere / capsule) in the guide's LOCAL
 * space (the guide transform is the body's rest pose, so the collider sits at
 * the local origin and moves with the guide — native Move-tool editing shows
 * up immediately).  The draw mode (none / wire / solid / wire+solid) is the
 * shape's `drawMode` attribute; the color comes from the PMX collision group.
 *
 * Registered by MayaMMD.mll under the pmxRigidBodyShape classification.
 */

#pragma once

#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>

// ===========================================================================
// RigidBodyShapeDrawOverride
// ===========================================================================
class RigidBodyShapeDrawOverride : public MHWRender::MPxDrawOverride
{
  public:
    static MHWRender::MPxDrawOverride* creator(const MObject& obj);

    explicit RigidBodyShapeDrawOverride(const MObject& obj);
    ~RigidBodyShapeDrawOverride() override;

    MHWRender::DrawAPI supportedDrawAPIs() const override;

    bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const override;
    bool hasUIDrawables() const override;

    // No per-frame work — addUIDrawables reads the attributes live.
    MUserData* prepareForDraw(const MDagPath& objPath, const MDagPath& cameraPath,
                              const MHWRender::MFrameContext& frameContext,
                              MUserData* oldData) override;

    void addUIDrawables(const MDagPath& objPath, MHWRender::MUIDrawManager& drawMgr,
                        const MHWRender::MFrameContext& frameContext,
                        const MUserData* data) override;
};
