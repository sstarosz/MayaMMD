/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape_geometry_override.hpp
 *
 * RigidBodyShapeGeometryOverride — viewport renderer for one pmxRigidBodyShape.
 *
 * Replaces the old flat-solid MUIDrawManager draw override with a proper
 * MPxGeometryOverride so the collider's SOLID surface is lit by the
 * viewport lights (stock k3dBlinn shader, per-frame group colour).  The
 * WIRE outline is still drawn with MUIDrawManager (addUIDrawables), and the
 * render item is pickable so the guide keeps its native selection.
 *
 * Drawing (in the guide's LOCAL space — the guide transform is the body's
 * current pose, so the collider sits at the local origin):
 *
 *   - solid   -> one MRenderItem (NonMaterialSceneItem, k3dBlinnShader,
 *                sDormantWireDepthPriority so it renders lit AND on top of
 *                the character mesh), position+normal streams.
 *   - wire    -> MUIDrawManager lines (same primitive wireframe as before).
 *
 * The draw mode (off / wire / solid / wire+solid) is the shape's `drawMode`
 * attribute; the colour comes from the PMX collision group (16-colour
 * palette), kinematic bodies are dimmed, disabled bodies greyed, and the
 * selected guide is highlighted with Maya's selection colour.
 *
 * Registered by MayaMMD.mll under the pmxRigidBodyShape classification.
 */

#pragma once

#include <maya/MDagPath.h>
#include <maya/MFrameContext.h>
#include <maya/MGeometry.h>
#include <maya/MHWGeometry.h>
#include <maya/MObject.h>
#include <maya/MPoint.h>
#include <maya/MPxGeometryOverride.h>
#include <maya/MShaderManager.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MUIDrawManager.h>

#include <array>

// ===========================================================================
// RigidBodyShapeGeometryOverride
// ===========================================================================
class RigidBodyShapeGeometryOverride : public MHWRender::MPxGeometryOverride
{
  public:
    static MHWRender::MPxGeometryOverride* creator(const MObject& obj);

    explicit RigidBodyShapeGeometryOverride(const MObject& obj);
    ~RigidBodyShapeGeometryOverride() override;

    MHWRender::DrawAPI supportedDrawAPIs() const override;

    // Always re-prepare: the group colour depends on the selection status
    // (displayStatus), which is not a DAG change on the shape node.
    bool requiresGeometryUpdate() const override;

    bool hasUIDrawables() const override;

    void updateDG() override;
    void updateRenderItems(const MDagPath& dagPath, MRenderItemList& renderItems) override;
    void addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawMgr,
                        const MFrameContext& frameContext) override;
    void populateGeometry(const MGeometryRequirements& requirements,
                          const MRenderItemList& renderItems, MGeometry& data) override;
    void cleanUp() override;

    // Selection — the solid render item is pickable; granularity is object-level.
    bool refineSelectionPath(const MSelectionInfo& selectInfo, const MRenderItem& hitItem,
                             MDagPath& dagPath, MObject& geomComponents,
                             MSelectionMask& objectMask) override;
    void updateSelectionGranularity(const MDagPath& dagPath,
                                    MSelectionContext& selectionContext) override;

  private:
    // Per-frame data pulled from the shape node in updateDG().
    struct DrawState
    {
        short drawMode = 0;
        short colliderType = 1;
        MPoint size; // PMX shape_size (box = half-extent verbatim)
        float color[3] = {1.00f, 0.25f, 0.25f};
    };

    MObject fShape;
    DrawState fState;
    MShaderInstance* fSolidShader = nullptr;
    MRenderItem* fSolidItem = nullptr;
};
