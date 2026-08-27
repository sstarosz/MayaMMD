/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape_geometry_override.hpp
 *
 * RigidBodyShapeGeometryOverride — viewport renderer for one pmxRigidBodyShape.
 *
 * Replaces the old flat-solid MUIDrawManager draw override with a proper
 * MPxGeometryOverride so the collider's SOLID surface is lit by the
 * viewport lights (stock k3dIsotropicOpenPBRSurfaceShader — the same OpenPBR
 * material Maya 2026+ uses for its default shader — with a per-frame
 * `baseColor` group colour).  The WIRE outline is still drawn with
 * MUIDrawManager (addUIDrawables), and the render item is pickable so the
 * guide keeps its native selection.
 *
 * Drawing (in the guide's LOCAL space — the guide transform is the body's
 * current pose, so the collider sits at the local origin):
 *
 *   - solid   -> one MRenderItem (NonMaterialSceneItem, OpenPBR surface
 *                shader, sDormantWireDepthPriority so it renders lit AND on
 *                top of the character mesh), position+normal(+uv) streams.
 *   - wire    -> MUIDrawManager lines (same primitive wireframe as before).
 *
 * The draw mode (off / wire / solid / wire+solid) is the shape's `drawMode`
 * attribute; the colour comes from the PMX collision group (16-colour
 * palette), kinematic bodies are dimmed and disabled bodies greyed.  When a
 * guide is selected the SOLID keeps its group colour and a native
 * selection-colour WIREFRAME is drawn on top (same as any mesh).
 *
 * LIFECYCLE (per the devkit footPrintNode_GeometryOverride sample): the
 * render item is created once and appended to the override's persistent
 * render-item list — VP2 owns it, so the plugin never calls
 * MRenderItem::Destroy.  `cleanUp()` is empty; the stock shader is created
 * lazily and released in the destructor (while Maya is still alive — a
 * release at DLL-exit time can crash Maya).
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

    // Re-prepare EVERY frame: drawMode / group colour / selection status are
    // DG (not DAG) changes on the shape node, which VP2's default dirty
    // tracking may otherwise miss (the solid then stays disabled and the
    // collider disappears when the user switches to Solid).  The meshes are
    // tiny, so the per-frame rebuild cost is negligible.
    bool requiresGeometryUpdate() const override;
    bool requiresUpdateRenderItems(const MDagPath& dagPath) const override;

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
    // Group colour (palette + kinematic dim + disabled grey) — the SOLID
    // always keeps this; selection is shown as a wireframe overlay instead.
    void groupColor(float& r, float& g, float& b) const;
    // Selection helpers (render thread — displayStatus/wireframeColor safe).
    static bool isSelected(const MDagPath& path);
    static void selectionColor(const MDagPath& path, float& r, float& g, float& b);

    // Per-frame data pulled from the shape node in updateDG().
    struct DrawState
    {
        short drawMode = 0;
        short colliderType = 1;
        MPoint size; // PMX shape_size (box = half-extent verbatim)
        short groupId = 0;
        short physicsMode = 0;
        bool enabled = true;
    };

    MObject fShape;
    DrawState fState;
    MShaderInstance* fSolidShader = nullptr;
};
