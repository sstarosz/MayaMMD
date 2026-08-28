/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint_geometry_override.hpp
 *
 * RigidBodyJointGeometryOverride — lit viewport renderer for one
 * pmxRigidBodyJoint, parallel to RigidBodyShapeGeometryOverride for the
 * per-body colliders.
 *
 * The joint node's DAG transform IS the joint frame, so the glyph sits at
 * the LOCAL origin: a solid box spanning the joint's linear-limit range
 * (jointLinearMin/Max, flat-shaded), colour = PMX joint type, drawn with the
 * stock OpenPBR surface shader (k3dIsotropicOpenPBRSurfaceShader) so it is
 * lit by the viewport lights.  The WIRE outline is drawn with MUIDrawManager
 * (addUIDrawables), and the render item is pickable so the joint keeps its
 * native selection.
 *
 * Drawing (in the joint node's LOCAL space — the transform is the frame):
 *
 *   - solid -> one MRenderItem (NonMaterialSceneItem, OpenPBR surface
 *              shader, sDormantWireDepthPriority so it renders lit AND on
 *              top of the character mesh), position+normal(+uv) streams.
 *   - wire  -> MUIDrawManager lines (the same box outline).
 *
 * The draw mode (off / wire / solid / wire+solid) is the joint's `drawMode`
 * attribute; the colour comes from the PMX joint type (6-colour palette).
 * When a joint is selected the SOLID keeps its colour and a native
 * selection-colour WIREFRAME is drawn on top (same as any mesh).
 *
 * LIFECYCLE (per the devkit footPrintNode_GeometryOverride sample): the
 * render item is created once and appended to the override's persistent
 * render-item list — VP2 owns it, so the plugin never calls
 * MRenderItem::Destroy.  `cleanUp()` is empty; the stock shader is created
 * lazily and released in the destructor (while Maya is still alive).
 *
 * Registered by MayaMMD.mll under the pmxRigidBodyJoint classification.
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
#include <maya/MUIDrawManager.h>

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

    // Re-prepare EVERY frame: drawMode / joint type are DG changes that VP2's
    // DAG dirty tracking may otherwise miss.
    bool requiresGeometryUpdate() const override;
    bool requiresUpdateRenderItems(const MDagPath& dagPath) const override;

    bool hasUIDrawables() const override;

    void updateDG() override;
    void updateRenderItems(const MDagPath& dagPath, MRenderItemList& renderItems) override;
    void populateGeometry(const MGeometryRequirements& requirements,
                          const MRenderItemList& renderItems, MGeometry& data) override;
    void addUIDrawables(const MDagPath& objPath, MUIDrawManager& drawMgr,
                        const MFrameContext& frameContext) override;
    void cleanUp() override;

    // Selection — the solid render item is pickable; granularity is object-level.
    bool refineSelectionPath(const MSelectionInfo& selectInfo, const MRenderItem& hitItem,
                             MDagPath& dagPath, MObject& geomComponents,
                             MSelectionMask& objectMask) override;
    void updateSelectionGranularity(const MDagPath& dagPath,
                                    MSelectionContext& selectionContext) override;

  private:
    // Joint colour (palette by PMX joint type) — the SOLID always keeps it;
    // selection is shown as a wireframe overlay instead.
    void jointColor(float& r, float& g, float& b) const;
    // Selection helpers (render thread — displayStatus/wireframeColor safe).
    static bool isSelected(const MDagPath& path);
    static void selectionColor(const MDagPath& path, float& r, float& g, float& b);

    // Per-frame data pulled from the joint node in updateDG().
    struct DrawState
    {
        short drawMode = 0;
        short type = 0; // PMX JointType (0..5)
        MPoint linearMin;
        MPoint linearMax;
    };

    MObject fJoint;
    DrawState fState;
    MHWRender::MShaderInstance* fSolidShader = nullptr;
};
