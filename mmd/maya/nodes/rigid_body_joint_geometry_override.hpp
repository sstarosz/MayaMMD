/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint_geometry_override.hpp
 *
 * RigidBodyJointGeometryOverride — lit viewport renderer for one
 * pmxRigidBodyJoint, parallel to RigidBodyShapeGeometryOverride for the
 * per-body colliders.
 *
 * The joint node's guide transform IS the joint frame, so the glyph sits at
 * the LOCAL origin.  A joint is a rigid CONSTRAINT BETWEEN TWO BODIES, so the
 * visualization leads with that relationship:
 *
 *   - connector -> two thick lines from the two constrained bodies' CURRENT
 *                  positions (read from the solver's outGuideTranslate[]
 *                  outputs, mapped into the shape's local space) through the
 *                  joint frame — you can see at a glance which two bodies a
 *                  joint links, live.
 *   - solid     -> a small anchor cube at the frame, lit with the stock OpenPBR
 *                  surface shader (k3dIsotropicOpenPBRSurfaceShader) and
 *                  coloured by PMX joint type; it is the pickable/selectable
 *                  element.
 *   - wire      -> the linear-limit range as a thin wire box + the frame's
 *                  X/Y/Z orientation triad, via MUIDrawManager.
 *
 * The draw mode (off / wire / solid / wire+solid) is the joint's `drawMode`
 * attribute; the colour comes from the PMX joint type (6-colour palette).
 * When a joint is selected the anchor keeps its colour and a native
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
        // The two constrained bodies' CURRENT positions (RigidBodies GROUP
        // space), from the solver's outGuideTranslate outputs — the connector
        // shows which two bodies this joint links.
        MPoint bodyAPos;
        MPoint bodyBPos;
        bool hasBodyA = false;
        bool hasBodyB = false;
        double anchorHalf = 0.04; // solid anchor-cube half-extent (adaptive)
    };

    MObject fJoint;
    MObject m_solver; // owning pmxRigidBodyNode (resolved in updateDG)
    DrawState fState;
    // Group-space body poses -> the shape's LOCAL space (for the connector
    // lines), computed in updateRenderItems() from the DAG-path matrices.
    MMatrix m_toLocal;
    MHWRender::MShaderInstance* fSolidShader = nullptr;
};
