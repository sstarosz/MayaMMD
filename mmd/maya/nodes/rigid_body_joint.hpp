/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint.hpp
 *
 * RigidBodyJoint — one selectable, movable Maya node per PMX rigid-body
 * JOINT (constraint), parallel to the per-body pmxRigidBodyShape.
 *
 * Each PMX joint becomes its own MPxLocatorNode under the model's
 * `{model}_Joints` sub-group (inside the `{model}_RigidBodies` group).  The
 * node is a plain data holder, exactly like pmxRigidBodyShape:
 *   - every PMX joint field — INCLUDING the FRAME
 *     (`jointRestTranslate` / `jointRestRotate`, in the bodies' rest space) —
 *     is a storable attribute, gathered in ONE `joint` compound that
 *     pmxRigidBodyConstraint connects into the solver's `joints[i]` input, so
 *     an edit travels as ordinary DG dirt and this node needs no notification
 *     logic.  The two BODIES are message connections from their
 *     pmxRigidBodyShape nodes (identity, not data).
 *   - the DAG transform above the node is the viewport GUIDE: placed at the
 *     rest frame by the command, then DRIVEN by the solver to the joint's
 *     live attachment on body A each frame
 *     (solver.outJointTranslate/outJointRotate -> transform.translate/rotate).
 *     The collider-style readout follows the animation, and its pivot sits on
 *     the drawn glyph.  To move the constraint, edit the rest-frame
 *     attributes, not the driven transform.
 *
 * The joint is drawn by RigidBodyJointGeometryOverride, registered for this
 * node's classification (`drawdb/geometry/pmxRigidBodyJoint`; see
 * docs/JointVisualization.md), with a per-node `drawMode` attribute.  The
 * node has no compute() — the solver reads the attributes through its input.
 *
 * The solver supplies the guide pose and the hidden jointDisplay inputs
 * (radius, activity and local bounds). Drawing state belongs to the geometry
 * override. This node has no private runtime state or solver binding.
 *
 * Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MBoundingBox.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MTypeId.h>

// ===========================================================================
// RigidBodyJoint
// ===========================================================================
class RigidBodyJoint : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "pmxRigidBodyJoint";
    // VP2 draw-database classification — the geometry override is registered
    // under this string.
    static constexpr const char* kNodeClassify = "drawdb/geometry/pmxRigidBodyJoint";
    // Custom selection type for the hub.  As a plain locator the hub ranks at
    // or below a Maya joint, so the bone it usually sits on always won the
    // pick.  The bone sphere is larger, so it stays pickable around the hub.
    static constexpr const char* kSelectionType = "pmxRigidBodyJoint";
    static constexpr int kSelectionPriority = 10;

    // Viewport draw mode (see docs/JointVisualization.md):
    //   0 Off    — nothing
    //   1 Hub    — the state-coloured hub; when selected, the A/B attachments
    //   2 Ranges — Hub, plus the selected joint's motion-range guides
    enum DrawMode : short
    {
        kDrawOff = 0,
        kDrawHub = 1,
        kDrawRanges = 2,
    };

    RigidBodyJoint();
    ~RigidBodyJoint() override;

    // MPxNode overrides — data-holder only (no compute()).
    // Bounded by what the joint draws around the node's origin: the node's
    // transform IS the live attachment on body A, so the box is static per
    // configuration (the limits and the hub radius) and does not follow the
    // pose — the DAG transform does.
    bool isBounded() const override { return true; }
    MBoundingBox boundingBox() const override;

    // Forward this node's input changes to the drawing implementation in both
    // DG and Evaluation Manager modes. No display state is kept on the node.
    MStatus setDependentsDirty(const MPlug& plug, MPlugArray& affected) override;
    MStatus preEvaluation(const MDGContext& context,
                          const MEvaluationNode& evaluationNode) override;

    // Registration helpers
    static void* creator();
    static MStatus initialize();

    // The pmxRigidBodyShape connected to `end` (aJointBodyA / aJointBodyB);
    // null when the end is unconnected or not a rigid body.
    static MObject bodyShape(const MObject& joint, const MObject& end);

    // ------------------------------------------------------------------
    // Attributes — the PMX joint data, including the FRAME.  The frame is
    // `jointRestTranslate` / `jointRestRotate` (the bodies' rest space, world
    // at import); the DAG transform above the node is a solver-driven
    // READOUT, not the frame (see the file header).
    // ------------------------------------------------------------------
    static MObject aSolver; // message -> owning pmxRigidBodyNode (wired at import)
    // Input <- solver.outStep: ordering for selected-joint overlays only.
    static MObject aSolverStep;
    // Derived display inputs <- solver.outJointDisplay[i]. These are separate
    // from the authored PMX compound and do not depend on simulation time.
    static MObject aJointDisplay;
    static MObject aJointHubRadius;
    static MObject aJointActive;
    static MObject aJointBoundsMin;
    static MObject aJointBoundsMax;
    // Per-joint simulation values, gathered in ONE compound that
    // pmxRigidBodyConstraint connects into the solver's `joints[i]`: an edit
    // is ordinary DG dirt along that edge (no attributeAffects relay).
    static MObject aJoint;
    static MObject aJointRestTranslate; // float3 — rest frame position (bodies' rest space)
    static MObject aJointRestRotate;    // float3 — rest frame XYZ rotation, degrees
    static MObject aJointNameLocal;     // string — PMX name_local; "" = none
    static MObject aJointNameUniversal; // string — PMX name_universal; "" = none
    // message <- body A / B's pmxRigidBodyShape.message (PMX
    // rigid_body_index_a / _b, linked by node so it survives body edits).
    static MObject aJointBodyA;
    static MObject aJointBodyB;
    static MObject aJointLinearMin; // float3 — linear limit min (joint-frame offsets)
    static MObject aJointLinearMax; // float3 — linear limit max (joint-frame offsets)
    static MObject
        aJointAngularMin; // angle3 — angular limit min (radians internally, UI-unit in scripts/AE)
    static MObject aJointAngularMax; // angle3 — angular limit max
    static MObject
        aJointLinearStiffness; // float3 — linear axis stiffness (Bullet spring stiffness)
    static MObject
        aJointAngularStiffness; // float3 — angular axis stiffness (Bullet spring stiffness)
    static MObject aDrawMode;   // enum — viewport draw mode (DrawMode)
};
